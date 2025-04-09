#include "aerocalcs/aerocalcsingle.hpp"
#include "central_difference.hpp"
#include "compTask.hpp"
#include "infMat.hpp"
#include "mat_reader/mat_reader.hpp"
#include "panel_geo/panel_geo.hpp"
#include "panel_method/source_doublet_single.hpp"
#include "singularity/const_doublet.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/const_source.hpp"
#include "singularity/const_source_far.hpp"
#include "solver/bicgstab_solver.hpp"
#include "solver/dense_solver.hpp"
#include "solver/gmres_solver.hpp"
#include "solver/gmres_solver_ilu.hpp"
#include "solver/sparse_solver.hpp"
#include "surface/surface_panel.hpp"
#include "surface/surface_reader.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <concepts>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <numeric>
#include <ranges>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include "helpers.hpp"
#include "post_processing.hpp"
// #define RANGE(n) views::iota(0, (int)n)
//
namespace fs = std::filesystem;
using ComputeTaskPair =
    std::pair<std::vector<ComputeTask>, std::vector<ComputeTask>>;



EvalPoints<double>
create_eval_points(std::span<const PanelGeometryPair> panelGeometries) {
  auto totalEvalPoints =
      std::accumulate(panelGeometries.begin(), panelGeometries.end(), 0,
                      [](std::size_t count, const PanelGeometryPair &panelGeo) {
                        return count + panelGeo.first.centrePoints.rows();
                      });

  EvalPoints<double> evalPoints(totalEvalPoints);
  std::size_t iPoints = 0;
  auto pointsView = panelGeometries | std::views::transform([](auto const &g) {
                      return g.first.centrePoints;
                    });
  std::ranges::for_each(pointsView, [&](auto const &pts) {
    const auto rows = pts.rows();
    evalPoints.mEvalPoints.middleRows(iPoints, rows) = pts;
    iPoints += rows;
  });
  return evalPoints;
}
ComputeTaskPair makeComputeTasksPairImpl(const PanelGeometryPair &panelGeometry,
                                         const EvalPoints<double> &evalPoints) {
  int surfPanels = panelGeometry.first.centrePoints.rows();
  int wakePanels = panelGeometry.second.centrePoints.rows();
  auto surfaceComputeTaskView =
      RANGE(surfPanels) |
      views::transform([&panelGeometry, &evalPoints](int faceIdx) {
        return createInfluenceComputeTask(panelGeometry.first, evalPoints,
                                          faceIdx);
      });
  auto wakeComputeTaskView =
      RANGE(wakePanels) |
      views::transform([&panelGeometry, &evalPoints](int faceIdx) {
        return createInfluenceComputeTask(panelGeometry.second, evalPoints,
                                          faceIdx);
      });

  ComputeTaskPair compTaskPair;
  compTaskPair.first.reserve(surfPanels);
  compTaskPair.second.reserve(wakePanels);
  std::ranges::copy(surfaceComputeTaskView,
                    std::back_inserter(compTaskPair.first));
  std::ranges::copy(wakeComputeTaskView,
                    std::back_inserter(compTaskPair.second));
  return compTaskPair;
};

auto makeChunkData(const auto &panelGeometry) {

  auto chunkSize = panelGeometry | views::transform([](const auto &pg) {
                     return pg.centrePoints.rows();
                   });
  std::vector<size_t> chunkStart(chunkSize.size(), 0);
  std::exclusive_scan(chunkSize.begin(), chunkSize.end(), chunkStart.begin(),
                      0);
  return std::pair{chunkStart, chunkSize};
}
std::vector<ComputeTaskPair>
makeComputeTaskPairs(std::span<PanelGeometryPair> panelGeometries,
                     const EvalPoints<double> &evalPoints) {

  int nBodies = panelGeometries.size();

  auto pairs =
      RANGE(nBodies) |
      views::transform([&panelGeometries, &evalPoints](int iBody) {
        return makeComputeTasksPairImpl(panelGeometries[iBody], evalPoints);
      });

  std::vector<ComputeTaskPair> compTaskPairs;
  compTaskPairs.reserve(nBodies);
  std::ranges::copy(pairs, std::back_inserter(compTaskPairs));

  // Generate correct faceIdx for both surfaces for 0 to the total number of
  // panels
  auto [surfChunkStart, surfChunkSize] =
      makeChunkData(panelGeometries | views::transform([](const auto &pgPair) {
                      return pgPair.first;
                    }));
  auto surfFaceIdxView =
      RANGE(nBodies) | views::transform([&](auto idx) {
        return views::iota(surfChunkStart[idx],
                           surfChunkStart[idx] + surfChunkSize[idx]);
      });

  for (auto i : RANGE(nBodies)) {
    for (auto j : RANGE(compTaskPairs[i].first.size())) {
      compTaskPairs[i].first[j].face.faceIdx = surfFaceIdxView[i][j];
    }
  }
  return compTaskPairs;
}
Eigen::MatrixXd
assembleLhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                std::span<const ComputeTask> wakePanelCompTasks,
                const PanelGeometry<WakePanel> &wakePanelGeo,
                const EvalPoints<double> &evalPoints) {

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  std::size_t surfDims = surfacePanelCompTasks.size();
  std::size_t wakeDims = wakePanelCompTasks.size();
  print("Eval Dims: ", evalDims, "Surf Dims: ", surfDims,
        "Wake Dims: ", wakeDims);
  Eigen::MatrixXd surfaceInfluenceMatrix = makeInfluenceMatrix<DoubletP, true>(
      evalDims, surfDims, surfacePanelCompTasks);

  if (wakeDims == 0) {
    return surfaceInfluenceMatrix;
  }
  Eigen::MatrixXd wakeInfluenceMatrix = makeInfluenceMatrix<DoubletP, false>(
      evalDims, wakeDims, wakePanelCompTasks);
  //  combine source and wake
  for (std::size_t iWakeP = 0;
       iWakeP < wakePanelGeo.mSurface.mTrailingEdgeIdx.rows(); iWakeP++) {

    // NOTE: bad for cache?

    int lowerFaceIdx = wakePanelGeo.mSurface.mTrailingEdgeIdx(iWakeP, 0);
    int upperFaceIdx = wakePanelGeo.mSurface.mTrailingEdgeIdx(iWakeP, 1);
    surfaceInfluenceMatrix(Eigen::placeholders::all, lowerFaceIdx) -=
        wakeInfluenceMatrix(Eigen::placeholders::all, iWakeP);
    surfaceInfluenceMatrix(Eigen::placeholders::all, upperFaceIdx) +=
        wakeInfluenceMatrix(Eigen::placeholders::all, iWakeP);
  }
  return surfaceInfluenceMatrix;
}

Eigen::MatrixXd assembleLhs(std::span<const ComputeTaskPair> compTaskPairs,
                            std::span<const PanelGeometryPair> panelGeometries,
                            const EvalPoints<double> &evalPoints) {

  auto lhsView =
      RANGE(compTaskPairs.size()) | views::transform([&](std::size_t idx) {
        return assembleLhsImpl(compTaskPairs[idx].first,
                               compTaskPairs[idx].second,
                               panelGeometries[idx].second, evalPoints);
      });

  std::size_t mDims = evalPoints.mEvalPoints.rows();
  Eigen::MatrixXd lhs(mDims, mDims);
  std::size_t iPoints = 0;
  std::ranges::for_each(lhsView, [&](const Eigen::Ref<const MatrixXd> &pts) {
    const auto cols = pts.cols();
    lhs.middleCols(iPoints, cols) = pts;
    iPoints += cols;
  });
  savetxt("lhs.txt", lhs);
  return lhs;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
assembleRhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                const EvalPoints<double> &evalPoints,
                const Eigen::Ref<Eigen::Array3d> &freeStream) {

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  std::size_t surfDims = surfacePanelCompTasks.size();
  Eigen::MatrixXd sourceInfluenceMat = makeInfluenceMatrix<SourceP, true>(
      evalDims, surfDims, surfacePanelCompTasks);
  Eigen::VectorXd sourceStrength =
      rowwiseDotProduct(surfacePanelGeo.normalVectors, freeStream);
  savetxt("rhs.txt", sourceInfluenceMat);
  return {-(sourceInfluenceMat * sourceStrength), sourceStrength};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
assembleRhs(std::span<const ComputeTaskPair> compTaskPairs,
            std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<double> &evalPoints,
            const Eigen::Ref<Eigen::Array3d> &freeStream) {

  auto rhsView =
      RANGE(compTaskPairs.size()) | views::transform([&](std::size_t idx) {
        return assembleRhsImpl(compTaskPairs[idx].first,
                               panelGeometries[idx].first, evalPoints,
                               freeStream);
      });

  std::size_t mDims = evalPoints.mEvalPoints.rows();
  Eigen::VectorXd rhs(mDims);

  rhs.setZero();
  Eigen::VectorXd sourceStrength(mDims);
  std::size_t iPoints = 0;
  std::ranges::for_each(rhsView, [&](const auto &pts) {
    rhs += pts.first;
    const auto rows = pts.second.rows();
    sourceStrength.middleRows(iPoints, rows) = pts.second;
    iPoints += rows;
  });
  return {rhs, sourceStrength};
}
auto parse_param(const fs::path &fpath) {
  std::ifstream pFile(fpath);
  if (!pFile.is_open()) {
    std::cerr << "Error opening params file: " << fpath.string() << std::endl;
  }
  FlowParams flowParams = {0, 1, 1};
  ReferenceGeom refGeom = {0};
  bool haveaoa = false;
  bool haveS = false;
  std::string line;

  while (std::getline(pFile, line)) {
    // Remove anything after "//"
    std::size_t pos = line.find("//");
    if (pos != std::string::npos) {
      line = line.substr(0, pos);
    }

    // Trim leading/trailing whitespace (simple approach)
    // You can write a more robust trim if needed.
    while (!line.empty() && (line.front() == ' ' || line.front() == '\t')) {
      line.erase(line.begin());
    }
    while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) {
      line.pop_back();
    }

    // Skip empty lines or lines that begin with '#'
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // First valid numeric line -> aoa, second -> S
    if (!haveaoa) {
      print("aoa: ", line.c_str());
      flowParams.aoa = std::atof(line.c_str());
      haveaoa = true;
    } else if (!haveS) {
      refGeom.refArea = std::atof(line.c_str());
      haveS = true;
    }
  }

  pFile.close();
  return std::make_pair(flowParams, refGeom);
}
auto parse_param_batch(const fs::path &fpath) {
  std::ifstream pFile(fpath);
  if (!pFile.is_open()) {
    std::cerr << "Error opening params file: " << fpath.string() << std::endl;
  }
  std::vector<FlowParams> flowParams;
  ReferenceGeom refGeom = {0};
  bool haveaoa = false;
  bool haveS = false;
  std::string line;

  while (std::getline(pFile, line)) {
    // Remove anything after "//"
    std::size_t pos = line.find("//");
    if (pos != std::string::npos) {
      line = line.substr(0, pos);
    }

    // Trim leading/trailing whitespace (simple approach)
    // You can write a more robust trim if needed.
    while (!line.empty() && (line.front() == ' ' || line.front() == '\t')) {
      line.erase(line.begin());
    }
    while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) {
      line.pop_back();
    }

    // Skip empty lines or lines that begin with '#'
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // First valid numeric line -> aoa, second -> S
    if (!haveaoa) {
      auto aoaS = split(line, " ");
      std::ranges::copy(
          aoaS | views::transform([](const std::string &aoa) -> double {
            return std::atof(aoa.c_str());
          }) | views::transform([](const auto &val) -> FlowParams {
            return {val, 1, 1};
          }),
          std::back_inserter(flowParams));
      haveaoa = true;
    } else if (!haveS) {
      refGeom.refArea = std::atof(line.c_str());
      haveS = true;
    }
  }

  pFile.close();
  return std::make_pair(flowParams, refGeom);
}
void writeBodyData(const std::string outfile, const PanelGeometryPair &ppair,
                   AeroResults &results) {

  std::vector<std::string> headers = {"x",   "y",   "z",   "A",  "dCp",
                                      "dVx", "dVy", "dVz", "dP", "dFx",
                                      "dFy", "dFz", "mu"};
  savetxt(outfile,
          (Eigen::ArrayXXd(ppair.first.centrePoints.rows(), headers.size())
               << ppair.first.centrePoints.col(0),
           ppair.first.centrePoints.col(1), ppair.first.centrePoints.col(2),
           ppair.first.areas, results.panelResults["dCp"],
           results.panelResults["dVx"], results.panelResults["dVy"],
           results.panelResults["dVz"], results.panelResults["dP"],
           results.panelResults["dFx"], results.panelResults["dFy"],
           results.panelResults["dFz"], results.panelResults["mu"])
              .finished(),
          " ", headers);
}
auto postProcessTotalPolars(auto &&outO) {
  auto out = std::move(outO);
  double q =
      0.5 * out.lastParams.rho * out.lastParams.Vinf * out.lastParams.Vinf;
  Eigen::Array3d F(3);
  F << out.polars["Fx"], out.polars["Fy"], out.polars["Fz"];
  Eigen::ArrayXd CF = F / (q * out.refGeom.refArea);
  double CL = (-CF(0) * std::sin(out.lastParams.aoa * M_PI / 180) +
               CF(2) * std::cos(out.lastParams.aoa * M_PI / 180));
  double CD = (CF(0) * std::cos(out.lastParams.aoa * M_PI / 180) +
               CF(2) * std::sin(out.lastParams.aoa * M_PI / 180));

  out.polars["aoa"] = out.lastParams.aoa;
  out.polars["CFx"] = CF(0);
  out.polars["CFy"] = CF(1);
  out.polars["CFz"] = CF(2);
  out.polars["CL"] = CL;
  out.polars["CD"] = CD;
  return out;
}

template <typename R>
concept AeroResults_range =
    std::ranges::range<R> &&
    std::is_same_v<std::ranges::range_value_t<R>, AeroResults>;

template <typename R>
concept AeroResults_range_range =
    std::ranges::range<R> && AeroResults_range<std::ranges::range_value_t<R>>;

void accumulateTotalPolars(std::string outdir,
                           AeroResults_range_range auto &&R) {

  // Accumulate Forces
  PRINT_TYPE(R[0]);
  auto totalResults =
      R | views::transform([](AeroResults_range auto &&results) {
        AeroResults accResults = results[0];
        std::ranges::for_each(results.begin() + 1, results.end(),
                              [&accResults](auto &&res) {
                                accResults.polars["Fx"] += res.polars["Fx"];
                                accResults.polars["Fy"] += res.polars["Fy"];
                                accResults.polars["Fz"] += res.polars["Fz"];
                              });
        return accResults;
      });
  auto totalPolars = views::transform(
      totalResults, [](auto &&res) { return postProcessTotalPolars(res); });

  std::vector<std::string> headers = {"aoa", "Fx",  "Fy", "Fz", "CFx",
                                      "CFy", "CFz", "CL", "CD"};
  Eigen::ArrayXXd polars(totalPolars.size(), headers.size());
  for (auto i : RANGE(totalPolars.size())) {
    for (auto j : RANGE(headers.size())) {
      polars(i, j) = totalPolars[i].polars[headers[j]];
    }
  }
  std::cout << polars;
  savetxt(outdir + "/polars.dat", polars, " ", headers);
};
auto run_analysis(const FlowParams &flowParams, const ReferenceGeom &refGeom,
                  const std::string &inputFile, const std::string &outputFile,
                  bool rotate_wake) {

  Eigen::Array3d freeStream = getFreeStream(flowParams.aoa, 1);
  auto pset = readConvertedComponentsFromFile(inputFile);
  if (rotate_wake) {
    print("Aligning Wake\n");
    align_wake_to_flow(pset, flowParams.aoa);
    ;
  }
  print("Wake Surface: ", pset[0].wake.mPoints.rows());
  auto panelGeometries = calc_panel_geometry(pset);
  print("Wake Size: ", panelGeometries[0].second.centrePoints.rows());
  auto evalPoints = create_eval_points(panelGeometries);
  auto compTaskPairs = makeComputeTaskPairs(panelGeometries, evalPoints);
  auto out =
      assembleRhs(compTaskPairs, panelGeometries, evalPoints, freeStream);
  Eigen::VectorXd sourceStrength = std::move(out.second);
  Eigen::VectorXd rhs = std::move(out.first);
  Eigen::MatrixXd lhs = assembleLhs(compTaskPairs, panelGeometries, evalPoints);
  Eigen::ArrayXd doubletStrength = GMRESSolver().solve(lhs, rhs);
  savetxt("solution", doubletStrength);
  auto results = postProcessBody(panelGeometries, doubletStrength,
                                 sourceStrength, flowParams, refGeom);

  for (auto i : RANGE(results.size())) {
    std::string outfile = outputFile + "/bodydata_S" + std::to_string(i) +
                          "_aoa" + std::to_string((int)flowParams.aoa) + ".dat";
    writeBodyData(outfile, panelGeometries[i], results[i]);
    print("Aoa: ", results[i].polars["aoa"], "CL: ", results[i].polars["CL"]);
  }
  return results;
}
int main(int argc, char *argv[]) {
  std::string inputFile;
  std::string outputFile;
  std::string paramsFile;
  bool batchAoa = false;
  bool rotate_wake = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-i") && (i + 1 < argc)) {
      inputFile = argv[++i];
    } else if ((arg == "-o") && (i + 1 < argc)) {
      outputFile = argv[++i];
    } else if ((arg == "-p") && (i + 1 < argc)) {
      paramsFile = argv[++i];
    } else if ((arg == "-b") && (i < argc)) {
      batchAoa = true;
    } else if ((arg == "-r") && (i < argc)) {
      rotate_wake = true;
    }
  }
  if (inputFile.empty() || outputFile.empty() || paramsFile.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " -i <input_file> -o <output_file> -p <params_file>\n";
    return 1;
  }
  if (!batchAoa) {
    auto [flowParams, refGeom] = parse_param(paramsFile);
    run_analysis(flowParams, refGeom, inputFile, outputFile, rotate_wake);
  } else {
    auto [flowParams, refGeom] = parse_param_batch(paramsFile);
    std::ranges::copy(flowParams | views::transform([](const auto &flowParams) {
                        return flowParams.aoa;
                      }),
                      std::ostream_iterator<double>(std::cout, " "));
    auto resultsView =
        flowParams | views::transform([&](const auto &flowParams) {
          return run_analysis(flowParams, refGeom, inputFile, outputFile,
                              rotate_wake);
        });
    std::vector<std::vector<AeroResults>> results;
    results.reserve(resultsView.size());
    std::ranges::copy(resultsView, std::back_inserter(results));
    accumulateTotalPolars(outputFile, results);
  }

  return 0;
}
