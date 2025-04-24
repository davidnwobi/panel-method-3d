#include "panel_geo/panel_geo.hpp"
#include "surface/surface_reader.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <ranges>

template <class T> auto makeChunkData(const auto &panelGeometry) {
  namespace views = std::ranges::views;

  auto chunkSize = panelGeometry | views::transform([](const auto &pg) {
                     return pg.centrePoints.rows();
                   });
  std::vector<size_t> chunkStart(chunkSize.size(), 0);
  std::exclusive_scan(chunkSize.begin(), chunkSize.end(), chunkStart.begin(),
                      0);
  return std::pair{chunkStart, chunkSize};
}

Eigen::Array3d getFreeStream(double aoa, double Vinf) {

  double angleOfAttack = aoa * M_PI / 180;
  return {std::cos(angleOfAttack), 0, std::sin(angleOfAttack)};
}
using namespace std::ranges;
template <typename Derived>
Derived rotate_2d_about_origin(const Eigen::MatrixBase<Derived> &points2d,
                               double angle_d) {
  double angle_r = angle_d * M_PI / 180.0;
  return (Eigen::Matrix2d{{std::cos(angle_r), -std::sin(angle_r)},
                          {std::sin(angle_r), std::cos(angle_r)}}) *
         points2d;
}

void rotate_3d_about_origin(Eigen::Ref<Eigen::ArrayX3d> points3d,
                            double angle_d) {
  Eigen::MatrixXd points = rotate_2d_about_origin(
      (Eigen::MatrixXd(2, points3d.rows()) << points3d.col(0).transpose(),
       points3d.col(2).transpose())
          .finished(),
      angle_d);
  points3d.col(0) = points.row(0).transpose();
  points3d.col(2) = points.row(1).transpose();
}

void rotate_points_about_start(Eigen::Ref<Eigen::ArrayX3d> points3d,
                               double angle_d) {
  if (points3d.rows() == 0)
    return;

  Eigen::RowVector3d original_loc(points3d(0, 0), 0, points3d(0, 2));
  points3d = points3d.rowwise() - original_loc.array(); // translate to origin
  rotate_3d_about_origin(points3d, angle_d);
  points3d = points3d.rowwise() + original_loc.array(); // translate from origin
}
void align_wake_to_flow(std::vector<PanelSet> &panel_sets, double aoa) {
  std::ranges::for_each(
      panel_sets,
      [&aoa](WakePanel &wake) {
        auto nX = wake.nXsecs + 1;
        if (nX > 1) {
          for (auto i : RANGE(wake.nYsecs + 1)) {
            rotate_points_about_start(wake.mPoints.middleRows(i * nX, nX), aoa);
            ;
          }
        }
      },
      &PanelSet::wake);
}

std::vector<PanelGeometryPair>
calc_panel_geometry(std::vector<PanelSet> &panel_sets) {
  std::vector<PanelGeometryPair> panelGeometryPair;
  panelGeometryPair.reserve(panel_sets.size());
  std::ranges::copy(panel_sets | std::views::transform([](PanelSet &pset) {
                      return std::make_pair(
                          PanelGeometry<SurfacePanel>(pset.body),
                          PanelGeometry<WakePanel>(pset.wake));
                    }),
                    std::back_inserter(panelGeometryPair));
  return panelGeometryPair;
}

std::pair<FlowParams, ReferenceGeom>
parse_param(const std::filesystem::path &fpath) {
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
      // print("aoa: ", line.c_str());
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
std::pair<std::vector<FlowParams>, ReferenceGeom>
parse_param_batch(const std::filesystem::path &fpath) {
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
