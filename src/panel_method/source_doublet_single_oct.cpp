#include "panel_method/source_doublet_single_oct.hpp"
#include "central_difference.hpp"
#include "evalPoints.hpp"
#include "infMat.hpp"
#include "panel_method/ipanel.hpp"
#include "pm_octree_defs.hpp"
#include "singularity/const_doublet.hpp"
#include "singularity/const_source.hpp"
#include "solver/dense_solver.hpp"
#include "solver/sparse_solver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <algorithm>

Eigen::MatrixXd SourceDoubletSingleOct::assembleLhs() {

  std::size_t evalDims = evalPointsRef.get().mEvalPoints.rows();
  std::size_t wakeDims = wakePanelCompTasks.size();
  Eigen::MatrixXd surfaceInfluenceMatrix = makeInfluenceMatrix<DoubletP, true>(
      evalDims, evalDims, surfacePanelCompTasks);
  std::cout << surfaceInfluenceMatrix.topLeftCorner(10, 10) << "\n";
  Eigen::MatrixXd wakeInfluenceMatrix = makeInfluenceMatrix<DoubletP, false>(
      evalDims, wakeDims, wakePanelCompTasks);

  // combine source and wake
  for (std::size_t iWakeP = 0;
       iWakeP < IPM::wakePanelRef.get().mSurface.mTrailingEdgeIdx.rows();
       iWakeP++) {

    // NOTE: bad for cache?

    int lowerFaceIdx =
        IPM::wakePanelRef.get().mSurface.mTrailingEdgeIdx(iWakeP, 0);
    int upperFaceIdx =
        IPM::wakePanelRef.get().mSurface.mTrailingEdgeIdx(iWakeP, 1);
    surfaceInfluenceMatrix(Eigen::placeholders::all, lowerFaceIdx) -=
        wakeInfluenceMatrix(Eigen::placeholders::all, iWakeP);
    surfaceInfluenceMatrix(Eigen::placeholders::all, upperFaceIdx) +=
        wakeInfluenceMatrix(Eigen::placeholders::all, iWakeP);
  }

  return surfaceInfluenceMatrix;
}

Eigen::VectorXd SourceDoubletSingleOct::assembleRhs() {
  std::size_t evalDims = evalPointsRef.get().mEvalPoints.rows();
  Eigen::MatrixXd sourceInfluenceMat = makeInfluenceMatrix<SourceP, true>(
      evalDims, evalDims, surfacePanelCompTasks);
  sourceStrength = rowwiseDotProduct(IPM::surfacePanelRef.get().normalVectors,
                                     IPM::freeStream);
  return -(sourceInfluenceMat * sourceStrength);
}

Eigen::MatrixXd SourceDoubletSingleOct::calculatePanelVelocities() {
  auto &panel = IPM::surfacePanelRef.get();
  std::size_t nYSecs = panel.mSurface.nYsecs;
  std::size_t nXsecs = panel.mSurface.nXsecs;

  // Is this being reshaped properly? yes

  // Convert points to panel coordinate surface chord-wise from trailing edge
  // bottom to trailing edge top
  // is solution solved  in the same order>	depends on if face has the same
  // order, yes compute task <- centerpoints <- face

  // 1): Convert the centrepoints to panel reference frame
  Eigen::ArrayXXd xPoints(nXsecs, nYSecs);
  xPoints.setZero();
  for (int iY = 0; iY < nYSecs; iY++) {
    for (int iX = 1; iX < nXsecs; iX++) {
      xPoints(iX, iY) = (panel.centrePoints.row(iX + iY * nXsecs) -
                         panel.centrePoints.row((iX - 1) + iY * nXsecs))
                            .matrix()
                            .norm() +
                        xPoints(iX - 1, iY);
    }
  }
  Eigen::ArrayXXd yPoints(nXsecs, nYSecs);
  yPoints.setZero();
  for (int iX = 0; iX < nXsecs; iX++) {
    for (int iY = 1; iY < nYSecs; iY++) {
      yPoints(iX, iY) = (panel.centrePoints.row(iX + iY * nXsecs) -
                         panel.centrePoints.row(iX + (iY - 1) * nXsecs))
                            .matrix()
                            .norm() +
                        yPoints(iX, iY - 1);
    }
  }

  // 2:) u = -d(mu)/d(x_l); v = -d(mu)/d(y_l); w = sigma
  Eigen::ArrayXXd fPoints(nXsecs, nYSecs);
  fPoints << IPM::solution.reshaped(nXsecs, nYSecs); // no minus

  Eigen::ArrayX3d inducedVelocities(nXsecs * nYSecs, 3);
  inducedVelocities << -centralDifference<true>(xPoints, fPoints).reshaped(),
      -centralDifference<false>(yPoints, fPoints).reshaped(), -sourceStrength;

  Eigen::ArrayX3d globalVelocites(nXsecs * nYSecs, 3);

  globalVelocites << rowwiseDotProduct(panel.tangentXVectors, IPM::freeStream),
      rowwiseDotProduct(panel.tangentYVectors, IPM::freeStream),
      rowwiseDotProduct(panel.normalVectors, IPM::freeStream);

  // NOTE: FOR DEBUGGING
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/xPoints.dat", xPoints);

  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/fPoints.dat", fPoints);

  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/ivelocities.dat",
                  inducedVelocities);
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/gvelocities.dat",
                  globalVelocites);

  std::cout << "Solve Completed\n";
  // 3:) V = V_g + V_l
  return globalVelocites + inducedVelocities;
}

SourceDoubletSingleOct::SourceDoubletSingleOct(
    const PanelGeometry<SurfacePanel> &surfacePanelGeo,
    const PanelGeometry<WakePanel> &wakePanelGeo,
    const EvalPoints<double> &evalPoints, std::unique_ptr<ISolver> &&solver,
    double AoAd)
    : IPanelMethod(surfacePanelGeo, wakePanelGeo, evalPoints,
                   std::move(solver)) {
  IPM::setFlowParams(AoAd);
}

template <typename PanelGeo>
std::vector<std::vector<std::size_t>>
getAllNeighbours(const PanelGeo &surfacePanelGeo,
                 const EvalPoints<double> &evalPoints,
                 const Eigen::ArrayXd &searchRadius) {

  const auto cloud = convertMat2Cloud(evalPoints.mEvalPoints);
  double resolution = 100;

  Octree octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  const auto searchPoints = createSearchPoints(surfacePanelGeo.centrePoints);
  const auto allNeighbours = queryOctree(octree, searchPoints, searchRadius);
  return allNeighbours;
}

template <typename PanelGeo>
auto getPanelDiagonalLength(const PanelGeo &panelGeo) -> Eigen::ArrayXd {
  const auto &faceIdx = panelGeo.mSurface.mFaceNodeIdx;
  const auto &surfPoints = panelGeo.mSurface.mPoints;

  return (surfPoints(faceIdx.col(0), Eigen::placeholders::all) -
          surfPoints(faceIdx.col(2), Eigen::placeholders::all))
      .rowwise()
      .norm();
}

template <typename PanelGeo>
auto makeComputeTasks(const PanelGeo &panelGeo,
                      const EvalPoints<double> &evalPoints) {
  int nPanels = panelGeo.centrePoints.rows();
  auto faceIdxs = RANGE(nPanels);
  auto allNeighbours = getAllNeighbours(panelGeo, evalPoints,
                                        getPanelDiagonalLength(panelGeo) * 10);
  std::size_t totalNum = 0;
  for (const auto &neighbours : allNeighbours) {
    totalNum += neighbours.size();
  }
  print("Points to evaluate: ", totalNum);
  print("No EvalPoints: ", evalPoints.mEvalPoints.rows());
  print("Full Search Space: ",
        evalPoints.mEvalPoints.rows() * panelGeo.centrePoints.rows());
  print("Potential Speed Up: ",
        (double)(evalPoints.mEvalPoints.rows() * panelGeo.centrePoints.rows()) /
            (double)totalNum);
  return initialize_transform<std::vector<ComputeTask>>(
      faceIdxs.begin(), faceIdxs.end(), [&](int faceIdx) {
        return createInfluenceComputeTask(panelGeo, evalPoints, faceIdx,
                                          allNeighbours[faceIdx]);
      });
};
void SourceDoubletSingleOct::run() {
  std::vector<std::size_t> idxs(IPM::evalPointsRef.get().mEvalPoints.rows());
  std::iota(idxs.begin(), idxs.end(), 0);

  surfacePanelCompTasks =
      makeComputeTasks(IPM::surfacePanelRef.get(), IPM::evalPointsRef.get());
  wakePanelCompTasks =
      makeComputeTasks(IPM::wakePanelRef.get(), IPM::evalPointsRef.get());

  IPM::solution = IPM::solver->solve(assembleLhs(), assembleRhs());
  IPM::velocities = calculatePanelVelocities();
}

Eigen::ArrayXd SourceDoubletSingleOct::getSource() const {
  return sourceStrength;
}
