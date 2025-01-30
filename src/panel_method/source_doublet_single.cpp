#include "panel_method/source_doublet_single.hpp"
#include "central_difference.hpp"
#include "infMat.hpp"
#include "panel_method/ipanel.hpp"
#include "singularity/const_doublet.hpp"
#include "singularity/const_source.hpp"
#include "solver/dense_solver.hpp"
#include "solver/sparse_solver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <algorithm>

Eigen::MatrixXd SourceDoubletSingle::assembleLhs() {

  Eigen::MatrixXd surfaceInfluenceMatrix =
      makeInfluenceMatrix<DoubletP<true>>(surfacePanelCompTasks);
  Eigen::MatrixXd wakeInfluenceMatrix =
      makeInfluenceMatrix<DoubletP<false>>(wakePanelCompTasks);

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

Eigen::VectorXd SourceDoubletSingle::assembleRhs() {
  Eigen::MatrixXd sourceInfluenceMat =
      makeInfluenceMatrix<SourceP<true>>(surfacePanelCompTasks);
  sourceStrength = rowwiseDotProduct(IPM::surfacePanelRef.get().normalVectors,
                                     IPM::freeStream);
  return -(sourceInfluenceMat * sourceStrength);
}

Eigen::MatrixXd SourceDoubletSingle::calculatePanelVelocities() {
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

SourceDoubletSingle::SourceDoubletSingle(
    const PanelGeometry<SurfacePanel> &surfacePanelGeo,
    const PanelGeometry<WakePanel> &wakePanelGeo,
    const EvalPoints<double> &evalPoints, std::unique_ptr<ISolver> &&solver,
    double AoAd)
    : IPanelMethod(surfacePanelGeo, wakePanelGeo, evalPoints,
                   std::move(solver)) {
  IPM::setFlowParams(AoAd);
}

void SourceDoubletSingle::run() {
  std::vector<std::size_t> idxs(IPM::evalPointsRef.get().mEvalPoints.rows());
  std::iota(idxs.begin(), idxs.end(), 0);

  auto makeComputeTasks =
      [this]<typename PanelGeo>(const PanelGeo &panelGeo,
                                const std::vector<std::size_t> &idxs) {
        int nPanels = panelGeo.centrePoints.rows();
        auto faceIdxs = RANGE(nPanels);
        return initialize_transform<std::vector<ComputeTask>>(
            faceIdxs.begin(), faceIdxs.end(), [&](int faceIdx) {
              return createInfluenceComputeTask(panelGeo, IPM::evalPointsRef,
                                                faceIdx, idxs);
            });
      };
  surfacePanelCompTasks = makeComputeTasks(IPM::surfacePanelRef.get(), idxs);
  wakePanelCompTasks = makeComputeTasks(IPM::wakePanelRef.get(), idxs);

  IPM::solution = IPM::solver->solve(assembleLhs(), assembleRhs());
  IPM::velocities = calculatePanelVelocities();
}

Eigen::ArrayXd SourceDoubletSingle::getSource() const { return sourceStrength; }
