#include "aerocalcs/aerocalcsingle.hpp"
#include "utils/utils.hpp"
#include <string>

void AeroCalcSingle::run(FlowParams &&params) {
  lastParams = std::move(params);
  pm->setFlowParams(lastParams.aoa);
  pm->run();
  post_process();
}

void AeroCalcSingle::post_process() {
  postProcessPanelResults();
  postProcessPolars();
  postProcessSpanResults();
  auto nXsecs = surfacePanelGeo.mSurface.nXsecs;
  auto nYsecs = surfacePanelGeo.mSurface.nYsecs;
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/velocities.dat",
                  pm->getComputedVelocites());
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/liftCoeff.dat",
                  spanResults["spanLift"]);
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/xPoints.dat",
                  surfacePanelGeo.centrePoints.col(0).reshaped(nXsecs, nYsecs));
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/yPoints.dat",
                  surfacePanelGeo.centrePoints.col(1).reshaped(nXsecs, nYsecs));
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/zPoints.dat",
                  surfacePanelGeo.centrePoints.col(2).reshaped(nXsecs, nYsecs));
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/doubletDist.dat",
                  pm->getSolution());
  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/pressure_" +
                      std::to_string((int)lastParams.aoa) + ".dat",
                  panelResults["dCp"]);

  print("Lift Coeff: ", polars["CL"]);
  print("Drag Coeff: ", polars["CD"]);
}

void AeroCalcSingle::postProcessPanelResults() {
  const Eigen::ArrayXXd &dV = pm->getComputedVelocites();

  Eigen::ArrayXd dCp = 1 - (pm->getComputedVelocites().rowwise().squaredNorm());

  double q = 0.5 * lastParams.rho * lastParams.Vinf * lastParams.Vinf;
  Eigen::ArrayXd dP = q * dCp;
  Eigen::ArrayX3d dF =
      surfacePanelGeo.normalVectors.colwise() * (dP * surfacePanelGeo.areas);

  std::cout << "issue\n";
  panelResults["dCp"] = dCp;
  panelResults["dVx"] = dV.col(0).eval();
  panelResults["dVy"] = dV.col(1).eval();
  panelResults["dVz"] = dV.col(2).eval();
  panelResults["dP"] = dP.eval();
  panelResults["dFx"] = dF.col(0).eval();
  panelResults["dFy"] = dF.col(1).eval();
  panelResults["dFz"] = dF.col(2).eval();
}
void AeroCalcSingle::postProcessPolars() {
  double q = 0.5 * lastParams.rho * lastParams.Vinf * lastParams.Vinf;
  Eigen::Array3d F(3);
  F << panelResults["dFx"].sum(), panelResults["dFy"].sum(),
      panelResults["dFz"].sum();
  Eigen::ArrayXd CF = F / (q * refGeom.refArea);
  double CL = (-CF(0) * std::sin(lastParams.aoa * M_PI / 180) +
               CF(2) * std::cos(lastParams.aoa * M_PI / 180));
  double CD = (F(0) * std::cos(lastParams.aoa * M_PI / 180) +
               F(2) * std::sin(lastParams.aoa * M_PI / 180));

  polars["aoa"] = lastParams.aoa;
  polars["Fx"] = F(0);
  polars["Fy"] = F(1);
  polars["Fz"] = F(2);
  polars["CFx"] = CF(0);
  polars["CFy"] = CF(1);
  polars["CFz"] = CF(2);
  polars["CL"] = CL;
  polars["CD"] = CD;
}
void AeroCalcSingle::postProcessSpanResults() {

  auto nXsecs = surfacePanelGeo.mSurface.nXsecs;
  auto nYsecs = surfacePanelGeo.mSurface.nYsecs;
  Eigen::ArrayXd yAvg = surfacePanelGeo.centrePoints.col(1)
                            .reshaped(nXsecs, nYsecs)
                            .colwise()
                            .mean();
  Eigen::ArrayXd spanLift =
      (-panelResults["dFy"] * std::sin(lastParams.aoa * M_PI / 180) +
       panelResults["dFz"] * std::cos(lastParams.aoa * M_PI / 180))
          .reshaped(nXsecs, nYsecs)
          .colwise()
          .sum();

  spanResults["yAvg"] = std::move(yAvg);
  spanResults["spanLift"] = std::move(spanLift);
}
