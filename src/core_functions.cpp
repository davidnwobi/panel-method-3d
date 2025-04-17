#include "aerocalcs/aerocalcsingle.hpp"
#include "compTask.hpp"
#include "helpers.hpp"
#include "lhs.hpp"
#include "mat_reader/mat_reader.hpp"
#include "post_processing.hpp"
#include "rhs.hpp"
#include "solver/bicgstab_solver.hpp"
#include "solver/dense_solver.hpp"
#include "solver/gmres_solver.hpp"
#include "solver/gmres_solver_ilu.hpp"
#include "solver/sparse_solver_qr.hpp"
#include "surface/surface_panel.hpp"
#include "surface/surface_reader.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <cmath>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

std::vector<AeroResults> run_analysis(const FlowParams &flowParams,
                                      const ReferenceGeom &refGeom,
                                      const std::string &inputFile,
                                      const std::string &outputFile,
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
  Eigen::MatrixXd lhs;
  Eigen::VectorXd rhs, sourceStrength;
  std::tie(lhs, rhs, sourceStrength) =
      assembleLhs(panelGeometries, evalPoints, freeStream);
  Eigen::ArrayXd doubletStrength = GMRESILUSolver().solve(lhs, rhs);
  return {};
  // savetxt("solution.txt", doubletStrength);
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
