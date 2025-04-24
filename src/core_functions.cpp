#include "aerocalcs/aerocalcsingle.hpp"
#include "compTask.hpp"
#include "helpers.hpp"
#include "lhs.hpp"
#include "mat_reader/mat_reader.hpp"
#include "post_processing.hpp"
#include "solver/dense_gmres_solver.hpp"
#include "solver/dense_gmres_solver_ilu.hpp"
#include "solver/gmres_solver.hpp"
#include "solver/gmres_solver_diag.hpp"
#include "solver/gmres_solver_ilu.hpp"
#include "solver/hodlr_dgmres.hpp"
#include "solver/hodlr_solver.hpp"
#include "solver/sparse_solver.hpp"
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

#include "solver/dense_solver.hpp"
#include <unsupported/Eigen/SparseExtra>

std::vector<AeroResults>
run_analysis(const FlowParams &flowParams, const ReferenceGeom &refGeom,
             const std::string &inputFile, const std::string &outputFile,
             float dropTol, float spTol, int solverType, bool rotate_wake) {

  Eigen::Array3f freeStream = getFreeStream(flowParams.aoa, 1);
  auto pset = readConvertedComponentsFromFile(inputFile);
  if (rotate_wake) {
    // print("Aligning Wake\n");
    align_wake_to_flow(pset, flowParams.aoa);
    ;
  }
  // print("Wake Surface: ", pset[0].wake.mPoints.rows());
  auto panelGeometries = calc_panel_geometry(pset);
  // print("Wake Size: ", panelGeometries[0].second.centrePoints.rows());
  auto evalPoints = create_eval_points(panelGeometries);
  auto [lhs, rhs, sourceStrength] =
      assembleLhs(panelGeometries, evalPoints, freeStream);
  Eigen::ArrayXf doubletStrength(rhs.rows());
  // print("Solver Type ", solverType);
  switch (solverType) {
  case 0: {
    DenseSolver solver;
    doubletStrength = solver.solve(lhs, rhs);
  } break;
  case 1: {
    DenseGMRESSolver solver;
    doubletStrength = solver.solve(lhs, rhs, 1e-6);
  } break;
  case 2: {
    DenseGMRESILUSolver solver;
    solver.setdropTol(dropTol);
    doubletStrength = solver.solve(lhs, rhs, 1e-6);
  } break;
  case 3: {
    HODLRSolver solver;
    doubletStrength = solver.solve(lhs, rhs, spTol);
  } break;
  case 4: {
    HodlrDgmres solver;
    solver.setdropTol(dropTol);
    solver.setspTol(spTol);
    doubletStrength = solver.solve(lhs, rhs, 1e-6);
  } break;
  case 5: {
    SparseSolver solver;
    solver.setspTol(spTol);
    doubletStrength = solver.solve(lhs, rhs);
  } break;
  case 6: {
    GMRESSolver solver;
    solver.setspTol(spTol);
    doubletStrength = solver.solve(lhs, rhs, 1e-6);
  } break;
  case 7: {
    GMRESILUSolver solver;
    solver.setspTol(spTol);
    solver.setspTol(dropTol);
    doubletStrength = solver.solve(lhs, rhs, 1e-6);
  } break;

  default:
    std::cerr << "Solver Not Implemented\n";
  }

  // Eigen::saveMarketDense(lhs, "lhs.txt");
  // Eigen::saveMarketDense(rhs, "rhs.txt");
  // Eigen::saveMarketDense(doubletStrength, "solution.txt");
  auto results = postProcessBody(panelGeometries, doubletStrength,
                                 sourceStrength, flowParams, refGeom);

  float sum = 0;
  for (auto i : RANGE(results.size())) {
    std::string outfile = outputFile + "/bodydata_S" + std::to_string(i) +
                          "_aoa" + std::to_string((int)flowParams.aoa) + ".dat";
    writeBodyData(outfile, panelGeometries[i], results[i]);
    // print("Aoa: ", results[i].polars["aoa"], "CL: ",
    // results[i].polars["CL"]);
    sum += results[i].polars["CL"];
  }

  printf("%1.6f\n", sum);
  return results;
}
