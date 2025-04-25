#include "solver/gmres_solver_ilu.hpp"
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <array>
#include <iostream>
#include <string>
#include <unsupported/Eigen/IterativeSolvers>
#include <unsupported/Eigen/SparseExtra>
#include <vector>

using namespace Eigen;
using namespace std;

int main() {
  // Parameters
  const int numTests = 1; // ILU(0), ILUT
  const int numMats = 5;
  const float tol = 1e-6;
  std::array<std::string, numMats> mats = {"c1", "c2", "c3", "swept_wing",
                                           "canardTest"};
  Eigen::ArrayXf sptols(5);

  // float dpTols[] = {1e-2, 1e-2, 3e-3, 1e-3, 3e-3};
  sptols << 1e-5, 1e-5, 1e-6, 3.125e-6, 1e-5;
  Eigen::ArrayXf dropTol(7);
  dropTol << 1e-1, 3e-2, 1e-2, 3e-3, 1e-3, 3e-4, 1e-4;
  std::vector<Eigen::MatrixXf> lhs(numMats);
  std::vector<Eigen::VectorXf> rhs(numMats);
  std::vector<bool> read(numMats);
  const string baseDir = "../../";

  // Loop over drop tolerances
  for (size_t j = 0; j < dropTol.size(); ++j) {
    // Results containers
    Eigen::ArrayXXi iters(mats.size(), numTests);
    Eigen::ArrayXXf errs(mats.size(), numTests);

    for (size_t i = 0; i < mats.size(); ++i) {
      // Paths for matrices
      string lhsPath = baseDir + mats[i] + "/lhs.txt";
      string rhsPath = baseDir + mats[i] + "/rhs.txt";

      // Load A
      if (!read[i]) {
        if (!loadMarketDense(lhs[i], lhsPath)) {
          cerr << "Error reading A from " << lhsPath << endl;
          continue;
        }
        if (!loadMarketDense(rhs[i], rhsPath)) {
          cerr << "Error reading b from " << rhsPath << endl;
          continue;
        }
        read[i] = true;
      }
      int n = lhs[i].rows();

      // Threshold matrices
      SparseMatrix<float> problemMat = lhs[i].sparseView(sptols[i], 1);
      SparseMatrix<float> precondMat = problemMat.pruned(dropTol[j], 1);
      problemMat.makeCompressed();
      precondMat.makeCompressed();

      cout << "Matrix " << mats[i] << " | dropTol=" << dropTol[j]
           << " | precond sparsity=" << float(precondMat.nonZeros()) / (n * n)
           << " | problem sparsity=" << float(problemMat.nonZeros()) / (n * n)
           << endl;

      // Build preconditioners outside solver
      const Eigen::IncompleteLUT<float> ilu0(
          precondMat, Eigen::NumTraits<float>::dummy_precision(), 1);

      // Test 1: ILU(0) preconditioner on 'precondMat', solve 'problemMat'
      {
        GMRESILUSolver solver;
        solver.setspTol(sptols[i]);
        solver.setdropTol(dropTol[j]);
        solver.solve(lhs[i], rhs[i], tol, 1000);
        iters(i, 0) = solver.iters;
        errs(i, 0) = solver.errs;
      }
    }

    // Display results for this drop tolerance
    cout << "Results for dropTol = " << dropTol[j] << ":\n";
    cout << " Iterations:\n";
    cout << iters;
    cout << endl;

    cout << " Errors:\n";
    cout << errs;
    cout << endl;
  }
  return EXIT_SUCCESS;
}
