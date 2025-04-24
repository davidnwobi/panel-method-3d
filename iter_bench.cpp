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
  sptols << 5e-5, 1e-5, 1e-6, 3.125e-6, 1e-5;
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
      IncompleteLUT<float> ilu0;
      ilu0.setDroptol(dropTol[j]);
      ilu0.setFillfactor(1);
      ilu0.compute(precondMat);

      // Test 1: ILU(0) preconditioner on 'precondMat', solve 'problemMat'
      {
        GMRES<SparseMatrix<float>, IncompleteLUT<float>> solver;
        Eigen::VectorXf x(n);
        x.setZero();
        Eigen::Index its = n;
        float error = tol;
        Eigen::internal::gmres(problemMat, rhs[i], x, ilu0, its, 200, error);
        iters(i, 0) = its;
        errs(i, 0) = error;
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
