#include <Eigen/Core>
#include <HODLR.hpp>
#include <chrono>
#include <iostream>
#include <solver/hodlr_solver.hpp>
#include <unsupported/Eigen/SparseExtra>

using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Clock = std::chrono::high_resolution_clock;
using Secs = std::chrono::duration<double>;

class EigenKernel : public HODLR_Matrix {
  const Mat &A_;

public:
  explicit EigenKernel(const Mat &A) : HODLR_Matrix(A.rows()), A_(A) {}
  dtype getMatrixEntry(int i, int j) override { return A_(i, j); }
};

double now() { return Clock::now().time_since_epoch().count() * 1e-9; }

int main() {

  // ------------------------------------------------------------------
  //  load data
  // ------------------------------------------------------------------
  Mat A;
  Vec rhs;

  std::string base = "/home/okwuchukwu-nwobi/Projects/PanelMethods/c1/";

  if (!Eigen::loadMarketDense(A, base + "lhs.txt")) {
    std::cerr << "Error reading A from " << base + "lhs.txt" << std::endl;
  }
  if (!Eigen::loadMarketDense(rhs, base + "rhs.txt")) {

    std::cerr << "Error reading b from " << base + "rhs.txt" << std::endl;
  }

  const int leaf = 64;
  const double tol = 1e-6;

  // ------------------------------------------------------------------
  //  HODLR build  +  factor
  // ------------------------------------------------------------------
  EigenKernel K(A);
  HODLR T(A.rows(), leaf, tol);
  //
  double t0 = now();
  T.assemble(&K, "rookPivoting", /*sym=*/false, /*pd=*/false);
  T.factorize();
  // double hodlr_build = now() - t0;
  //
  // // ------------------------------------------------------------------
  // //  HODLR solve
  // // ------------------------------------------------------------------
  Vec b = rhs;
  // t0 = now();

  HODLRSolver solver;
  Vec x_hodlr = solver.solve(A, rhs, tol);

  // Vec x_hodlr = T.solve(rhs);
  double hodlr_solve = now() - t0;

  // residual check
  std::cout << "HODLR ‖Ax-b‖/‖b‖ = " << (A * x_hodlr - b).norm() / b.norm()
            << '\n';

  // ------------------------------------------------------------------
  //  dense LU  (Eigen::PartialPivLU)
  // ------------------------------------------------------------------
  Eigen::PartialPivLU<Mat> lu;

  t0 = now();
  lu.compute(A); // factor only
  double lu_fact = now() - t0;

  t0 = now();
  Vec x_lu = lu.solve(b); // solve only
  double lu_solve = now() - t0;

  std::cout << "LU    ‖Ax-b‖/‖b‖ = " << (A * x_lu - b).norm() / b.norm()
            << '\n';

  // ------------------------------------------------------------------
  //  timings
  // ------------------------------------------------------------------
  std::cout << "\n===== wall times (s) =====\n"
            << "hodlr_solve : " << hodlr_solve << '\n'
            << "lu_fact     : " << lu_fact << '\n'
            << "lu_solve    : " << lu_solve << '\n';

  double sumR2 = (x_lu - x_hodlr).squaredNorm();
  std::cout << "Residual Difference: " << sumR2 << "\n";
}
