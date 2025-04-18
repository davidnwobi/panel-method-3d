#include <Eigen/Core>
#define CLAMP_TO 1e9

void R12_Q12_J12_NORM(double *__restrict R12_, double *__restrict Q12_,
                      double *__restrict J12_, double *__restrict x,
                      double *__restrict y, double *__restrict z,
                      double *__restrict node1, double *__restrict node2,
                      size_t N);

void J12_NORM(double *__restrict J12_, double *__restrict x, double *__restrict y,
              double *__restrict z, double *__restrict node1,
              double *__restrict node2, size_t N);
