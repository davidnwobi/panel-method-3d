#include <Eigen/Core>
#define CLAMP_TO 1e9

void R12_Q12_J12_NORM(float *__restrict R12_, float *__restrict Q12_,
                      float *__restrict J12_, float *__restrict x,
                      float *__restrict y, float *__restrict z,
                      float *__restrict node1, float *__restrict node2,
                      size_t N);

void J12_NORM(float *__restrict J12_, float *__restrict x, float *__restrict y,
              float *__restrict z, float *__restrict node1,
              float *__restrict node2, size_t N);
