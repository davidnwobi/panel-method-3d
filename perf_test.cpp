#include <Eigen/Core>
#include <asm/unistd.h>
#include <inttypes.h>
#include <iostream>
#include <linux/perf_event.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <unsupported/Eigen/IterativeSolvers>
#include <unsupported/Eigen/SparseExtra>

static long perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                            int cpu, int group_fd, unsigned long flags) {
  int ret;

  ret = syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
  return ret;
}

template <typename Callable> long long count_instructions(Callable func) {
  struct perf_event_attr pe;
  long long count;
  int fd;

  memset(&pe, 0, sizeof(struct perf_event_attr));
  pe.type = PERF_TYPE_HARDWARE;
  pe.size = sizeof(struct perf_event_attr);
  pe.config = PERF_COUNT_HW_INSTRUCTIONS;
  pe.disabled = 1;
  pe.exclude_kernel = 1;
  // Don't count hypervisor events.
  pe.exclude_hv = 1;

  fd = perf_event_open(&pe, 0, -1, -1, 0);
  if (fd == -1) {
    fprintf(stderr, "Error opening leader %llx\n", pe.config);
    exit(EXIT_FAILURE);
  }

  ioctl(fd, PERF_EVENT_IOC_RESET, 0);
  ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);

  func();

  ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
  read(fd, &count, sizeof(long long));

  printf("Used %lld instructions\n", count);

  close(fd);
  return count;
}

int main(int argc, char **argv) {
  using namespace Eigen;
  MatrixXf lhs;
  MatrixXf rhs;
  std::string base = "../../c3/";

  if (!loadMarketDense(lhs, base + "lhs.txt")) {
    std::cerr << "Error reading A from " << base + "lhs.txt" << std::endl;
  }
  if (!loadMarketDense(rhs, base + "rhs.txt")) {

    std::cerr << "Error reading b from " << base + "rhs.txt" << std::endl;
  }

  SparseMatrix<float> precondMat = lhs.sparseView(3e-3, 1);
  printf("%ld, \n", precondMat.nonZeros());
  IncompleteLUT<float> ilu0;
  ilu0.setFillfactor(1);
  ilu0.compute(precondMat);

  int n = lhs.rows();
  auto gmres_bench = [&lhs, &rhs, &ilu0, n] {
    for (int i = 0; i < 50; i++) {
      VectorXf x(n);
      x.setZero();
      Index its = n;
      float error = 1e-6;
      internal::gmres(lhs, rhs, x, ilu0, its, 200, error);
      printf("Its: %ld\n", its);
    }
  };
  auto lu_bench = [&lhs, &rhs] {
    for (int i = 0; i < 50; i++) {
      lhs.lu().solve(rhs);
    }
  };

  count_instructions(gmres_bench);
}
