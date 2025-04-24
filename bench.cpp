#include "singularity/const_source_doublet.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <benchmark/benchmark.h>
#include <cstdio>
#include <cstdlib>
#include <immintrin.h>
#include <memory>
#include <singularity/internal_functions.hpp>

static const float MULTIPLIER = 10;
using ul = long long;

void bench_kernel2(benchmark::State &state) {
  const size_t N = state.range(0);
  Eigen::ArrayXf sourceMat(N);
  Eigen::ArrayXf doubleMat(N);
  ComputeTask compTask;
  compTask.points = Eigen::ArrayX3f::Random(N, 3);
  compTask.face.points = Eigen::ArrayX3f::Random(4, 3);
  for (auto _ : state) {
    SourceDoubletP::calcInfluenceImpl(sourceMat, doubleMat, compTask);
  }
  state.SetBytesProcessed((ul)state.iterations() * (ul)N * (ul)8);
}

BENCHMARK(bench_kernel2)
    ->RangeMultiplier(2)
    ->Arg(512)
    ->Arg(2048)
    ->Arg(4096)
    ->Arg(8192)
    ->DisplayAggregatesOnly(true);

BENCHMARK_MAIN();
