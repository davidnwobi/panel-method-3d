#include <Eigen/Core>
#include <cstdlib>
#include <memory>
#include <benchmark/benchmark.h>

static const float MULTIPLIER = 10;
using ul = long long;

template<typename PlainObjectType, typename T>
Eigen::Map<PlainObjectType, Eigen::Aligned32> init (T* data, size_t size){
    return Eigen::Map<PlainObjectType, Eigen::Aligned32>(data, size);
}

template<typename MapType, typename T>
void resize (MapType& mapped, T* data, size_t size){
    new (&mapped) MapType(data, size);
}

template<typename T, size_t Alignment>
void* get_aligned_mem(size_t nbytes){
    void* raw_ptr = nullptr;
    ul SUCCESS = posix_memalign(&raw_ptr, Alignment, nbytes);
    if (SUCCESS != 0){
        std::abort();
    }
    return raw_ptr;
}

auto free_deleter = [](float* p) {
    std::free(p);
};

template<typename Derived>
inline auto expr1(const Eigen::ArrayBase<Derived>& d){
    return d*100/((float) MULTIPLIER);
}
template<typename Derived>
inline auto expr2(const Eigen::ArrayBase<Derived>& d){
    return d.sin()/((float) MULTIPLIER);
}
template<typename Derived>
inline auto copyinto(const Eigen::ArrayBase<Derived>& d){
    return d*10.0/50.0 * expr1(d) * expr2(d);
}

template<typename Derived>
inline Eigen::ArrayXf expr1_2(const Eigen::ArrayBase<Derived>& d){
    return d*100/((float) MULTIPLIER);
}
template<typename Derived>
inline Eigen::ArrayXf expr2_2(const Eigen::ArrayBase<Derived>& d){
    return d.sin()/((float) MULTIPLIER);
}
template<typename Derived>
inline Eigen::ArrayXf copyinto_2(const Eigen::ArrayBase<Derived>& d){
    return d*10.0/50.0 * expr1_2(d) * expr2_2(d);
}

float bench_1(ul no_panels) {
    ul n = no_panels;
    float* raw_ptr = (float*) get_aligned_mem<float, 32>(sizeof(float)*n);
    std::unique_ptr<float, decltype(free_deleter)> data(raw_ptr, free_deleter);
    auto test = init<Eigen::ArrayXf, float>(data.get(), n);
    Eigen::ArrayXf test2 = Eigen::ArrayXf::Random(no_panels/2);
    resize(test, data.get(), no_panels/2);
    test = copyinto(test2);

    float* raw_ptr_2 = (float*) get_aligned_mem<float, 32>(sizeof(float)*n);
    std::unique_ptr<float, decltype(free_deleter)> data_2(raw_ptr_2, free_deleter);
    auto test3 = init<Eigen::ArrayXf, float>(data_2.get(), n/2);
    test3 = copyinto(test);
    return test3.sum();
}
float bench_1_2(ul no_panels) {
    ul n = no_panels;
    float* raw_ptr = (float*) get_aligned_mem<float, 32>(sizeof(float)*n);
    std::unique_ptr<float, decltype(free_deleter)> data(raw_ptr, free_deleter);
    auto test = init<Eigen::ArrayXf, float>(data.get(), n);
    Eigen::ArrayXf test2 = Eigen::ArrayXf::Random(no_panels/2);
    resize(test, data.get(), no_panels/2);
    test = copyinto_2(test2);


    float* raw_ptr_2 = (float*) get_aligned_mem<float, 32>(sizeof(float)*n);
    std::unique_ptr<float, decltype(free_deleter)> data_2(raw_ptr_2, free_deleter);
    auto test3 = init<Eigen::ArrayXf, float>(data_2.get(), n/2);
    test3 = copyinto_2(test);
    return test3.sum();
}
float bench_2(ul no_panels) {
    ul n = no_panels;
    Eigen::ArrayXf test =  Eigen::ArrayXf::Random(no_panels);
    Eigen::ArrayXf test2 = Eigen::ArrayXf::Random(no_panels/2);
    Eigen::ArrayXf test3 =  Eigen::ArrayXf::Random(no_panels);
    test = copyinto_2(test2);
    test3 = copyinto_2(test);
    return test3.sum();
}
float bench_2_2(ul no_panels) {
    ul n = no_panels;
    Eigen::ArrayXf test =  Eigen::ArrayXf::Random(no_panels);
    Eigen::ArrayXf test2 = Eigen::ArrayXf::Random(no_panels/2);
    Eigen::ArrayXf test3 =  Eigen::ArrayXf::Random(no_panels);
    test = copyinto_2(test2);
    test3 = copyinto_2(test);
    return test3.sum();
}

void bench_e1(benchmark::State &state) {
  ul no_panels = state.range(0) * state.range(0);
  float CL;
  for (auto _ : state) {
    CL = bench_1(no_panels);
  }
  benchmark::DoNotOptimize(CL);
  state.SetBytesProcessed((ul)state.iterations()*(ul)no_panels*(ul)8);
}
void bench_e1_2(benchmark::State &state) {
  ul no_panels = state.range(0) * state.range(0);
  float CL;
  for (auto _ : state) {
    CL = bench_1_2(no_panels);
  }
  benchmark::DoNotOptimize(CL);
  state.SetBytesProcessed(ul(state.iterations())*(ul)no_panels*(ul)8);
}
void bench_e2(benchmark::State &state) {
  ul no_panels = state.range(0) * state.range(0);
  float CL;
  for (auto _ : state) {
    CL = bench_2(no_panels);
  }
  benchmark::DoNotOptimize(CL);
  state.SetBytesProcessed(ul(state.iterations())*(ul)no_panels*(ul)8);
}
void bench_e2_2(benchmark::State &state) {
  ul no_panels = state.range(0) * state.range(0);
  float CL;
  for (auto _ : state) {
    CL = bench_2_2(no_panels);
  }
  benchmark::DoNotOptimize(CL);
  state.SetBytesProcessed(ul(state.iterations())*(ul)no_panels*(ul)8);
}

BENCHMARK(bench_e2_2)->RangeMultiplier(4)->Range(8, 8 << 12)->MinTime(10)->DisplayAggregatesOnly(true);

BENCHMARK_MAIN();
// int main(){

//     prulf("%f\n", bench_1(65536*20000));
// }
