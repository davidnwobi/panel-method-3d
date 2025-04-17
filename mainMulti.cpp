#include "aerocalcs/aerocalcsingle.hpp"
#include "core_functions.hpp"
#include "helpers.hpp"
#include "post_processing.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>
#include <string>
#include <vector>

int main(int argc, char *argv[]) {
  std::string inputFile;
  std::string outputFile;
  std::string paramsFile;
  bool batchAoa = false;
  double dropTol = 1e-6;
  bool rotate_wake = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-i") && (i + 1 < argc)) {
      inputFile = argv[++i];
    } else if ((arg == "-o") && (i + 1 < argc)) {
      outputFile = argv[++i];
    } else if ((arg == "-p") && (i + 1 < argc)) {
      paramsFile = argv[++i];
    } else if ((arg == "-b") && (i < argc)) {
      batchAoa = true;
    } else if ((arg == "-r") && (i < argc)) {
      rotate_wake = true;
    } else if ((arg == "-d") && (i + 1 < argc)) {
      dropTol = std::stod(argv[++i]);
    }
  }
  if (inputFile.empty() || outputFile.empty() || paramsFile.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " -i <input_file> -o <output_file> -p <params_file>\n";
    return 1;
  }
  if (!batchAoa) {
    auto [flowParams, refGeom] = parse_param(paramsFile);
    run_analysis(flowParams, refGeom, inputFile, outputFile, dropTol,
                 rotate_wake);
  } else {
    auto [flowParams, refGeom] = parse_param_batch(paramsFile);
    std::ranges::copy(flowParams | views::transform([](const auto &flowParams) {
                        return flowParams.aoa;
                      }),
                      std::ostream_iterator<float>(std::cout, " "));
    auto resultsView =
        flowParams | views::transform([&](const auto &flowParams) {
          return run_analysis(flowParams, refGeom, inputFile, outputFile,
                              dropTol, rotate_wake);
        });
    std::vector<std::vector<AeroResults>> results;
    results.reserve(resultsView.size());
    std::ranges::copy(resultsView, std::back_inserter(results));
    accumulateTotalPolars(outputFile, results);
  }

  return 0;
}
