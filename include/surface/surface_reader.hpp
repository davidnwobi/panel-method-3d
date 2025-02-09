#pragma once
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>

template <class ScalarType> ScalarType stringToNum(const std::string &str) {
  if constexpr (std::is_integral<ScalarType>::value) {
    return static_cast<ScalarType>(std::stoi(str));
  } else if constexpr (std::is_floating_point<ScalarType>::value) {
    auto out = static_cast<ScalarType>(std::stod(str));
    return std::abs(out) < 1e-10 ? 0.0 : out;
  } else {
    throw std::invalid_argument(
        std::string("Can't convert string to type of ") +
        typeid(ScalarType).name());
  }
}

struct PanelSet {
  SurfacePanel body;
  WakePanel wake;

  PanelSet() = default;
  PanelSet(SurfacePanel &&body, WakePanel &&wake) noexcept
      : body(body), wake(wake) {}
};

std::vector<PanelSet>
readConvertedComponentsFromFile(const std::filesystem::path &filePath,
                                char delim = ',');

class SurfaceFileReader {
private:
  std::fstream mSurfaceFile;
  char mDelim;
  std::vector<std::string> splitLine(std::string &data);

public:
  SurfaceFileReader(const std::filesystem::path &filePath, char delim = ',');
  std::vector<std::string> getNextLine();
  [[nodiscard]] bool isEOF();
};

template <class EigenMatrixType>
void readTable(SurfaceFileReader &fileReader, EigenMatrixType &table) {
  // Each data row is preceed by an index. This is skipped when storing in the
  // matrix
  using ScalarType = typename EigenMatrixType::Scalar;

  // Skip headers
  fileReader.getNextLine();
  for (auto tableRow : table.rowwise()) {
    auto line = fileReader.getNextLine();
    std::transform(line.begin() + 1, line.end(), tableRow.begin(),
                   stringToNum<ScalarType>);
  }
}
