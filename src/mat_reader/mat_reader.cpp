#include "mat_reader/mat_reader.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>
#include <vector>

std::vector<std::string> split(const std::string &line,
                               const std::string &delimiter) {
  std::vector<std::string> result;
  std::string token;
  std::istringstream tokenStream(line);
  char c;

  // Process each character in the input line
  while (tokenStream.get(c)) {
    // Check if the current character is a delimiter or space
    if (std::string{c} == std::string{" "} || delimiter == std::string{c}) {
      if (!token.empty()) {
        // If token is not empty, add it to the result and clear token
        result.push_back(token);
        token.clear();
      }
    } else {
      // Append character to token
      token += c;
    }
  }

  // Add the last token to the result if not empty
  if (!token.empty()) {
    result.push_back(token);
  }

  return result;
}

using T = double;
struct TableData {

  using SerialData = std::vector<T>;
  std::size_t rows;
  std::size_t cols;
  SerialData serial_data;
};

TableData read_table_data(const std::filesystem::path &file_loc,
                          const std::string &delim, int skip_rows) {
  std::fstream file(file_loc);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file at " + file_loc.string());
  }
  std::string line;
  while (skip_rows--) {
    std::getline(file, line);
  }

  // deduce no of cols
  TableData::SerialData data;
  getline(file, line);
  std::vector<std::string> nums = split(line, delim);
  std::transform(nums.begin(), nums.end(), std::back_inserter(data),
                 [](const std::string &s) { return std::stod(s); });
  std::size_t n_cols = nums.size();
  std::size_t n_rows = 1;
  while (getline(file, line)) {

    std::vector<std::string> nums = split(line, delim);
    std::transform(nums.begin(), nums.end(), std::back_inserter(data),
                   [](const std::string &s) { return std::stod(s); });
    n_rows++;
  }

  return TableData{n_rows, n_cols, data};
}

Eigen::MatrixXd table_to_mat(TableData &&table_data) {
  return Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      std::move(table_data.serial_data.data()), table_data.rows,
      table_data.cols);
}

Eigen::MatrixXd loadtxt(const std::filesystem::path &file_loc,
                        const std::string &delim, int skip_rows) {
  return table_to_mat(read_table_data(file_loc, delim, skip_rows));
}
