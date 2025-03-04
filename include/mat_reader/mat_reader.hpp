#pragma once
#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <ranges>
#include <span>
Eigen::MatrixXd loadtxt(const std::filesystem::path &file_loc,
                        const std::string &delim = " ", int skip_rows = 0);

template <typename Derived>
void savetxt(const std::filesystem::path &fileName,
             const Eigen::DenseBase<Derived> &matrix,
             const std::string &delim = " ",
             std::span<const std::string> headers = {}) {

  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, delim, "\n");

  // Output file stream to write the CSV file
  std::ofstream file(fileName);

  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file at " + fileName.string());
  }
  if (file.is_open()) {

    if (headers.size() > 0) {
      // add a delimeter and write to file
      auto headerView = headers | std::views::transform([&delim](auto val) {
                          return val + delim;
                        });
      std::ranges::for_each(headerView, [&file](auto val) {
        std::ranges::copy(val, std::ostreambuf_iterator<char>(file));
      });
      file << '\n';
    }

    file << matrix.format(CSVFormat);
    file.close();
  }
}
