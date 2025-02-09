#include <Eigen/Core>
#include <filesystem>
Eigen::MatrixXd loadtxt(const std::filesystem::path &file_loc,
                        const std::string &delim = " ", int skip_rows = 0);
