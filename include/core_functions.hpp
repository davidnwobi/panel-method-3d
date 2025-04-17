#include "aerocalcs/aerocalcsingle.hpp"
#include <Eigen/Core>
#include <string>
#include <vector>

std::vector<AeroResults> run_analysis(const FlowParams &flowParams,
                                      const ReferenceGeom &refGeom,
                                      const std::string &inputFile,
                                      const std::string &outputFile,
                                      double dropTol, bool rotate_wake);
