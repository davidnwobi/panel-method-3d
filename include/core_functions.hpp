#include "aerocalcs/aerocalcsingle.hpp"
#include <Eigen/Core>
#include <string>
#include <vector>

std::vector<AeroResults>
run_analysis(const FlowParams &flowParams, const ReferenceGeom &refGeom,
             const std::string &inputFile, const std::string &outputFile,
             float dropTol, float spTol, int solver, bool rotate_wake);
