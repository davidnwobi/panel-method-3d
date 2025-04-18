#include <Eigen/Core>
#include "panel_geo/panel_geo.hpp"
#include "surface/surface_reader.hpp"
#include <numeric>
#include <ranges>
Eigen::Array3d getFreeStream(double aoa, double Vinf);

  
using namespace std::ranges;

void rotate_3d_about_origin(Eigen::Ref<Eigen::ArrayX3d> points3d,
                            double angle_d);
  

void rotate_points_about_start(Eigen::Ref<Eigen::ArrayX3d> points3d,
                               double angle_d);
  
void align_wake_to_flow(std::vector<PanelSet> &panel_sets, double aoa);
  

std::vector<PanelGeometryPair>
calc_panel_geometry(std::vector<PanelSet> &panel_sets);


  


 


std::pair<FlowParams, ReferenceGeom> parse_param(const std::filesystem::path &fpath);
  
std::pair<std::vector<FlowParams>, ReferenceGeom> 
parse_param_batch(const std::filesystem::path &fpath);
  
