#include <Eigen/Core>
#include "panel_geo/panel_geo.hpp"
#include "surface/surface_reader.hpp"
#include <numeric>
#include <ranges>
Eigen::Array3d getFreeStream(double aoa, double Vinf);

  
using namespace std::ranges;
template <typename Derived>
Derived rotate_2d_about_origin(const Eigen::MatrixBase<Derived> &points2d,
                               double angle_d);
void rotate_3d_about_origin(Eigen::Ref<Eigen::ArrayX3d> points3d,
                            double angle_d);
  

void rotate_points_about_start(Eigen::Ref<Eigen::ArrayX3d> points3d,
                               double angle_d);
  
void align_wake_to_flow(std::vector<PanelSet> &panel_sets, double aoa);
  

std::vector<PanelGeometryPair>
calc_panel_geometry(std::vector<PanelSet> &panel_sets);
  


template <class T>auto makeChunkData(const auto &panelGeometry) {

  auto chunkSize = panelGeometry | views::transform([](const auto &pg) {
                     return pg.centrePoints.rows();
                   });
  std::vector<size_t> chunkStart(chunkSize.size(), 0);
  std::exclusive_scan(chunkSize.begin(), chunkSize.end(), chunkStart.begin(),
                      0);
  return std::pair{chunkStart, chunkSize};
}
 
