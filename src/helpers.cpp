#include <Eigen/Core>
#include <cmath>
#include "panel_geo/panel_geo.hpp"
#include "surface/surface_reader.hpp"
#include <ranges>

Eigen::Array3d getFreeStream(double aoa, double Vinf) {

  double angleOfAttack = aoa * M_PI / 180;
  return {std::cos(angleOfAttack), 0, std::sin(angleOfAttack)};
}
using namespace std::ranges;
template <typename Derived>
Derived rotate_2d_about_origin(const Eigen::MatrixBase<Derived> &points2d,
                               double angle_d) {
  double angle_r = angle_d * M_PI / 180.0;
  return (Eigen::Matrix2d{{std::cos(angle_r), -std::sin(angle_r)},
                          {std::sin(angle_r), std::cos(angle_r)}}) *
         points2d;
}

void rotate_3d_about_origin(Eigen::Ref<Eigen::ArrayX3d> points3d,
                            double angle_d) {
  Eigen::MatrixXd points = rotate_2d_about_origin(
      (Eigen::MatrixXd(2, points3d.rows()) << points3d.col(0).transpose(),
       points3d.col(2).transpose())
          .finished(),
      angle_d);
  points3d.col(0) = points.row(0).transpose();
  points3d.col(2) = points.row(1).transpose();
}

void rotate_points_about_start(Eigen::Ref<Eigen::ArrayX3d> points3d,
                               double angle_d) {
  if (points3d.rows() == 0)
    return;

  Eigen::RowVector3d original_loc(points3d(0, 0), 0, points3d(0, 2));
  points3d = points3d.rowwise() - original_loc.array(); // translate to origin
  rotate_3d_about_origin(points3d, angle_d);
  points3d = points3d.rowwise() + original_loc.array(); // translate from origin
}
void align_wake_to_flow(std::vector<PanelSet> &panel_sets, double aoa) {
  std::ranges::for_each(
      panel_sets,
      [&aoa](WakePanel &wake) {
        auto nX =  wake.nXsecs+1;
        if (nX > 1){
          for (auto i : RANGE(wake.nYsecs+1)){ 
             rotate_points_about_start(wake.mPoints.middleRows(i*nX, nX), aoa);
            ;
          }
        }
      },
      &PanelSet::wake);
}

std::vector<PanelGeometryPair>
calc_panel_geometry(std::vector<PanelSet> &panel_sets) {
  std::vector<PanelGeometryPair> panelGeometryPair;
  panelGeometryPair.reserve(panel_sets.size());
  std::ranges::copy(panel_sets | std::views::transform([](PanelSet &pset) {
                      return std::make_pair(
                          PanelGeometry<SurfacePanel>(pset.body),
                          PanelGeometry<WakePanel>(pset.wake));
                    }),
                    std::back_inserter(panelGeometryPair));
  return panelGeometryPair;
}
