#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <vector>

using Octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;

inline auto convertMat2Cloud(const Eigen::ArrayX3d &mat) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = mat.rows();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    (*cloud)[i].x = mat(i, 0);
    (*cloud)[i].y = mat(i, 1);
    (*cloud)[i].z = mat(i, 2);
  }
  return cloud;
}

inline auto createSearchPoints(const Eigen::ArrayX3d &mat) {
  std::vector<pcl::PointXYZ> searchPoints;
  searchPoints.reserve(mat.rows());
  auto createPoints = [](const Eigen::RowVector3d &row) {
    pcl::PointXYZ out;
    out.x = row(0);
    out.y = row(1);
    out.z = row(2);
    return out;
  };
  std::transform(mat.rowwise().begin(), mat.rowwise().end(),
                 std::back_inserter(searchPoints), createPoints);
  return searchPoints;
}

inline auto queryOctree(const Octree &octree,
                        const std::vector<pcl::PointXYZ> &searchPoints,
                        const Eigen::ArrayXd &searchRadius) {
  std::vector<std::vector<std::size_t>> neighbours;
  neighbours.reserve(searchPoints.size());

  std::transform(searchPoints.begin(), searchPoints.end(), searchRadius.begin(),
                 std::back_inserter(neighbours),
                 [octree](const pcl::PointXYZ &searchPoint, double radius) {
                   std::vector<int> pointIdxRadiusSearch;
                   std::vector<float> pointRadiusSquaredDistance;
                   octree.radiusSearch(searchPoint, radius,
                                       pointIdxRadiusSearch,
                                       pointRadiusSquaredDistance);

                   std::vector<std::size_t> idxs(pointIdxRadiusSearch.size());
                   std::copy(pointIdxRadiusSearch.begin(),
                             pointIdxRadiusSearch.end(), idxs.begin());
                   return idxs;
                 });
  return neighbours;
}

constexpr bool operator==(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
  return std::abs(p1.x - p2.x) < 1e-10 && std::abs(p1.y - p2.y) < 1e-10 &&
         std::abs(p1.z - p2.z) < 1e-10;
}
struct PointLess {
  constexpr bool operator()(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    return std::tie(p1.x, p1.y, p1.z) < std::tie(p2.x, p2.y, p2.z);
  }
};
