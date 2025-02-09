#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <ctime>
#include <iostream>
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
  std::vector<std::vector<int>> neighbours;
  neighbours.reserve(searchPoints.size());

  std::transform(searchPoints.begin(), searchPoints.end(), searchRadius.begin(),
                 std::back_inserter(neighbours),
                 [octree](const pcl::PointXYZ &searchPoint, double radius) {
                   std::vector<int> pointIdxRadiusSearch;
                   std::vector<float> pointRadiusSquaredDistance;
                   octree.radiusSearch(searchPoint, radius,
                                       pointIdxRadiusSearch,
                                       pointRadiusSquaredDistance);
                   return pointIdxRadiusSearch;
                 });
  return neighbours;
}
std::vector<std::vector<std::size_t>>
getAllNeighbours(const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                 const EvalPoints<double> &evalPoints,
                 const Eigen::ArrayXd &searchRadius) {

  const auto cloud = convertMat2Cloud(evalPoints.mEvalPoints);
  double resolution = 100;

  Octree octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  const auto searchPoints = convertMat(surfacePanelGeo.centrePoints);
  const auto allNeighbours = queryOctree(octree, searchPoints, searchRadius);
  return allNeighbours;
}
int main() {
  srand((unsigned int)time(NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
  }

  float resolution = 128.0f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

  // Neighbors within voxel search

  std::vector<int> pointIdxVec;

  if (octree.voxelSearch(searchPoint, pointIdxVec)) {
    std::cout << "Neighbors within voxel search at (" << searchPoint.x << " "
              << searchPoint.y << " " << searchPoint.z << ")" << std::endl;

    for (std::size_t i = 0; i < pointIdxVec.size(); ++i)
      std::cout << "    " << (*cloud)[pointIdxVec[i]].x << " "
                << (*cloud)[pointIdxVec[i]].y << " "
                << (*cloud)[pointIdxVec[i]].z << std::endl;
  }

  // K nearest neighbor search

  int K = 10;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  std::cout << "K nearest neighbor search at (" << searchPoint.x << " "
            << searchPoint.y << " " << searchPoint.z << ") with K=" << K
            << std::endl;

  if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
      std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x << " "
                << (*cloud)[pointIdxNKNSearch[i]].y << " "
                << (*cloud)[pointIdxNKNSearch[i]].z
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")"
                << std::endl;
  }

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x << " "
            << searchPoint.y << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

  if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                          pointRadiusSquaredDistance) > 0) {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
      std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x << " "
                << (*cloud)[pointIdxRadiusSearch[i]].y << " "
                << (*cloud)[pointIdxRadiusSearch[i]].z
                << " (squared distance: " << pointRadiusSquaredDistance[i]
                << ")" << std::endl;
  }
}
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
