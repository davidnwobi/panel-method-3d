#include "pcl/impl/point_types.hpp"
#include "pm_octree_defs.hpp"
#include "gtest/gtest.h"
#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <vector>

using Octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;

TEST(PM_OCTREE, PM_OCTREE) {

  std::size_t n = 100;
  Eigen::ArrayX3d mat(n * n, 3); // make serialized regular grid;
  for (std::size_t j = 0; j < n; j++) {
    for (std::size_t k = 0; k < n; k++) {
      mat.row(k + j * n) << 0.0, (double)j, (double)k;
    }
  }

  // Typical point tree  workflow for panel methods
  auto const cloud = convertMat2Cloud(mat);
  double resolution = 1;
  Octree octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  const auto searchPoints = createSearchPoints(mat);
  int scaleSize = 1;
  const auto allNeighbours = queryOctree(
      octree, searchPoints, Eigen::ArrayXd::Ones(n * n) * scaleSize);

  int testLoc = 5934; // arbitary point somewhere in the grid
  const auto &testNeigbours = allNeighbours[testLoc];
  auto areaScalingRG = [](double scaleSize) -> std::size_t {
    return 4 * (std::floor(scaleSize) + 1) * (std::floor(scaleSize)) + 1;
  };
  EXPECT_TRUE((areaScalingRG(scaleSize) - testNeigbours.size()) ==
              0); // this is what the math expects.

  auto centerPoint = (*cloud)[testLoc];
  std::vector<pcl::PointXYZ> testQueryPoints;
  testQueryPoints.reserve(testNeigbours.size());
  std::ranges::transform(testNeigbours, std::back_inserter(testQueryPoints),
                         [&cloud](std::size_t i) { return (*cloud)[i]; });
  std::sort(testQueryPoints.begin(), testQueryPoints.end(), PointLess{});
  for (auto point : testQueryPoints) {
    std::cout << point;
  }
  std::cout << "\n";

  int iPoint = 0;
  int lim = scaleSize - 1;
  // Permute points from lexographic smallest to largest and compare
  for (auto j = centerPoint.y - lim; j <= centerPoint.y + lim; j++) {
    for (auto k = centerPoint.z - lim; k <= centerPoint.z + lim; k++) {
      EXPECT_TRUE((testQueryPoints[iPoint++] == pcl::PointXYZ{0, j, k}));
    }
  }
}
