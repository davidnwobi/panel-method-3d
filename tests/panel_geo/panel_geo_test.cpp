#include <gtest/gtest.h>
#include <filesystem>
#include <iterator>
#include "panel_geo/panel_geo.hpp"
#include "surface/surface_reader.hpp"
#include "panel_geo/panel_geo.hpp"
#include "evalPoints.hpp"
#include <iomanip>
#include "compTask.hpp"

std::filesystem::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");

TEST(SinglePanelTest, testCenterPointsandNormals) {
	
	std::filesystem::path filePath(testDataLoc.string() + "/singlepanel.txt");
	auto [surfaces, wakes] = readConvertedComponentsFromFile(filePath);
	PanelGeometry panelGeo(surfaces[0]);

	EXPECT_TRUE(panelGeo.centrePoints.row(0).isApprox(Eigen::RowVector3d(0.5, 0.5, 0))) << panelGeo.centrePoints.row(0) << " is not equal to " << (Eigen::RowVector3d(0.5, 0.5, 0));
	EXPECT_TRUE(panelGeo.tangentXVectors.row(0).isApprox(Eigen::RowVector3d(1.0, 0.0, 0.0))) << panelGeo.tangentXVectors.row(0) << " is not equal to " << (Eigen::RowVector3d(1.0, 0.0, 0.0));
	EXPECT_TRUE(panelGeo.tangentYVectors.row(0).isApprox(Eigen::RowVector3d(0.0, 1.0, 0.0))) << panelGeo.tangentYVectors.row(0) << " is not equal to  " << (Eigen::RowVector3d(0.0, 1.0, 0.0));
	EXPECT_TRUE(panelGeo.normalVectors.row(0).isApprox(Eigen::RowVector3d(0.0, 0.0, 1.0))) << panelGeo.normalVectors.row(0) << " is not equal to " << (Eigen::RowVector3d(0.0, 0.0, 1.0));
}


TEST(TestLocalConversion, testLocalConversion) {

	std::filesystem::path filePath(testDataLoc.string() + "/naca0010lowres.txt");
	auto [surfaces, wakes] = readConvertedComponentsFromFile(filePath);
	auto panelGeoSurf = PanelGeometry(surfaces[0]);
	
	EvalPoints evalPoints;
	evalPoints.mEvalPoints = panelGeoSurf.centrePoints + panelGeoSurf.normalVectors;

	std::vector<std::size_t> idxs(evalPoints.mEvalPoints.rows());
	std::iota(idxs.begin(), idxs.end(), 0);

	for (int i = 0; i < 5; i++) {
		EXPECT_TRUE(createInfluenceComputeTask(panelGeoSurf, evalPoints, i, idxs).points.row(i).isApprox(Eigen::RowVector3d(0.0, 0.0, 1.0))); 
	}
}

TEST(SinglePanelTest, testArea) {
	
	std::filesystem::path filePath(testDataLoc.string() + "/singlepanel.txt");
	auto [surfaces, wakes] = readConvertedComponentsFromFile(filePath);
	PanelGeometry panelGeo(surfaces[0]);

	EXPECT_TRUE(std::abs(panelGeo.areas(0) - 1) < 1e-6) << panelGeo.areas(0) << " is not equal to " << 1;
}
