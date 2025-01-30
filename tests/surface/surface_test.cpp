#include <gtest/gtest.h>
#include "surface/surface_reader.hpp"
#include "panel_geo/panel_geo.hpp"
#include <numeric>
#include <filesystem>
#include <Eigen/Core>

std::filesystem::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");

TEST(VMERGE, SimpleMerge) {
	Eigen::MatrixXd matA = MatrixXd::Random(10, 4);
	Eigen::MatrixXd matB = MatrixXd::Random(10, 4);


	
	auto matC = vMerge(matA, matB);

	EXPECT_TRUE(matC.rows() == matA.rows() + matB.rows());
	EXPECT_TRUE(matC.middleRows(0, matA.rows()).isApprox(matA));
	EXPECT_TRUE(matC.middleRows(matA.rows(), matB.rows()).isApprox(matB));
}
TEST(VMERGE, SimpleMergeIntoSameArray) {
	//Test Merging into the same array;
	
	Eigen::MatrixXd matA = MatrixXd::Random(4, 4);
	Eigen::MatrixXd matB = MatrixXd::Random(5, 4);
	Eigen::MatrixXd matD = MatrixXd::Random(3, 4);


	
	Eigen::MatrixXd matC(Eigen::Map<Eigen::MatrixXd>(matA.data(), matA.rows(), matA.cols()));
	matC = vMerge(matC, matB);
	matC = vMerge(matC, matD);

	EXPECT_TRUE(matC.rows() == matA.rows() + matB.rows() + matD.rows());
	EXPECT_TRUE(matC.middleRows(0, matA.rows()).isApprox(matA));
	EXPECT_TRUE(matC.middleRows(matA.rows(), matB.rows()).isApprox(matB));
	EXPECT_TRUE(matC.middleRows(matA.rows()+matB.rows(), matD.rows()).isApprox(matD));


}

TEST(HMERGE, SimpleMerge) {
	Eigen::MatrixXd matA = MatrixXd::Random(10, 4);
	Eigen::MatrixXd matB = MatrixXd::Random(10, 4);

	
	auto matC = hMerge(matA, matB);

	EXPECT_TRUE(matC.cols() == matA.cols() + matB.cols());
	EXPECT_TRUE(matC.middleCols(0, matA.cols()).isApprox(matA));
	EXPECT_TRUE(matC.middleCols(matA.cols(), matB.cols()).isApprox(matB));
}
TEST(HMERGE, SimpleMergeIntoSameArray) {
	//Test Merging into the same array;
	
	Eigen::MatrixXd matA = MatrixXd::Random(4, 4);
	Eigen::MatrixXd matB = MatrixXd::Random(4, 5);
	Eigen::MatrixXd matD = MatrixXd::Random(4, 3);


	
	Eigen::MatrixXd matC(Eigen::Map<Eigen::MatrixXd>(matA.data(), matA.rows(), matA.cols()));
	matC = hMerge(matC, matB);
	matC = hMerge(matC, matD);

	EXPECT_TRUE(matC.cols() == matA.cols() + matB.cols() + matD.cols());
	EXPECT_TRUE(matC.middleCols(0, matA.cols()).isApprox(matA));
	EXPECT_TRUE(matC.middleCols(matA.cols(), matB.cols()).isApprox(matB));
	EXPECT_TRUE(matC.middleCols(matA.cols()+matB.cols(), matD.cols()).isApprox(matD));


}
TEST(SurfaceTest, testSurfaceConcatenation) {
	

	std::cout << "Next\n";
	std::filesystem::path filePath(testDataLoc.string() + "/converted_4elementairfoil_DegenGeom.txt");
	auto [surfaces, wakes] = readConvertedComponentsFromFile(filePath);
	
	int finalmPointsRows = surfaces[0].getPoints().rows() + surfaces[1].getPoints().rows();
	int surf0mPointsSize = surfaces[0].getPoints().rows();

	int finalmFaceNodeIdxRows = surfaces[0].getFaceNodeIdx().rows() + surfaces[1].getFaceNodeIdx().rows();
	int surf0mFaceNodeIdxSize = surfaces[0].getFaceNodeIdx().rows();
	
	
	surfaces[0] += surfaces[1];

	EXPECT_TRUE(surfaces[0].getFaceNodeIdx().rows() == finalmFaceNodeIdxRows);
	EXPECT_TRUE(surfaces[0].getFaceNodeIdx().middleRows(surf0mFaceNodeIdxSize, surfaces[1].getFaceNodeIdx().rows()).isApprox(surfaces[1].getFaceNodeIdx()+surf0mPointsSize));

	EXPECT_TRUE(surfaces[0].getPoints().rows() == finalmPointsRows);
	EXPECT_TRUE(surfaces[0].getPoints().middleRows(surf0mPointsSize, surfaces[1].getPoints().rows()).isApprox(surfaces[1].getPoints()));

}

TEST(SurfaceTest, testGetWithIndex) {
	std::filesystem::path filePath(testDataLoc.string() + "/converted_4elementairfoil_DegenGeom.txt");
	auto [surfaces, wakes] = readConvertedComponentsFromFile(filePath);

	std::vector<int> idx(10);
	std::iota(idx.begin(), idx.end(), 0);

	assert(surfaces[0].getPoints().topRows(10).isApprox(surfaces[0].getWithIndex(idx)));

}


