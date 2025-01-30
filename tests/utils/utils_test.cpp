#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <functional>
#include "utils/utils.hpp"  // Replace with the actual header file containing the templates
#include <Eigen/Core>

// Test for accumulate_move
TEST(AccumulateMoveTest, HandlesIntegers) {
    std::vector<int> nums = {1, 2, 3, 4};
    auto result = accumulate_move(nums.begin(), nums.end(), 0,
        [](int&& acc, int val) {
            return acc + val;
        });
    EXPECT_EQ(result, 10);
}

TEST(AccumulateMoveTest, HandlesStrings) {
    std::vector<std::string> strings = {"Hello", " ", "World", "!"};
    auto result = accumulate_move(strings.begin(), strings.end(), std::string(),
        [](std::string&& acc, const std::string& val) {
            acc += val;
            return acc;
        });
    EXPECT_EQ(result, "Hello World!");
}

//  Genuinely not sure if this is a good test case.
TEST(AccumulateMoveTest, HandlesVectorToString) {
    std::vector<std::string> strings = {"Hello", "World", "!"};

    auto result = accumulate_move(
        std::make_move_iterator(strings.begin()),
        std::make_move_iterator(strings.end()),
        std::string(),
        []<class String>(String acc, String val) {
            acc += val;
            return acc;
        });

    // Check that the result is the concatenated string
    EXPECT_EQ(result, "HelloWorld!");

    // Check that the original vector is empty
    for (const auto& str : strings) {
        EXPECT_TRUE(str.empty());
    }
}

TEST(AccumulateMoveTest, HandlesVectorOfEigenArrayXXd) {
    using EigenArray = Eigen::ArrayXXd;

    // Initialize some Eigen arrays with the same number of rows but different columns
    EigenArray mat1 = EigenArray::Constant(2, 3, 1.0); // 2 rows, 3 cols, filled with 1.0
    EigenArray mat2 = EigenArray::Constant(2, 2, 2.0); // 2 rows, 2 cols, filled with 2.0
    EigenArray mat3 = EigenArray::Constant(2, 4, 3.0); // 2 rows, 4 cols, filled with 3.0

    std::vector<EigenArray> matrices = {std::move(mat1), std::move(mat2), std::move(mat3)};

    // Accumulate using accumulate_move and hMerge
    auto result = accumulate_move(std::make_move_iterator(matrices.begin()), std::make_move_iterator(matrices.end()), EigenArray(2,0),
        []<class EigenMatType>(EigenMatType acc, EigenMatType mat) {
            return hMerge(std::move(acc), std::move(mat));
        });

    // Check dimensions of the resulting matrix
    EXPECT_EQ(result.rows(), 2); // Rows should match
    EXPECT_EQ(result.cols(), 3 + 2 + 4); // Total columns from all matrices

    // Check values in the resulting matrix
    EXPECT_TRUE(result.block(0, 0, 2, 3).eval().isApprox(EigenArray::Constant(2, 3, 1.0).eval()));
    EXPECT_TRUE(result.block(0, 3, 2, 2).eval().isApprox(EigenArray::Constant(2, 2, 2.0).eval()));
    EXPECT_TRUE(result.block(0, 5, 2, 4).eval().isApprox(EigenArray::Constant(2, 4, 3.0).eval()));

    // Check that the original matrices are empty
    for (const auto& mat : matrices) {
        EXPECT_EQ(mat.size(), 0); // Empty matrices should have zero size
    }
}

// Test for initialize_transform with unary operation
TEST(InitializeTransformUnaryTest, HandlesSquareOperation) {
    std::vector<int> nums = {1, 2, 3, 4};
    auto result = initialize_transform<std::vector<int>>(nums.begin(), nums.end(),
        [](int val) {
            return val * val;
        });
    EXPECT_EQ(result, std::vector<int>({1, 4, 9, 16}));
}

TEST(InitializeTransformUnaryTest, HandlesStringLength) {
    std::vector<std::string> words = {"a", "ab", "abc"};
    auto result = initialize_transform<std::vector<size_t>>(words.begin(), words.end(),
        [](const std::string& word) {
            return word.length();
        });
    EXPECT_EQ(result, std::vector<size_t>({1, 2, 3}));
}

// Test for initialize_transform with binary operation
TEST(InitializeTransformBinaryTest, HandlesSumOfTwoRanges) {
    std::vector<int> nums1 = {1, 2, 3};
    std::vector<int> nums2 = {4, 5, 6};
    auto result = initialize_transform<std::vector<int>>(nums1.begin(), nums1.end(), nums2.begin(),
        [](int val1, int val2) {
            return val1 + val2;
        });
    EXPECT_EQ(result, std::vector<int>({5, 7, 9}));
}

TEST(InitializeTransformBinaryTest, HandlesStringConcatenation) {
    std::vector<std::string> nums1 = {"a", "b", "c"};
    std::vector<std::string> nums2 = {"x", "y", "z"};
    auto result = initialize_transform<std::vector<std::string>>(nums1.begin(), nums1.end(), nums2.begin(),
        [](const std::string& val1, const std::string& val2) {
            return val1 + val2;
        });
    EXPECT_EQ(result, std::vector<std::string>({"ax", "by", "cz"}));
}
