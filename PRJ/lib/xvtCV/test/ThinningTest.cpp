/**
 * @example{lineno} test/ThinningTest.cpp
 * This example demonstrates how to use xvt::Thinning function.
 */

#include "pch.h"
#include "xvtCV/Thinning.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

TEST(ThinningTest, ThinImage) {
    // Create a sample binary image
    cv::Mat input(10, 10, CV_8UC1, cv::Scalar(0));
    cv::rectangle(input, cv::Point(2, 2), cv::Point(8, 8), cv::Scalar(255), cv::FILLED);

    // Apply thinning
    cv::Mat output;
    xvt::Thinning(input, output, xvt::ThinningTypes::THINNING_ZHANGSUEN);

    // Check that the output is a binary image
    EXPECT_EQ(output.type(), CV_8UC1);

    // Check that the output has the same dimensions as the input
    EXPECT_EQ(output.rows, input.rows);
    EXPECT_EQ(output.cols, input.cols);

    // Check that the output has a thinner boundary than the input
    int inputBoundary = cv::countNonZero(input);
    int outputBoundary = cv::countNonZero(output);
    EXPECT_GT(inputBoundary, outputBoundary);
}
