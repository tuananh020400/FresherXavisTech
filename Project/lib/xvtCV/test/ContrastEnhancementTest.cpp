/**
 * @example{lineno} test/ContrastEnhancementTest.cpp
 * This example demonstrates how to enhance image by xvt::enhance.
 */

#include "pch.h"
#include "xvtCV/ContrastEnhancement.h"
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

TEST(ContrastEnhancement, TestAGCIE) {
	cv::Mat test_image = cv::Mat(100,100, CV_8UC1, cv::Scalar(100));
	cv::Mat dst;
	xvt::enhance::AGCIE(test_image, dst);
	ASSERT_FALSE(dst.empty());
	EXPECT_EQ(test_image.size(), dst.size());
}

TEST(ContrastEnhancement, TestIAGCWD) {
	cv::Mat test_image = cv::Mat(100, 100, CV_8UC1, cv::Scalar(100));
	cv::Mat dst;
	xvt::enhance::IAGCWD(test_image, dst);
	ASSERT_FALSE(dst.empty());
	EXPECT_EQ(test_image.size(), dst.size());
}

TEST(ContrastEnhancement, TestPerformSUACE) {
	cv::Mat test_image = cv::Mat(100, 100, CV_8UC1, cv::Scalar(100));
	cv::Mat dst;
	xvt::enhance::PerformSUACE(test_image, dst, 3, 3);
	cv::waitKey(0);
	ASSERT_FALSE(dst.empty());
	EXPECT_EQ(test_image.size(), dst.size());
}

