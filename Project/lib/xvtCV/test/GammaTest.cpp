/**
 * @example{lineno} test/GammaTest.cpp
 * This example demonstrates how to use xvt::enhance::GammaCorrector class.
 */

#include "pch.h"
#include "xvtCV/GammaCorrector.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

TEST(GammaCorrector, colorImage) {
    std::string path = "../samples/UPPER/2022-10-19 091225__2022-10-19 091232_H26_0010_22-2438R_UPPER_123_OK_Color_8bit.tif";
    cv::Mat src = cv::imread(path, 1);
    cv::Mat dst;
    float const g = 1.8f;
    xvt::enhance::GammaCorrector gamma;
    gamma.SetGamma(g);
    gamma.Apply(src, dst);
    //cv::imwrite(path, src);
    EXPECT_FLOAT_EQ(g, gamma.GetGamma());
    EXPECT_TRUE(!gamma.GetLUT().empty());
}