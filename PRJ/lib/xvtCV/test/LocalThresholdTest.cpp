/**
 * @example{lineno} test/LocalThresholdTest.cpp
 * This example demonstrates how to use xvt::threshold classes.
 */

#include "pch.h"
#include "xvtCV/LocalThresholding.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>


TEST(LocalThreshold, ImageEmpty)
{
    cv::Mat inputImg, outputImg;
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_TRUE(outputImg.empty());
}

TEST(LocalThreshold, ImageColor)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_8UC3);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_EQ(outputImg.type(), CV_8UC1);
    EXPECT_TRUE(outputImg.empty());
}

TEST(LocalThreshold, Image16BitsGray)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_16UC1);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_TRUE(outputImg.empty());
    EXPECT_EQ(outputImg.depth(), CV_8UC1);
    EXPECT_EQ(outputImg.channels(), 1);
}


TEST(LocalThreshold, WindowSizeEqual0)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_8UC1);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(0, 0));
    EXPECT_EQ(inputImg.size(), outputImg.size());
    EXPECT_EQ(outputImg.type(), CV_8UC1);
}

TEST(LocalThreshold, InputSizeDiffOutputSize)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_8UC1);
    outputImg.create(400, 500, CV_8UC1);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_EQ(inputImg.size(), outputImg.size());
    EXPECT_EQ(outputImg.type(), CV_8UC1);
}

TEST(LocalThreshold, InputSizeDiffOutputSize1)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_8UC1);
    outputImg.create(400, 300, CV_8UC1);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_EQ(inputImg.size(), outputImg.size());
    EXPECT_EQ(outputImg.type(), CV_8UC1);
}

TEST(LocalThreshold, InputSizeDiffOutputSize2)
{
    cv::Mat inputImg, outputImg;
    outputImg.create(400, 300, CV_8UC1);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_TRUE(outputImg.empty());
    EXPECT_EQ(outputImg.type(), CV_8UC1);
}

TEST(LocalThreshold, InOutDiffChannel)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_8UC1);
    outputImg.create(300, 400, CV_8UC3);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_EQ(outputImg.size(), inputImg.size());
    EXPECT_EQ(outputImg.type(), CV_8UC1);
}

TEST(LocalThreshold, InOutDiffChannel1)
{
    cv::Mat inputImg, outputImg;
    inputImg.create(300, 400, CV_8UC3);
    outputImg.create(300, 400, CV_8UC1);
    xvt::threshold::ThresholdSauvola(inputImg, outputImg, cv::Size(5, 5));
    EXPECT_TRUE(outputImg.empty());
    EXPECT_EQ(outputImg.type(), CV_8UC1);
}
