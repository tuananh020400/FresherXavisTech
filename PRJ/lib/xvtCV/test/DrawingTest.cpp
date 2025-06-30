/**
 * @example{lineno} test/DrawingTest.cpp
 * This example demonstrates how to use xvt::Drawing class.
 */

#include "pch.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtConvert.h"
#include "xvtCV/IInspection.h"
#include <opencv2/highgui.hpp>

TEST(DrawingPlotTest, EmptyPointVector) {
    xvt::Drawing drawing;
    cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<cv::Point2f> emptyPoints;
    bool isUpdateScale = true;

    cv::Mat originalImg = img.clone();

    drawing.Plot(img, emptyPoints, isUpdateScale);

}

TEST(DrawingTest, EmptyImageMatrix) {
    xvt::Drawing drawing;
    cv::Mat emptyImg;
    std::vector<cv::Point2f> points(5);

    bool isUpdateScale = false;

    cv::Mat originalImg = emptyImg.clone();

    drawing.Plot(emptyImg, points, isUpdateScale);
}

TEST(DrawingTest, NegativeCoordinatePoint) {
    xvt::Drawing drawing;
    cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<cv::Point2f> points = { cv::Point2f(-10.5f, 30.0f), cv::Point2f(20.0f, -15.0f) };
    bool isUpdateScale = true;

    drawing.Plot(img, points, isUpdateScale);

}

TEST(DrawingTest, UpdateScaleTrue) {
    xvt::Drawing drawing;
    cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<cv::Point2f> points = { cv::Point2f(10.0f, 20.0f), cv::Point2f(30.0f, 40.0f) };
    bool isUpdateScale = true;

    drawing.Plot(img, points, isUpdateScale);

}

TEST(DrawingTest, UpdateScaleFalse) {
    xvt::Drawing drawing;
    cv::Mat img(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
    std::vector<cv::Point2f> points = { cv::Point2f(10.0f, 20.0f), cv::Point2f(30.0f, 40.0f) };
    bool isUpdateScale = false;

    drawing.Plot(img, points, isUpdateScale);
}

TEST(DrawingTest, DrawRulerTest1) {
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    xvt::Drawing drawing;
    cv::Point start(-10, 10);
    cv::Point end(490, 490);
    drawing.DrawRuler(img, start, end);
    //cv::imshow("DrawRuler", img);
    cv::waitKey(0);
}

TEST(DrawingTest, EmptyImage) {
    xvt::Drawing drawing;
    cv::Mat emptyImg;
    cv::Point2f point(10, 10);

    ASSERT_NO_THROW(drawing.Plot(emptyImg, point));
}

TEST(DrawingTest, GrayImage_Conversion) {
    xvt::Drawing drawing;
    cv::Mat grayImg(100, 100, CV_8UC1); 
    cv::Point2f point(50, 50);

    ASSERT_NO_THROW(drawing.Plot(grayImg, point));

    ASSERT_EQ(grayImg.channels(), 3);
}

TEST(DrawingTest, Plot_WithColorChange) {
    xvt::Drawing drawing;
    cv::Mat image(100, 100, CV_8UC3); 
    cv::Point2f point(50, 50);
    cv::Scalar color(0, 255, 255); 

    ASSERT_NO_THROW(drawing.Plot(image, point, color));

    cv::Vec3b pixelColor = image.at<cv::Vec3b>(point);
}

TEST(DrawingTest, PlotRectangle_ValidConversion) {
    xvt::Drawing drawing;
    cv::Mat image(100, 100, CV_8UC3); 
    cv::Rect rect(10, 10, 50, 50); 

    ASSERT_NO_THROW(drawing.Plot(image, rect));

}

TEST(DrawingTest, EmptyImg) {
    xvt::Drawing drawing;
    cv::Mat image; 
    cv::Rect rect(10, 10, 50, 50); 

    ASSERT_NO_THROW(drawing.Plot(image, rect));

    ASSERT_TRUE(image.empty()); 
}

TEST(DrawingTest, DrawColorRect) {
    xvt::Drawing drawing;
    cv::Mat image(100, 100, CV_8UC3);
    cv::Rect rect(10, 10, 50, 50);

    ASSERT_NO_THROW(drawing.Plot(image, rect, cv::Scalar(255, 255, 0))); 
}

TEST(DrawingTest, DrawMultiRect) {
    xvt::Drawing drawing;
    cv::Mat image(100, 100, CV_8UC3);
    std::vector<cv::Rect> rectList;
    cv::Rect rect1(10, 10, 50, 50);
    rectList.push_back(rect1);
    cv::Rect rect2(20, 10, 50, 50);
    rectList.push_back(rect2);

    ASSERT_NO_THROW(drawing.Plot(image, rectList, cv::Scalar(255, 255, 0))); 
}

//! [xvtCV Drawing Example] //Snippet
TEST(DrawingTest, DrawExample)
{
    // Read the signal image
    cv::Mat im = cv::imread("line_image_test/Ver2W2B.jpg", 0);
    if (im.empty()) return;

    // Reduce image to single row
    std::vector<float> reduce;
    cv::reduce(im, reduce, 0, cv::ReduceTypes::REDUCE_AVG);

    // Method 1: Use Sobel operator and filter to extract edge information

    // Get the gausian kernel
    cv::Mat gauKernel = cv::getGaussianKernel(9, -1, CV_32F);

    cv::Mat sobKernelX;
    // Derivate the gausian kernel
    cv::Sobel(-gauKernel, sobKernelX, CV_32F, 0, 1, 3);

    cv::Mat sobelImg;
    // Get the edge information
    cv::filter2D(reduce, sobelImg, CV_32F, sobKernelX.t());

    // Method 2: Manually smooth the signal
    // and then differentiate it to obtain edge information

    cv::Mat im2 = cv::Mat_<float>(reduce);
    // Smooth the signal
    cv::GaussianBlur(reduce, reduce, cv::Size(5, 1), -1);
    std::vector<int> xList(im.cols,0);
    for (int i = 0; i < im.cols; i++)
    {
        xList[i] = i;
    }

    std::vector<float> kernel = { -2, 0 , 2 };

    std::vector<float> diff;
    // Get the edge information
    cv::filter2D(reduce, diff, -1, kernel);

    xvt::Drawing drawtool;
    drawtool.yTopOrigin = false;

    // Draw the signal by the yellow color
    drawtool.Plot(im, xList, reduce, true);

    // Draw the edge signal by medthod 1 the red color
    drawtool.color = cv::Scalar(0, 0, 255);
    drawtool.Plot(im, xList, diff, true);

    // Draw the edge signal by medthod 2 the green color
    drawtool.color = cv::Scalar(0, 255, 0);
    drawtool.Plot(im, xList, std::vector<float>(sobelImg), true);

}
//! [xvtCV Drawing Example] //Snippet