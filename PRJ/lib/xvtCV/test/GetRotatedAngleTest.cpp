/**
 * @example{lineno} test/GetRotatedAngleTest.cpp
 * This example demonstrates how to use xvt::GetRotatedAngle class.
 */

#include "pch.h"
#include "xvtCV/Contour.h"
#include "xvtCV/Utils.h"
#include "xvtCV/xvtConvert.h"

#define M_PI 3.14159265358979323846

TEST(GetRotateAngleTest, Case01)
{
    //case 1 (45 degree)
    double angle = 45.0 * M_PI / 180.0; // convert to radians
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on negative sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, 45.0);
}
TEST(GetRotateAngleTest, Case02)
{
    //case 2 (90 degree)
    double angle = 90.0 * M_PI / 180.0; // convert to radians
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on negative sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, 90.0);
}
TEST(GetRotateAngleTest, Case03)
{
    //case 3 (-45 degree)
    int sizePoint = 1000;
    double angle = -45.0 * M_PI / 180.0; // convert to radians
    std::vector<cv::Point> listPoints;
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on sine of angle
        listPoints.push_back(cv::Point(x,y));
    }
    auto rect = cv::boundingRect(listPoints);
    cv::Mat drawImg = cv::Mat::zeros(sizePoint, sizePoint, CV_8UC3);
    cv::line(drawImg, listPoints[0], listPoints.back(), cv::Scalar(255, 0, 255));

    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    if (angleDegree > 0) {
        angleDegree = 90 - angleDegree;
    }
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, -45.0);
}
TEST(GetRotateAngleTest, Case04)
{
    //case 4 (-20 degree)
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    double angle = -20.0 * M_PI / 180.0; // convert to radians
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    auto rect = cv::boundingRect(listPoints);
    cv::Mat drawImg = cv::Mat::zeros(sizePoint, sizePoint, CV_8UC3);
    cv::line(drawImg, listPoints[0], listPoints.back(), cv::Scalar(255, 0, 255));

    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    if (angleDegree > 0) {
        angleDegree = 90 - angleDegree;
    }
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, -20.0);
}
TEST(GetRotateAngleTest, Case05)
{
    //case 5 (-32 degree)
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    double angle = -32.0 * M_PI / 180.0; // convert to radians
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    auto rect = cv::boundingRect(listPoints);
    cv::Mat drawImg = cv::Mat::zeros(sizePoint, sizePoint, CV_8UC3);
    cv::line(drawImg, listPoints[0], listPoints.back(), cv::Scalar(255, 0, 255));

    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    if (angleDegree > 0) {
        angleDegree = 90 - angleDegree;
    }
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, -32.0);
}
TEST(GetRotateAngleTest, Case06)
{
    //case 6 (32 degree)
    double angle = 32.0 * M_PI / 180.0; // convert to radians
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on negative sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    auto rect = cv::boundingRect(listPoints);
    cv::Mat drawImg = cv::Mat::zeros(sizePoint, sizePoint, CV_8UC3);
    cv::line(drawImg, listPoints[0], listPoints.back(), cv::Scalar(0, 0, 255));
    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, 32.0);
}

TEST(GetRotateAngleTest, Case07)
{
    //case 7 (0.5 degree)
    double angle = 0.5 * M_PI / 180.0; // convert to radians
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on negative sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    auto rect = cv::boundingRect(listPoints);
    cv::Mat drawImg = cv::Mat::zeros(sizePoint, sizePoint, CV_8UC3);
    cv::line(drawImg, listPoints[0], listPoints.back(), cv::Scalar(0, 0, 255));
    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, 0.5);
}
TEST(GetRotateAngleTest, Case08)
{
    //case 8 (1 degree)
    double angle = 1 * M_PI / 180.0; // convert to radians
    int sizePoint = 1000;
    std::vector<cv::Point> listPoints;
    for (int i = 0; i < sizePoint; ++i) {
        double x = i * cos(angle); // x-coordinate depends on cosine of angle
        double y = i * sin(angle); // y-coordinate depends on negative sine of angle
        listPoints.push_back(cv::Point(x, y));
    }
    auto rect = cv::boundingRect(listPoints);
    cv::Mat drawImg = cv::Mat::zeros(sizePoint, sizePoint, CV_8UC3);
    cv::line(drawImg, listPoints[0], listPoints.back(), cv::Scalar(0, 0, 255));
    EXPECT_EQ(listPoints.size(), sizePoint);
    double radian;
    cv::Point2d mainAxis, center;
    radian = xvt::GetRotatedAngle(listPoints, mainAxis, center);
    double angleDegree = xvt::Rad2Deg(radian);
    int rounded = std::round(angleDegree * 100.0);
    double result = rounded / 100.0;
    EXPECT_DOUBLE_EQ(result, 1);
}