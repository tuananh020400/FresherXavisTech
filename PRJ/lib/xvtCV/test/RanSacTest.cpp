/**
 * @example{lineno} test/RanSacTest.cpp
 * This example demonstrates how to use xvt::Ransac classes.
 */

#include "pch.h"
#include <opencv2/opencv.hpp>
#include <random>
#include "xvtCV/CircleDetector.h"
#include "xvtCV/LineDetector.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/Drawing.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct CircleData {
    std::string filepath;
    std::vector<cv::Point> listPoints;
    double innerRadius;
    double outerRadius; // Corrected typo from outerRatius to outerRadius
    double dataThreshold;
    int modelThreshold;
    int timeOut;
    // Constructor with default parameter for listPoints
    CircleData(const std::string& path,
        std::vector<cv::Point> lPoints = {}, // Default empty vector
        double r1 = 0.0,
        double r2 = 0.0,
        double d1 = 0.0,
        int m1 = 0,
        int t1=0)
        : filepath(path),
        listPoints(lPoints),
        innerRadius(r1),
        outerRadius(r2),
        dataThreshold(d1),
        modelThreshold(m1),
        timeOut(t1){}
};

template<typename DataGenerator>
auto GeneratePoints(int numPoints, float dataRange, int noiseRange, float ratioNoise, DataGenerator dataGenerator)->std::vector<cv::Point>
{
    std::vector<cv::Point> circlePoints;
    circlePoints.reserve(numPoints);

    int numNoisePoints = static_cast<int>(numPoints * ratioNoise);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> data(-dataRange, dataRange);
    std::uniform_int_distribution<int> noise(-noiseRange, noiseRange);
    std::uniform_int_distribution<int> noiseIdx(0, numPoints - 1);

    for (int i = 0; i < numPoints; ++i) {
        auto p = dataGenerator(i);

        bool isNoise = ratioNoise > 0 && noiseIdx(gen) < numNoisePoints;
        if (isNoise)
        {
            auto e = decltype(p)(noise(gen), noise(gen));
            circlePoints.emplace_back(p + e);
        }
        else
        {
            if(dataRange>0)
            {
                auto d = decltype(p)(data(gen), data(gen));
                circlePoints.emplace_back(p + d);
            }
            else
            {
                circlePoints.emplace_back(p);
            }
        }
    }

    return circlePoints;
}

auto GenerateCirclePoints(const cv::Point2f& center, double radius, int numPoints, float dataRange, int noiseRange, float ratioNoise)->std::vector<cv::Point>
{

    const double angleIncrement = 2 * CV_PI / numPoints;
    std::vector<cv::Point> points = GeneratePoints(numPoints, dataRange, noiseRange, ratioNoise
                                                         , [&](int i)
                                                         {
                                                             double theta = i * angleIncrement;
                                                             int x = static_cast<int>(center.x + radius * cos(theta) + 0.5);
                                                             int y = static_cast<int>(center.y + radius * sin(theta) + 0.5);
                                                             return cv::Point(x, y);
                                                         }
    );
   
    return points;
}

auto GenerateLinePoints(cv::Point start, cv::Point end, int numPoints, float dataRange, int noiseRange, float ratioNoise)->std::vector<cv::Point>
{
    std::vector<cv::Point> points;
    if (end == start) return points;

    auto v = cv::Point2d(end - start);
    double l = cv::norm(v);
    v /= numPoints;
    //numPoints = std::min(numPoints, int(l));
    points = GeneratePoints(numPoints, dataRange, noiseRange, ratioNoise
                                                         , [&](int i)
                                                         {
                                                             return cv::Point2d(start)+i * v;
                                                         }
    );

    return points;
}

auto GetCircleTxtFiles(bool isDebug) -> std::vector<CircleData> {
    std::vector<CircleData> listTxt = {
        CircleData("./test/MicOuter.txt", {}, 140, 155, 5, 20, isDebug ? 1000 : 300),
        CircleData("./test/MicInner.txt", {}, 80, 100, 5, 20, isDebug ? 1000 : 300),
        CircleData("./test/MicNoise1.txt", {}, 140, 155, 5, 100, isDebug ? 5000 : 1000),
        CircleData("./test/MicNoise2.txt", {}, 120, 145, 5, 50, isDebug ? 1500 : 500)
    };
    for (CircleData& circleData : listTxt) {
        std::ifstream file(circleData.filepath); // replace "points.txt" with the name of your file
        std::string line;
        std::vector<cv::Point> points;
        if (!file) {
            std::cerr << "Unable to open file";
        }
        else {
            while (std::getline(file, line)) {
                cv::Point point;
                std::size_t posX = line.find("x=");
                std::size_t posY = line.find("y=");

                if (posX != std::string::npos && posY != std::string::npos) {
                    point.x = std::stoi(line.substr(posX + 2, line.find(' ', posX) - posX - 2));
                    point.y = std::stoi(line.substr(posY + 2, line.find(' ', posY) - posY - 2));
                    points.push_back(point);
                }
            }
        }
        circleData.listPoints=points;
        file.close();
    }

    return listTxt;
}

template<typename ModelResult>
auto TestRansac(xvt::Ransac<ModelResult> const& detector, xvt::VecPoint const& points)->xvt::RansacResult<ModelResult>
{
    auto rect = cv::boundingRect(points);
    auto rectCenter = (rect.tl() + rect.br()) / 2;
    int imgW = rectCenter.x*2;
    int imgH = rectCenter.y*2;

    std::vector<cv::Point> rChoosenPointList;
    auto c = detector.FitRANSAC(points, rChoosenPointList);
    cv::Mat colorImg = cv::Mat::zeros(imgH, imgW, CV_8UC3);

    // Draw the points on the image
    xvt::DrawPoints(colorImg, points, cv::Scalar(255, 255, 255));
    cv::Mat oriImg = colorImg.clone();

    c.DrawResult(colorImg);

    auto c2 = detector.FitLSQE(points);
    c2.DrawResult(colorImg, cv::Point(), xvt::CVPen(COLOR_CV_VIOLET));

    // Display the image
    //cv::imshow("Fitting", colorImg);
    //cv::imshow("Original", oriImg);
    //cv::waitKey(200);

    EXPECT_NO_THROW();
    EXPECT_NO_FATAL_FAILURE();
    EXPECT_TRUE(c.IsOK());
    EXPECT_TRUE(c.mPoints.size() <= points.size());
    return c;
}

TEST(Ransac, FitCircleTxt) {
#ifdef _DEBUG
    std::vector<CircleData> listPoints = GetCircleTxtFiles(true);
#else
    std::vector<CircleData> listPoints = GetCircleTxtFiles(false);
#endif
    for (CircleData circleData : listPoints) {
        xvt::CircleDetector circleFit;
        circleFit.mRadiusRange.Set(circleData.innerRadius, circleData.outerRadius);
        circleFit.mDataThreshold= circleData.dataThreshold;
        circleFit.mModelThreshold=circleData.modelThreshold;
        circleFit.mTimeOut = circleData.timeOut;

        circleFit.mIsRefineFinal =1;
        circleFit.mIsRefineModel =1;

        //circleFit.SetModelSize(5);

        for (int i = 0; i < 5; i++) {
            auto c = TestRansac(circleFit, circleData.listPoints);
            std::cout << c.mCenter << c.mRadius<<", "<<c.GetRMSE() << ", " << c.mPoints.size() << ", " << c.mPInlier << std::endl;
        }
    }
}

TEST(Ransac, FitCircle) {
    int     numPoints    = 500;
    float   ratioNoise   = 0.7f;
    double  circleRadius = 200.0;
    int     noiseRange   = 80.0;
    float   dataRange    = 2;

    cv::Point2f circleCenter(circleRadius+ noiseRange+10, circleRadius + noiseRange + 10);

    // Generate the circle points
    auto points = GenerateCirclePoints(circleCenter, circleRadius, numPoints, dataRange, noiseRange, ratioNoise);

    auto points2 = GenerateCirclePoints(circleCenter+cv::Point2f(3,0), circleRadius, numPoints, dataRange, noiseRange, ratioNoise);
    //points.insert(points.end(), points2.begin(), points2.end());

    xvt::CircleDetector circleFit;
    circleFit.mMaxCoverRate = 0.01;
    circleFit.mRadiusRange.Set(circleRadius*0.9, circleRadius*1.1);
    circleFit.mDataThreshold = dataRange > 0 ? dataRange * 2.5 : 1.0;
#ifdef _DEBUG
    circleFit.mTimeOut = 1000;
#else
    circleFit.mTimeOut = 100;
#endif // _DEBUG


    circleFit.SetModelSize(7);

    for (int i = 0; i < 1; i++) {

        auto c = TestRansac(circleFit, points);
        std::cout << c.mCenter << c.mRadius << ", " << c.GetRMSE() << std::endl;
    }

}

TEST(Ransac, FitLine)
{
    int     numPoints = 500;
    float   ratioNoise = 0.5f;
    int     noiseRange = 80.0;
    float   dataRange = 2;

    // Generate the circle points
    auto points = GenerateLinePoints(cv::Point(10,400), cv::Point(400, 100), numPoints, dataRange, noiseRange, ratioNoise);

    xvt::LineDetector circleFit;
    circleFit.mDataThreshold = dataRange > 0 ? dataRange * 2.5 : 1.0;
#ifdef _DEBUG
    circleFit.mTimeOut = 400;
#else
    circleFit.mTimeOut = 10;
#endif // _DEBUG
    circleFit.SetModelSize(7);

    for (int i = 0; i < 1; i++)
    {
        TestRansac(circleFit, points);
    }
}