/**
 * @example{lineno} test/PolyFitTest.cpp
 * This example demonstrates how to use xvt::Polyfit class.
 */

#include "pch.h"
#include "xvtCV/Polyfit.h"
#include "xvtCV/Utils.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/xvtTypes.h"

using namespace xvt;

TEST(PolyFitTest, Case01)
{
    double a = 1.0;
    double b = 2.0;
    double c = 2.0;
    int n = 100;
    double step = 0.123;
    VecDouble x = VecDouble(n, 0.0);
    VecDouble y = VecDouble(n, 0.0);
    for (int i = 0; i < n; i++)
    {
        x[i] = i*step;
        y[i] = a * x[i] * x[i] + b * x[i] + c;
    }
    double e = 1.e-4;
    double order = 2;
    Polyfit fitor(x, y, order);
    auto& coffe = fitor.GetCoeff();
    EXPECT_EQ(fitor.GetOrder(), order);
    EXPECT_LE(fitor.GetRMS(), e);
    EXPECT_NEAR(coffe[0], c, e);
    EXPECT_NEAR(coffe[1], b, e);
    EXPECT_NEAR(coffe[2], a, e);

    VecDouble y2;
    fitor.f(x, y2);
    Drawing drawTool;
    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
    drawTool.color = COLOR_CV_BLUE;
    drawTool.yTopOrigin = 0;
    drawTool.Plot(img, x, y);
    drawTool.color = COLOR_CV_RED;
    drawTool.Plot(img, x, y2, false);
}

TEST(PolyFitTest, Case02)
{
    double a = 1.0;
    double b = 2.0;
    double c = 2.0;
    int n = 100;
    double step = 0.123;

    VecDouble x = VecDouble(n, 0.0);
    VecDouble y = VecDouble(n, 0.0);
    VecDouble noise = VecDouble(n, 0.0);
    VecPoint points = VecPoint(n);
    double d = 2.0;
    cv::randn(noise, 0.0, d);
    int n2 = n / 2;
    for (int i = -n2; i < n2; i++)
    {
        auto idx = i + n2;
        x[idx] = i * step;
        y[idx] = a * x[idx] * x[idx] + b * x[idx] + c + noise[idx];
        points[idx] = cv::Point(int(x[idx]), int(y[idx]));
    }

    double order = 2;
    Polyfit fitor(x, y, order);
    auto& coffe = fitor.GetCoeff();
    EXPECT_EQ(fitor.GetOrder(), order);
    EXPECT_LE(fitor.GetRMS(), d+0.1);
    EXPECT_NEAR(coffe[0], c, 0.33);
    EXPECT_NEAR(coffe[1], b, 0.15);
    EXPECT_NEAR(coffe[2], a, 0.01);

    VecDouble y2;
    fitor.f(x, y2);
    Drawing drawTool;
    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
    drawTool.color = COLOR_CV_BLUE;
    drawTool.yTopOrigin = 0;
    drawTool.Plot(img, x, y);
    drawTool.color = COLOR_CV_RED;
    drawTool.Plot(img, x, y2, false);
}

TEST(PolyFitTest, CaseRansac)
{
    double a = 1.0;
    double b = 2.0;
    double c = 2.0;
    int n = 20;
    double step = 1;

    VecDouble x = VecDouble(n, 0.0);
    VecDouble y = VecDouble(n, 0.0);
    VecDouble noise = VecDouble(n, 0.0);
    VecPoint points = VecPoint(n);
    double d = 2;
    cv::randn(noise, 0.0, d);
    int n2 = n / 2;
    for (int i = 0; i < n; i++)
    {
        auto idx = i;
        x[idx] = int(i);
        y[idx] = int(a * x[idx] * x[idx] + b * x[idx] + c + noise[idx]);
        points[idx] = cv::Point(int(x[idx]), int(y[idx]));
    }

    double order = 2;
    PolynomialDetector fitor(order);
    fitor.mDataThreshold = 3;
#ifdef _DEBUG
    fitor.mTimeOut = 2;
#else
    fitor.mTimeOut = 1;
#endif // _DEBUG
    auto res = fitor.FitRANSAC(points);
    auto& coffe = res.GetCoeff();
    EXPECT_EQ(res.GetOrder(), order);
    EXPECT_LE(res.GetRMSE(points), d*1.15);
    EXPECT_TRUE(res.IsOK());

    VecDouble y2 = res.f(x);
    Drawing drawTool;

    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    drawTool.color = COLOR_CV_BLUE;
    drawTool.yTopOrigin = 0;
    drawTool.Plot(img, x, y);
    drawTool.color = COLOR_CV_RED;
    drawTool.Plot(img, x, y2, false);

    cv::Mat img2 = cv::Mat::zeros(500, 500, CV_8UC3);
    std::sort(res.mPoints.begin(), res.mPoints.end(), [](auto const& p1, auto const& p2) {return p1.x < p2.x; });
    res.DrawResult(img2);
}
