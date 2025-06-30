/**
 * @example{lineno} test/LineDetectorTest.cpp
 * 
 * This example demonstrates how to use xvt::Line, xvt::LineDetector class.
 */

#include "pch.h"
#include "xvtCV/LineDetector.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/Utils.h"
#include "xvtCV/ColorDefine.h"
#include <fstream>
#include <random>

using namespace cv;

void CheckLine(xvt::Line l1, xvt::Line l2, double eVec, double ed)
{
    auto vec1 = l1.GetNormalizeVector();
    auto vec2 = l2.GetNormalizeVector();
    double d = abs(vec1.ddot(vec2));
    EXPECT_NEAR(d, 1, eVec);
    EXPECT_NEAR(abs(l1.GetDistance()), abs(l2.GetDistance()), ed);
}

//Contruct the line from two points on the line
TEST(LineTest, LineConstructor)
{
    xvt::Line l1 = xvt::Line(Point(1, 1), Point(1, 2));
    EXPECT_NEAR(l1.GetAngle(), CV_PI/2, DBL_EPSILON);
    EXPECT_EQ(l1.GetDistance(), -1);

    xvt::Line l2(Point(1, 2), Point(2, 2));
    EXPECT_NEAR(l2.GetAngle(), 0.0, DBL_EPSILON);
    EXPECT_EQ(l2.GetDistance(), 2);

    xvt::Line l3(Point(0, 0), Point(2, 2));
    EXPECT_NEAR(l3.GetAngle(), CV_PI / 4, DBL_EPSILON);
    EXPECT_NEAR(l3.GetDistance(), 0, DBL_EPSILON);

    xvt::Line l32(Point(0, 0), Point(2, -2));
    EXPECT_NEAR(l32.GetAngle(), -CV_PI / 4, DBL_EPSILON);
    EXPECT_NEAR(l32.GetDistance(), 0, DBL_EPSILON);

    xvt::Line l4(Point(1, 0), Point(0, 1));
    EXPECT_NEAR(l4.GetAngle(), -CV_PI / 4, DBL_EPSILON);
    EXPECT_DOUBLE_EQ(l4.GetDistance(), -sqrt(2) / 2.0);

    xvt::Line l5(Point(1, 2), Point(1, 3));
    EXPECT_TRUE(l5 == l1);

    xvt::Line l06 = xvt::Line(Point(1, 0), Point(0, 1));
    xvt::Line l07 = xvt::Line(Point2d(1, 0), Point2d(cos(-CV_PI/4), sin(-CV_PI / 4)), true);
    xvt::Line l08 = xvt::Line(CV_PI / 4, sqrt(2)/2);
    xvt::Line l09 = xvt::Line(Point(-1, 2), Point(2, -1));
    xvt::Line l10 = xvt::Line(cos(CV_PI / 4), sin(CV_PI / 4), -sqrt(2) / 2);
    xvt::Line l11 = xvt::Line(1, 1, -1);
    EXPECT_TRUE(l07 == l06);
    EXPECT_TRUE(l08 == l06);
    EXPECT_TRUE(l09 == l06);
    EXPECT_TRUE(l10 == l06);
    EXPECT_TRUE(l11 == l06);
}

TEST(LineTest, GetLineVector)
{
    xvt::Line l1(Point(1, 0), Point(0, 2));

    auto normalize_vec = Point(-1, 2);
    auto v = l1.GetNormalizeVector();
    auto a = v.ddot(normalize_vec) / norm(normalize_vec) / norm(v);
    EXPECT_DOUBLE_EQ(abs(a), 1.0);

    auto normal_vec = Point(2, 1);
    auto n = l1.GetNormalVector();
    auto a2 = n.ddot(normal_vec) / norm(normal_vec) / norm(n);
    EXPECT_DOUBLE_EQ(abs(a2), 1.0);

    EXPECT_DOUBLE_EQ(v.ddot(n), 0.0);
}

TEST(LineTest, GetPoint)
{
    xvt::Line l1(Point(1, 0), Point(0, 1));
    auto n = l1.GetNormalVector();
    auto v = l1.GetNormalizeVector();

    //EXPECT_EQ(l1.GetPoint0(), Point2d(1.0 / 2, 1.0 / 2));

    auto px = l1.GetPointX(Point(2, 2));
    EXPECT_DOUBLE_EQ(px.x, 2.0);
    EXPECT_DOUBLE_EQ(px.y, -1.0);

    auto py = l1.GetPointY(Point(2, 2));
    EXPECT_DOUBLE_EQ(py.x, -1.0);
    EXPECT_DOUBLE_EQ(py.y, 2.0);

    auto p = l1.GetPoint(Point(2, 2));
    EXPECT_DOUBLE_EQ(p.x, 2.0);
    EXPECT_DOUBLE_EQ(p.y, -1.0);

    auto p0 = l1.GetPoint0();

}

TEST(LineTest, GetDistance)
{
    xvt::Line l1(Point(1, 0), Point(0, 1));
    auto v1 = l1.GetNormalizeVector();
    auto n1 = l1.GetNormalVector();
    EXPECT_NEAR(l1.GetDistance(Point(1, 1)), sqrt(2) / 2, DBL_EPSILON);
    EXPECT_NEAR(l1.GetDistance(Point(1, 0)), 0, DBL_EPSILON);
    EXPECT_NEAR(l1.GetDistance(Point(0, 1)), 0, DBL_EPSILON);
    EXPECT_NEAR(l1.GetDistance(Point(0, 0)), -sqrt(2) / 2, DBL_EPSILON);

    xvt::Line l2(Point(1, 0), Point(0, -1));
    auto v2 = l2.GetNormalizeVector();
    auto n2 = l2.GetNormalVector();
    EXPECT_NEAR(l2.GetDistance(Point(1, -1)), -sqrt(2) / 2, DBL_EPSILON);
    EXPECT_NEAR(l2.GetDistance(Point(1, 0)), 0, DBL_EPSILON);
    EXPECT_NEAR(l2.GetDistance(Point(0, -1)), 0, DBL_EPSILON);
    EXPECT_NEAR(l2.GetDistance(Point(0, 0)), sqrt(2) / 2, DBL_EPSILON);
}

TEST(LineTest, GetAngle)
{
    xvt::Line l1(Point(1, 0), Point(0, 1));

    EXPECT_NEAR(l1.GetAngle(Point2d( 1,  0)), -CV_PI / 4, DBL_EPSILON);
    EXPECT_NEAR(l1.GetAngle(Point2d(-1,  0)), -CV_PI / 4, DBL_EPSILON);
    EXPECT_NEAR(l1.GetAngle(Point2d( 0,  1)),  CV_PI / 4, DBL_EPSILON);
    EXPECT_NEAR(l1.GetAngle(Point2d( 0, -1)),  CV_PI / 4, DBL_EPSILON);

}

//Contruct the line from two points on the line
TEST(LineTest, GetIntersectPoint)
{
    xvt::Line l1(Point(1, 0), Point(0, 1));
    xvt::Line ox(Point(0, 0), Point(1, 0), true);
    auto px = l1.GetIntersectPoint(ox);
    EXPECT_NEAR(px.x, 1.0, DBL_EPSILON);
    EXPECT_NEAR(px.y, 0.0, DBL_EPSILON);

    xvt::Line oy(Point(0, 0), Point(0, 1), true);
    auto py = l1.GetIntersectPoint(oy);
    EXPECT_NEAR(py.x, 0.0, DBL_EPSILON);
    EXPECT_NEAR(py.y, 1.0, DBL_EPSILON);

    //parallel line
    xvt::Line l2(Point(1, 1), Point(0, 2));
    auto p = l1.GetIntersectPoint(l2);
    EXPECT_EQ(p.x, std::numeric_limits<double>::infinity());
    EXPECT_EQ(p.y, std::numeric_limits<double>::infinity());

    //Same line
    xvt::Line l3(Point(2, 1), Point(-1, 2));
    p = l1.GetIntersectPoint(l3);
}

TEST(LineTest, LineDraw)
{
    cv::Mat img=cv::Mat::zeros(300, 300, CV_8UC3);
    auto center = cv::Point2d(img.cols / 2.0, img.rows / 2.0);
    xvt::CVPen pen;
    pen.mColor = COLOR_CV_BLUE;
    xvt::Line l;
    for (int i = 0; i < 360; i+=10)
    {
        l=xvt::Line(xvt::Deg2Rad((double)i), 100);
        pen.mColor = l.GetDistance(-center)*l.GetDistance(center) > 0 ? COLOR_CV_BLUE : COLOR_CV_GREEN;
        l.Draw(img, pen, center);
    }

    pen.mColor = COLOR_CV_RED;
    xvt::Line l1(Point(50, 0), Point(0, 50));
    l1.Draw(img, pen, center, true);
 
    pen.mColor = COLOR_CV_CYAN;
    xvt::DrawPlusSign(img, center, pen, 4);
}

TEST(LineTest, FitLine)
{
    cv::Mat img = cv::Mat::zeros(300, 300, CV_8UC3);

    std::vector<cv::Point_<int>> testPointsInt{
        {-1, -1},
        {20, 20},
        {30, 30},
        {40, 40},
        {50, 50}
    };

    for (const auto& point : testPointsInt)
    {
        cv::circle(img, point, 2, cv::Scalar(0, 255, 255), -1);
    }

    xvt::Line resultLine1 = xvt::LineDetector::FitLineLSQE(testPointsInt);
    xvt::CVPen pen1(COLOR_CV_RED);
    resultLine1.Draw(img, pen1);

    auto expectVec = cv::Point2d(cos(CV_PI / 4), cos(CV_PI / 4));
    auto vec = resultLine1.GetNormalizeVector();
    EXPECT_NEAR(vec.x, expectVec.x, DBL_EPSILON);
    EXPECT_NEAR(vec.y, expectVec.y, DBL_EPSILON);

    // Seed the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the distribution for the range [0, CV_2PI]
    std::uniform_real_distribution<double> angleDis(0.0, CV_2PI);
    std::uniform_real_distribution<double> xDis(0.0, img.cols);

    std::vector<cv::Point_<double>> testPointsFloat;
    std::vector<cv::Point_<int>> testPointInt;
    xvt::Line expectLine(cv::Point(0, img.rows), cv::Point(img.cols, 0));
    for (int i = 0; i < 150; i++)
    {
        auto x = xDis(gen);
        auto angle = angleDis(gen);
        auto p = cv::Point2d(x, img.rows / 2.0);
        p = expectLine.GetPoint(p) + cv::Point2d(cos(angle), sin(angle));
        testPointsFloat.push_back(p);
        testPointInt.push_back(p);
    }

    xvt::Line resultLine2 = xvt::LineDetector::FitLineLSQE(testPointsFloat);
    auto rms = resultLine2.GetRMSE(testPointsFloat);
    EXPECT_LE(rms, 1.0);
    CheckLine(resultLine2, expectLine, 1e-3, 1.0001);

    xvt::Line resultLine3 = xvt::LineDetector::FitLineOpenCV(testPointInt);
    auto rms3 = resultLine3.GetRMSE(testPointInt);
    EXPECT_LE(rms3, 1.0);
    CheckLine(resultLine3, expectLine, 1e-3, 1.0001);


    xvt::CVPen pen2(COLOR_CV_GREEN);
    resultLine2.Draw(img, pen2);
    xvt::DrawPoints(img, testPointsFloat, cv::Scalar(255, 0, 255));
}

TEST(LineTest, FitLineDiffAngle)
{
    
    // Seed the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    cv::Mat img = cv::Mat::zeros(300, 300, CV_8UC3);

    // Define the distribution for the range [0, CV_2PI]
    std::uniform_real_distribution<double> angleDis(0.0, CV_2PI);
    std::uniform_real_distribution<double> xDis(0.0, img.cols);

    for (float a = 0.0; a < CV_PI; a += CV_PI / 16)
    {
        std::vector<cv::Point_<double>> testPointsFloat;
        std::vector<cv::Point_<int>> testPointInt;
        xvt::Line expectLine(a, 150);
        for (int i = 0; i < 150; i++)
        {
            auto x = xDis(gen);
            auto angle = angleDis(gen);
            auto p = cv::Point2d(x, x);
            p = expectLine.GetPoint(p) + cv::Point2d(cos(angle), sin(angle));
            testPointsFloat.push_back(p);
            testPointInt.push_back(p);
        }

        xvt::Line resultLine2 = xvt::LineDetector::FitLineLSQE(testPointsFloat);
        auto rms = resultLine2.GetRMSE(testPointsFloat);
        EXPECT_LE(rms, 1.0);
        CheckLine(resultLine2, expectLine, 1e-3, 1.0001);

        xvt::Line resultLine3 = xvt::LineDetector::FitLineOpenCV(testPointInt);
        auto rms3 = resultLine3.GetRMSE(testPointInt);
        EXPECT_LE(rms3, 1.0);
        CheckLine(resultLine3, expectLine, 1e-3, 1.0001);

        xvt::CVPen pen2(COLOR_CV_GREEN);
        resultLine2.Draw(img, pen2);
        xvt::DrawPoints(img, testPointsFloat, cv::Scalar(255, 0, 255));
    }
}

TEST(LineDetectorTest, FindVerticalLine) {
    cv::Mat img(300, 300, CV_8UC3, cv::Scalar::all(0));
    cv::line(img, cv::Point(150, 0), cv::Point(150, 299), cv::Scalar::all(255), 1);

    xvt::LineDetector lineDetector;
    xvt::LineResult result = lineDetector.FindVerticalLine(img);
    auto vec = result.GetNormalizeVector();
    EXPECT_NEAR(vec.x, 0, 1e-3);
    EXPECT_NEAR(vec.y, 1, 1e-3);
    EXPECT_NEAR(abs(result.GetDistance()), 150, 1.0001);

    cv::Mat imgWithLine(300, 300, CV_8UC3, cv::Scalar::all(0));
    xvt::CVPen pen(COLOR_CV_BLUE);
    result.DrawResult(img,cv::Point(), pen);

}

TEST(LineDetectorTest, FindHorLine) {
    cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::line(img, cv::Point(0,150), cv::Point(299, 150), cv::Scalar(0, 0, 0), 1);

    xvt::LineDetector lineDetector;
    xvt::LineResult result = lineDetector.FindHorizontalLine(img);
    auto vec = result.GetNormalizeVector();
    EXPECT_NEAR(vec.x, 1, 1e-3);
    EXPECT_NEAR(vec.y, 0, 1e-3);
    EXPECT_NEAR(abs(result.GetDistance()), 150, 1.0001);

    xvt::CVPen pen(COLOR_CV_RED);
    result.DrawResult(img, cv::Point(), pen);

}

TEST(LineDetectorTest, FindLine)
{
    //Hor
    cv::Mat img(200, 300, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::line(img, cv::Point(150, 0), cv::Point(150, 299), cv::Scalar(0, 0, 0), 1);

    xvt::LineDetector lineDetector;
    xvt::LineResult resultVer = lineDetector.FindLine(img, false);
    auto vec = resultVer.GetNormalizeVector();
    EXPECT_NEAR(vec.x, 0, 1e-3);
    EXPECT_NEAR(vec.y, 1, 1e-3);
    EXPECT_NEAR(abs(resultVer.GetDistance()), 150, 1.0001);

    //Ver
    cv::Mat img2(200, 300, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::line(img2, cv::Point(0, 150), cv::Point(299, 150), cv::Scalar(0, 0, 0), 1);

    xvt::LineResult resultHor = lineDetector.FindLine(img2, true);
    vec = resultHor.GetNormalizeVector();
    EXPECT_NEAR(vec.x, 1, 1e-3);
    EXPECT_NEAR(vec.y, 0, 1e-3);
    EXPECT_NEAR(abs(resultHor.GetDistance()), 150, 1.0001);

    lineDetector.mIsBlack2White = false;
    xvt::LineResult resultVerW2B = lineDetector.FindLine(img, false);
    xvt::LineResult resultHorW2B = lineDetector.FindLine(img2, true);

    xvt::CVPen penG(COLOR_CV_GREEN);
    xvt::CVPen penR(COLOR_CV_RED);

    resultVer.Draw(img, penG);
    resultVerW2B.Draw(img, penR);

    resultHor.Draw(img2, penG);
    resultHorW2B.Draw(img2, penR);
}

TEST(LineDetectorTest, FindLineGrayImage) {
    std::string folderPath = ".\\line_image_test\\";
    std::vector<std::string> horNameList = {
         "Hor1B2W.jpg"
        ,"Hor2W2B.jpg"
    };
    std::vector<xvt::Line> expHorLines = {
         xvt::Line(-0.0092877663809109324, -0.99995686776763204, 25.766190740898143)
        ,xvt::Line(cv::Point2d{32.50,24.530303955078125}, cv::Point2d{0.99585533142089844,0.090951584279537201}, true)
    };

    std::vector<std::string> verNameList = {
         "Ver1W2B.jpg"
        ,"Ver2W2B.jpg"
    };
    std::vector<xvt::Line> expVerLines = {
         xvt::Line(cv::Point2d{17.716049194335938, 40.0}, cv::Point2d{0.0010844030184671283,-0.99999940395355225}, true)
        ,xvt::Line(cv::Point2d{20.415384292602539, 32.0}, cv::Point2d{0.16790005564689636,-0.98580402135848999}, true)
    };

    for (int i = 0; i < 2; i++) {
        xvt::LineDetector lineDetector;

        std::string horName = folderPath + horNameList[i];
        cv::Mat horImage = cv::imread(horName, 0);
        EXPECT_TRUE(!horImage.empty());

        lineDetector.mIsBlack2White = i < 2;
        xvt::LineResult horLine = lineDetector.FindLine(horImage, true);
        xvt::CVPen penG(COLOR_CV_GREEN);
        horLine.DrawResult(horImage, cv::Point(), penG);
        CheckLine(horLine, expHorLines[i], 1e-3, 1.0001);

        std::string verName = folderPath + verNameList[i];
        cv::Mat verImage = cv::imread(verName, 0);
        EXPECT_TRUE(!verImage.empty());

        lineDetector.mIsBlack2White = false;
        xvt::LineResult verLine = lineDetector.FindLine(verImage, false);
        xvt::CVPen penR(COLOR_CV_RED);
        verLine.DrawResult(verImage, cv::Point(), penR);
        CheckLine(verLine, expVerLines[i], 1e-3, 1.0001);
    }
}

TEST(LineDetectorTest, Transform) {
    double ang = 90;
    double dis = 0;

    xvt::Line originalLine(xvt::Deg2Rad(ang), dis);
    cv::Point2d p = originalLine.GetPoint(cv::Point2d{ 50, 50 });

    cv::Mat image(300, 300, CV_8UC3, cv::Scalar(0, 0, 0));
    auto center = cv::Point2d(image.cols / 2.0, image.rows / 2.0);
    xvt::DrawPlusSign(image, center, xvt::CVPen(COLOR_CV_YELLOW), 10);
    for(int i= 10; i<180;i+=10)
    {
        cv::Mat m = cv::getRotationMatrix2D(cv::Point2f(0, 0), i, 1);
        xvt::Line transformedLine = xvt::Transform(originalLine, m);

        transformedLine.Draw(image, xvt::CVPen(COLOR_CV_RED), center);

        auto vec = cv::Point2d(cos(xvt::Deg2Rad(-i)), sin(xvt::Deg2Rad(-i)));
        
        auto angle = xvt::Rad2Deg(transformedLine.GetAngle());
        EXPECT_DOUBLE_EQ(abs(vec.ddot(transformedLine.GetNormalizeVector())), 1);
        EXPECT_DOUBLE_EQ(abs(transformedLine.GetDistance()), dis);

        auto p2 = xvt::Transform(p, m);
        xvt::DrawPlusSign(image, p2+center, xvt::CVPen(COLOR_CV_CYAN), 2);
        xvt::DrawPlusSign(image, vec*25+center, xvt::CVPen(COLOR_CV_VIOLET), 2);
        
    }

    originalLine.Draw(image, xvt::CVPen(COLOR_CV_GREEN), center);
}

TEST(LineDetectorTest, Transform_Translation) {

    double dis = 10;
    auto shift = cv::Point(20, 20);
    auto p0 = cv::Point(0, 0);
    auto p1 = cv::Point(0, dis);
    auto p2 = cv::Point(dis, dis);
    auto p3 = cv::Point(-dis, dis);

    xvt::Line originalLine1(p1, p2);
    xvt::Line originalLine2(p0, p3);

    xvt::Line expLine1(p1 + shift, p2 + shift);
    xvt::Line expLine2(p0 + shift, p3 + shift);

    cv::Mat transformationMatrix = (cv::Mat_<double>(2, 3) << 1, 0, shift.x, 0, 1, shift.y);
    xvt::Line transformedLine1 = xvt::Transform(originalLine1, transformationMatrix);
    xvt::Line transformedLine2 = xvt::Transform(originalLine2, transformationMatrix);

    EXPECT_TRUE(expLine1 == transformedLine1);
    EXPECT_TRUE(expLine1 == (originalLine1+shift));
    EXPECT_TRUE(expLine2 == transformedLine2);
    EXPECT_TRUE(expLine2 == (originalLine2 + shift));

    cv::Mat image(300, 300, CV_8UC3, cv::Scalar(0, 0, 0));

    xvt::CVPen penG(COLOR_CV_GREEN);
    xvt::CVPen penR(COLOR_CV_RED);
    auto center = cv::Point{ 150,150 };
    xvt::DrawPlusSign(image, center, xvt::CVPen(COLOR_CV_CYAN), 2);

    originalLine1.Draw(image, penG, center, true);
    transformedLine1.Draw(image, penR, center, true);
    originalLine2.Draw(image, penG, center, true);
    transformedLine2.Draw(image, penR, center, true);
}

TEST(LineDetectorTest, Transform_Translation_Scaling) {
    double dis = 10;
    xvt::Line originalLine(cv::Point(0, dis), cv::Point(10, dis));
    xvt::Line originalLine2(cv::Point(0, 0), cv::Point(-dis, dis));

    cv::Mat transformationMatrix = (cv::Mat_<double>(2, 3) << 2, 0, 20, 0, 2, 20);
    xvt::Line transformedLine = xvt::Transform(originalLine, transformationMatrix);
    xvt::Line transformedLine2 = originalLine2;
    transformedLine2.Transform(transformationMatrix);

    EXPECT_DOUBLE_EQ(abs(transformedLine.GetDistance()), dis*2+20);
    EXPECT_DOUBLE_EQ(abs(transformedLine2.GetDistance()), cv::norm(cv::Point(20, 20)));

    cv::Mat image(300, 300, CV_8UC3, cv::Scalar(0, 0, 0));

    xvt::CVPen penG(COLOR_CV_GREEN);
    xvt::CVPen penR(COLOR_CV_RED);
    auto center = cv::Point{ 150,150 };
    xvt::DrawPlusSign(image, center, xvt::CVPen(COLOR_CV_CYAN), 7);

    originalLine.Draw(image   , xvt::CVPen(cv::Scalar{ 0,125,0}), center, true);
    transformedLine.Draw(image, xvt::CVPen(cv::Scalar{ 0,255,0 }), center, true);

    originalLine2.Draw(image, xvt::CVPen(cv::Scalar{ 125,0,0 }), center, true);
    transformedLine2.Draw(image, xvt::CVPen(cv::Scalar{ 255,0,0 }), center, true);
}

TEST(LineDetectorTest, zigzag_img) {
    std::string filename = ".\\line_image_test\\zigzag1.jpg";
    cv::Mat image = cv::imread(filename, 0);
    EXPECT_TRUE(!image.empty());

    xvt::LineDetector lineDetector;
    xvt::LineResult result = lineDetector.FindLine(image, true);
    xvt::CVPen penG(COLOR_CV_GREEN);
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    result.Draw(image, penG);
    EXPECT_NEAR(result.GetAngle(), 0 / CV_PI * 180, 0.1);
}

TEST(LineDetectorTest,curve) {
    std::string filename = ".\\line_image_test\\Cur1.jpg";
    cv::Mat image = cv::imread(filename, 0);
    EXPECT_TRUE(!image.empty());

    xvt::LineDetector lineDetector;
    xvt::LineResult result = lineDetector.FindLine(image, true);
    xvt::CVPen penG(COLOR_CV_GREEN);
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    result.Draw(image, penG);
    EXPECT_NEAR(xvt::Rad2Deg(result.GetAngle()), -19, 0.5);
}

void TestIntersect(xvt::Line l, cv::Rect rect, std::vector<cv::Point2f> vec)
{
    auto pnts = xvt::Intersect(l, rect);
    EXPECT_EQ(pnts.size(), vec.size());

    for (int i = 0, n = std::min(pnts.size(), vec.size()); i < n; i++)
    {
        EXPECT_EQ(pnts[0].x, vec[0].x);
        EXPECT_EQ(pnts[0].y, vec[0].y);
    }

    cv::Point shift = cv::Point(30, 30) - rect.tl();
    rect = rect + shift;
    l.Transform(shift);
    cv::Mat img = cv::Mat::zeros(rect.height + 60, rect.width + 60, CV_8UC3);
    cv::rectangle(img, rect, cv::Scalar(0, 255, 0));
    l.Draw(img, xvt::CVPen(cv::Scalar(0, 0, 255)));
    for (auto& p : pnts)
    {
        xvt::DrawPlusSign(img, p + cv::Point2f(shift), xvt::CVPen(cv::Scalar(0, 255, 255)));
    }
}

TEST(IntersectTest, linenRect)
{
    cv::Rect rect(0, 0, 100, 100);

    xvt::Line l(cv::Point(0, 0), cv::Point(100, 100));
    TestIntersect(l, rect, { cv::Point2f(0, 0), cv::Point2f(100, 100) });

    l = xvt::Line(cv::Point(0, 0), cv::Point(50, 100));
    TestIntersect(l, rect, { cv::Point2f(0, 0), cv::Point2f(50, 100) });

    l = xvt::Line(cv::Point(100, 0), cv::Point(50, 100));
    TestIntersect(l, rect, { cv::Point2f(100, 0), cv::Point2f(50, 100) });

    l = xvt::Line(cv::Point(0, 100), cv::Point(100, 50));
    TestIntersect(l, rect, { cv::Point2f(100, 50), cv::Point2f(0, 100) });

    l = xvt::Line(cv::Point(50, 0), cv::Point(50, 50));
    TestIntersect(l, rect, { cv::Point2f(50, 0), cv::Point2f(50, 100) });

    l = xvt::Line(cv::Point(0, 50), cv::Point(50, 50));
    TestIntersect(l, rect, { cv::Point2f(100, 50), cv::Point2f(0, 50) });

    l = xvt::Line(cv::Point(10, 10), cv::Point(30, 60));
    TestIntersect(l, rect, { cv::Point2f(6, 0), cv::Point2f(46, 100) });

    l = xvt::Line(cv::Point(0, 0), cv::Point(10, -10));
    TestIntersect(l, rect, { cv::Point2f(0, 0)});


    //Test not intersect
    l = xvt::Line(cv::Point(5, -10), cv::Point(-35, 30));
    TestIntersect(l, rect, { });

    l = xvt::Line(cv::Point(105, 105), cv::Point(1, -1), true);
    TestIntersect(l, rect, { });

    l = xvt::Line(cv::Point(105, 0), cv::Point(1, 1), true);
    TestIntersect(l, rect, { });

    l = xvt::Line(cv::Point(0, 105), cv::Point(1, 1), true);
    TestIntersect(l, rect, { });

}