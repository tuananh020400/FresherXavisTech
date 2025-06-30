/**
 * @example{lineno} test/FindPeakTest.cpp
 * This example demonstrates how to use xvt::FindPeaks class.
 */

#include "pch.h"
#include "xvtCV/Peak.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>

using namespace xvt;
using namespace cv;

const VecFloat mSig1     = { 3,3,3,1,1,2,4,6,6,7,8,8,6,4,1,8,8 };
const VecPeak  mSig1Peak = { Peak{2, 3, 0}, Peak{7, 6, 0}, Peak{10, 8, 7}, Peak{11, 8, 7}, Peak{15, 8, 0} };

const VecFloat mSig2     = { 1,1,2,4,6,6,5,4,3,3,6,4,1,8,8 };
const VecPeak  mSig2Valley = { Peak{1, 1, 0}, Peak{8, 3, -3}, Peak{9, 3, -3}, Peak{12, 1, -5} };

const VecFloat mSig3 = { 1,6,3,2,6,3,1,6,5,4 };
const VecPeak  mSig3Peak = { Peak{1, 6, 4}, Peak{4, 6, 4}, Peak{7, 6, 2} };

const VecFloat mSig4 = { 9,4,5,6,4,7,10,4,4,7 };
const VecPeak  mSig4Valley = { Peak{1, 4, -2}, Peak{4, 4, -2}, Peak{7, 4, -3}  , Peak{8, 4, -3} };

const VecFloat mSig5 = { 5,5,5,5,5,5,5,5,5,5 };
const VecPeak  mSig5Peak = { };
const VecPeak  mSig5Valley = { };

const VecFloat mSig7 = { 1,6,3,-2,6,3,1,6,5,4,10,-1,6,0 };
const VecPeak  mSig7Peak = { Peak{1, 6, 5}, Peak{4, 6, 5}, Peak{7, 6, 2}, Peak{10, 10, 11}, Peak{12, 6, 6} };

const VecFloat mSig8 = { 9,4,5,6,10,4,4,7,1,5,4,8 };
const VecPeak  mSig8Valley = { Peak{1, 4, -5}, Peak{5, 4, -3}, Peak{6, 4, -3}  , Peak{8, 1, -7}, Peak{10, 4, -1} };

VecFloat Load(std::string const& fileName)
{
    std::ifstream f(fileName, std::iostream::in);

    VecFloat buffer;
    if (f.is_open())
    {
        f.seekg(0, f.end);// seek to end of file
        int fileSize = f.tellg();// get current file pointer
        f.seekg(0, f.beg);// seek back to beginning of file
        std::string str;
        if (fileSize > 0)
        {
            while (std::getline(f, str))
            {
                //std::istringstream in(str);
                buffer.push_back(std::atof(str.c_str()));
            }
        }
    }
    f.close();
    return buffer;
}

void TestFindPeak(const VecFloat& signal, const xvt::VecPeak& peakRef , xvt::PeakType type) {

    cv::Mat sigImg;
    xvt::FindPeaks peakFinder(type, xvt::PeakFindingMethod::Prominence);
    peakFinder.Process(signal);

    auto&& peaks = peakFinder.GetPeaks();
    auto drawTool = xvt::DrawPeaks(sigImg, signal, peaks, cv::Scalar(0, 0, 255));

    auto isSizeOK = (int)peakRef.size() == (int)peaks.size();
    EXPECT_TRUE(isSizeOK);
    if (isSizeOK)
    {
        for (int i = 0; i < peakRef.size(); i++)
        {
            EXPECT_EQ(peakRef[i], peaks[i]);
        }
    }
}

TEST(PeakTest, PeakEQfirst)
{
    TestFindPeak(mSig1 , mSig1Peak , PeakType::Peak );
}

TEST(PeakTest, ValleyEQfirst)
{
    TestFindPeak(mSig2, mSig2Valley, PeakType::Valley);
}

TEST(PeakTest, PeakAdjacentEQ)
{
    TestFindPeak(mSig3, mSig3Peak, PeakType::Peak);
}

TEST(PeakTest, ValleyAdjacentEQ)
{
    TestFindPeak(mSig4, mSig4Valley, PeakType::Valley);
}

TEST(PeakTest, noPeakValley)
{
    TestFindPeak(mSig5, mSig5Peak, PeakType::Peak);
    TestFindPeak(mSig5, mSig5Valley, PeakType::Valley);
}

TEST(PeakTest, PeakEQsuddenHigher)
{
    TestFindPeak(mSig7, mSig7Peak, PeakType::Peak);
}

TEST(PeakTest, ValleyEQsuddenHigher)
{
    TestFindPeak(mSig8, mSig8Valley, PeakType::Valley);
}

//! [Peak Finding Examples] it is use for snippet code referencing
TEST(PeakTest, PeakExamples)
{
    // Read the signal image
    cv::Mat im = cv::imread("xvtcvLib/Docs/peaksnolineorg.png");
    if (im.empty()) return;

    // Threshold image and extract the signal x, y data
    cv::Mat bin;
    cv::cvtColor(im, bin, cv::ColorConversionCodes::COLOR_BGR2GRAY);
    cv::threshold(bin, bin, 190, 255, THRESH_BINARY_INV);
    VecInt x;
    VecFloat y;
    for (int c = 0; c < bin.cols; c++)
    {
        for (int r = 0; r < bin.rows; r++)
        {
            if (bin.at<uchar>(r, c) > 0)
            {
                y.emplace_back(r);
                x.emplace_back(c);
                break;
            }
        }
    }

    // smooth the signal to avoid noise
    cv::blur(y, y, cv::Size(5, 1));

    int pro = 10;// prominece
    int dis = 5; // ditance

    // Create the peak finder.
    FindPeaks peakFinder(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
    peakFinder.Process(y);

    // Find the peak that has prominence > 10.
    auto peaks = peakFinder.GetPeakResult(pro, dis);

    // Create the valley finder.
    FindPeaks valleyFinder(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
    valleyFinder.Process(y);

    // Find the valley that has prominence < -10.
    auto valleys = valleyFinder.GetPeakResult(pro, dis);

    // Create the draw tool
    xvt::Drawing drawTool;
    drawTool.yTopOrigin = true;
    drawTool.mLineType = cv::LineTypes::LINE_8;
    // Draw the signal
    drawTool.Plot(bin, x, y, 0);
    CVPen pen(COLOR_CV_RED);
    // Draw the peak by red color
    for (auto& p : peaks)
    {
        xvt::DrawPlusSign(im, cv::Point(x[p.index], p.value), pen, 5);
    }

    pen = CVPen(COLOR_CV_BLUE);
    // Draw the valley by blue color
    for (auto& p : valleys)
    {
        xvt::DrawPlusSign(im, cv::Point(x[p.index], p.value), pen, 5);
    }
}

//! [Peak Finding Examples] it is use for snippet code referencing