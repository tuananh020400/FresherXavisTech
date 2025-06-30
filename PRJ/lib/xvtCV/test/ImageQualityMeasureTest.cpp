#include "pch.h"
#include "xvtCV/ImageQualityMeasure.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class LinePairsTest : public ::testing::Test {
protected:
    float numOfLp = 4;
    xvt::LinePairs mLinePairs;
    std::unique_ptr<xvt::LinePairsResult> res = std::make_unique<xvt::LinePairsResult>();
};


TEST_F(LinePairsTest, Process)
{
    std::string imgPath = ".\\line_image_test\\LinePair.jpg";
    cv::Mat img = cv::imread(imgPath, 0);
    mLinePairs.mStep = 20;
    mLinePairs.mMinProminance = 0.0f;
    mLinePairs.mMinDistance = 0.0f;
    mLinePairs.mNumOfLinePairs = 4;
    mLinePairs.mOffset = 0.1f;
    res = xvt::DynamicCastUniquePtr<xvt::LinePairsResult, xvt::IInspectionResult>(mLinePairs.Inspect(img));

    res->mPixelSize = 0.0093;
    auto LPPerMM = res->Convert2LinePairPerMM();
    EXPECT_TRUE(!res->mListContrast.empty());
    EXPECT_TRUE(!res->mListLinePair.empty());

    EXPECT_EQ(res->mListContrast.size(), res->mListLinePair.size());
    EXPECT_EQ(res->mListLinePair.size(), LPPerMM.size());

    bool inRangeCtr = std::all_of(res->mListContrast.begin(), res->mListContrast.end(), [](auto& val)
        {return val >= 0.0f && val <= 1.0f; });
    EXPECT_TRUE(inRangeCtr);

    bool inRangeLp = std::all_of(res->mListLinePair.begin(), res->mListLinePair.end(), [](auto& val)
        {return val >= 0.0f; });

    EXPECT_TRUE(inRangeLp);

    bool inRangeLpMM = std::all_of(LPPerMM.begin(), LPPerMM.end(), [](auto& val)
        {return val >= 0.0f; });

    EXPECT_TRUE(inRangeLpMM);

}


TEST_F(LinePairsTest, ImageEmpty)
{
    cv::Mat inputImg;
    res = xvt::DynamicCastUniquePtr<xvt::LinePairsResult, xvt::IInspectionResult>(mLinePairs.Inspect(inputImg));
    EXPECT_TRUE(res->mListContrast.empty());
    EXPECT_TRUE(res->mListLinePair.empty());
}

TEST_F(LinePairsTest, ImageColor)
{
    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    res = xvt::DynamicCastUniquePtr<xvt::LinePairsResult, xvt::IInspectionResult>(mLinePairs.Inspect(img));
    EXPECT_TRUE(res->mListContrast.empty());
    EXPECT_TRUE(res->mListLinePair.empty());

}

TEST_F(LinePairsTest, Image16Bits)
{
    cv::Mat img = cv::Mat::zeros(500, 500, CV_16UC3);
    res = xvt::DynamicCastUniquePtr<xvt::LinePairsResult, xvt::IInspectionResult>(mLinePairs.Inspect(img));
    EXPECT_TRUE(res->mListContrast.empty());
    EXPECT_TRUE(res->mListLinePair.empty());
}

TEST_F(LinePairsTest, Image4Channels1)
{
    cv::Mat bgraImage(100, 100, CV_16FC4, cv::Scalar(255, 0, 0, 255));
    res = xvt::DynamicCastUniquePtr<xvt::LinePairsResult, xvt::IInspectionResult>(mLinePairs.Inspect(bgraImage));
    EXPECT_TRUE(res->mListContrast.empty());
    EXPECT_TRUE(res->mListLinePair.empty());
}

TEST_F(LinePairsTest, Image4Channels2)
{
    cv::Mat bgraImage(100, 100, CV_8UC4, cv::Scalar(255, 0, 0, 255));
    res = xvt::DynamicCastUniquePtr<xvt::LinePairsResult, xvt::IInspectionResult>(mLinePairs.Inspect(bgraImage));
    EXPECT_TRUE(res->mListContrast.empty());
    EXPECT_TRUE(res->mListLinePair.empty());
}

