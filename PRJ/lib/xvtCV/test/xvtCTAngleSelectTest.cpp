/**
 * @example{lineno} test/xvtCTAngleSelectTest.cpp
 * This example demonstrates how to use xvt::ct::AngleSelector class.
 */

#include "pch.h"
#include "xvtCV/xvtCTAngleSelect.h"
#include "xvtCV/Utils.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/CircleDetector.h"
#include "xvtCV/GammaCorrector.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/xvtConvert.h"
#include "xvtCV/LineDetector.h"
#include "xvtCV/xvtFile.h"

class AngleSelectorTest : public testing::Test
{
public:
    static void SetUpTestSuite();
    static void TearDownTestSuite() {}
    static void ReadVolume();
    static bool IsValidVolume();
    void SetUp();

    static int mZIdx;
    static std::string mFolder  ;
    static std::string mFileName;
    static std::vector<cv::Mat> mVolume;
};
int AngleSelectorTest::mZIdx = 0;
std::string AngleSelectorTest::mFolder = "";
std::string AngleSelectorTest::mFileName = "";
std::vector<cv::Mat> AngleSelectorTest::mVolume;

TEST_F(AngleSelectorTest, BuildTest1)
{
    if (!IsValidVolume()) return;

    cv::Mat inImg = mVolume[mZIdx];

    //auto imCenter = cv::Point2f(zImg.size().width / 2, zImg.size().height / 2);
    cv::Point2f imCenter;
    xvt::ct::AngleSelector angleSelector;
    angleSelector.mCenterThreshold = 0;
    angleSelector.mCircleThreshold = 140;
    angleSelector.mVarianceBlurSize = 21;

    angleSelector.GetCenter(inImg, imCenter);

    auto sliceAngle = angleSelector.GetSliceImage(mVolume, mZIdx);

    auto r = sliceAngle.mRadius * 1.05;

    sliceAngle.mAvgNo = 2;
    angleSelector.mAvgNo = sliceAngle.mAvgNo;
    angleSelector.mAvgStep = 0.4f;
    sliceAngle.mAngleList[0] = 0;
    sliceAngle.mAngleList[1] = sliceAngle.mAngleList[0] + 180;
    auto sliceAvgAngle = xvt::ct::GetSliceMPR(mVolume
                                                   , sliceAngle.mCenter
                                                   , sliceAngle.mAngleList
                                                   , r
                                                   , angleSelector.mAvgNo
                                                   , xvt::ct::SliceBlurType::Angle
                                                   , angleSelector.mAvgStep
    );

    auto l = xvt::Line(xvt::Deg2Rad(0), 512);
    auto test = xvt::ct::GetSliceImage(mVolume, l);

    //float lineAngle = (sliceAngle.mAngleList[0] + sliceAngle.mAngleList[1]) / 2.0f - 90.f;
    //float lineAngle = 70;
    float lineAngle = sliceAngle.mAngleList[0];
    auto  lineVec = -cv::Point2f(cos(xvt::Deg2Rad(lineAngle)), sin(xvt::Deg2Rad(lineAngle)));
    //auto  linePoint = imCenter + cv::Point2f(30, 0);
    auto  linePoint = sliceAngle.mCenter + cv::Point2f(0, 0);
    auto  start = linePoint - r * lineVec;
    auto  end   = linePoint + r * lineVec;
    auto  sliceLine = xvt::ct::GetSliceImage(mVolume, start, lineVec, r*2, angleSelector.mRadiusStep);

    angleSelector.mAvgNo = 5;
    angleSelector.mAvgStep = 1.f;
    auto sliceAvgParrallel = xvt::ct::GetSliceMPRLine(mVolume, start, lineVec, r*2, angleSelector.mAvgNo, angleSelector.mAvgStep);

    //auto rect = cv::Rect(0, 245, sliceLine.cols, 350);
    auto rect = xvt::CreateROI(0, 0, sliceLine.cols, sliceLine.rows, sliceLine.size());
    xvt::enhance::GammaCorrector gama(0.8);
    gama.Apply(sliceAngle.mSliceImage, sliceAngle.mSliceImage);
    gama.Apply(sliceLine, sliceLine);
    gama.Apply(sliceAvgAngle, sliceAvgAngle);
    gama.Apply(sliceAvgParrallel, sliceAvgParrallel);
    gama.Apply(inImg, inImg);

    {//Drawing result
        cv::Mat drawZImg;
        cv::Mat drawVerImg;
        xvt::ConvertRGB(inImg, drawZImg);

        sliceAngle.DrawResult(drawZImg, drawVerImg, mZIdx);

        l.Draw(drawZImg, xvt::CVPen(COLOR_CV_LIME));
        cv::line(drawZImg, start, end, COLOR_CV_YELLOW, 1, cv::LineTypes::LINE_AA);

        auto n = cv::Point2f(end.y - start.y, start.x - end.x);
        auto l = cv::norm(n);
        n /= l;
        cv::line(drawZImg, start, end, COLOR_CV_YELLOW, 1, cv::LineTypes::LINE_AA);
        cv::line(drawZImg, start - angleSelector.mAvgNo * n, end - angleSelector.mAvgNo * n, COLOR_CV_CYAN, 1, cv::LineTypes::LINE_AA);
        cv::line(drawZImg, start + angleSelector.mAvgNo * n, end + angleSelector.mAvgNo * n, COLOR_CV_CYAN, 1, cv::LineTypes::LINE_AA);

        xvt::DrawPlusSign(drawZImg, imCenter, xvt::CVPen(COLOR_CV_RED_LONESTAR), 20);


        std::string angleStr  = xvt::ToString(sliceAngle.mAngleList[0], 1) + "_" + xvt::ToString(sliceAngle.mAngleList[1], 1);
        std::string centerStr = "x" + xvt::ToString(sliceAngle.mCenter.x, 1) + "_y" + xvt::ToString(sliceAngle.mCenter.y, 1);
        std::string zIdxStr   = "z" + std::to_string(mZIdx);

        std::string resName = "/res_" + angleStr;

        std::string resName1 = resName + "_" + xvt::ToString(lineAngle, 2) + "_x" + xvt::ToString(linePoint.x, 1) + "_y" + xvt::ToString(linePoint.y, 1);

        std::string output_filename = mFolder + "/res_" + angleStr + "_angle.tif";
        cv::imwrite(output_filename, sliceAngle.mSliceImage);

        output_filename = mFolder + "/res_" + zIdxStr + ".tif";
        cv::imwrite(output_filename, drawZImg);

        output_filename = mFolder + "/res_select_z.tif";
        cv::imwrite(output_filename, drawVerImg);

        output_filename = mFolder + "/res_slice_z" + std::to_string(mZIdx) + ".tif";
        cv::imwrite(output_filename, inImg);

        output_filename = mFolder + resName1 + "_line.tif";
        cv::imwrite(output_filename, sliceLine);

        output_filename = mFolder + resName1 + "_line_avg.tif";
        cv::imwrite(output_filename, sliceAvgParrallel);

        output_filename = mFolder + resName + "_angle_avg.tif";
        cv::imwrite(output_filename, sliceAvgAngle);
    }

    return;
}

TEST_F(AngleSelectorTest, BuildTest2)
{
    if (!IsValidVolume()) return;

    cv::Mat inImg = mVolume[mZIdx];

    float angle = 0.f;
    cv::Point center = cv::Point(inImg.cols/2.0, inImg.rows / 2.0);
    float r = 600;

    auto c = xvt::ct::GetCircle(inImg, 0);
    center = c.mCenter;
    r = c.mRadius * 1.05;

    cv::Mat slice_a, slice_p;
    for (float i = 0; i < 1; i+=1.33)
    {
        std::string str = "Angle: " + std::to_string(i);
        std::cout << str << std::endl;
        xvt::ScopeTimer t(str);

        slice_a = xvt::ct::GetSliceMPR(mVolume, center, i, r, 3, xvt::ct::SliceBlurType::Angle, 0.25f);
        t.GetElapsedTime(L"Angle");
        slice_p = xvt::ct::GetSliceMPR(mVolume, center, i, r, 10, xvt::ct::SliceBlurType::ParrallelLine, 1.f);
        t.GetElapsedTime(L"Parallel");

        EXPECT_EQ(slice_a.rows, mVolume.size());
        EXPECT_EQ(slice_a.cols, int(r) * 2);

        EXPECT_EQ(slice_p.rows, mVolume.size());
        EXPECT_EQ(slice_p.cols, int(r) * 2);

    #if 0
        cv::Mat res;
        cv::vconcat(slice_a, slice_p, res);
        cv::imshow("Slice", res);
        cv::waitKey(0);

       /* auto output_filename = mFolder + "avg_line.tif";
        cv::imwrite(output_filename, slice_p);

        output_filename = mFolder + "avg_angle.tif";
        cv::imwrite(output_filename, slice_a);*/
    #endif
    }
}

TEST_F(AngleSelectorTest, SlicerTest)
{
    if (!IsValidVolume()) return;

    cv::Mat& zImg = mVolume[mZIdx];

    float angle = 0.f;
    cv::Point2f center = cv::Point2f(zImg.cols / 2.0, zImg.rows / 2.0);
    float r = 600;

    xvt::ct::Slicer slicer(center, angle, r, 5);
    slicer.mBlurSize = 2;
    slicer.mBlurStep = 0.5;
    slicer.mBlurType = xvt::ct::SliceBlurType::Angle;
    auto slice_a = slicer.GetSlice(mVolume);

    cv::Mat drawImg = zImg.clone();
    slicer.Draw(drawImg, xvt::CVPen());

    slicer.mBlurSize = 10;
    slicer.mBlurStep = 1;
    slicer.mBlurType = xvt::ct::SliceBlurType::ParrallelLine;
    auto slice_p = slicer.GetSlice(mVolume);
    slicer.Draw(drawImg, xvt::CVPen());

    EXPECT_EQ(slice_a.rows, mVolume.size());
    EXPECT_EQ(slice_a.cols, int(r) * 2);

    EXPECT_EQ(slice_p.rows, mVolume.size());
    EXPECT_EQ(slice_p.cols, int(r) * 2);
}

void AngleSelectorTest::SetUpTestSuite()
{
}

bool AngleSelectorTest::IsValidVolume()
{
    return !mVolume.empty() && mVolume.size() > mZIdx;
}

void AngleSelectorTest::ReadVolume()
{
    mFolder   = "D:/GiangND/Battery/Cylinder/Images/4680 CT/volume_l/";
    mFileName = mFolder + "/volume.raw";

    if (mFileName.empty()) return;

    auto ext = xvt::GetFileExtension(mFileName);
    xvt::ScopeTimer t("Read Volume");
    if (ext == ".tif")
    {
        cv::imreadmulti(mFileName, mVolume, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
    }

    if (ext == ".raw")
    {
        mVolume = xvt::ReadImageRaw(xvt::ToWString(mFileName), 256, 1024, 1024, CV_32FC1);
    }

    mZIdx = 150;
}

void AngleSelectorTest::SetUp()
{
    if(mVolume.empty())
        ReadVolume();
}
