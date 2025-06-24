#include "pch.h"
#include "xvtBattery/CylinderBatteryLower.h"
#include "xvtBattery/CylinderUtils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace xvt::battery;
using namespace xvt;

class CylinderBatteryLowerTest : public ::testing::Test
{
public:
    void SetUp()
    {
        src_lower = cv::imread(path_lower, 0);
        src_lower_NG = cv::imread(path_lower_NG, 0);
        pole_region = cv::imread(path_pole_region, 0);
        src_lower_black_cloud = cv::imread(path_black_cloud, 0);
        lower = CylinderBatteryLower();
        lower.mRoi = cv::Rect(0, 0, 5000, 5000);
        lower.mEnableAutoMode = 1;
        lower.mPixelSize = 0.0167;
        lower.mValidCellWidthRange.Set(0.1, 50.0);
        lower.setGamma(10); // 0 - 40
        lower.mValidCathode2AnodeRange.Set(0.0, 3.0);
        lower.mValidAnode2CaseRange.Set(0.0, 1.6);
        lower.mValidCathode2CaseRange.Set(0.0, 2.0);
        lower.mCenterNeglectionWidth = 250;
    }

    void TearDown()
    {

    }

public:
    BatteryInspectionResult BResult;
    CylinderBatteryLower lower;
    cv::Mat src_lower, src_lower_NG, src_lower_black_cloud;
    cv::Mat pole_region;
    std::string path_lower = "./samples/LOWER/2022-06-08 121610_H47_0019_iBC221-NO1_LOWER_Recipe1_NG.tif";
    std::string path_lower_NG = "./samples/LOWER/2022-06-26 033603__2022-06-26 033610_H17_57442_iBC221-NO1_LOWER_Recipe(old)_NG.tif";
    std::string path_pole_region = "./samples/POLE_REGION/2022-10-19 160723_H09_0149_22-2475R_LOWER_123_OK.bmp";
    std::string path_black_cloud = "./samples/LOWER/2023-01-06 140723_H07_91358_22-2476R_LOWER_123_OK.tif";
};
bool writeImg = false;
TEST_F(CylinderBatteryLowerTest, Inspection) {

    lower.mInspectingItems.ANODE_TO_CASE_GAP = true;
    lower.mInspectingItems.ANODE_TO_CATHODE_LENGTH = true;
    lower.mInspectingItems.CATHODE_TO_CASE_GAP = false;
    ERR_CODE err = lower.Inspection(src_lower, BResult);
    //show_window("lower", BResult.resImg);
    if (writeImg)
    {
        std::string path = path_lower;
        xvt::battery::replace_all(path, path.substr(path.find_last_of(".") + 1), "jpg");
        cv::imwrite(path, BResult.resImg);
    }
    
    //cv::waitKey(0);

    EXPECT_EQ(err, ERR_CODE::OK);
}
TEST_F(CylinderBatteryLowerTest, Inspection_BlackCloud)
{
    lower.mInspectingItems.ANODE_TO_CASE_GAP = true;
    lower.mInspectingItems.ANODE_TO_CATHODE_LENGTH = true;
    lower.mInspectingItems.CATHODE_TO_CASE_GAP = false;
    lower.mInspectingItems.CHECK_BLACK_CLOUD = true;
    ERR_CODE err = lower.Inspection(src_lower_black_cloud, BResult);
    //show_window("lower", BResult.resImg);
    EXPECT_EQ(err, ERR_CODE::OK);
}
TEST_F(CylinderBatteryLowerTest, InspectionNG) {
    ERR_CODE err = lower.Inspection(src_lower_NG, BResult);
    //show_window("lower", BResult.resImg);
    if (writeImg)
    {
        std::string path = path_lower_NG;
        xvt::battery::replace_all(path, path.substr(path.find_last_of(".") + 1), "jpg");
        cv::imwrite(path, BResult.resImg);
    }
    //cv::waitKey(0);

    EXPECT_EQ(err, ERR_CODE::errPoleXDetection4);
}

TEST_F(CylinderBatteryLowerTest, FindAllPolesPos) {

    //return;
    std::vector<int> lstPolePosXAll, anodePos, cathodePos;
    std::string descript = "";
    cv::Rect ROI = cv::Rect(cv::Point(0, 0), pole_region.size());
    ERR_CODE err = lower.FindAllPolesPos(pole_region, ROI, lstPolePosXAll, anodePos, cathodePos, descript, false);
    //std::string res = ERR_CODE_Message(err);
    /*if (writeImg)
    {
        std::string path = path_pole_region;
        xvt::battery::replace_all(path, path.substr(path.find_last_of(".") + 1), "jpg");
        cv::imwrite(path, BResult.resImg);
    }*/
    //cv::waitKey(0);

    EXPECT_EQ(err, ERR_CODE::OK);
}