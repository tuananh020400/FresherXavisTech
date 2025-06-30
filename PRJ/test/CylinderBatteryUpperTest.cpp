#include "pch.h"
#include "xvtBattery/CylinderBatteryUpper.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace xvt::battery;
//std::string path_upper = "./samples/UPPER/2022-10-19 091225__2022-10-19 091232_H26_0010_22-2438R_UPPER_123_OK.tif";
std::string path_upper = "./samples/UPPER/2022-11-04 173500__2022-11-04 173507_H33_8549_22-2438R_UPPER_123_OK.tif";
cv::Mat src_upper;

TEST(CylinderBatteryUpperTest, TestMatInput) {
    src_upper = cv::imread(path_upper, 0);
    EXPECT_TRUE(!src_upper.empty());
}

TEST(CylinderBatteryUpperTest, Inspection) {
    CylinderBatteryUpper upper;
    upper.mRoi = cv::Rect(0, 0, 3000, 3000);
    upper.mEnableAutoMode = true;
    upper.mPixelSize = 0.0167;
    upper.mValidCellWidthRange.Set(0, 40.0);
    upper.mValidCathode2AnodeRange.Set(0, 3);
    upper.mValidAnode2CaseRange.Set(0, 3);
    upper.setGamma(18);
    //upper.mInspectingItems.SetALL(false);
    //upper.mInspectingItems.COVER_HEIGHT = false;
    //upper.mInspectingItems.SetPolesInspect(true);
    upper.mInspectingItems.ANODE_TO_CASE_GAP = true;
    upper.mInspectingItems.ANODE_TO_CATHODE_LENGTH = true;
    //upper.mInspectingItems.CATHODE_TO_CASE_GAP = true;
    //upper.mInspectingItems.FIND_JR_ROI = false;
    //upper.mInspectingItems.CHECK_LEANING = false;
    upper.mTextFontScale = 0.6;
    //upper.mDisplayMode = DisplayMode::DALL;
    BatteryInspectionResult BResult;
    ERR_CODE err = upper.Inspection(src_upper, BResult);
    //show_window("upper", BResult.resImg);
    //std::string path = path_upper;
    //xvt::battery::replace_all(path, path.substr(path.find_last_of(".") + 1), ".jpg");
    //cv::imwrite(path, BResult.resImg);
    //cv::waitKey(0);
    //upper.MinAnode2Case = 1;
    //upper.mEnableAutoMode = true;
    EXPECT_EQ(err, ERR_CODE::OK);
}