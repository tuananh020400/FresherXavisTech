#include "xvtBattery/CylinderBatteryUpper.h"
#include "xvtBattery/CylinderBatteryLower.h"
#include "header.h"

//#define INSPECT_UPPER

std::vector<std::wstring> loadCylinderData()
{
    std::vector<std::wstring> lstCylinderInspect;
#ifdef INSPECT_UPPER
    //lstCylinderInspect.emplace_back(L"../samples/UPPER/2022-10-19 091225__2022-10-19 091232_H26_0010_22-2438R_UPPER_123_OK.tif");
    //lstCylinderInspect.emplace_back(L"../samples/UPPER/2022-11-04 173500__2022-11-04 173507_H33_8549_22-2438R_UPPER_123_OK.tif");
    lstCylinderInspect.emplace_back(L"../samples/UPPER/1 1 3.tif");
#else
    //lstCylinderInspect.emplace_back(L"../samples/LOWER/2022-06-08 121610_H47_0019_iBC221-NO1_LOWER_Recipe1_NG.tif");
    //lstCylinderInspect.emplace_back(L"../samples/LOWER/2022-06-26 033603__2022-06-26 033610_H17_57442_iBC221-NO1_LOWER_Recipe(old)_NG.tif");
    //lstCylinderInspect.emplace_back(L"../samples/LOWER/2022-10-19 091225_H26_0010_22-2438R_LOWER_123_OK.tif");
    //lstCylinderInspect.emplace_back(L"../samples/LOWER/2023-01-06 140723_H07_91358_22-2476R_LOWER_123_OK.tif");
    lstCylinderInspect.emplace_back(L"../samples/LOWER/1 2 3.tif");
    //lstCylinderInspect.emplace_back(L"../samples/LOWER/M2_F10_M2_F10.tif");
#endif


    return lstCylinderInspect;
}

using namespace xvt;
using namespace xvt::battery;

#ifdef INSPECT_UPPER
void InlineSetting(CylinderBatteryUpper& cUpper)
{
    cUpper.mRoi = cv::Rect(0, 0, 3000, 3000);
    cUpper.JR_ROIX = 0;
    cUpper.JR_ROIY = -50;
    cUpper.mEnableAutoMode = true;
    cUpper.mCenterNeglectionWidth = 300;
    cUpper.mPixelSize = 0.0167;
    cUpper.mSkipPolesDistance = 0;;
    cUpper.mValidCellWidthRange.Set(0, 40.0);
    cUpper.mValidCathode2AnodeRange.Set(0, 3);
    cUpper.mValidAnode2CaseRange.Set(0, 3);
    cUpper.setGamma(18);
    //cUpper.mInspectingItems.SetALL(false);
    //cUpper.mInspectingItems.COVER_HEIGHT = false;
    //cUpper.mInspectingItems.SetPolesInspect(true);
    cUpper.mInspectingItems.ANODE_TO_CASE_GAP = false;
    cUpper.mInspectingItems.ANODE_TO_CATHODE_LENGTH = true;
    cUpper.mInspectingItems.ANODE_TO_CASE_VARIATION = false;
    //cUpper.mInspectingItems.CATHODE_TO_CASE_GAP = true;
    //cUpper.mInspectingItems.FIND_JR_ROI = false;
    //cUpper.mInspectingItems.CHECK_LEANING = false;
    cUpper.mInspectingItems.CHECK_BEADING = false;
    cUpper.mTextFontScale = 0.7;
    cUpper.mTextLineSpace = 30;
    cUpper.mTextPosition = cv::Point(0,200);
    //cUpper.mDisplayMode = DisplayMode::DALL;
}

#else
void InlineSetting(CylinderBatteryLower& lower)
{
    lower.mRoi = cv::Rect(0, 0, 3000, 1200);
    lower.mThreshold = 50;
    lower.JR_ROIX = 0;
    lower.JR_ROIY = -50;
    lower.mEnableAutoMode = 1;
    lower.mPixelSize = 0.0167;
    lower.mValidCellWidthRange.Set(0.1, 50.0);
    lower.setGamma(20); // 0 - 40
    lower.mValidCathode2AnodeRange.Set(0.0, 3.0);
    lower.mValidAnode2CaseRange.Set(0.0, 1.6);
    lower.mValidCathode2CaseRange.Set(0.0, 2.0);
    lower.mSkipPolesDistance = 0;;
    lower.mCenterNeglectionWidth = 300;
    lower.mInspectingItems.CHECK_BLACK_CLOUD = false;
    lower.mInspectingItems.ANODE_TO_CASE_VARIATION = false;
    lower.mInspectingItems.CATHODE_TO_CASE_GAP = false;
    lower.mInspectingItems.ANODE_TO_CASE_GAP = false;
    lower.mInspectingItems.ANODE_TO_CATHODE_LENGTH = true;
    lower.mLineType = 0;
    lower.mTextFontScale = 0.7;
    lower.mTextLineSpace = 30;
    lower.mTextPosition = cv::Point(0,1200);
}
#endif

int cylinder_main()
{
    auto listData = loadCylinderData();
    std::for_each(listData.begin(), listData.end(), [&](const auto &path)
    {
        cv::Mat src = xvt::ReadImage(path, cv::ImreadModes::IMREAD_ANYDEPTH);
        cv::Mat res = src.clone();
        if (!src.empty())
        {
            BatteryInspectionResult BResult;
#ifdef INSPECT_UPPER
            CylinderBatteryUpper cUpper;
            InlineSetting(cUpper);
            ERR_CODE err = cUpper.Inspection(src, BResult);
#else
            CylinderBatteryLower lower;
            InlineSetting(lower);
            ERR_CODE err = lower.Inspection(src, BResult);
#endif
            std::wstring newPath = xvt::GetParentDirectory(path) + L"/" + xvt::GetFileNameWithoutExtension(path);
            xvt::WriteImage(newPath + L".jpg", BResult.resImg);
        }
    });

    return 0;
}
