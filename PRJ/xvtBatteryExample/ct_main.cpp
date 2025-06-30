#include <iostream>
#include "xvtBattery/CTBatteryInspectorBase.h"

std::vector<std::wstring> loadData()
{
    std::vector<std::wstring> lstCTInspect;
    xvt::VecString lstPath;
    cv::glob("../samples/CT/4680 Can/*upper*.tif", lstPath, true);
    for(auto path : lstPath) lstCTInspect.emplace_back(xvt::ToWString(path));
    return lstCTInspect;
}

using namespace xvt;
using namespace xvt::battery;

void InlineSetting(CTBatteryInspector &ctIsp, bool loadRecipe = false, bool saveRecipe = false)
{
    auto& baseIsp = ctIsp.mIspBase;
    auto& beadingIsp = ctIsp.mIspBeading;
    auto& jrIsp = ctIsp.mIspJR;
    auto& poleIsp = ctIsp.mIspPole;
    auto& cathodeIsp = ctIsp.mIspCathode;
    auto& anodeIsp = ctIsp.mIspAnode;
    auto& displayPen = ctIsp.mDisplayPen;

    ctIsp.mEnable = 1;

    baseIsp.mRoi.x = 0;
    baseIsp.mRoi.y = 265;
    baseIsp.mRoi.width = 1000;
    baseIsp.mRoi.height = 230;
    baseIsp.mThreshold = 70;
    baseIsp.mDirection = 0;
    baseIsp.mValidBatteryWidthRange.mIsEnable = true;
    baseIsp.mValidBatteryWidthRange.Set(10, 50);
    ctIsp.mDisplayMode = xvt::battery::DisplayMode::POLE;
    ctIsp.mAvgSliceNo = 3;
    ctIsp.mSliceNo = 7;
    ctIsp.mPixelSize = 0.050;

    beadingIsp.mEnable = false;
    beadingIsp.mBeadingHeightMin = 50;
    beadingIsp.mD1StartPosition = 0.5;

    jrIsp.mJROffsetX = 20;
    jrIsp.mJROffsetY = 65;
    jrIsp.mHeight = 80;
    jrIsp.mCenterNeglectionWidth = 130;
    jrIsp.mBaseLineOffset = 0;
    jrIsp.mOneSidePoleNumber = 70;
    jrIsp.mEnableCheckLeaning = false;
    jrIsp.mLeaningThreshold = 0;
    jrIsp.mMinLeaningDistance = 0;

    cathodeIsp.mCathodeLineThresholdInner = 0;
    cathodeIsp.mCathodeLineThresholdMiddle = 0;
    cathodeIsp.mCathodeLineThresholdOuter = 0;
    cathodeIsp.mCathodeLineWindowSize = 25;

    anodeIsp.mAnodeThresholdInner = 1.2;
    anodeIsp.mAnodeThresholdMiddle = 2.2;
    anodeIsp.mAnodeThresholdOuter = 2.2;

    poleIsp.mPoleHeight = 80;
    poleIsp.mPolesProminenceThreshold = 1;
    poleIsp.mPolesDistanceRange.Set(3, 30);
    poleIsp.mSkipPolesDistance = 0;
    poleIsp.mAnode2CaseOffset = 0;
    poleIsp.mCathode2AnodeOffset = 0;
    poleIsp.mCathode2CaseOffset = 0;
    poleIsp.mValidCathode2AnodeRange.mIsEnable = true;
    poleIsp.mValidCathode2AnodeRange.Set(0.45, 4.0);
    poleIsp.mValidAnode2CaseRange.mIsEnable = true;
    poleIsp.mValidAnode2CaseRange.Set(0.8, 3.0);
    poleIsp.mValidCathode2CaseRange.mIsEnable = true;
    poleIsp.mValidCathode2CaseRange.Set(3.5, 7);
    poleIsp.mVariationAnode2Case = 3;

    displayPen.mFontScale = 0.4;
	displayPen.mSpace = 15;
    ctIsp.mGamma = 1.5;
}

int ct_main()
{
    auto listData = loadData();
    if(!listData.empty())
    {
        for(const auto &path : listData)
        {
            cv::Mat src = xvt::ReadImage(path, cv::ImreadModes::IMREAD_ANYDEPTH);
            
            if (!src.empty())
            {
                xvt::battery::CTBatteryInspector CTInspect;
                xvt::battery::CTBatteryInspectorResult IResult;
                InlineSetting(CTInspect);

                auto result = CTInspect.Inspect(src, IResult);
                cv::Mat res;
                IResult.DrawResult(res);
                xvt::WriteImage(path + L".png", res);
            }
        }
    }
    return 0;
}