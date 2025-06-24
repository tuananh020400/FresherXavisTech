#include "pch.h"
#include "xvtBattery/JRBatteryInspector.h"
#include "xvtCV/Utils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <execution>

std::vector<std::wstring> loadData()
{
    std::vector<std::wstring> lstJRInspect;

    for (int i = 1; i < 5; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230623_JR 21/S" + std::to_wstring(i) + L"P10000.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230623_JR 21/S" + std::to_wstring(i) + L"P20000.tif");
    }

    for (int i = 1; i < 7; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230628_JR 17/S" + std::to_wstring(i) + L"P10000.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230628_JR 17/S" + std::to_wstring(i) + L"P20000.tif");
    }

    for (int i = 1; i < 10; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230802_JR/S" + std::to_wstring(i) + L"P10000.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230802_JR/S" + std::to_wstring(i) + L"P20000.tif");
    }
    return lstJRInspect;
}

using namespace xvt;
using namespace xvt::battery;

void InlineSetting(JRBatteryInspector& jrIsp)
{
    auto& baseInspector = jrIsp.mIspBase;
    auto& jrInspector = jrIsp.mIspJR;
    auto& poleInspector = jrIsp.mIspPole;
    auto& cathodeInspector = jrIsp.mIspCathode;
    auto& anodeInspector = jrIsp.mIspAnode;

#pragma region Setting Paras
    jrIsp.mOffset = 3;
    jrIsp.mThresholdCathodeDefault = 3;
    jrIsp.mIsRotated = true;
    jrIsp.mPixelSize = 0.0016;
    jrIsp.mAlgo = 2; // 0: Manully Threshold, 1: Auto Threshold, 2: Find Peak
    jrIsp.mManullyThreshold = 100;
    jrIsp.mOffsetAno = 0;

    baseInspector.mThreshold = 0;
    baseInspector.mRoi = cv::Rect(800, 600, 1000, 1000);

    jrInspector.mNoRegion = 3;
    jrInspector.mHeight = 150;
    jrInspector.mJROffsetX = -20;
    jrInspector.mJROffsetY = -200;

    poleInspector.mPoleHeight = -120;
    poleInspector.mPolesProminenceThreshold = 1.5;
    poleInspector.mPolesDistanceRange.Set(18, 45);
    poleInspector.mMaximumNumberPoles = 21;
    poleInspector.mMinimumNumberPoles = 18;
    poleInspector.mEnableValidCathode2Anode = true;
    poleInspector.mValidCathode2AnodeRange.Set(0, 0.5);

    cathodeInspector.mEnableAutoMode = true;
    cathodeInspector.mCathodeLineThresholdOuter = 8.0;
    cathodeInspector.mCathodeLineThresholdMiddle = 10.0;
    cathodeInspector.mCathodeLineThresholdInner = 12.0;

    anodeInspector.mEnableAutoMode = true;
    anodeInspector.mAnodeThresholdInner = 12.0;
    anodeInspector.mAnodeThresholdMiddle = 12.0;
    anodeInspector.mAnodeThresholdOuter = 11.5;
#pragma endregion Setting Paras region Setting Paras
}

TEST(JRBatteryInspectTest, inspect) {
    auto listData = loadData();
    for(auto path: listData)
    {
        xvt::battery::JRBatteryInspector jrInspect;
        InlineSetting(jrInspect);
        cv::Mat src = xvt::ReadImage(path, cv::ImreadModes::IMREAD_ANYDEPTH);
        cv::Mat res = src.clone();
        if (!src.empty())
        {
            auto jrInspectResult = jrInspect.Inspect(src);

            xvt::CVPen pen;
            pen.mThickness = 1;
            jrInspectResult.DrawResult(res, cv::Point(), pen);
            //jrInspectResult.Save(xvt::GetParentDirectory(path), L"result.csv", true);
            xvt::WriteImage(path + L".jpg", res);
        }
    }
}