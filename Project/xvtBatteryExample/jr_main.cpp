#include <iostream>
#include "xvtCV/Utils.h"
#include "xvtBattery/JRBatteryInspector.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "header.h"

std::vector<std::wstring> loadJRData()
{
    std::vector<std::wstring> lstJRInspect;
    lstJRInspect.emplace_back(L"../samples/JR/AL_P1.tif");
    lstJRInspect.emplace_back(L"../samples/JR/AL_P2.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230814_JR/TEST1/AL_P1.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230807_JR//6/AL_P2.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/JR10/AL_P1.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/JR10/AL_P2.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/JR11/AL_P1.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/JR11/AL_P2.tif");
    //lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/JR5/AL_P1.tif");
    
    /*for (int i = 1; i < 5; i++)
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

    for (int i = 1; i < 4; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230809_JR/AL" + std::to_wstring(i) + L"_P1.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230809_JR/AL" + std::to_wstring(i) + L"_P2.tif");
    }

    for (int i = 1; i < 11; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230810_JR/" + std::to_wstring(i) + L"/AL_P1.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230810_JR/" + std::to_wstring(i) + L"/AL_P2.tif");
    }

    for (int i = 1; i < 11; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230810_JR2/" + std::to_wstring(i) + L"/AL_P1.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230810_JR2/" + std::to_wstring(i) + L"/AL_P2.tif");
    }

    for (int i = 1; i < 11; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230810_JR3/" + std::to_wstring(i) + L"/AL_P1.tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230810_JR3/" + std::to_wstring(i) + L"/AL_P2.tif");
    }*/

    
    /*for (int i = 1; i < 12; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/Ori/AL_P1" + std::to_wstring(i) + L".tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230818_JR/Ori/AL_P2" + std::to_wstring(i) + L".tif");
    }

    for (int i = 1; i < 15; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230824_JR/Ori/AL_P1" + std::to_wstring(i) + L".tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230824_JR/Ori/AL_P2" + std::to_wstring(i) + L".tif");
    }

    for (int i = 1; i < 12; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20230905_JR/Ori/AL_P1_S" + std::to_wstring(i) + L".tif");
        lstJRInspect.emplace_back(L"../samples/JR/20230905_JR/Ori/AL_P2_S" + std::to_wstring(i) + L".tif");
    }

    for (int i = 1; i < 5; i++)
    {
        lstJRInspect.emplace_back(L"../samples/JR/20231006_JR/Ori/AL_P1_S" + std::to_wstring(i) + L".tif");
        lstJRInspect.emplace_back(L"../samples/JR/20231006_JR/Ori/AL_P2_S" + std::to_wstring(i) + L".tif");
    }*/

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
    jrIsp.mPixelSize = 0.011;
    jrIsp.mAlgo = 2; // 0: Manully Threshold, 1: Auto Threshold, 2: Find Peak
    jrIsp.mManullyThreshold = 100;
    jrIsp.mOffsetAno = 0;
    jrIsp.mProminenceValue = 40.0;

    baseInspector.mThreshold = 0;
    baseInspector.mRoi = cv::Rect(800, 600, 1000, 1000);

    jrInspector.mNoRegion = 5;
    jrInspector.mHeight = 150;
    jrInspector.mJROffsetX = -30;
    jrInspector.mJROffsetY = -200;

    poleInspector.mPoleHeight = -120;
    poleInspector.mPolesProminenceThreshold = 0.5;
    poleInspector.mPolesDistanceRange.Set(23, 45);
    poleInspector.mMaximumNumberPoles = 21;
    poleInspector.mMinimumNumberPoles = 18;
    poleInspector.mEnableValidCathode2Anode = true;
    poleInspector.mValidCathode2AnodeRange.Set(0.0, 10.0);

    cathodeInspector.mEnableAutoMode = true;
    cathodeInspector.mIsApplyEnhanced = true;
    cathodeInspector.mCathodeLineThresholdOuter = 8.0;
    cathodeInspector.mCathodeLineThresholdMiddle = 10.0;
    cathodeInspector.mCathodeLineThresholdInner = 12.0;

    anodeInspector.mEnableAutoMode = true;
    anodeInspector.mIsApplyEnhanced = false;
    anodeInspector.mAnodeThresholdInner = 12.0;
    anodeInspector.mAnodeThresholdMiddle = 12.0;
    anodeInspector.mAnodeThresholdOuter = 11.5;
#pragma endregion Setting Paras region Setting Paras
}

int jr_main()
{
    auto listData = loadJRData();
    std::for_each(listData.begin(), listData.end(), [&](const auto &path)
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
            jrInspectResult.Save(xvt::GetParentDirectory(path), L"result.csv", true);
            std::wstring newPath = xvt::GetParentDirectory(path) + L"/" + xvt::GetFileNameWithoutExtension(path);
            xvt::WriteImage(newPath + L".jpg", res);
        }
    });

    return 0;
}
