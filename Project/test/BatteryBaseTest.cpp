#include "pch.h"
#include "xvtBattery/CylinderInspectorUpper.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/Utils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <execution>
#include <memory>
using namespace xvt;
using namespace xvt::battery;

class BatteryBaseTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        /*listData.push_back(
            std::make_pair("../samples/UPPER/2022-10-19 091225__2022-10-19 091232_H26_0010_22-2438R_UPPER_123_OK.tif",
                           cv::Rect(370, 780, 1100, 230)));*/
        std::string path ="//BUILD-SERVER-WI/Xavis-NewDisk/Battery/Cylinder/GP Line/20221216/1UPPER 16 NG/1UPPER NG/*.tif";
        std::vector<std::string> folderPaths;

        // cv::glob(path, folderPaths,true);
        for (const auto& p : folderPaths)
        {
            listData.push_back(std::make_pair(p,cv::Rect()));
        }
    }

    void TearDown()
    {
    }

public:
    std::vector<std::pair<std::string, cv::Rect>> listData;
    float pixelSize = 1.0f;
};

TEST_F(BatteryBaseTest, BaseTest)
{
    for(auto data: listData)
    {
        cv::Mat src = cv::imread(data.first, 0);
        cv::Mat res = src.clone();
        if (!src.empty())
        {
            // find battery contour
            CylinderInspectorUpper upperInspect;
            auto& baseInspector     = upperInspect.mIspBase;
            auto& beadingInspector = upperInspect.mIspBeading;
            auto jrInspector = std::make_shared<battery::BatteryInspectorJR>(upperInspect.mIspJR);
            auto& cathodeInspector = upperInspect.mIspCathode;
            auto& anodeInspector = upperInspect.mIspAnode;
            auto poleInspector = std::make_shared<battery::BatteryInspectorPole>(upperInspect.mIspPole);

            baseInspector.mThreshold = 0;
            baseInspector.mValidBatteryWidthRange.Set(0, 5000);

            jrInspector->mHeight = 250;

            cathodeInspector.mEnableAutoMode = true;
            cathodeInspector.mCathodeLineThresholdOuter = 8.0;
            cathodeInspector.mCathodeLineThresholdMiddle = 10.0;
            cathodeInspector.mCathodeLineThresholdInner = 12.0;

            anodeInspector.mEnableAutoMode = false;
            anodeInspector.mAnodeThresholdInner = 12.0;
            anodeInspector.mAnodeThresholdMiddle = 12.0;
            anodeInspector.mAnodeThresholdOuter = 11.5;

            auto upperResult = upperInspect.Inspect(src);
            CVPen pen;
            pen.mThickness = 1;
            upperResult.DrawResult(res, cv::Point(), pen);

            cv::imwrite(data.first + ".jpg", res);
        }
    }
    EXPECT_TRUE(true);
}