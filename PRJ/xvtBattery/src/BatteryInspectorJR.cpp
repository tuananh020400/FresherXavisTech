#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/Contour.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/Polyfit.h"
#include "xvtCV/Thinning.h"
#include "xvtCV/Utils.h"
#include <xvtCV/GlobalThresholding.h>
#include <CvPlot/cvplot.h>

namespace xvt
{
namespace battery
{

auto BatteryInspectorJR::Inspect(cv::Mat const &inImg, cv::Rect roi) const -> BatteryInspectorJRResult
{
    auto ispResult = BatteryInspectorJRResult();

    ispResult.mEnable = mEnable;

    if (!ispResult.mEnable)
    {
        ispResult(EResult::UC, "Disabled!");
        return ispResult;
    }

    cv::Mat src;
    if (!xvt::Convert8Bits(inImg, src, false))
    {

        cv::Rect subROI = CreateROI(cv::Point(roi.x, roi.y + mJROffsetY), cv::Size(roi.width, mHeight), inImg.size());
        double LeftBorder = 0;
        double RightBorder = subROI.width;
        if (mEnableCheckLeaning && !subROI.empty())
        {
            int leaningThreshold = mEnableCheckLeaningAuto ? 0 : mLeaningThreshold;
            CheckPoleLeaning(src(subROI), mHeight, leaningThreshold, LeftBorder, RightBorder);
        }

        // JRROI
        int maxLeaningDistance = std::max(LeftBorder, subROI.width - RightBorder);

        cv::Rect topRefROI =
            CreateROI(cv::Point(subROI.x + maxLeaningDistance, subROI.y), cv::Size(subROI.width - maxLeaningDistance * 2, mHeight), inImg.size());
        if (mEnableCheckTopReferenceLine && !topRefROI.empty())
        {
            auto &topRefResult = dynamic_cast<InspectionResult &>(ispResult);
            topRefResult = FindTopReferenceLine(inImg(topRefROI), ispResult.mCenterContourPos, topRefROI.tl());
        }
        else
        {
            ispResult(EResult::OK, "");
            ispResult.mCenterContourPos = topRefROI.tl() + cv::Point(topRefROI.width / 2, 0);
        }

        ispResult.mLeftLeaningPos = cv::Point(subROI.tl().x + LeftBorder, ispResult.mCenterContourPos.y);
        ispResult.mRightLeaningPos = cv::Point(subROI.tl().x + RightBorder, ispResult.mCenterContourPos.y);

        subROI.x += LeftBorder + mJROffsetX;
        subROI.width -= (LeftBorder + (subROI.width - RightBorder)) + mJROffsetX * 2;
        ispResult.mPoleRoi = subROI;
    }
    else
    {
        ispResult(EResult::ER, "Image type is not supported!");
    }
    return ispResult;
}

auto BatteryInspectorJR::FindLeaning(const cv::Mat &image, cv::Point offset) const -> InspectionResult
{
    return InspectionResult();
}

auto BatteryInspectorJR::FindTopReferenceLine(const cv::Mat &image, cv::Point &center, cv::Point offset) const -> InspectionResult
{
    InspectionResult finalResult;

    if (!image.empty())
    {
        std::vector<int> thresholds = xvt::threshold::ThresholdOtsuMulti(image);
        auto contour = xvt::FindMaxContour(image, thresholds[1], false, xvt::CompareArea);

        if (contour.size() > 0)
        {
            cv::Moments shapeMoments = cv::moments(contour);

            center = cv::Point(shapeMoments.m10 / shapeMoments.m00, shapeMoments.m01 / shapeMoments.m00) + offset;

            for (auto &p : contour)
                p += offset;
            finalResult.mPoints = contour;
            cv::Rect tmpROI = finalResult.GetROI();
            finalResult(EResult::OK, "");
        }
        else
        {
            finalResult(EResult::ER, "FindTopReferenceLine: Can't find any contour");
        }
    }
    else
    {
        finalResult(EResult::ER, "FindTopReferenceLine: Image Empty");
    }

    return finalResult;
}


void BatteryInspectorJRResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    pen.mColor = cv::Scalar(0, 150, 150);
    if (!mPoints.empty())
        cv::drawContours(img, VecVecPoint{mPoints}, -1, pen.mColor, 1);

    if (!IsER())
    {
        pen.mColor = cv::Scalar(150, 150, 0);
        xvt::DrawText(img, "Center", mCenterContourPos, TextAlign::BOTTOM_CENTER, pen, cv::Point(0, -10));
        cv::circle(img, mCenterContourPos, 2, pen.mColor, 3);
    }

    //Draw pole leaning position
    cv::line(img, mLeftLeaningPos, mLeftLeaningPos + cv::Point(0, mPoleRoi.height), cv::Scalar(255, 0, 255));
    cv::line(img, mRightLeaningPos, mRightLeaningPos + cv::Point(0, mPoleRoi.height), cv::Scalar(255, 0, 255));
}

auto BatteryInspectorJRResult::DrawResultStr(cv::Mat &image,
                                             std::string const &name,
                                             CVPen const &pen,
                                             cv::Point const &offset,
                                             bool isDrawOKResult) const -> cv::Point
{
    return cv::Point();
}
}; // namespace battery
}; // namespace xvt
