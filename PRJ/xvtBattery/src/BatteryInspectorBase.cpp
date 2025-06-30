#include "xvtBattery/BatteryInspectorBase.h"
#include "xvtCV/Contour.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/Utils.h"
#include <xvtCV/ScopeTimer.h>

namespace xvt
{
namespace battery
{
auto BatteryInspectorBase::Inspect(cv::Mat const &img) const -> BatteryInspectorBaseResult
{
    cv::Mat src;

    auto ispResult = BatteryInspectorBaseResult();
    auto &findBatteryResult = dynamic_cast<InspectionResult &>(ispResult);

    auto imgSize = img.size();
    cv::Rect ROI = GetInspectROI(imgSize);

    if (!xvt::Convert8Bits(img, src, false))
    {
        cv::Mat binaryImg;
        int thresholdType = mIsInvert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
        if (mThreshold == 0)
        {
            thresholdType |= cv::THRESH_OTSU;
        }

        cv::threshold(src(ROI), binaryImg, mThreshold, 255, thresholdType);
        //if(mEnableMedian) cv::medianBlur(binaryImg, binaryImg, 9);

        if (mEnableMorp)
        {
            cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
            cv::morphologyEx(binaryImg, binaryImg, cv::MORPH_CLOSE, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);
        }

        if(mUseTwoSides)
        {
            findBatteryResult = FindBatteryTwoSides(binaryImg);
        }
        else
        {
            findBatteryResult = FindBattery(binaryImg);
        }

        if(findBatteryResult.IsOK())
        {
            double mbatteryROIWidth = round_f(findBatteryResult.GetROI().width * mPixelSize, 2);
            if (mValidBatteryWidthRange.IsInRange(mbatteryROIWidth))
            {
                findBatteryResult.SetResult(EResult::OK);
            }
            else
            {
                findBatteryResult(EResult::NG, "FindBattery: Battery width out of range w=" + std::to_string(mbatteryROIWidth));
            }
            ispResult.mPoints = xvt::Transform(ispResult.mPoints, ROI.tl());
        }
    }
    else
    {
        ispResult(EResult::ER, "Image type is not supported!");
    }

    ispResult.mRoiSetting = ROI;

    return ispResult;
}

auto BatteryInspectorBase::FindBattery(cv::Mat const &img) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!img.empty())
    {
        auto contour = FindMaxContour(img, 0, false, CompareRectArea, cv::CHAIN_APPROX_NONE);
        
        if (!contour.empty())
        {
            auto boundRect = cv::boundingRect(contour);
            finalResult.SetROI(boundRect);
            finalResult.mPoints = std::move(contour);
            finalResult.SetResult(EResult::OK);
        }
        else
        {
            finalResult.SetMsg("FindBattery: Cannot find contour");
        }
    }
    else
    {
        finalResult.SetMsg("FindBattery: Image Empty");
    }
    return finalResult;
}

auto BatteryInspectorBase::FindBatteryTwoSides(cv::Mat const& img) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!img.empty())
    {
        cv::Size sizeImg = img.size();
        cv::Rect leftRect = CreateROI(0, 0, sizeImg.width/2, sizeImg.height, sizeImg);
        auto leftResult = FindBattery(img(leftRect));
        auto leftCornerPoints = FindCornerPointsIdx(leftResult.mPoints);
        if(leftCornerPoints.size() == 4)
        {
            int idxTopLeft;
            int idxBottomLeft;
            idxTopLeft = leftCornerPoints[0];
            idxBottomLeft = leftCornerPoints[3];
            if(idxTopLeft > idxBottomLeft) idxTopLeft = 0;

            //finalResult.mPoints.insert(finalResult.mPoints.begin(), leftResult.mPoints.begin() + idxTopLeft, leftResult.mPoints.begin() + idxBottomLeft);
            finalResult.mPoints.insert(finalResult.mPoints.begin(), leftResult.mPoints.begin(), leftResult.mPoints.end());
        }

        cv::Rect rightRect = CreateROI(leftRect.br().x, 0, sizeImg.width/2, sizeImg.height, sizeImg);
        auto rightResult = FindBattery(img(rightRect));
        auto rightCornerPoints = FindCornerPointsIdx(rightResult.mPoints);
        if(rightCornerPoints.size() == 4)
        {
            int idxTopRight;
            int idxBottomRight;
            idxTopRight = rightCornerPoints[1];
            idxBottomRight = rightCornerPoints[2];
            if(idxTopRight < idxBottomRight) idxTopRight = rightResult.mPoints.size() - 1;
            rightResult.mPoints = xvt::Transform(rightResult.mPoints, rightRect.tl());
            //finalResult.mPoints.insert(finalResult.mPoints.end(), rightResult.mPoints.begin() + idxBottomRight, rightResult.mPoints.begin() + idxTopRight);
            finalResult.mPoints.insert(finalResult.mPoints.end(), rightResult.mPoints.begin(), rightResult.mPoints.end());
        }

        if(!finalResult.mPoints.empty())
        {
            auto boundRect = cv::boundingRect(finalResult.mPoints);
            finalResult.SetROI(boundRect);
            finalResult.SetResult(1);
        }
        else
        {
            finalResult(EResult::ER, "FindBattery: Can't find corner points");
        }
    }
    else
    {
        finalResult(EResult::ER, "FindBattery: Image Empty");
    }
    return finalResult;
}

auto BatteryInspectorBase::GetInspectROI(cv::Size imgSize) const -> cv::Rect
{
    cv::Rect ROI = mRoi.empty() ? cv::Rect(cv::Point(0, 0), imgSize) : xvt::CreateROI(mRoi, imgSize);
    if (mDirection == 1)
    {
        cv::Point topLeft(ROI.x, ROI.y);
        cv::Point bottomRight(ROI.x + ROI.width, ROI.y + ROI.height);

        cv::Point center(imgSize.width / 2, imgSize.height / 2);

        float pX = center.x - (topLeft.x - center.x);
        float pY = center.y - (topLeft.y - center.y);
        topLeft = cv::Point(pX, pY);

        pX = center.x - (bottomRight.x - center.x);
        pY = center.y - (bottomRight.y - center.y);
        bottomRight = cv::Point(pX, pY);

        ROI = CreateROI(cv::Rect(topLeft, bottomRight), imgSize);
    }
    return ROI;
}

void BatteryInspectorBaseResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    pen.mColor = GetResultColor();
    cv::Rect subRoi = !mRoiSetting.empty() ? mRoiSetting : cv::Rect(cv::Point(0, 0), img.size());
    cv::Scalar color(150, 150, 0);
    if ((IsER() || IsNG()) && !mPoints.empty() && !subRoi.empty())
    {
        cv::drawContours(img(subRoi), VecVecPoint{mPoints}, -1, pen.mColor, 1);
        //cv::rectangle(img, GetROI(), color, 2);
    }

    //Draw top right and bottom right
    int cornerSize = 15;
    auto mCornerPoints = GetCornerPoint();
    if (!mCornerPoints.empty() && !subRoi.empty())
    {
        xvt::DrawPlusSign(img, mCornerPoints[0] + offSetPoint + subRoi.tl(), color, pen.mThickness, cornerSize);
        xvt::DrawPlusSign(img, mCornerPoints[1] + offSetPoint + subRoi.tl(), color, pen.mThickness, cornerSize);
        xvt::DrawPlusSign(img, mCornerPoints[2] + offSetPoint + subRoi.tl(), color, pen.mThickness, cornerSize);
        xvt::DrawPlusSign(img, mCornerPoints[3] + offSetPoint + subRoi.tl(), color, pen.mThickness, cornerSize);
    }

    if (!mRoiSetting.empty())
        cv::rectangle(img, mRoiSetting, COLOR_CV_ORANGE, 3);
    ;
}

} // namespace battery
} // namespace xvt
