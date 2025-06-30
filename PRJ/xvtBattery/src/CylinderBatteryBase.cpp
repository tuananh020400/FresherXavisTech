#include "xvtBattery/CylinderBatteryBase.h"

using namespace xvt::enhance;
namespace xvt {
namespace battery {

void CylinderBatteryBase::setGamma(float _gamma)
{
    _gamma = std::max<float>(0.0f, std::min<float>(4.0f, _gamma / 10.0f));
    mGamma.SetGamma(_gamma);
}

CylinderBatteryBase::CylinderBatteryBase()
{
    mTextPosition = cv::Point(0, 150);

    // Debug mode
    mDisplayMode = DisplayMode::ALL;
    mGamma = GammaCorrector();
}

ERR_CODE CylinderBatteryBase::Inspection(const cv::Mat& src, BatteryInspectionResult& BIresult)
{
    BatteryInspectionResult iResult;
    ERR_CODE resValue = Inspection2(src, iResult);

    if ((int)resValue > (int)ERR_CODE::NG) {

        cv::Mat resImg;
        cv::cvtColor(src, resImg, cv::COLOR_GRAY2BGR);

        cv::Rect settingROI(mRoi);
        cv::rectangle(resImg, settingROI, cv::Scalar(0, 255, 255), 3);

        cv::Point errOutPosition = cv::Point(100, resImg.rows - 200);

        std::string description = ToString(resValue) + " " + iResult.Description;
        cv::putText(resImg, "Result: NG", cv::Point(640, 100), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
        cv::putText(resImg, description, errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        iResult.resImg = resImg;
    }

    BIresult = iResult;
    return resValue;
}

void InspectingItem::SetALL(bool flag)
{
    SetBeadingALL(flag);
    ANODE_TO_CATHODE_LENGTH = flag;
    ANODE_TO_CASE_GAP = flag;
    CATHODE_TO_CASE_GAP = flag;
    CHECK_BEADING = flag;
    CHECK_LEANING = flag;
    FIND_JR_ROI = flag;
    CHECK_CENTER_PIN = flag;
}

void InspectingItem::SetBeadingALL(bool flag)
{
    COVER_HEIGHT = flag;
    INNER_DIAMETER = flag;
    OUTER_DIAMETER = flag;
    GROOVE_DEPTH = flag;
    GROOVE_HEIGHT = flag;
}

void InspectingItem::SetPolesInspect(bool flag)
{
    ANODE_TO_CATHODE_LENGTH = flag;
    ANODE_TO_CASE_GAP = flag;
    FIND_JR_ROI = flag;
    CHECK_LEANING = flag;
}

void InspectingItem::SetUpperInspectDefail()
{
    SetBeadingALL(true);
    CHECK_BEADING = true;
    CHECK_LEANING = true;
    ANODE_TO_CATHODE_LENGTH = true;
    ANODE_TO_CASE_GAP = true;
    CATHODE_TO_CASE_GAP= false;
    FIND_JR_ROI = true;
    CHECK_CENTER_PIN = false;
}

void InspectingItem::SetLowerInspectDefail()
{
    SetBeadingALL(false);
    CHECK_BEADING = false;
    CHECK_LEANING = true;
    ANODE_TO_CATHODE_LENGTH = true;
    ANODE_TO_CASE_GAP = true;
    CATHODE_TO_CASE_GAP = false;
    FIND_JR_ROI = true;
    CHECK_CENTER_PIN = false;
}

void InspectingItem::SetJRInspectDefail()
{
    SetBeadingALL(false);
    CHECK_BEADING = false;
    CHECK_LEANING = false;
    ANODE_TO_CATHODE_LENGTH = true;
    ANODE_TO_CASE_GAP = true;
    CATHODE_TO_CASE_GAP = false;
    FIND_JR_ROI = false;
    CHECK_CENTER_PIN = false;
}

}
}