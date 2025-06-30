#pragma once
#include "xvtCV/xvtInspection.h"
#include "xvtCV/PixelRef.h"
#include "xvtCV/xvtRange.h"
#include "xvtBattery/BatteryUtils.h"

#pragma region Setting Battery Base
#define BATTERY_BASE_SETTING_SECTION                   L"INSPECT BATTERY BASE"
#define BATTERY_BASE_ENABLE                            L"Battery Enable()"
#define BATTERY_BASE_ROI                               L"Battery ROI(px)"
#define BATTERY_BASE_THRESHOLD                         L"Battery Threshold(0~255)"
#define BATTERY_BASE_THRESHOLD_INVERT                  L"Threshold Invert"
#define BATTERY_BASE_VALID_WIDTH_RANGE                 L"Battery Valid Width Range(mm)"
#pragma endregion

namespace xvt
{
namespace battery
{
/**
* @addtogroup Base
* @{
*/
class XVT_EXPORTS BatteryInspectorBaseResult : public InspectionResult
{
public:
    /**
     * @brief 
     * @param img 
     * @param offSetPoint 
     * @param pen 
     */
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

public:
    cv::Rect mRoiSetting{};
};

/**
 * @brief Battery Inspector Base
 * 
 */
class XVT_EXPORTS BatteryInspectorBase : public PixelRef
{
public:
    explicit BatteryInspectorBase(double const &pxSize) : PixelRef{pxSize}
    {
    }
    explicit BatteryInspectorBase(double &&pxSize) = delete;

    /**
     * @brief Inspect Battery Base
     * 
     *
     * @param src input array of 8-bit or 16-bit elements.
     * @return BatteryInspectorBaseResult
     */
    auto Inspect(cv::Mat const &src) const -> BatteryInspectorBaseResult;

    /**
     * @brief Find Battery object
     * @param img 
     * @param threshold 
     * @param autoMode 
     * @return 
     */
    auto FindBattery(cv::Mat const &img) const -> InspectionResult;

    auto FindBatteryTwoSides(cv::Mat const &img) const -> InspectionResult;

    auto GetInspectROI(cv::Size imgSize) const->cv::Rect;
public:
    bool mEnable = true;
    bool mEnableMedian = true;
    bool mEnableMorp = true;
    bool mIsInvert = true;
    bool mUseTwoSides = true;

    cv::Rect mRoi = {};

    // Battery Direction
    int	mDirection = 1;

    //The rotate angle
    double mRotateAngle = 0.0;

    //Threshold to find the battery ROI, 0 is auto OTSU, other is manual threshold
    int mThreshold = 0;

    // Valid Cell Width Range[mm]
    Ranged mValidBatteryWidthRange = Ranged(0, 100);
};

/**@}*/ //end of group Base

} // namespace battery
} // namespace xvt
