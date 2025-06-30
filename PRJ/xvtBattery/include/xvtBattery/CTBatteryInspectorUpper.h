#pragma once
#include "xvtBattery/CTBatteryInspectorBase.h"
#include "xvtBattery/BatteryInspectorBeading.h"
#if 0
namespace xvt {
namespace battery {

/**
* @addtogroup CT
* @{
*/

/**
* @class CTBatteryInspectorUpperResult
* @brief Class for representing and visualizing inspection results for battery poles.
*
* This class inherits from `InspectionResult` and `BatteryInspectionResult` and provides methods
* to draw inspection results on images, generate CSV output, and display textual results related
* to battery pole inspections.
*/
class XVT_EXPORTS CTBatteryInspectorUpperResult : public InspectionResult, public BatteryInspectionResult
{
public:

    /**
    * @brief Draws the inspection results on the given image.
    *
    * This method overlays the inspection results, anodes, cathodes, and pole information onto the provided image.
    *
    * @param img The image (cv::Mat) to draw results on.
    * @param offSetPoint Optional offset point for positioning results.
    * @param pen Optional CVPen object for customizing drawing properties.
    */
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    /**
     * @brief Draws string of results on the image.
     *
     * @param image The image (cv::Mat) to draw results on.
     * @param name Optional name to display.
     * @param pen Optional CVPen object for customizing text properties.
     * @param offset Optional point for positioning the text.
     * @param isDrawOKResult Flag indicating whether to draw OK results.
     * @return cv::Point The position of the next text to be drawn.
     */
    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawOKResult = false) const -> cv::Point override;

    /**
     * @brief Exports inspection data to CSV format.
     *
     * @param out CSVOutput object for storing the output data.
     * @param prefix Optional prefix for each entry in the output.
     * @param isRecursive Flag indicating whether to include nested data.
     */
    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

    /**
     * @brief Draws text results related to pole inspections on the image.
     *
     * @param resImg The image (cv::Mat) to draw the text results on.
     */
    void DrawPoleTextResult(cv::Mat& resImg, CVPen const& pen = CVPen(), cv::Point const& offset = cv::Point()) const;
public:
    cv::Rect mOuterRoi;         ///< The outer region of battery (ROI) in the image.
    cv::Rect mPoleRegionRoi;    ///< The region of interest specific to the poles.
    cv::Point mCenter;          ///< The center point.
    cv::Point mCaseRef;          ///< The Case Ref.
    cv::Rect mCenterDetected;   ///< The detected center.
    int mCenterWidth;           ///< The width center Neglection or CenterDetected;
    VecPoint mCathodePos;       ///< Positions of the cathode points.
    PoleResult mPoles;       ///< Inspection results related to the anode poles.
    DisplayMode mDisplayMode = DisplayMode::ALL;    ///< The display mode for showing results.
    double mPixelSize = 1.0;    ///< Pixel size for scaling measurements.
    BatteryInspectorBeadingResult mBeadingResult;
    VecString mErrMsg;            ///< List error message.
    Ranged mValidCathode2Case;
};

/**
 * @class CTBatteryInspectorUpper
 * @brief A class that performs upper battery inspection, find the anode and cathode and check the battery OK/NG.
 */
class XVT_EXPORTS CTBatteryInspectorUpper : public CTBatteryInspectorBase
{
public:
    /**
     * @brief Default constructor for the CTBatteryInspectorUpper class.
     */
    CTBatteryInspectorUpper();

    /**
     * @brief Performs the battery inspection on the given image.
     *
     * This function takes an input image and inspects the battery, returning the results in a CTBatteryInspectorUpperResult object.
     *
     * @param src The input image to be inspected.
     * @return The inspection result.
     */
    auto Inspect(cv::Mat const &src) const -> CTBatteryInspectorUpperResult;

    /**
     * @brief Inspects the battery and outputs the result in the provided result object.
     *
     * This method processes the input image and stores the inspection result in the passed CTBatteryInspectorUpperResult object.
     *
     * @param inImg The input image to be inspected.
     * @param BIresult A reference to a CTBatteryInspectorUpperResult object to store the inspection result.
     * @return An error code indicating success or failure.
     */
    ERR_CODE Inspect(const cv::Mat& inImg, CTBatteryInspectorUpperResult& BIresult) const;

    /**
     * @brief Inspects average battery images.
     *
     * This method takes a vector of images and inspects the average battery image, storing the results in the provided CTBatteryInspectorUpperResult object.
     *
     * @param inImg A vector of input images to be inspected.
     * @param BIresult A reference to a CTBatteryInspectorUpperResult object to store the inspection results.
     * @return An error code indicating success or failure.
     */
    ERR_CODE Inspect(const std::vector<cv::Mat>& inImg, CTBatteryInspectorUpperResult& BIresult) const;

    /**
     * @brief Inspects the cathode region of a battery from the provided image.
     *
     * This method performs an inspection of the cathode area in a given battery image. It utilizes
     * image processing techniques, including filtering and peak finding, to identify and verify the
     * position of cathodes within the specified region of interest (ROI).
     *
     * The method processes the input image and refines the region of interest before applying a filter.
     * It then reduces the image to focus on the relevant data, locates cathode positions based on peaks,
     * and may utilize both automatic and threshold-based detection methods to finalize the cathode positions.
     *
     * @param src The source image (cv::Mat) to inspect.
     * @param subROI The region of interest (cv::Rect) within the source image to focus the inspection on.
     * @param ispResult The result object (CTBatteryInspectorUpperResult) to store the inspection results, including found cathode positions.
     *
     * @return InspectionResult The result of the inspection, indicating success or failure and containing any relevant error messages.
     */
    auto InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorUpperResult& ispResult) const->InspectionResult;

public:
    int mSliceNo = 1;
    int mAvgSliceNo = 3;
    double mGamma = 1.0;
    int mBatteryDirection = 0;
    BatteryInspectorBeading mIspBeading = BatteryInspectorBeading(mPixelSize);
};
/**@}*/ //end of group CT
}
}

#endif