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
* @class CTBatteryInspectorLowerResult
* @brief This class handles the inspection results of CT battery, particularly focusing on drawing results and exporting data.
*
* CTBatteryInspectorLowerResult inherits from InspectionResult and BatteryInspectionResult. It provides functions to
* draw inspection results onto images, format results as strings, and export results to CSV format.
*/
class XVT_EXPORTS CTBatteryInspectorLowerResult : public InspectionResult, public BatteryInspectionResult
{
public:
    /**
    * @brief Draws the inspection result onto the input image.
    *
    * It includes details like battery poles, region of interest (ROI), and specific measurement data.
    *
    * @param img The image on which the results are drawn.
    * @param offSetPoint The starting point where the results will be drawn (default is (0, 0)).
    * @param pen The drawing pen specifying color, thickness, etc. (default is CVPen()).
    */
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    /**
     * @brief Draws the result as text onto the image.
     *
     * @param image The image on which the text result will be drawn.
     * @param name Optional name to be displayed with the result (default is an empty string).
     * @param pen The drawing pen specifying color, font scale, and thickness (default is CVPen()).
     * @param offset The position offset for drawing the text (default is (0, 0)).
     * @param isDrawStrResult Whether or not to draw the string result (default is false).
     * @return The final position of the drawn text.
     */
    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawStrResult = false) const -> cv::Point override;

    /**
     * @brief Exports the inspection data to CSV format.
     *
     * @param out The CSV output object to which the data will be written.
     * @param prefix A prefix string for each result entry (default is an empty string).
     * @param isRecursive Whether to recursively export nested results (default is true).
     */
    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

    /**
     * @brief Draws the text result related to the pole inspection.
     *
     * This method draws detailed text information about the battery pole inspection onto the image.
     * It displays pole positions, measurements between anode and cathode, and any error results.
     *
     * @param resImg The image on which the pole inspection text will be drawn.
     */
    void DrawPoleTextResult(cv::Mat& resImg, CVPen const& pen = CVPen(), cv::Point const& offset = cv::Point()) const;
public:
    cv::Rect mOuterRoi;           ///< The outer region of battery (ROI) in the image.
    cv::Rect mPoleRegionRoi;      ///< The region of interest specific to the poles.
    cv::Point mCenter;            ///< The center point.
    cv::Rect mCenterDetected;     ///< The detected center.
    int mCenterWidth;             ///< The width center Neglection or CenterDetected;
    PoleResult mAnodePoles;       ///< Inspection results related to the anode poles.
    VecPoint mCathodePos;         ///< Positions of the cathode points.
    DisplayMode mDisplayMode = DisplayMode::ALL;  ///< The display mode for showing results.
    double mPixelSize = 1.0;      ///< Pixel size for scaling measurements.
    int sliceIdex;                ///< The index of the slice being inspected.
    VecDouble mCTFs;              ///< Custom data or flags related to the inspection.
    VecString mErrMsg;            ///< List error message.
    BatteryInspectorBeadingResult mBeadingResult;
    int mDirection = 0;
};

/**
 * @class CTBatteryInspectorLower
 * @brief A class that performs lower battery inspection, find the anode and cathode and check the battery OK/NG.
 */

class XVT_EXPORTS CTBatteryInspectorLower
    : public CTBatteryInspectorBase
    , public IInspection
{
public:

    /**
     * @brief Default constructor for the CTBatteryInspectorLower class.
     */
    CTBatteryInspectorLower();

    /**
     * @brief Performs the battery inspection on the given image.
     *
     * This function takes an input image and inspects the battery, returning the results in a CTBatteryInspectorLowerResult object.
     *
     * @param src The input image to be inspected.
     * @return The inspection result.
     */
    auto Inspect(cv::Mat const &src) const -> std::unique_ptr<xvt::IInspectionResult> override;
    auto Inspect(std::vector<cv::Mat> const& srcs) const->std::unique_ptr<IInspectionResult> override { return {}; }
    auto Load(std::wstring const& path) & ->bool override { return false; }
    auto Save(std::wstring const& path)const->bool override { return false; }
    auto Clone() const->std::unique_ptr<IInspection> override { return {}; }
    /**
     * @brief Inspects the battery and outputs the result in the provided result object.
     *
     * This method processes the input image and stores the inspection result in the passed CTBatteryInspectorLowerResult object.
     *
     * @param inImg The input image to be inspected.
     * @param BIresult A reference to a CTBatteryInspectorLowerResult object to store the inspection result.
     * @return An error code indicating success or failure.
     */
    auto Inspect(const cv::Mat& inImg, CTBatteryInspectorLowerResult& BIresult) const -> ERR_CODE;

    /**
     * @brief Inspects average battery images.
     * 
     * This method takes a vector of images and inspects the average battery image, storing the results in the provided CTBatteryInspectorLowerResult object.
     * 
     * @param inImg A vector of input images to be inspected.
     * @param BIresult A reference to a CTBatteryInspectorLowerResult object to store the inspection results.
     * @return An error code indicating success or failure.
     */
    ERR_CODE Inspect(const std::vector<cv::Mat>& inImg, CTBatteryInspectorLowerResult& BIresult) const;

    /**
     * @brief Inspects the cathode within the provided image region.
     *
     * This function processes the input image to detect the cathode position within the specified `subROI` region.
     * It first checks if the input image and the region of interest (ROI) are valid.
     * After that, it applies a convolution filter to the image and extracts the potential cathode peaks from the signal.
     * If sufficient peaks are detected, the function identifies the cathode points and refines their positions based on thresholds.
     * If no valid peaks are found or the image/ROI is invalid, an error message is returned.
     *
     * @param src Input image.
     * @param subROI Region of interest where the cathode should be inspected.
     * @param ispResult Result object storing the detected cathode positions and other inspection details.
     * @return InspectionResult Returns the inspection status and results.
     */
    auto InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorLowerResult& ispResult) const->InspectionResult;

    /**
     * @brief Inspects the anode within the provided image region.
     *
     * This function processes the input image to detect the anode position within the specified `subROI` region.
     * It first checks if the input image and the region of interest (ROI) are valid.
     * If sufficient cathode positions are detected, the function iterates through them,
     * refining the region of interest for each pole and performing peak detection on the processed image.
     * It applies different algorithms based on the configuration to locate the anode.
     * The results are then combined, and validations are performed to ensure the anode's position is within specified ranges.
     * If no valid peaks are found or if the input image/ROI is invalid, an error message is returned.
     *
     * @param src Input image.
     * @param subROI Region of interest for the anode inspection.
     * @param ispResult Result object storing the detected anode positions and other inspection details.
     * @return InspectionResult Returns the inspection status and results.
     */
    auto InspectAnode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorLowerResult& ispResult) const->InspectionResult;

private:
    auto FindAnodeByLineTracing(const cv::Mat &inputImg,
                                VecPoint &anodes,
                                cv::Point startPoint,
                                int limitVal,
                                int stepHorizontal,
                                int stepVertical,
                                float breakThreshold,
                                int borderDistance,
                                bool restrictMove,
                                int moveAllow,
                                int borderCheck = 0) const -> InspectionResult;

    auto FindAnodeByProminence(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode) const->InspectionResult;
    
    auto FindAnodeByShape(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode, cv::Rect& subAnoRoi, int disconnectLimit = 8, int widthAnode = 2, int startRegion = 10) const->InspectionResult;

    void FindEndpointCathodeThreshold(const cv::Mat& src, VecPeak& polePeak, cv::Rect& subROI, CTBatteryInspectorLowerResult& ispResult, VecPoint& midPos, int kSize) const;
public:
    bool mIsRotated = true; ///< Indicates if the battery image is rotated (true) or not (false).

    int mChooseAnodeAlgorithm = 0; ///< Specifies the algorithm to use for anode detection.

    int mSliceNo = 1;
    int mAvgSliceNo = 3;
    double mGamma = 1.0;

    BatteryInspectorBeading mIspBeading = BatteryInspectorBeading(mPixelSize);
};
/**@}*/ //end of group CT
}
}
#endif