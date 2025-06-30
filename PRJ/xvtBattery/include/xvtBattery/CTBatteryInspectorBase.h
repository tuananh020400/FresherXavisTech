#pragma once

#include "xvtCV/xvtInspection.h"
#include "xvtCV/xvtProperty.h"
#include "xvtCV/xvtTable.h"
#include "xvtBattery/CylinderUtils.h"
#include "xvtBattery/CylinderBatteryBase.h"
#include "xvtBattery/CylinderBatteryResult.h"
#include "xvtBattery/PoleInfo.h"
#include "xvtBattery/BatteryInspectorBase.h"
#include "xvtBattery/BatteryInspectorBeading.h"
#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryInspectorPole.h"
#include "xvtBattery/BatteryInspectorCathode.h"
#include "xvtBattery/BatteryInspectorAnode.h"
#include "xvtBattery/BatteryUtils.h"

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
class XVT_EXPORTS CTBatteryInspectorResult
    : public InspectionResult
    , public BatteryInspectionResult
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
    void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

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
    auto DrawResultStr(cv::Mat& image,
                       std::string const& name = "",
                       CVPen const& pen = CVPen(),
                       cv::Point const& offset = cv::Point(),
                       bool isDrawStrResult = false) const->cv::Point override;

    /**
     * @brief Exports the inspection data to CSV format.
     *
     * @param out The CSV output object to which the data will be written.
     * @param prefix A prefix string for each result entry (default is an empty string).
     * @param isRecursive Whether to recursively export nested results (default is true).
     */
    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive = true)const->void override;

    /**
     * @brief Draws the text result related to the pole inspection.
     *
     * This method draws detailed text information about the battery pole inspection onto the image.
     * It displays pole positions, measurements between anode and cathode, and any error results.
     *
     * @param resImg The image on which the pole inspection text will be drawn.
     */
    void DrawPoleTextResult(cv::Mat& resImg, CVPen const& pen = CVPen(), cv::Point const& offset = cv::Point()) const;

    void SaveDetailsCSV(std::wstring path);

    /// CSV support functions
    virtual auto GetTitleStr()const->std::string override
    {
        return std::string("CT Battery Inspection Result\n");
    }
private:
    void DrawValidCaseLineResult(cv::Mat& resImg, CVPen pen = CVPen()) const;

    void DrawPoleResult(cv::Mat& resImg) const;

    void UpdateTableData() const;
public:
    cv::Rect mOuterRoi;           ///< The outer region of battery (ROI) in the image.
    cv::Rect mPoleRegionRoi;      ///< The region of interest specific to the poles.
    cv::Point mCenter;            ///< The center point.
    cv::Rect mCenterDetected;     ///< The detected center.
    int mCenterWidth;             ///< The width center Neglection or CenterDetected;
    PoleResult mAnodePoles;       ///< Inspection results related to the anode poles.
    VecPoint mCathodePos;         ///< Positions of the cathode points.
    VecDouble mXCathodePos = {};
    DisplayMode mDisplayMode = DisplayMode::ALL;  ///< The display mode for showing results.
    double mPixelSize = 1.0;      ///< Pixel size for scaling measurements.
    int sliceIdex;                ///< The index of the slice being inspected.
    VecDouble mCTFs;              ///< Custom data or flags related to the inspection.
    VecString mErrMsg;            ///< List error message.
    BatteryInspectorBeadingResult mBeadingResult;
    int mDirection = 0;
    int nNoCathodeLeft = 0;
    double mGamma = 0.0;
    int mOneSidePoleNo = 0;
    int minLenghtIdx = -1;
    int maxLenghtIdx = -1;
    std::array<cv::Point, 4> mCornerPoint;
    int mDecimalNo = 2;

private:
    mutable Table mPoleTableL;
    mutable Table mPoleTableR;
    mutable Table mPoleNoTableL;
    mutable Table mPoleNoTableR;
};

/**
* @class CTBatteryInspector
* @brief Base class to include common functions for Upper and Lower to perform battery component inspection and detection.
*/
class XVT_EXPORTS CTBatteryInspector : public IInspection
{
public:
    CTBatteryInspector();

    auto Inspect(cv::Mat const& src) const->std::unique_ptr<xvt::IInspectionResult> override;
    auto Inspect(std::vector<cv::Mat> const& srcs) const->std::unique_ptr<IInspectionResult> override { return {}; }
    auto Load(std::wstring const& path) & ->bool override { return false; }
    auto Save(std::wstring const& path)const->bool override { return false; }
    auto Clone() const->std::unique_ptr<IInspection> override { return {}; }

    auto Inspect2(cv::Mat const& src) const->std::unique_ptr<xvt::IInspectionResult>;

    /**
     * @brief Inspects the battery and outputs the result in the provided result object.
     *
     * This method processes the input image and stores the inspection result in the passed CTBatteryInspectorLowerResult object.
     *
     * @param inImg The input image to be inspected.
     * @param BIresult A reference to a CTBatteryInspectorLowerResult object to store the inspection result.
     * @return An error code indicating success or failure.
     */
    auto Inspect(const cv::Mat& inImg, CTBatteryInspectorResult& BIresult) const->ERR_CODE;

    /**
     * @brief Inspects average battery images.
     *
     * This method takes a vector of images and inspects the average battery image, storing the results in the provided CTBatteryInspectorLowerResult object.
     *
     * @param inImg A vector of input images to be inspected.
     * @param BIresult A reference to a CTBatteryInspectorLowerResult object to store the inspection results.
     * @return An error code indicating success or failure.
     */
    auto Inspect(const std::vector<cv::Mat>& inImg, CTBatteryInspectorResult& BIresult) const -> ERR_CODE;

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
    auto InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorResult& ispResult) const->InspectionResult;

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
    auto InspectAnode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorResult& ispResult) const->InspectionResult;

    /**
     * @brief Finds the endpoint of the cathode automatically using the input image and pole peak data.
     *
     * This method processes the input image to detect the cathode endpoint, applying gamma correction,
     * Gaussian kernel smoothing, peak and valley detection, and reference position adjustment.
     *
     * @param inputImg The input image (cv::Mat) to process.
     * @param polePeak Vector of peaks indicating the pole positions.
     * @param enableReference Boolean flag to enable reference position adjustment.
     * @param kernelSize Size of the Gaussian kernel.
     * @param thicknessPole Thickness of the pole for detection.
     * @param disVerifi Distance verification threshold.
     * @param numVariance Variance value used to smooth the results.
     * @return VecPoint A vector of cv::Point containing the cathode endpoints.
     */
    auto FindEndpointCathodeAuto(const cv::Mat& inputImg,
                                 VecPeak& polePeak,
                                 bool enableReference = true,
                                 int thicknessPole = 3,
                                 int disVerifi = 10,
                                 float numVariance = 0.1
    ) const -> VecPoint;

    /**
     * @brief Finds the cathode position by reducing the image and analyzing pixel differences.
     *
     * @param src The source image.
     * @return int The cathode position (y-coordinate).
     */
    auto FindCathodePos(cv::Mat const& src) const-> int;

    /**
     * @brief Finds the top reference line in the given image based on battery ROI.
     *
     * @param src The source image.
     * @param batteryROI Region of interest representing the battery.
     * @param batteryCenter Output parameter representing the center of the battery.
     * @return InspectionResult The result of the top reference line detection.
     */
    auto FindTopReferenceLine(const cv::Mat& src, const cv::Rect& batteryROI, cv::Rect& batteryCenter) const->InspectionResult;

    //Remove the Cathode pos that has the position close to anode with distance d
    //If d<0 the function is skiped
    //only search around 3 pos in start, mid, end
    void RemoveCathodeByAnode(CTBatteryInspectorResult& ispResult, float d = 1.f) const;

protected:
    ERR_CODE VerifyParameters(BatteryInspectionResult& BIresult) const;

    auto FindCathodeXPositionAuto(const VecFloat& signal, xvt::Rangei& polesDistanceRange, int centerNeglectionWidth, xvt::PeakType type) const -> VecPeak;

    auto FindMode(VecInt numVector) const -> int;

    auto GetPeakResult(xvt::FindPeaks& findPeaks, VecFloat reduceVec, float prominenceTh, float& minDis, float& maxDis, float alpha) const -> VecPeak;

private:
    auto FindAnodeByLineTracing(const cv::Mat& inputImg,
                                VecPoint& anodes,
                                cv::Point startPoint,
                                int limitVal,
                                int stepHorizontal,
                                int stepVertical,
                                float breakThreshold,
                                int borderDistance,
                                bool restrictMove,
                                int moveAllow,
                                cv::Point offset = cv::Point(0,0),  
                                int borderCheck = 0
    ) const->InspectionResult;

    auto FindAnodeByProminence(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode) const->InspectionResult;

    auto FindAnodeByShape(const cv::Mat& inputImg,
                          VecPoint& anode,
                          cv::Point& startAnode,
                          cv::Rect& subAnoRoi,
                          int disconnectLimit = 8,
                          int widthAnode = 2,
                          int startRegion = 10
    ) const->InspectionResult;

    void FindEndpointCathodeThreshold(const cv::Mat& src,
                                      VecPeak& polePeak,
                                      cv::Rect& subROI,
                                      CTBatteryInspectorResult& ispResult,
                                      int kSize
    ) const;

    VecInt CheckDominantPoles(const VecFloat& avgInt, float firstThresh = 1.5, float lastThresh = 1.5, int limitIntensity = 200) const;

    // Remove poles that have intensity bigger than near poles
    void RemovePoles(const cv::Mat& img, CTBatteryInspectorResult& ispResult) const;

    auto RefineCathodeLine(const cv::Mat& img, cv::Rect centerROI, VecPoint& ispResult) const -> VecPoint;

    void RefineAnodePoles(CTBatteryInspectorResult& ispResult) const;

    void GetCathodeLeftRight(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const;

    void AddMissingAnodeEndingPoles(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const;

    void RemoveNoiseAnodeEndingPoles(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const;

    void RemoveCathodeCenterPoles(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const;

    //if it is cathode, the difference intensity of bottom part and top part will be big
    void RemoveCathodeByIntensity(cv::Mat const& img, CTBatteryInspectorResult& ispResult, float d = 1.f) const;

    auto RemoveTab(cv::Mat & img, int tabThd=0) const -> xvt::VecVecPoint;
public:
    bool mEnable = true;

    bool mIsRotated = true; ///< Indicates if the battery image is rotated (true) or not (false).

    int mChooseAnodeAlgorithm = 0; ///< Specifies the algorithm to use for anode detection.

    int mSliceNo = 1;
    int mAvgSliceNo = 3;
    int mDecimalNo = 2;
    double mGamma = 1.0;

    /** @brief Pixel size for image processing (default is 1.0). */
    double mPixelSize = 1.0;

    /** @brief Flag to enable cathode reference position adjustment. */
    bool mCathodeEnableReference = false;

    /** @brief Gaussian kernel size for cathode detection. */
    int mCathodeKernelSize = 3;

    /** @brief Thickness of the cathode pole for detection. */
    int mCathodeThicknessPole = 3;

    /** @brief Distance verification threshold for cathode detection. */
    int mCathodeDistanceVerifi = 15;

    /** @brief Variance threshold used for cathode result smoothing. */
    float mCathodeNumVariance = 0.1;

    /** @brief List of inspecting items. */
	InspectingItem mInspectingItems;

    BatteryInspectorBase    mIspBase    = BatteryInspectorBase(mPixelSize);
    BatteryInspectorBeading mIspBeading = BatteryInspectorBeading(mPixelSize);
    BatteryInspectorJR      mIspJR      = BatteryInspectorJR(mPixelSize);
    BatteryInspectorPole    mIspPole    = BatteryInspectorPole(mPixelSize);
    BatteryInspectorCathode mIspCathode = BatteryInspectorCathode(mPixelSize, mIspPole, mIspJR);
    BatteryInspectorAnode   mIspAnode   = BatteryInspectorAnode(mPixelSize, mIspPole, mIspJR);
    // Debug mode

    /** @brief Display mode for showing various inspection results. [1: Show Outer ROI, 2: Show Pole Grid, 4: Show Text, 7: Show All ] */
    DisplayMode mDisplayMode = DisplayMode::ALL;

    CVPen mDisplayPen;

    /** @brief Position of the text result display. */
    cv::Point mTextPosition = cv::Point(0, 150);
};
/**@}*/ //end of group CT
}
}