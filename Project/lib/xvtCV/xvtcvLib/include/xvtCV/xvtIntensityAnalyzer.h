#pragma once
#include <opencv2/core/types.hpp>
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include <xvtCV/xvtPen.h>
#include <xvtCV/xvtInspection.h>
#include <string>
#include <vector>
#include <iostream>
#include <numeric>

namespace xvt {
//! @addtogroup Utils
//! @{

class RoiInfo;
class IntensityAnalyzerResult;

/**
 * @brief Get the intensity of list of points that is inside the image by bilinear interpolation.
 * The intensity of point that is out side of the image will not be output.
 * @warning just support uchar(8UC1) image becareful when using it
 * @param image Input 8UC1 image.
 * @param points List of points
 * @return List of intensity of point that is inside the image.
*/
XVT_EXPORTS
auto GetIntensity(const cv::Mat1b& image, std::vector<cv::Point2f>const& points, bool isKeepOuter = false)->std::vector<uchar>;

/**
 * @brief Get the intensity of list of points that is inside the image by bilinear interpolation.
 * The intensity of point that is out side of the image will not be output.
 * @warning just support ushort(16UC1) image becareful when using it
 * @param image Input 8UC1 image.
 * @param points List of points
 * @return List of intensity of point that is inside the image.
*/
XVT_EXPORTS
auto GetIntensity(const cv::Mat1w& image, std::vector<cv::Point2f>const& points, bool isKeepOuter = false)->std::vector<ushort>;

XVT_EXPORTS
auto GetIntensity(const cv::Mat1f& image, std::vector<cv::Point2f>const& points, bool isKeepOuter = false)->std::vector<float>;

XVT_EXPORTS
auto GetPoints(cv::Mat const& img, cv::Point2f start, cv::Point2f dir, int size, float scale, bool isKeepOuter = false)->std::vector<cv::Point2f>;

/**
 * @brief Get the list of points in the line from start to end points and point is inside image.
 * 
 * The length of the line segment: l = |start-end|
 * The unit direction vector of line(start, end) is: v = |start-end| / l
 * The distance between each point is: step = radiusStep * |v|
 * The number of point output will be: n = l / step
 * Point ith: p_ith = start + step*i
 * 
 * @param img Image that contain the line
 * @param start Start point
 * @param end End point
 * @param scale The scale value to unit direction vector of line start-end
 * @param isKeepOuter if it is true, the point is outside of image will be keep.
 * Otherwise it will be removed.
 * @return List of points
*/
XVT_EXPORTS
auto GetPoints(cv::Mat const& img, cv::Point2f start, cv::Point2f end, float scale = 1.f, bool isKeepOuter=false)->std::vector<cv::Point2f>;

/**
 * @overload
 * @brief Get the list of points in the line from center follow in the angle direction
 * @param img Image that contain the line
 * @param center Point that start to calculation
 * @param angle Direction of points
 * @param size How many point will be calculated
 * @param scale Unit movement, Point next = point previous + unit vector * scale
 * @param isKeepOuter if it is true, the point is outside of image will be keep.
 * Otherwise it will be removed.
 * @return List of points
*/
XVT_EXPORTS
auto GetPoints(cv::Mat const& img, cv::Point2f center, float angle, int size, float scale = 1.f, bool isKeepOuter=false)->std::vector<cv::Point2f>;

/**
 * @brief Get the variance of the list of data
 * @tparam T data type
 * @param data list of data
 * @return the variance of the data
 * @return 0 if the data is empty
*/
template<typename T>
auto GetVariance(const std::vector<T>& data)->double;

/**
 * @brief The Class handle to compute: average, minimum, maximum, standart variance intensity of an image.
*/
class XVT_EXPORTS IntensityAnalyzer : public IInspection
{
public:
    /**
     * @brief Colect the xvt::RoiInfo of the input image
     * @param inImg Input image
     * @return Intensity statistical information
    */
    auto Inspect(cv::Mat const& inImg) const->std::unique_ptr<IInspectionResult> override;
    auto Inspect(std::vector<cv::Mat> const& imgList) const->std::unique_ptr<IInspectionResult> override;

    /**
     * @brief Load the setting ROI from file
     * @return true if load successfully
     * @return false otherwise
     * @note The data format as follow: x,y,w,h
     *       each line will store one rect value.
     *       if can not load any ROI information the inspection will be disbaled.
     */
    auto Load(const std::wstring& settingPath) & -> bool override;
    auto Save(std::wstring const& path)const->bool override;

    auto Clone() const->std::unique_ptr<IInspection> override;
public:
    /// list of ROI need to collect the intensity information
    VecRect mRoiList;
    /// Enable inspection
    bool mEnable    = true;
    /// Convert intensity to 8UC1.
    bool mIsUse8Bit = true;
};

/**
 * @brief Struct to store the intenisty information.
 * average, minimum, maximum, standart variance intensity
*/
struct XVT_EXPORTS RoiInfo
{
    /// Setting ROI
    cv::Rect mSetRoi{};
    /// ROI used to caculate the intensity
    cv::Rect mUseRoi{};
    /// Average intensity in mUseRoi.
    double mAvg=0.0;
    /// Min intensity in mUseRoi.
    double mMin=0.0;
    /// Max intensity in mUseRoi.
    double mMax=0.0;
    /// Standart variance intensity in mUseRoi.
    double mStd=0.0;

    /**
     * @brief Function to generate data to csv format.
     * @param out ouput csv Data format
     * @param prefix addition information to the key string.
     * @param isRecursive Get the child data.
     * @see xvt::CSV
    */
    auto GetCSVData(VecKeyValueStr& out, std::string prefix = "", bool isRecursive = true)const->void;
};

/**
 * @brief Result of xvt::IntensityAnalyzer, that storage the xvt::RoiInfo inspection result.
*/
class XVT_EXPORTS IntensityAnalyzerResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    virtual auto GetCSVData(VecKeyValueStr& out, std::string prefix = "", bool isRecursive = true)const->void override;

    virtual auto GetResultStr() const->std::string override;

public:
    /// Inspected ROI.
    std::vector<RoiInfo> mRois;
};

template<typename T>
auto GetVariance(const std::vector<T>& data)->double
{
    double variance = 0.0;
    if (data.empty()) return variance;

    double mean = 0.0;
    mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    for (auto const& value : data)
    {
        auto error = value - mean;
        variance += error * error;
    }

    variance /= data.size();
    return variance;
}

using UniquePtrIntensityResult = std::unique_ptr<xvt::IntensityAnalyzerResult>;
using SharedPtrIntensityResult = std::shared_ptr<xvt::IntensityAnalyzerResult>;

//! @} end of group Utils

}
