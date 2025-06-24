#pragma once
#include "xvtCV/xvtDefine.h"
#include <xvtCV/xvtInspection.h>
#include <opencv2/core/types.hpp>


namespace xvt {


class  LinePairsResult;
/**
*  @brief The class perform measure image quality from linepair image based on LinePairs and Contrast calculation method.
*/
class XVT_EXPORTS LinePairs : public IInspection
{
public:
    LinePairs();
    /**
     * @brief Inspects single images.
     *
     * This method analyzes input images and returns a pointer to the inspection result.
     *
     * @param inImg: Input images to be inspected.
     * @return A unique pointer to the inspection result.
    */
    auto Inspect(cv::Mat const& inImg) const->std::unique_ptr<IInspectionResult> override;
    auto Inspect(std::vector<cv::Mat> const& imgList) const->std::unique_ptr<IInspectionResult> override;
    auto Load(const std::wstring& settingPath) & -> bool override;
    auto Save(std::wstring const& path)const->bool override;
    auto Clone() const->std::unique_ptr<IInspection> override;


public:
    /// Step interval on signal
    int mStep;
    /// Filter peak points based on minProminance
    float mMinProminance;
    /// Filter peak points based on minDistance
    float mMinDistance;
    /// true is black line and false is white line
    bool mLineColor = false;
    /// Number of line pairs
    int mNumOfLinePairs;
    /// An optional offset value for the find peak (default is 0.0f).
    float mOffset = 0.0f;

private:
    /**
    * @brief Calculates the line pair contrast of an input image.
    *
    * This function computes the contrast of line pairs within the given input image.
    * The number of line pairs and an optional offset can be specified.
    *
    * @param inputImage: The input image (cv::Mat) in which the line pair contrast will be calculated.
    * @return void: This function does not return any value.
    */
    auto CalculateLinePairContrast(const cv::Mat& inputImage) const->std::unique_ptr<LinePairsResult>;

};
/**
*  @brief Result of xvt::LinePairs, that storage list contrast value and linepairs per px value
*/
class XVT_EXPORTS LinePairsResult : public InspectionResult
{
public:
    LinePairsResult();
    /**
    *  @brief Convert result line pair per px to line pair per mm based on predefined mPixslSize.
    *  @return std::vector<double>: This function return a vector linepairs per mm.
    */
    auto Convert2LinePairPerMM() const->std::vector<double>;
    /**
        * @brief Generates a CSV result file at the specified file path.
        *
        * This function creates a CSV file and writes the necessary result data to it.
        *
        * @param filePath: The full path, including the file name, where the CSV file will be generated.
        * @return void: This function does not return a value.
        */
    auto GenerateCSVResult(const std::string& filePath) const->void;

public:
    /// List result of contrast
    std::vector<double> mListContrast;
    /// List result of Line Pairs
    std::vector<double> mListLinePair;

    double mPixelSize = 1;
};
}
