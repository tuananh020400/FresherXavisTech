#pragma once
#include "xvtCV/Drawing.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include <opencv2/core.hpp>

namespace xvt {

//! @addtogroup Shape
//! @{

/**
 * @brief Represents information about a peak in a signal.
 */
struct Peak
{
    int index=0;            //!< index of the peak in the container
    float value=0.0f;       //!< value of the peak.
    float prominence=0.0f;  //!< prominence of the peak.

    /**
     * @brief Compares two Peak objects.
     *
     * @param other The other Peak object to compare with.
     * @return true if they are the same peak.
     * @return false otherwise.
     */
    bool operator==(Peak const& other) const
    {
        return (index == other.index && value == other.value && prominence == other.prominence);
    }
};

/**
 * @brief Enum class for specifying the type of peak.
*/
enum class PeakType
{
    Valley = -1, //!< Prominence is negative value (valley).
    None = 0,    //!< Not a peak or valley.
    Peak = 1,    //!< Prominence is positive value (peak).
    Both = 2     //!< It can be either a peak or a valley.
};

/**
 * @brief Enum class for specifying the method to find peaks.
 */
enum class PeakFindingMethod
{
      Prominence     = 0 //!< Filter peak by prominence
    , RelativeHeight = 1 //!< Filter peak by relative height
};

using VecPeak = std::vector<Peak>; //!< Vector of peak

/**
 * @brief Class for finding peaks in a signal.
 * 
 * @see @ref PeakFinding
 */
class XVT_EXPORTS FindPeaks
{
public:
    /**
    * @brief Default constructor.
    */
    FindPeaks();

    /**
     * @brief Initializes the FindPeaks object.
     *
     * @param peakType Type of turning point to find: peak or valley.
     * @param findPeaksMethod Method for finding peaks (see #PeakFindingMethod)
     * @param period Estimated period of the signal for calculating relative height (distance).
     * @param includeEndPoints Whether to include the endpoints as peak or valley.
     */
    FindPeaks(PeakType peakType
              , PeakFindingMethod findPeaksMethod = PeakFindingMethod::Prominence
              , int period = 0
              , bool includeEndPoints = false
    );

    /**
     * @brief Runs the Find Peaks algorithm on the given signal.
     *
     * @param signal The input signal in which to find peaks.
     *
     * @see @ref PeakFinding
     */
    void Process(const std::vector<float>& signal);

    /**
    * @brief Sets the type of peak to find.
    *
    * @param peakType The type of peak.
    */
    void SetPeakType(PeakType peakType);

    /**
     * @brief Sets the method for finding peaks.
     *
     * @param peakFindingMethod Method for finding peaks.
     */
    void SetPeakFindingMethod(PeakFindingMethod peakFindingMethod);

    /**
     * @brief Sets the period of the signal.
     *
     * @param period The estimated period of the signal.
     */
    void SetPeriod(int period);

    /**
     * @brief Sets whether to include the endpoints as peaks.
     *
     * @param includeEndPoints Whether to include the endpoints as peaks.
     */
    void SetIncludeEndPoints(bool includeEndPoints);

    /**
    * @brief Gets the current peak type.
    *
    * @return The current #PeakType.
    */
    auto GetPeakType() const -> PeakType;

    /**
     * @brief Gets the current method for finding peaks.
     *
     * @return The current peak #PeakFindingMethod.
     */
    auto GetPeakFindingMethod() const -> PeakFindingMethod;

    /**
     * @brief Gets the period of the signal.
     *
     * @return The current period of the signal setting.
     */
    auto GetPeriod() const -> int;

    /**
     * @brief Gets the filtered peaks based on the given criteria.
     *
     * @param minProminence The minimum prominence of peaks to consider.
     * @param minDistance The minimum distance between peaks.
     * @param peakRefinement Whether to use peak refinement.
     * @return A vector of filtered peaks.
     *
     * @see @ref PeakFinding
     */
    auto GetPeakResult(  float minProminence = 0
                       , float minDistance = 0
                       , float peakRefinement = 0
    ) const->std::vector< Peak >;

    /**
     * @brief Gets whether the endpoints are included as peaks.
     *
     * @return True if endpoints are included, false otherwise.
     */
    bool GetIncludeEndPoints() const;

    /**
     * @brief Returns all peak candidates found in the signal.
     *
     * @return A reference to a vector of peaks.
     */
    auto GetPeaks() & ->std::vector<Peak> const&
    {
        return mPeakList;
    }

    /**
     * @overload
    */
    auto GetPeaks() && ->std::vector<Peak>
    {
        return std::move(mPeakList);
    }
private:
    std::vector<float> mSignal;               //!< The signal in which to find peaks.
    PeakType mPeakType;                       //!< The type of peak to find.
    PeakFindingMethod mPeakFindingMethod;     //!< The method used for finding peaks.
    std::vector<Peak> mPeakList;              //!< List of peaks found.
    int mPeriod;                              //!< The estimated period of the signal.
    bool mIncludeEndPoints;                   //!< Whether to include endpoints as peaks.

    /**
     * @brief Finds peak and valley candidates based on the left and right difference.
     * If diffLeft*diffright > 0 it is a peak.
     *
     * @param peakCandidatesList The output list of peak candidates.
     * @param valleyCandidatesList The output list of valley candidates.
     */
    void FindPeaksCandidate(std::vector<Peak>& peakCandidatesList, std::vector<Peak>& valleyCandidatesList) const;

    /**
     * @brief Finds the left and right indices where reference points around the peak/valley can be found.
     *
     * @param peakPoint The peak/valley for which to find reference points.
     * @param leftClosestSubPeak The closest sub-peak to the left.
     * @param rightClosestSubPeak The closest sub-peak to the right.
     * @return A pair of indices representing the left and right reference points.
     */
    std::pair<int, int> FindTwoIntervals(const Peak& peakPoint, const Peak& leftClosestSubPeak, const Peak& rightClosestSubPeak) const;

    /**
     * @brief Finds the left and right indices where reference points around the peak/valley can be found, based on relative height.
     *
     * @param signal The input signal.
     * @param peaktype The type of peak: Valley or Peak.
     * @param peakPoint The peak/valley for which to find reference points.
     * @param leftClosestSubPeak The closest sub-peak to the left.
     * @param rightClosestSubPeak The closest sub-peak to the right.
     * @param period The estimated period of the signal.
     * @return A pair of indices representing the left and right reference points.
     */
    std::pair<int, int> findTwoIntervalsRelativeHeight(const std::vector<float>& signal, PeakType peaktype, Peak peakPoint, Peak leftClosestSubPeak, Peak rightClosestSubPeak, int period);

    /**
     * @brief Finds the reference point of a peak for calculating prominence or relative height.
     *
     * @param peakPoint The peak/valley for which to find the reference point.
     * @param twoIntervals A pair of indices representing the left and right intervals.
     * @param leftClosestSubPeak The closest sub-peak to the left.
     * @param rightClosestSubPeak The closest sub-peak to the right.
     * @return A pair representing the index and value of the reference point.
     */
    std::pair<int, float> FindKeyCol(const Peak& peakPoint
                                     , std::pair<int, int> twoIntervals
                                     , const Peak& leftClosestSubPeak
                                     , const Peak& rightClosestSubPeak
    ) const;

    /**
     * @brief Finds the reference point of a peak for calculating relative height.
     *
     * @param signal The input signal.
     * @param peaktype The type of peak: Valley or Peak.
     * @param peakPoint The peak/valley for which to find the reference point.
     * @param twoIntervals A pair of indices representing the left and right intervals.
     * @param leftClosestSubPeak The closest sub-peak to the left.
     * @param rightClosestSubPeak The closest sub-peak to the right.
     * @return A pair representing the index and value of the reference point.
     */
    std::pair<int, float> findKeyColRelativeHeight(const std::vector<float>& signal
                                                   , PeakType peaktype
                                                   , Peak peakPoint
                                                   , std::pair<int, int> twoIntervals
                                                   , Peak leftClosestSubPeak
                                                   , Peak rightClosestSubPeak
    );

     /**
     * @brief Removes peaks that are too close to each other based on a given distance.
     *
     * @param vecList The list of peaks.
     * @param minDistance The minimum distance between peaks.
     */
    void RemoveClosePeaks(std::vector<Peak>& filteredPeakList, float minDistance) const;

    /**
     * @brief Removes peaks that prominence are less than a given minProminence.
     * @param filteredPeakList The list of peaks.
     * @param minProminence The minimum prominence of the peaks.
    */
    void RemoveLowProminencePeaks(std::vector<Peak>& filteredPeakList, float minProminence) const;

    /**
     * @brief Find the peak base on the relative height. This is a modified version of #findPeakByProminence
     * @param signal The input signal
     * @param peaktype The type of peak
     * @param minProminence Threshold for consider one as a peak/valley
     * @param minDistance Distance so that the peaks in this range will be removed
     * @param period Assumed period of signal
     * @param p Percent from the peak/valley value for refinding the peak postion. if p==0 it will not perform refind
     * @param includeEndpoints Whether to include the endpoints as peaks
     * @return vector of peak
    */
    std::vector<Peak> findPeakByRelativeHeight(const std::vector<float>& signal
                                               , PeakType peaktype
                                               , float minProminence
                                               , int minDistance
                                               , int period
                                               , float p
                                               , bool includeEndpoints = false
    );

    /**
     * @brief Function to  refine the peak list
     * @param filteredPeakList Ouput filtered peak list
     * @param peakRefinement Percent from the peak/valley value for refinding the peak postion.
     * if p==0 it will not perform refind
    */
    void RefinePeaks(std::vector<Peak>& filteredPeakList, float peakRefinement) const;

    /**
     * @brief Draw prominence graph for debugging
     * @param filteredPeakList The list of peak
    */
    void DrawGraph(std::vector<Peak>& filteredPeakList) const;
};

/**
 * @brief Draws the poles position, signal, and prominence information on the input image.
 * 
 * It mainly supports cylinder project.
 * 
 * @param src Input image where the peaks will be drawn.
 * @param signal Input signal containing the data points.
 * @param lstPeak List of detected peaks to be drawn.
 * @param drawTool Drawing tool used to render the peaks on the image.
 * @param centerNeglectionWidth Optional parameter to neglect peaks in the center of the image.
 * @return Image with the peaks information drawn.
*/
XVT_EXPORTS
cv::Mat DrawPeakInfo(const cv::Mat& src
                     , std::vector<float> const& signal
                     , std::vector<Peak> const& lstPeak
                     , Drawing & drawTool
                     , int centerNeglectionWidth = 0
);

/**
 * @brief Draws the peaks on the input image.
 *
 * @param img Input image where peaks will be drawn.
 * @param sig Signal data points used for detecting peaks.
 * @param lstPeak List of detected peaks.
 * @param pen Drawing pen used to render the peaks on the image.
 * @return Drawing object with the peaks rendered on the image.
 */
XVT_EXPORTS
Drawing DrawPeaks(cv::Mat& img, VecFloat const& sig, VecPeak const& lstPeak, CVPen pen);

/**
 * @brief Comparator to check if the value of the first peak is smaller than the second peak.
 *
 * @param rhs Right-hand side peak.
 * @param lsh Left-hand side peak.
 * @return true if the value of rhs is smaller than lsh.
 * @return false otherwise.
 */
inline
bool SmallerValue(Peak const& rhs, Peak const& lsh)
{
    return rhs.value < lsh.value;
}

/**
 * @brief Comparator to check if the value of the first peak is greater than the second peak.
 *
 * @param rhs Right-hand side peak.
 * @param lsh Left-hand side peak.
 * @return true if the value of rhs is greater than lsh.
 * @return false otherwise.
 */
inline
bool GreaterValue(Peak const& rhs, Peak const& lsh)
{
    return rhs.value > lsh.value;
}

/**
 * @brief Comparator to check if the prominence of the first peak is smaller than the second peak.
 *
 * @param rhs Right-hand side peak.
 * @param lsh Left-hand side peak.
 * @return true if the prominence of rhs is smaller than lsh.
 * @return false otherwise.
 */
inline
bool SmallerProminence(Peak const& rhs, Peak const& lsh)
{
    return rhs.prominence < lsh.prominence;
}

/**
 * @brief Comparator to check if the prominence of the first peak is greater than the second peak.
 *
 * @param rhs Right-hand side peak.
 * @param lsh Left-hand side peak.
 * @return true if the prominence of rhs is greater than lsh.
 * @return false otherwise.
 */
inline
bool GreaterProminence(Peak const& rhs, Peak const& lsh)
{
    return rhs.prominence > lsh.prominence;
}

//! @} end of group Shape
}//namespace xvt