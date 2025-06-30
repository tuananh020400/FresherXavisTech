#pragma once
#include "xvtCV/xvtInspection.h"
#include "xvtCV/Peak.h"

namespace xvt {
namespace battery {
/**
* @addtogroup Utils
* @{
*/

/**
* @brief Class that provides information about poles.
*/
class PoleInfo;

/**
 * @brief Class for comparing poles based on their lengths and positions.
 *
 * This class encapsulates the logic for comparing two pole objects,
 * including evaluating their lengths, types, and the potential need for adjustments.
 */
class PoleCompare;

/**
 * @enum PoleType
 * @brief Enumeration of different types of poles.
 *
 * This enum defines the various types of poles that can be detected or classified.
 */
enum class PoleType : int
{
    Real,            ///< Real detected pole
    Incorrect,       ///< Pole detected but marked as incorrect
    Adjusted,        ///< Adjusted pole by copying the left pole to the right pole
    NotFound,        ///< Unable to find the matched pole on the other side
    Inserted,        ///< A new pole has been inserted
    InsertedOutRange ///< The pole is inserted but it is out of range
};

/**
 * @enum PoleCompareStatus
 * @brief Enumeration of pole comparison statuses.
 *
 * This enum indicates the trustworthiness of a matched detected pole based on length differences.
 */
enum class PoleCompareStatus
{
    Untrust = 0, ///< The matched detected pole that has a length difference >= MaxDiff
    Trust = 1,   ///< The matched detected pole that has a length difference < MaxDiff
    Adjusted     ///< The pole has been adjusted
};

/**
 * @struct PoleRemoveInfo
 * @brief Structure to hold information about pole removal.
 *
 * This structure contains the x-coordinate and index of the pole to be removed.
 */
struct PoleRemoveInfo
{
    int pointX; ///< The x-coordinate of the pole to be removed
    int index; ///< The index of the pole in the list
};

/**
 * @brief Linearly interpolates the position of a new pole between two trusted poles.
 *
 * This function computes the position of a reference pole at the given `x` coordinate by
 * linearly interpolating between two trusted poles: `trustedPoleBefore` and `trustedPoleAfter`.
 * The y-coordinates for both the anode and cathode of the new pole are computed based on the
 * relative position of `x` between the x-coordinates of the two trusted poles.
 *
 * @param x The x-coordinate of the new pole to be computed.
 * @param trustedPoleBefore The pole with known values before the desired x-coordinate.
 * @param trustedPoleAfter The pole with known values after the desired x-coordinate.
 * @return PoleInfo The interpolated pole information at the given x-coordinate, with updated
 *         anode and cathode positions, and the type set to `PoleType::Inserted`.
 */
XVT_EXPORTS PoleInfo GetRefPole(float x, PoleInfo const &trustedPoleBefore, PoleInfo const &trustedPoleAfter);

/**
 * @brief Evaluates poles for potential refinement based on their positions and characteristics.
 *
 * This function processes a list of pole positions and their corresponding anode and cathode
 * positions, categorizing them into left and right poles. It fits polynomial models to the poles
 * and evaluates them for potential refinement based on specified distance and difference thresholds.
 * The resulting list of pole comparisons is returned, indicating the relationship between left and
 * right poles, including any interpolated values for missing poles.
 *
 * @param lstPolePosXAll A vector containing the x-coordinates of all poles.
 * @param anodePos A vector containing the y-coordinates of the anodes corresponding to the poles.
 * @param cathodePos A vector containing the y-coordinates of the cathodes corresponding to the poles.
 * @param widthROI The width of the region of interest for pole evaluation.
 * @param maxDis The maximum allowable distance for matching poles.
 * @param maxDiff The maximum allowable difference in pole lengths for evaluation.
 * @param refinement A boolean flag indicating whether to refine the pole evaluations further.
 * @return std::vector<PoleCompare> A vector containing the comparisons of left and right poles,
 *         with their fitted positions and types.
 *
 * @note The function assumes that the input vectors are of equal length and contain valid pole
 *       positions. Polynomial fitting is applied to the left and right poles to derive their
 *       expected anode and cathode positions.
 */
XVT_EXPORTS std::vector<PoleCompare> evaluatePoleForRefinement(const VecInt &lstPolePosXAll, const VecInt &anodePos, const VecInt &cathodePos,
                                                               int widthROI, double maxDis, double maxDiff, bool refinement);

/**
 * @brief Class that represents pole information, including its anode, cathode.
 */
class XVT_EXPORTS PoleInfo : public InspectionResult {
public:

    /**
     * @brief Default constructor for PoleInfo.
     *
     * Initializes the pole with default values.
     *
     * @param anode The position of the anode.
     * @param cathode The position of the cathode.
     * @param type The type of the pole.
     */
    PoleInfo(cv::Point anode = cv::Point(), cv::Point cathode = cv::Point(), PoleType type = PoleType::NotFound);

    /**
     * @brief Constructor for PoleInfo with a Peak value.
     *
     * Initializes the pole with a Peak value along with its anode and cathode.
     *
     * @param peakValue The peak value associated with the pole.
     * @param anode The position of the anode.
     * @param cathode The position of the cathode.
     * @param type The type of the pole.
     */
    PoleInfo(Peak peakValue, cv::Point anode = cv::Point(), cv::Point cathode = cv::Point(), PoleType type = PoleType::NotFound);
    
    /**
     * @brief Get the length of the pole.
     *
     * @return The length of the pole as a double value.
     */
    auto length() const -> double;

public:
    //Type of left and right pair pole
    PoleType mType;     ///< Type of the pole (e.g., Real, Incorrect)
    cv::Point mAnode;   ///< Position of the anode
    cv::Point mCathode; ///< Position of the cathode
    Peak mPeakValue;    ///< Peak value associated with the pole
};

class PoleCompare {
public:

    /**
    * @brief Default constructor for PoleCompare.
    */

    PoleCompare();

    /**
    * @brief Constructor for comparing two poles with a given maximum difference.
    *
    * @param leftPole The left pole to compare.
    * @param rightPole The right pole to compare.
    * @param maxdiff The maximum difference allowed between the poles for them to be considered valid.
    */
    PoleCompare(PoleInfo &&leftPole, PoleInfo &&rightPole, double const &maxdiff);

    /**
     * @brief Get the length error between the left and right poles.
     *
     * @return The absolute difference in lengths between the two poles.
     */
    double GetLengthError();

    /**
     * @brief Set the poles for comparison and evaluate their trustworthiness.
     *
     * @param leftPole The left pole to set.
     * @param rightPole The right pole to set.
     * @param maxdiff The maximum difference allowed between the poles.
     */
    void SetPole(PoleInfo &&leftPole, PoleInfo &&rightPole, double const &maxdiff);


    /**
     * @brief Adjusts uncertain poles by attempting to correct errors based on length differences.
     *
     * @param maxdiff The maximum allowed difference in lengths for adjustments.
     * @return True if an adjustment was made, false otherwise.
     */
    bool AdjustUncertainType(double maxdiff);
public:
    PoleCompareStatus mType; ///< The status of the pole comparison (e.g., Trust, Untrust)
    PoleInfo mLeftPole;      ///< The left pole being compared
    PoleInfo mLeftReferencePole; ///< Reference information for the left pole
    PoleInfo mRightPole;     ///< The right pole being compared
    PoleInfo mRightReferencePole; ///< Reference information for the right pole
};
/**@}*/ //end of group Utils
} // namespace battery
} // namespace xvt
