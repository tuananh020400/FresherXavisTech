#pragma once

#include "xvtCV/Utils.h"
#include "xvtCV/xvtInspection.h"
#include "xvtCV/xvtDefine.h"
#include "xvtCV/Peak.h"
#include "xvtCV/xvtRange.h"

#define SECRET_CODE 255
#define BORDER_WIDTH_RATIO 0.012

namespace xvt {
namespace battery {
/**
* @addtogroup Utils
* @{
*/

/**
* @brief Enumeration for different types of anode algorithms.
*/
enum class anodeAlgoType
{
    LineTracing, ///< Line tracing algorithm.
    Edge,        ///< Edge detection algorithm.
    Kmean,       
    Projection, 
};

/**
 * @brief Structure representing a point value with intensity and position.
 */
struct pointValue
{
    int intensity;      ///< Intensity value of the point.
    cv::Point position; ///< Position of the point.
    /**
     * @brief Constructor for pointValue.
     *
     * @param i Intensity value (default is 0).
     * @param p Position (default is (0, 0)).
     */
    pointValue(int i = 0, cv::Point p = cv::Point()) : intensity{i}, position{p}{}
};

/**
 * @brief Structure to hold candidate edge properties.
 */
struct candiEdP
{
    int sumDerivative;
    int position;
    int firstPos;
    int countDerivative;
    int weightedSum;
    int var;
};

/**
 * @brief A structure to store the results of calculating the average cathode line.
 *
 * This structure holds the computed local averages for cathode lines, both left and right segments,
 * as well as statistical data (such as sum and size) for further calculations like standard deviation.
 *
 * @tparam T The data type of the cathode line values (typically int or float).
 */
template <typename T>
struct AverageCathodeLineResult
{
    /**
    * @brief The list containing the locally averaged cathode line values.
    */
    std::vector<T> localAverageCathodeLine{};

    /**
    * @brief The list containing the locally averaged cathode line values on the left side.
    */
    std::vector<T> localAverageCathodeLineLeft{};

    /**
     * @brief The list containing the locally averaged cathode line values on the right side.
     */
    std::vector<T> localAverageCathodeLineRight{};

    /**
    * @brief The list used to store cathode line values for mean and standard deviation calculation.
    */
    std::vector<T> lst4MeanStd{};

    /**
    * @brief The number of valid cathode line elements used for averaging.
    */
    int lstCathodeSize = 0;

    /**
     * @brief The sum of all valid cathode line elements.
     */
    double sum = 0;

    /**
    * @brief Indicates whether the calculation of the local average was successful.
    */
    bool success = false;
};

/**
 * @brief Draws a list of text strings with corresponding colors on the provided image, starting from the given position.
 *
 * @param resImg The image on which the text will be drawn.
 * @param startPos The starting position (top-left corner) where the first text string will be drawn.
 * @param textList A vector of pairs where each pair contains a string and its corresponding color (cv::Scalar).
 * @param fontScale Scale factor that is multiplied by the base font size.
 * @param fontWeight Thickness of the lines used to draw the text.
 * @return The position (cv::Point) immediately after the last drawn text.
 */
XVT_EXPORTS
cv::Point drawText( cv::Mat &resImg,
                    cv::Point const &startPos,
                    std::vector<std::pair<std::string, cv::Scalar>> const &textList,
                    float fontScale,
                    float fontWeight);

/**
 * @brief Draws a string of text on the provided image and returns the next position after the text.
 *
 * @param resImg The image on which the text will be drawn.
 * @param pos The starting position (top-left corner) where the text will be drawn.
 * @param str The text string to be displayed.
 * @param color The color of the text in BGR format (cv::Scalar).
 * @param fontScale Scale factor that is multiplied by the base font size.
 * @param fontWeight Thickness of the lines used to draw the text.
 * @return The position (cv::Point) immediately after the drawn text.
 */
XVT_EXPORTS
cv::Point drawText( cv::Mat &resImg,
                    cv::Point const &pos,
                    std::string const &str,
                    cv::Scalar const &color,
                    float fontScale,
                    float fontWeight);

/**
 * @brief Finds a reference line in the blurred image using a specified leaning threshold.
 *
 * @param blurImg The blurred image to search for the reference line.
 * @param leaningThreshold The threshold value for leaning detection.
 * @return The index of the found reference line.
 */
XVT_EXPORTS
int FindRefLine(cv::Mat &blurImg, int leaningThreshold);

/**
 * @brief Gets the current time as a formatted string.
 *
 * @return A string representing the current time.
 */
XVT_EXPORTS
std::string GetStrCurrentTime();

/**
 * @brief Gets the region index based on the provided coordinates.
 *
 * @param xCord The x-coordinate of the pixel to be checked.
 * @param regions The total number of regions to divide the image into (must be even and >= 2).
 * @param imgWidth The width of the entire image.
 * @param midWidth The width of the central region that does not belong to any side regions.
 * @return int The region index (1-based) the pixel belongs to, or 0 if the pixel is out of bounds or the number of regions is invalid.
 */
XVT_EXPORTS
int GetRegionIndex(int xCord, int regions, int imgWidth, int midWidth);

/**
 * @brief Checks which region a pixel's x-coordinate belongs to based on the image width, mid width, and number of regions.
 *
 * This function determines which region of an image a pixel falls into by dividing the image into symmetrical regions on either side of a central region.
 * It accounts for an even number of regions, with half of them on the left side and half on the right side of the central region.
 * If the x-coordinate is within the middle region or the number of regions is invalid (odd or less than 2), the function returns 0.
 * Otherwise, it returns the region index the pixel belongs to, with the regions starting at 1.
 *
 * @param xCord      The x-coordinate of the pixel to be checked.
 * @param regions    The total number of regions to divide the image into (must be even and >= 2).
 * @param imgWidth   The width of the entire image.
 * @param midWidth   The width of the central region that does not belong to any side regions.
 *
 * @return int       The region index (1-based) the pixel belongs to, or 0 if the pixel is out of bounds or the number of regions is invalid.
 */

XVT_EXPORTS
int CheckRegion(int xCord, int regions, int imgWidth, int midWidth);

/**
 * @brief Generates a lookup table (LUT) for contrast stretching between two intensity levels.
 *
 * @param a The first intensity level (between 0 and 255).
 * @param b The second intensity level (between 0 and 255).
 * @return A vector (VecFloat) representing the LUT for transforming pixel values.
 */
XVT_EXPORTS
VecFloat GetLUT(int a, int b);

/**
 * @brief Applies a diagonal filter to an input image, using a combination of Sobel filters and Gaussian kernels.
 *
 * @param src The source image to apply the diagonal filter on.
 * @param ksize Size of the Gaussian kernel to be used.
 * @param sigmaX Standard deviation of the Gaussian filter in the x-direction.
 * @param borderMode Border mode used for extrapolation (e.g., cv::BORDER_DEFAULT).
 * @return The resulting image (cv::Mat) after applying the diagonal filter.
 */
XVT_EXPORTS
cv::Mat DiagonalFilter(const cv::Mat &src, int ksize, double sigmaX, int borderMode = cv::BORDER_DEFAULT);

/**
 * @brief Applies local histogram equalization to enhance contrast in different regions of the input image.
 *
 * @param src The source image on which to apply the equalization.
 * @param region The number of regions to divide the image into for localized processing.
 * @return The resulting image (cv::Mat) after applying local histogram equalization.
 */
XVT_EXPORTS
cv::Mat LocalHistogramEqualization(const cv::Mat &src, const int region);

/**
 * @brief Checks for leaning in the poles of a battery image.
 *
 * This function returns the most left and right pole region positions.
 *
 * @param image Input whole battery image.
 * @param poleRegionHeight Height of the pole region to analyze.
 * @param leaningThreshold Threshold value to determine leaning.
 * @param leftx Reference to store the most left position + x offset. Returns batteryROI.x + padding + xoffset if not found.
 * @param rightx Reference to store the most right position - x offset. Returns batteryROI.x + batteryROI.width - (padding + xoffset) if not found.
 * @return true if leaning is detected, false otherwise.
 */
XVT_EXPORTS
bool CheckPoleLeaning(const cv::Mat &image, int poleRegionHeight, int leaningThreshold, double &leftx, double &rightx);

/**
 * @brief Checks for leaning in the poles of a battery image.
 *
 * This function returns the most left and right pole region positions.
 *
 * @param image Input whole battery image.
 * @param poleRegionHeight Height of the pole region to analyze.
 * @param leaningThreshold Threshold value to determine leaning.
 * @param leftx Reference to store the most left position + x offset. Returns batteryROI.x + padding + xoffset if not found.
 * @param rightx Reference to store the most right position - x offset. Returns batteryROI.x + batteryROI.width - (padding + xoffset) if not found.
 * @return true if leaning is detected, false otherwise.
 */
XVT_EXPORTS
bool CheckPoleLeaningAuto(const cv::Mat &image, int windowSize, int &leftx, int &rightx);

/**
 * @brief Manually checks if the battery pole is leaning using a specified leaning threshold.
 *
 * This function processes the input image using a specified threshold to detect leaning poles. It calculates
 * column sums from the image and applies a threshold to detect the leaning region.
 *
 * @param image The input image (cv::Mat) of the battery pole.
 * @param leaningThreshold The threshold value for detecting leaning. Higher values make the function more sensitive to leaning.
 * @param leftx Reference to an integer that will hold the detected x-coordinate of the left leaning pole.
 * @param rightx Reference to an integer that will hold the detected x-coordinate of the right leaning pole.
 *
 * @return A boolean value indicating whether the pole is leaning (true) or not (false).
 */
XVT_EXPORTS
bool CheckPoleLeaningManual(const cv::Mat &image, int leaningThreshold, int &leftx, int &rightx);

/**
 * @brief Refines the contour of a mask by flattening the bottom line and adjusting the top and bottom shifts.
 *
 * This function processes the input contour by flattening the left and right outer lines,
 * adjusting the bottom and top lines based on specified shifts, and refining the mask contour.
 * The function identifies key points on the left and right sides of the contour and uses
 * them to generate smooth outer lines and shift the contour lines accordingly.
 *
 * @param inContour The input vector of points (VecPoint) representing the contour of the mask.
 * @param imageWidth The width of the image for reference when processing the contour.
 * @param bottomLineShift The amount by which the bottom line of the contour will be shifted.
 * @param TopLineShift The amount by which the top line of the contour will be shifted.
 * @param outerPoint A vector of two points (VecPoint) representing the outermost points on the left and right sides of the contour.
 *
 * @return A vector of points (VecPoint) representing the refined contour after processing.
 */
XVT_EXPORTS
VecPoint refineMaskingContour(const VecPoint& inContour,
                                            int imageWidth,
                                            int bottomLineShift,
                                            int TopLineShift,
                                            VecPoint outerPoint);

/**
 * @brief Finds the closest point in a vector of points based on the given X coordinate.
 *
 * @param vtDst The vector of points to search in.
 * @param xValue The X coordinate to find the closest point to.
 * @param xAfter The reference X coordinate after which points are searched.
 * @param maxRange The maximum allowable range for the search.
 * @return The index of the closest point found. Returns -1 if no point is found within the range.
 */
XVT_EXPORTS
int FindClosestPoint(const VecPoint &vtDst, const int xValue, int xAfter, const int maxRange);

/**
 * @brief Finds the closest point in a vector of integers based on the given X coordinate.
 *
 * @param vtDst The vector of integers to search in.
 * @param xValue The X coordinate to find the closest integer to.
 * @param xAfter The reference X coordinate after which points are searched.
 * @param maxRange The maximum allowable range for the search.
 * @return The index of the closest integer found. Returns -1 if no point is found within the range.
 */
XVT_EXPORTS
int FindClosestPoint(const std::vector<int> &vtDst, const int xValue, int xAfter, const int maxRange);

/**
 * @brief Finds the positions of cathode peaks in a reduced signal vector using prominence and minimum distance criteria.
 *
 * @param reduceVec The input vector containing the reduced signal.
 * @param minProminence The minimum prominence (height difference between peak and neighboring troughs) for detecting peaks.
 * @param polesMinDistance The minimum distance between detected peaks.
 * @return A vector (VecPeak) containing the positions of the detected peaks.
 */
XVT_EXPORTS
std::vector<Peak> FindCathodeXPositionManual(const VecFloat &reduceVec,
                                                             const float &minProminence,
                                                             const float &polesMinDistance);

/**
 * @brief Automatically finds the X positions of cathodes in a signal by analyzing the poles' distances.
 *
 * @param signal The signal vector to be analyzed for cathode X positions.
 * @param polesDistanceRange Output parameter for the range of distances between poles.
 * @param centerNeglectionWidth The width of the signal center to be neglected.
 * @return A vector of detected cathode X positions.
 */
XVT_EXPORTS
std::vector<Peak> FindCathodeXPositionAuto(const VecFloat &signal,
                                                           Rangei &polesDistanceRange,
                                                           const int &centerNeglectionWidth);

/*
@param src input image; the image can have any number of channels, which are processed independently, but the depth should be CV_8U, CV_16U, CV_16S, CV_32F or CV_64F.
@param dim dimension index along which the matrix is reduced. 0 means that the matrix is reduced to a single row. 1 means that the matrix is reduced to a single column.
@param ksize Gaussian kernel ksize can differ but it must be positive and odd.Or, it can be zero's and then they are computed from sigma.
*/

/**
 * @brief Reduces the background of a given image, focusing on non-black regions, and applies a smoothing filter.
 *
 * This function calculates the average pixel value in non-black regions of each row or column
 * (based on the `dim` parameter) in the input image. The result is then smoothed using a Gaussian filter.
 *
 * @param src Input image (cv::Mat) to be processed.
 * @param dim Dimension along which the operation is performed:
 *            - 0 for columns.
 *            - 1 for rows.
 * @param kSize Kernel size for the Gaussian blur filter.
 * @return VecFloat A vector containing the reduced values.
 */
XVT_EXPORTS
VecFloat ReduceBlackBackgroundMat(const cv::Mat &src, int dim, unsigned int kSize = 5);

/**
 * @brief Check the tilt of a line by calculating the derivative and checking for changes in direction.
 *
 * @param line The input vector of points (VecPoint) representing the line.
 * @param radiusPole The threshold for determining significant changes in the line's direction.
 *
 * @return A vector of floats (VecFloat) representing the tilt information of the line.
 */
XVT_EXPORTS
VecFloat CheckTiltedLine(const VecPoint& line, int radiusPole = 1);

/**
 * @brief Compute the average of many image slices.
 *
 * This function takes a vector of images, sums them up, and computes the average image.
 *
 * @param imgs The input vector of images (cv::Mat).
 *
 * @return The resulting average image (cv::Mat).
 */
XVT_EXPORTS
auto AverageSliceImage(const std::vector<cv::Mat>& imgs) -> cv::Mat;

/**
 * @brief Compute the average of slices from a single image (maximum number of slice is 11).
 *
 * This function slices a given image into smaller sections and computes the average of those slices.
 *
 * @param imgs The input image (cv::Mat).
 * @param noSlice The number of slices to divide the image into.
 *
 * @return The resulting averaged image (cv::Mat).
 */
XVT_EXPORTS
auto AverageSliceImage(const cv::Mat& imgs, int noSlice = 1) -> cv::Mat;

/**
 * @brief Get image slices based on the number of slices specifie (maximum number of slice is 11).
 *
 * This function splits an image into smaller sections and returns them as a vector of cv::Mat.
 *
 * @param imgs The input image (cv::Mat).
 * @param noSlice The number of slices to divide the image into.
 *
 * @return A vector of image slices (std::vector<cv::Mat>).
 */
XVT_EXPORTS
auto GetSliceImages(const cv::Mat& imgs, int noSlice = 1) -> std::vector<cv::Mat>;

/**
 * @brief Find peaks and valleys in a given signal.
 *
 * This function smoothens the signal using Gaussian blur, then finds peaks and valleys based on prominence and distance.
 *
 * @param signal The input vector of floats (VecFloat) representing the signal.
 * @param prominence The prominence required for peaks and valleys.
 * @param distance The minimum distance between peaks and valleys.
 * @param ksize The kernel size for the Gaussian blur.
 *
 * @return A vector of peaks and valleys (VecPeak).
 */
XVT_EXPORTS
auto GetPeakAndValley(const VecFloat& signal, float prominence = 0.01, int distance = 0, int ksize = 3) -> VecPeak;

/**
 * @brief Generate an image of peaks and valleys from a source image.
 *
 * This function processes each row of the source image to detect peaks and valleys, outputting an image representing the prominence.
 *
 * @param src The input image (cv::Mat).
 * @param prominence The prominence required for peaks and valleys.
 * @param distance The minimum distance between peaks and valleys.
 * @param ksize The kernel size for Gaussian blur.
 *
 * @return The output image representing peaks and valleys (cv::Mat).
 */
XVT_EXPORTS
auto GetPeakAndValleyImage(const cv::Mat& src, float prominence = 0.01, int distance = 4, int ksize = 3) -> cv::Mat;

/**
 * @brief Calculate Contrast Transfer Function (CTF) from an image.
 *
 * This function calculates the CTF value by analyzing the prominence of peaks and valleys in a reduced vector from the image.
 *
 * @param src The input image (cv::Mat).
 * @param prominence The prominence required for peaks and valleys.
 * @param distance The minimum distance between peaks and valleys.
 * @param ksize The kernel size for Gaussian blur.
 *
 * @return The calculated CTF value (double).
 */
XVT_EXPORTS
auto CalculateCTF(cv::Mat const& src, double prominence = 0.01, int distance = 0, int ksize = 3) -> double;

/**
 * @brief Detect the battery ROI in an image.
 *
 * @param img The input image (cv::Mat).
 * @param thd The threshold used for edge detection.
 * @param corners The output array of corner points (std::array<cv::Point, 4>).
 *
 * @return The result of the battery detection (InspectionResult).
 */
XVT_EXPORTS
auto FindBatteryByTurnPoint(cv::Mat const& img, int thd, std::array<cv::Point, 4>& corners) -> InspectionResult;

/**
 * @brief Find the turn point of a contour in the image.
 *
 * This function detects the turning point of a contour using a specified threshold and minimum length.
 *
 * @param img The input image (cv::Mat).
 * @param thd The threshold for contour detection.
 * @param isLeft Boolean indicating whether to search on the left or right side.
 * @param min_length The minimum length required for detecting a turn.
 *
 * @return A tuple containing two points (cv::Point) representing the turn point, the contour (VecPoint), and a boolean indicating if a turn was found.
 */
XVT_EXPORTS
auto FindTurnPoint(cv::Mat const& img, int thd, bool isLeft, int min_length = 50) -> std::tuple<cv::Point, cv::Point, VecPoint, bool>;

/**
 * @brief Finds the average cathode line from a given list of cathode lines, excluding specified regions.
 *
 * This function processes a list of cathode lines to calculate a local average while neglecting
 * certain columns and regions. It computes the sum and average of valid cathode lines and builds a result structure.
 *
 * @tparam T The data type of the cathode line (typically int or float).
 * @param cathodeLineList The list of cathode line values.
 * @param neglectionWidth The width of the region to neglect on either side of the central columns.
 * @param startNeglectCol The starting column to neglect in the center region.
 * @param endNeglectCol The ending column to neglect in the center region.
 * @param rightEndBounder The right boundary beyond which values are neglected.
 * @param localAverageWindowSize The window size used for local averaging of cathode lines.
 * @param wSize The width size of each segment in the cathode line.
 * @return AverageCathodeLineResult<T> A result object containing the computed averages, sums, and success flag.
 */
template <typename T>
AverageCathodeLineResult<T> FindAverageCathodeLine(std::vector<T> &cathodeLineList,
                                                   int neglectionWidth,
                                                   int startNeglectCol,
                                                   int endNeglectCol,
                                                   int rightEndBounder,
                                                   int localAverageWindowSize,
                                                   int wSize);

/**
 * @brief Refines the cathode line by identifying and correcting outliers.
 *
 * This function detects outliers in the cathode line data based on the standard deviation from the average
 * and replaces outlier values with interpolated or neighboring values.
 *
 * @tparam T The data type of the cathode line (typically int or float).
 * @param avgCathodeLineResult The result structure containing average cathode line data.
 * @param lstCathodeLine The list of cathode line values to be refined.
 * @param wSize The width size of each segment in the cathode line.
 * @param centerLeft The left center boundary index.
 * @param centerRight The right center boundary index.
 */
template <typename T>
void RefineCathodeLine(AverageCathodeLineResult<T> &avgCathodeLineResult,
                       std::vector<T> &lstCathodeLine,
                       int wSize,
                       int centerLeft,
                       int centerRight);
}

/**
 * @brief Calculates the mean and standard deviation of a signal.
 *
 * This utility function computes the mean and standard deviation of a signal using OpenCV's `meanStdDev` function.
 *
 * @tparam T The data type of the signal (typically int or float).
 * @param signal The vector containing the signal values.
 * @return std::pair<double, double> A pair containing the mean and standard deviation of the signal.
 */
template <typename T>
std::pair<double, double> MeanStdDevLite(const std::vector<T>& signal);

XVT_EXPORTS
inline cv::Rect CreateROI(cv::Rect tmp, cv::Size const& imageSize)
{
    RefineROI(tmp, imageSize);
    return tmp;
}

XVT_EXPORTS
bool SaveCSV(std::wstring const& path, std::wstring const& imgName, xvt::CSVOutput const& output);
}
/**@}*/ //end of group Utils

#include "xvtBattery/BatteryUtils.inl.h"