#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtFile.h"

#pragma warning(push, 0)
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#pragma warning(pop)

#include <type_traits>
#include <string>

/**
 * @brief Xavis Tech name space
*/
namespace xvt {
//! @addtogroup Utils
//! @{

/// <summary>
/// Check if a ROI is inside the range: x in [0, imgW) and y in [0, imgH)
/// </summary>
/// <param name="x">ROI's x position</param>
/// <param name="y">ROI's y position</param>
/// <param name="w">ROI's width</param>
/// <param name="h">ROI's height</param>
/// <param name="imgW">image's width</param>
/// <param name="imgH">image's height</param>
/// <returns>true if roi is valid, false if it is invalid</returns>
constexpr
bool IsValidROI(int x, int y, int w, int h, int imgW, int imgH)
{
    return (x >= 0 && y >= 0 && w >= 0 && h >= 0 && x < imgW&& y < imgH&& x + w <= imgW && y + h <= imgH);
}

/**
 * @brief Adjusts a selected ROI (Region of Interest) to ensure it remains inside an image.
 * 
 * This function refines the coordinates of a `selectROI` so that it lies within the bounds of
 * an `image`. It does this by adjusting the `selectROI` relative to the top-left corner 
 * of the `image`. The result is stored back in `selectROI`.
 * 
 * @param selectROI The ROI that will be refined to fit inside the parent ROI. It is passed 
 *                 by reference, so it will be modified in place.
 * @param imageSize Size of the image
 * @return true if the refined ROI is valid (i.e., non-empty), false otherwise.
*/
inline
bool RefineROI(cv::Rect& selectROI, cv::Size const& imageSize)
{
    if (imageSize.empty())
    {
        selectROI = cv::Rect();
    }
    else
    {
        cv::Point br = selectROI.br();

        selectROI.x = (std::min)((std::max)(0, selectROI.x), imageSize.width);
        selectROI.y = (std::min)((std::max)(0, selectROI.y), imageSize.height);

        br.x = (std::min)((std::max)(0, br.x), imageSize.width);
        br.y = (std::min)((std::max)(0, br.y), imageSize.height);

        selectROI.width = (std::max)(0, br.x - selectROI.x);
        selectROI.height = (std::max)(0, br.y - selectROI.y);

        if (selectROI.x == imageSize.width) selectROI.x -= 1;
        if (selectROI.y == imageSize.height) selectROI.y -= 1;
    }

    return !selectROI.empty();
}

/**
 * @overload
 * @brief Adjusts a child ROI (Region of Interest) to ensure it remains inside a parent ROI.
 * 
 * This function refines the coordinates of a `childROI` so that it lies within the bounds of
 * a `parentROI`. It does this by adjusting the `childROI` relative to the top-left corner 
 * of the `parentROI`. The result is stored back in `childROI`.
 * 
 * @param childROI The ROI that will be refined to fit inside the parent ROI. It is passed 
 *                 by reference, so it will be modified in place.
 * @param parentROI The parent ROI, which defines the boundary in which the `childROI` 
 *                  must fit. It is passed as a constant reference.
 * @return true if the refined ROI is valid (i.e., non-empty), false otherwise.
 */
inline
bool RefineROI(cv::Rect& childROI, cv::Rect const& parentROI)
{
    auto tl = parentROI.tl();
    auto refineROI = childROI - tl;
    RefineROI(refineROI, parentROI.size());
    childROI = refineROI + tl;

    return !childROI.empty();
}

/// <summary>
/// Create ROI with predefined x, y, w, h
/// and only keep the part of the ROI that is in side the image.
/// </summary>
/// <param name="x">ROI's x position</param>
/// <param name="y">ROI's y position</param>
/// <param name="w">ROI's width</param>
/// <param name="h">ROI's height</param>
/// <param name="imageSize">side of the image</param>
/// <returns>ROI in the image</returns>
inline
auto CreateROI(int x, int y, int w, int h, cv::Size const& imageSize)->cv::Rect
{
    cv::Rect tmp{ x, y , w, h };
    RefineROI(tmp, imageSize);
    return tmp;
}

/// <summary>
/// Create ROI with predefined top left position and size
/// and only keep the part of the ROI that is in side the image.
/// </summary>
/// <param name="p">ROI's top left position</param>
/// <param name="s">ROI's size</param>
/// <param name="imageSize">side of the image</param>
/// <returns>ROI in the image</returns>
inline
auto CreateROI(cv::Point const& p, cv::Size const& s, cv::Size const& imageSize)->cv::Rect
{
    cv::Rect tmp{ p , s };
    RefineROI(tmp, imageSize);
    return tmp;
}

/**
 * @overload
 * @brief Creates a new ROI from given dimensions and ensures it fits inside a parent ROI.
 * 
 * This function constructs an ROI using a given starting point (`p`) and size (`s`).
 * It then refines the ROI to ensure it remains within the bounds of the specified `parentROI`.
 * 
 * @param p The top-left point (cv::Point) of the ROI.
 * @param s The size (cv::Size) of the ROI.
 * @param parentROI The parent ROI that bounds the new ROI.
 * @return The resulting ROI that lies within the bounds of the parent ROI.
 */
inline
auto CreateROI(cv::Point const& p, cv::Size const& s, cv::Rect const& parentROI)->cv::Rect
{
    cv::Rect tmp{ p , s };
    RefineROI(tmp, parentROI);
    return tmp;
}

/**
 * @overload
 * @brief Creates a new ROI from given dimensions and ensures it fits inside a parent ROI.
 * 
 * This function constructs an ROI using the provided x, y, width (w), and height (h)
 * values. It then refines the ROI to ensure it remains within the bounds of the specified 
 * `parentROI`.
 * 
 * @param x The x-coordinate of the top-left corner of the ROI.
 * @param y The y-coordinate of the top-left corner of the ROI.
 * @param w The width of the ROI.
 * @param h The height of the ROI.
 * @param parentROI The parent ROI that bounds the new ROI.
 * @return The resulting ROI that lies within the bounds of the parent ROI.
 */
inline
auto CreateROI(int x, int y, int w, int h, cv::Rect const& parentROI) -> cv::Rect
{
    cv::Rect tmp{ x, y , w, h };
    RefineROI(tmp, parentROI);
    return tmp;
}

/// <summary>
/// Refine the ROI and then return the img(roi)
/// </summary>
/// <param name="img">input image</param>
/// <param name="roi">interested region</param>
/// <returns>the valid image in ROI</returns>
inline
auto GetImage(cv::Mat const& img, cv::Rect& roi)->cv::Mat
{
    xvt::RefineROI(roi, img.size());
    return img(roi);
}

/// <summary>
/// Refine the ROI and then return the img(roi)
/// </summary>
/// <param name="img">input image</param>
/// <param name="roi">interested region</param>
/// <returns>the valid image in ROI</returns>
inline
auto GetImage(cv::Mat const& img, cv::Rect&& roi)->cv::Mat
{
    xvt::RefineROI(roi, img.size());
    return img(roi);
}

/// <summary>
/// Calculate the distance between two openCV points.
/// distance = | p1 - p2 |
/// </summary>
/// <typeparam name="T">Type of points</typeparam>
/// <param name="p1"> First point</param>
/// <param name="p2">Second point</param>
/// <returns>the distance between two points in double</returns>
template <class T>
inline
auto Distance(T const& p1, T const& p2)->double
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

/**
 * @brief Get the distance from the 2D point p1 to p2.
 * 
 * distance = |p1-p2|
 * @tparam T point type that have member .x, .y
 * @param p1 2D point(x,y)
 * @param p2 2D point(x,y)
 * @return length from point to origin
 * @note Can use cv::norm to get the length.
*/
template <class T>
inline
auto GetLength(T const& p1, T const& p2)->double
{
    T d = p2 - p1;
    return sqrt((double)d.x * d.x + (double)d.y * d.y);
}

/**
 * @brief Get the distance from the 2D point to origin (length of vector).
 * 
 * distance = |p|
 * @tparam T point type that have member .x, .y
 * @param p 2D point(x,y)
 * @return length from point to origin
 * @note Can use cv::norm to get the length.
*/
template <class T>
inline
auto GetLength(T const& p)->double
{
    return sqrt((double)p.x * p.x + (double)p.y * p.y);
}

/**
 * @brief Convert an image to the 8 bit gray image (8UC1).
 * 
 * @param src Input image of RBG, 16bits(16UC1), float (32FC1) image to 8UC1 image
 * @param dst Output 8 bit gray image (8UC1)
 * @param isClone whether dst image is copy image of src
 *
 * @return 0 if converting successfully, there are no error
 * @return 1 if image is empty 
 * @return 2 if image type is not supported
*/
XVT_EXPORTS
auto Convert8Bits(const cv::Mat& src, cv::Mat& dst, bool isClone = true)->int;

/**
 * @brief Convert an image to the 8 bits RGB image.
 * @param src An input image of 8UC3, 16UC1, 8UC1 image to 8UC3
 * @param dst Output color 8UC3(RBG) image 
 * @param isClone when src is U8C3, isClone=true dst=src.clone() otherwise dst=src (reference data)
 * @return 0 if converting successfully, there are no error
 * @return 1 if image is empty 
 * @return 2 if image type is not supported
*/
XVT_EXPORTS
auto ConvertRGB(const cv::Mat& src, cv::Mat& dst, bool isClone = true) -> int;

/**
 * @brief Read an image from a unicode path.
 * @param filePath input image unicode path 
 * @param flags How to read image, see cv::ImreadModes
 * @return Read image
*/
XVT_EXPORTS
auto ReadImage(std::wstring const& filePath, int flags = 2)->cv::Mat;

/**
 * @brief Read an image from a unicode path. User should provide the width, height, and type of image.
 * @param filePath input image unicode path 
 * @param w width of image
 * @param h height of image
 * @param type deepth of a pixel such: 8bits-CV_8UC1, 16bits-CV_16UC1, float-CV_32FC1
 * @return Read image
*/
XVT_EXPORTS
auto ReadImageRaw(std::wstring const& filePath, int w, int h, int type)->cv::Mat;

/**
 * @overload 
 * @brief Read an image from a unicode path. User should provide the width, height, and type of image.
 * @param filePath input image unicode path 
 * @param n number of image should be read
 * @param w width of image
 * @param h height of image
 * @param type deepth of a pixel such: 8bits-CV_8UC1, 16bits-CV_16UC1, float-CV_32FC1
 * @return vector of image
 * @note should check size of output vector to make sure the output is correct.
*/
XVT_EXPORTS
auto ReadImageRaw(std::wstring const& filePath, int n, int w, int h, int type)->std::vector<cv::Mat>;

/**
 * @brief Write an image to a unicode path.
 * @param filePath an unicode path 
 * @param image cv::Mat image to write
*/
XVT_EXPORTS
void WriteImage(std::wstring const& filePath, cv::Mat const& image);

/// Get unqiue points in the list of point.
/// it use LexicoCompare to sort the points.
/// points will be sorted after funtion call.
XVT_EXPORTS
void RemoveDuplicatePoint(std::vector<cv::Point>& points);

/// Function to find points where each point has only one neighbor in list of neighbor directions
XVT_EXPORTS
auto FindPointsWithOneNeighbor(const std::vector<cv::Point>& points, const std::vector<cv::Point>& directions)->std::vector<cv::Point>;

/**
 * @brief Lexicographic compare, same as for ordering words in a dictionnary.
 * It prefers to compare by x.
 * 
 * Test first 'letter of the word' (x coordinate), if same then test 
 * second 'letter' (y coordinate).
 * 
 * @param p1 first point
 * @param p2 second point
 * @return true if p1.x < p2.x else if p1.x == p2.x return p1.y < p2.y
*/
XVT_EXPORTS
bool LexicoCompare(const cv::Point& p1, const cv::Point& p2);

//! Round a float number
inline
double round_f(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

template<class T>
inline
/**
 * @brief Transform point by a matrix
 * @tparam T Point type
 * @param p A point
 * @param m Transform matrix
 * @return Transformed point
*/
auto Transform(cv::Point_<T> const& p, cv::Mat const& m)->cv::Point_<T>
{
    cv::Point_<T> tmp = p;
    if (!m.empty())
    {
        tmp.x = m.at<double>(0, 0) * p.x + m.at<double>(0, 1) * p.y + m.at<double>(0, 2);
        tmp.y = m.at<double>(1, 0) * p.x + m.at<double>(1, 1) * p.y + m.at<double>(1, 2);
    }
    return tmp; 
}

template<class T>
inline
/**
 * @brief Transform list of point by a matrix
 * @tparam T Point type
 * @param src List of point
 * @param m Transform matrix
 * @return Transformed points
*/
auto Transform(std::vector<cv::Point_<T>> const& src, cv::Mat const& m)->std::vector<cv::Point_<T>>
{
    std::vector<cv::Point_<T>> dst;
    if (m.empty())
    {
        dst = src;
    }
    else if(!src.empty())
    {
        if(m.rows==2)
            cv::transform(src, dst, m);

        if (m.rows == 3)
            cv::perspectiveTransform(src, dst, m);
    }

    return dst;
}

template<class T>
inline
/**
 * @brief Shift list of point by shift value
 * @tparam T Point type
 * @param src List of point
 * @param shift Shift value
 * @return Transformed points
*/
auto Transform(std::vector<cv::Point_<T>> const& src, cv::Point_<T> const& shift)->std::vector<cv::Point_<T>>
{
    std::vector<cv::Point_<T>> dst;
    if (!src.empty())
    {
        dst.reserve(src.size());
        for (auto p : src)
        {
            dst.push_back(p + shift);
        }
    }

    return dst;
}

template<class T>
inline
/**
 * @brief Shift list of point by shift value
 * @tparam T Point type
 * @param src List of point
 * @param shift Shift value
 * @return Transformed points
*/
auto Transform(std::vector<std::vector<cv::Point_<T>>> const& src, cv::Point_<T> const& shift)->std::vector<std::vector<cv::Point_<T>>>
{
    std::vector<std::vector<cv::Point_<T>>> dst;
    for (auto& c : src)
    {
        dst.emplace_back(xvt::Transform(c, shift));
    }
    return dst;
}

template<class T>
inline
/**
 * @brief Transform point by a matrix
 * @tparam T Point type
 * @param src List of point
 * @param m Transform matrix
 * @return Transformed points
*/
auto Transform(std::vector<std::vector<cv::Point_<T>>> const& src, cv::Mat const& m)->std::vector<std::vector<cv::Point_<T>>>
{
    std::vector<std::vector<cv::Point_<T>>> dst;
    for (auto& c : src)
    {
        dst.emplace_back(xvt::Transform(c, m));
    }
    return dst;
}

template<class T>
inline
/**
 * @brief Rotate a list of point
 * @tparam T Point type
 * @param points List of point
 * @param imgSize image size
 * @param angle Rotated angle
*/
void RotatePoints(std::vector<cv::Point_<T>>& points, cv::Size imgSize, double angle)
{
    imgSize -= cv::Size(1, 1);
    cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(imgSize.width / 2.0f, imgSize.height / 2.0f), angle, 1.0);
    rotMat.at<double>(0, 2) += -imgSize.width / 2.0f + imgSize.height / 2.0f;
    rotMat.at<double>(1, 2) += imgSize.width / 2.0f - imgSize.height / 2.0f;
    points = Transform(points, rotMat);
}

/**
 * @brief Find all max element in the list
 * @tparam ForwardIt Forward itterator
 * @tparam Compare Compare function
 * @param first Begining itterator
 * @param last Ending itterator
 * @param com Compare function
 * @return vector of pointer that have max value in the list
*/
template<typename ForwardIt, typename Compare >
auto FindAllMax(ForwardIt first, ForwardIt last, Compare com)->std::vector<ForwardIt>
{
    std::vector<ForwardIt> tmp;
    if (first != last)
    {
        ForwardIt largest = first++;
        tmp.emplace_back(largest);
        for (; first != last; ++first)
        {
            if (com(*largest, *first))
            {
                tmp.clear();
                largest = first;
                tmp.emplace_back(first);
            }
            else if (!com(*first, *largest))
            {
                tmp.emplace_back(first);
            }
        }
    }
    return tmp;
}

//! output image = |Sobel(X)|*wx + wy*|Sobel(Y)| and the output is signed 16bits type (CV_16S)
XVT_EXPORTS
auto Sobel(const cv::Mat& inputImage, float wx = 0.5f, float wy = 0.5f)->cv::Mat;

//! output image = |Sobel(X)|*wx + wy*|Sobel(Y)| and the output is double type (64F)
XVT_EXPORTS
auto Sobel(const cv::Mat& inputImage, float wx, float wy, int kSize, int order)->cv::Mat;

//! @} end of group Utils

}//xvt
