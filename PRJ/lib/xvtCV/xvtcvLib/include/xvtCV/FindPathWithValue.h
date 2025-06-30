#pragma once
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include "opencv2/core/types.hpp"
namespace xvt
{
//! @addtogroup Utils
//! @{

/**
 * @brief Find the shortest path using the Pixel value.
 * @param image The input grayscale 8-bit image.
 * @param start The starting point for the path.
 * @param end The target or end point for the path.
 * @param directions Searching directions
 * @param pixelThreshold The threshold value for considering pixels in the pathfinding process.
 * @return A vector of points representing the shortest path.
*/
XVT_EXPORTS
auto FindPathWithPixelValue(  const cv::Mat& image
                            , const cv::Point& start
                            , const cv::Point& end
                            , std::vector<cv::Point> const& directions
                            , int pixelThreshold=255
)->std::vector<cv::Point>;

//! @} end of group Utils
}
