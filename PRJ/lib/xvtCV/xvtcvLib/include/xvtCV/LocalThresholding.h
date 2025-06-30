#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>

namespace xvt {
/// @brief Set of global and local thresholding fucntions and classes.
namespace threshold {
//! @addtogroup Thresholding
//! @{

/// <summary>
/// Niblack recommends K_VALUE = -0.2 for images with black foreground objects
/// and K_VALUE = +0.2 for images with white foreground objects.
/// <para>
/// Reference: Niblack W. (1986) "An introduction to Digital Image Processing" Prentice-Hall.
/// </para>
/// </summary>
/// <param name="src">input image</param>
/// <param name="dst">output image</param>
/// <param name="windowsSize">local window size</param>
/// <param name="K"></param>
/// <param name="r"></param>
void XVT_EXPORTS ThresholdNiBlack(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.3, int r = 128);

/// <summary>
/// Adaptive Document Image Binarization Pattern Recognition.
/// Sauvola recommends K_VALUE = 0.5 and R_VALUE = 128.
/// </summary>
/// <para>
/// reference: 
/// <a href="https://www.sciencedirect.com/science/article/abs/pii/S0031320399000552">
/// Sauvola J. and Pietaksinen M. (2000) Adaptive Document Image Binarization Pattern Recognition, 33(2): 225-236
/// </a>
/// </para>
/// <param name="src">[in] input image</param>
/// <param name="dst">[out] binary image</param>
/// <param name="windowsSize">[in] window size</param>
/// <param name="K"></param>
/// <param name="r"></param>
void XVT_EXPORTS ThresholdSauvola(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.25, int r = 128);

/// <summary>
/// Wolf binarization thresholding.
/// </summary>
/// <param name="src">input image</param>
/// <param name="dst">ouput image</param>
/// <param name="windowsSize">window size</param>
/// <param name="K"></param>
/// <param name="r"></param>
void XVT_EXPORTS ThresholdWolf(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.1, int r = 128);

/// <summary>
/// Nick binarization thresholding.
/// </summary>
/// <param name="src">input image</param>
/// <param name="dst">ouput image</param>
/// <param name="windowsSize">window size</param>
/// <param name="K"></param>
/// <param name="r"></param>
void XVT_EXPORTS ThresholdNick(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.1, int r = 128);

/// <summary>
/// Run all the thresholds for testing purpose.
/// </summary>
/// <param name="src">input image</param>
/// <param name="size">window size</param>
void XVT_EXPORTS TryAllLocalThreshold(cv::Mat const& src, cv::Size size);

/**@}*/ //end of group Thresholding

}//Threshold
}//cvt