#pragma once

#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtInspection.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace xvt {

class CircleResult;
class Line;

/**
 * @brief Computed Tomography name space. All function/class related to CT.
*/
namespace ct {

enum class SliceBlurType
{
    Angle = 0
   ,ParrallelLine  = 1
};

// Angle selector's result
class AngleSelectorResult;
using VecMat = std::vector<cv::Mat>;
using Volume = VecMat;

/// <summary>
/// The angle selector class, that detect the angle in the X-Ray image of cylinder battery,
/// based on the variant of intensity
/// </summary>
class XVT_EXPORTS AngleSelector{
public:
    void GetCenter(cv::Mat const& img, cv::Point2f center) const;

    auto GetAngle(cv::Mat const& img, AngleSelectorResult& res) const-> void;

    auto GetSliceImage(Volume const& volume, int zSliceIdx, float r = -1.f) const->AngleSelectorResult;
public:

    // Finding the angle by variance
    int   mAngleNo = 2;
    float mAngleStart= 0.f;
    float mAngleEnd = 360.f;
    float mAngleStep = 1.f;

    float mVarianceThreshold = 0.f;
    int   mVarianceBlurSize = 15;

    // Setting for pixel distance
    float mRadiusStep=1.0f;

    // Setting for finding the volume center
    int   mCenterThreshold = 0;
    int   mCircleThreshold = 0;

    // Setting for average image
    float mAvgNo = 3;
    float mAvgStep = 0.5f;
};

/**
 * @brief Angle selector result class
*/
class XVT_EXPORTS AngleSelectorResult :public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    void DrawResult(cv::Mat& zImg, cv::Mat& sideImg, int zSliceIdx) const;

public:
    float              mRadius=0;
    float              mAvgNo =0;
    cv::Point2f        mCenter;
    cv::Mat            mSliceImage;
    std::vector<float> mAngleList;      //list of angle in degree
    std::vector<float> mVarianceVec;

};

/**
 * @brief Get the fitting circle by binary image
 * @param img Input image
 * @param threshold Threshold value to binary image. If 0 OTSU will be use.
 * @return Fitted circle
*/
XVT_EXPORTS
auto GetCircle(cv::Mat const& img, int threshold)->xvt::CircleResult;

/**
 * @brief Get the slice image by the list of points
 * @param volume volume image (3D)
 * @param points Position to get the values
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceImage(Volume const& volume, xvt::VecPoint2f const& points)->cv::Mat;

/**
 * @overload
 * @brief Get the slice image by line information
 * @param volume Volume image (3D)
 * @param l A line to cut the slice image
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceImage(Volume const& volume, xvt::Line const& l)->cv::Mat;

/**
 * @overload
 * @brief Get the slice image by two angles direction from center
 * @param volume Volume image (3D)
 * @param center Reference postion
 * @param angle1 First angle in degree
 * @param angle2 Second angle in degree
 * @param size How many point from center will be get following the angle direction
 * @param scale Movement scale of unit vector in angle direction.
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceImage(Volume const& volume
                   , cv::Point2f center
                   , std::vector<float> angles
                   , int   size = 0
                   , float scale = 1.f
)->cv::Mat;

/**
 * @brief Get the slice image by angles direction from center
 * @param volume Volume image (3D)
 * @param center Reference postion
 * @param angle Angle in degree
 * @param size How many point from center will be get following the angle direction
 * @param scale Movement scale of unit vector in angle direction.
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceImage(Volume const& volume
                   , cv::Point2f center
                   , float angle
                   , int   size
                   , float scale = 1.f
                   , bool isHalf = false
)->cv::Mat;

/**
 * @brief Get the slice image by the segment between two points
 * @param volume Volume image (3D)
 * @param start First point
 * @param dir direction vector
 * @param size Number of point from start position
 * @param scale Movement scale of unit vector in angle direction.
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceImage(Volume const& volume
                   , cv::Point2f start
                   , cv::Point2f dir
                   , int         size
                   , float       scale = 1.f
)->cv::Mat;

XVT_EXPORTS
auto GetSliceMPRAngle(Volume const& volume
                      , cv::Point2f center
                      , float       angle
                      , int         size
                      , float       d
                      , float       step
                      , bool        cvt28bits = true
                      , bool        isHalf=false
)->cv::Mat;

/**
 * @brief Get the slice image by average (blur) multiple slices
 * @param volume Volume image (3D)
 * @param start First position
 * @param end Second position
 * @param d Distance to get the slice
 * @param step Distance resolution
 * @param cvt28bits if output image will be converted to CV_8UC1
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceMPRLine(Volume const& volume
                      , cv::Point2f start
                      , cv::Point2f dir
                      , int         size
                      , float       d
                      , float       step=1.f
                      , bool        cvt28bits = true
)->cv::Mat;

/**
 * @brief Get the slice image by average (blur) multiple slices
 * @param volume Volume image (3D)
 * @param angle Direction to get the slice
 * @param center Reference postion
 * @param size How many point from center will be get following the angle direction
 * @param k Angle/distace to get the slice
 * @param type Medthode to get the slices
 * @param step angle/distance resolution
 * @param cvt28bits if output image will be converted to CV_8UC1
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceMPR(Volume const& volume
                      , cv::Point2f center
                      , float angle
                      , int   size
                      , float k
                      , SliceBlurType type = SliceBlurType::ParrallelLine
                      , float step = 1.f
                      , bool  cvt28bits = true
)->cv::Mat;

/**
 * @brief Get the slice image by average (blur) multiple slices
 * @param volume Volume image (3D)
 * @param center Reference postion
 * @param angle1 First angle in degree
 * @param angle2 Second angle in degree
 * @param size How many point from center will be get following the angle direction
 * @param angle Angle to get the slices from angle1 and angle2.
 * @param step Angle resolution
 * @param cvt28bits if output image will be converted to CV_8UC1
 * @return Slice image
*/
XVT_EXPORTS
auto GetSliceMPR(Volume const& volume
                 , cv::Point2f center
                 , std::vector<float> angleList
                 , int   size
                 , float angle
                 , SliceBlurType type
                 , float step = 1.f
                 , bool  cvt28bits = true
)->cv::Mat;

class XVT_EXPORTS Slicer
{
public:
    Slicer() = default;
    Slicer(cv::Point2f     c
           , float         angle
           , int           size
           , float         blurSize
           , float         blurStep = 1.0f
           , SliceBlurType blurType = SliceBlurType::ParrallelLine
           , bool          cvt8bit = false
    );

    auto GetSlice(Volume const& volume)const->cv::Mat;
    auto GetSlice(Volume const& volume, float angle)&->cv::Mat;
    void Draw(cv::Mat& img, CVPen pen) const;

public:
    cv::Point2f   mCenter;
    float         mAngle = 0;
    int           mSize = 0;

    float         mBlurSize = 5;
    float         mBlurStep = 1.f;
    SliceBlurType mBlurType = SliceBlurType::ParrallelLine;

    bool          mIsConvert8bit = false;
};

}
}