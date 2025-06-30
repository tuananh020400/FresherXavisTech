#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/imgproc.hpp>
#include <utility>
#include <iostream>

namespace xvt {

//! @addtogroup Drawing
//! @{

/**
* @brief CVPen class stores the drawings information such as: thickness, color, font...
*/
class XVT_EXPORTS CVPen
{
public:
    /**
     * @brief Constructor
     * @param color Drawing color
     * @param thickness Line or text thickness
     * @param fontScale Font size scale from base size
     * @param space Space between each text line
     * @param lineType Drawing line type
     * @param fontFace Font family
    */
    CVPen(cv::Scalar color = cv::Scalar(0, 0, 0)
          , int thickness = 1
          , double fontScale = 0.7
          , int space = 10
          , int lineType = cv::LINE_8
          , int fontFace = cv::FONT_HERSHEY_SIMPLEX // cv::FONT_HERSHEY_PLAIN
    );

    /// <summary>
    /// Get the size of tab in pixel based on the font information
    /// </summary>
    /// <returns>size in pixel</returns>
    int GetTabSize() const
    {
        return GetTextSize().width * mTabSize;
    }

    /// <summary>
    /// Get the size of a character in pixel based on the font information
    /// </summary>
    /// <returns>size in pixel</returns>
    cv::Size GetTextSize() const
    {
        return cv::getTextSize(" ", mFontFace, mFontScale, mThickness, 0);
    }

    /// <summary>
    /// Get the size of the text in pixel based on the font information
    /// </summary>
    /// Font informations:
    /// <see cref="CVPen.mFontFace">font family</see>
    /// <see cref="CVPen.mFontScale">, font size</see>
    /// <see cref="CVPen.mThickness">, font thickness</see>
    /// <param name="text">int put text</param>
    /// <returns>size of the text in pixel</returns>
    cv::Size GetTextSize(std::string const& text) const
    {
        return cv::getTextSize(text, mFontFace, mFontScale, mThickness, 0);
    }

public:
    /// Drawing line thickness
    int mThickness = 1;
    /// Font type
    int mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    /// Font scale (width)
    double mFontScale = 0.7;
#pragma warning(suppress : 4251)
    /// Drawing color
    cv::Scalar mColor = cv::Scalar(255, 0, 0);
    /// How to draw the a line see cv::LineTypes
    int mLineType = cv::LINE_8;
    /// Space between the line of text
    int mSpace = 10;
    /// Size of a tab.
    int mTabSize = 4;
};

/**
 * @brief How to align text position
*/
enum class TextAlign
{
    TOP             = 1,               ///< Align to the top in the vertical direction.
    MIDDLE          = 1 << 1,          ///< Align to the middle in the vertical direction (centered between top and bottom).
    BOTTOM          = 1 << 2,          ///< Align to the bottom in the vertical direction.
    LEFT            = 1 << 3,          ///< Align to the left in the horizontal direction.
    CENTER          = 1 << 4,          ///< Align to the center in the horizontal direction (centered between left and right).
    RIGHT           = 1 << 5,          ///< Align to the right in the horizontal direction.
    TOP_LEFT        = TOP | LEFT,      ///< Align to the top-left corner.
    TOP_CENTER      = TOP | CENTER,    ///< Align to the top and horizontally centered.
    TOP_RIGHT       = TOP | RIGHT,     ///< Align to the top-right corner.
    MIDDLE_LEFT     = MIDDLE | LEFT,   ///< Align to the middle in the vertical direction and to the left in the horizontal direction.
    MIDDLE_CENTER   = MIDDLE | CENTER, ///< Align to the center both vertically and horizontally.
    MIDDLE_RIGHT    = MIDDLE | RIGHT,  ///< Align to the middle in the vertical direction and to the right in the horizontal direction.
    BOTTOM_LEFT     = BOTTOM | LEFT,   ///< Align to the bottom-left corner.
    BOTTOM_CENTER   = BOTTOM | CENTER, ///< Align to the bottom and horizontally centered.
    BOTTOM_RIGHT    = BOTTOM | RIGHT,  ///< Align to the bottom-right corner.
};

/**
 * @brief Check if align match with alignType
 * @param align Align information
 * @return true if they are match, otherwise fasle
*/
template<TextAlign alignType>
bool IsAlign(TextAlign align)
{
    return (static_cast<int>(align) & static_cast<int>(alignType)) == static_cast<int>(alignType);
}

/**
 * @brief Get the size of a character in pixel based on the font information
 * @param text A string
 * @param pen Font information
 * @return Size of text when drawing
*/
inline
auto GetTextSize(std::string const& text, CVPen const& pen)->cv::Size
{
    return cv::getTextSize(text, pen.mFontFace, pen.mFontScale, pen.mThickness, 0);
}

/**
 * @brief Draw the text on the image
 * @param img Draw image.
 * @param text String to be printed on the image.
 * @param p Start point to draw the text
 * @param pen CVPen information
 * @return Bottom right position
*/
XVT_EXPORTS
auto DrawText(cv::Mat& img, std::string const& text, cv::Point p, CVPen const& pen)->cv::Point;

/// @brief Draw the text on the image
/// 
/// TextAlign
/// 
/// ||||
/// |-------:|:---------:|:---------|
/// |TopLeft | TopCenter | Top Right|
/// |MidLeft | MidCenter | Mid Right|
/// |BotLeft | BotCenter | Bot Right|
/// 
/// @param src draw image.
/// @param text string to be printed on the image.
/// @param point start point to draw the text
/// @param pos position of the text: TopLeft  -  TopCenter  -  Top Right
/// @param pen CVPen information
/// @param offset offset value from the points
XVT_EXPORTS
void DrawText(cv::Mat& src, const std::string& text, const cv::Point& point, TextAlign pos, CVPen const& pen, cv::Point offset = cv::Point(0, 0));

/// @brief Draw the list of text on the image.
/// @param src image that text will be drawn on
/// @param textList list of drawing text
/// @param points list of point that text will be drawn at
/// @param pos TextAlign type
/// @param pen CVPen information
/// @param offset offset value from the points
XVT_EXPORTS
void DrawTextList(cv::Mat& src, const std::vector<std::string>& textList, const std::vector<cv::Point>& points, TextAlign pos, CVPen const& pen, cv::Point offset = cv::Point(0, 0));

/**
 * @brief Draw a line on image
 * @param img An image to drawn on
 * @param p1 First point of line
 * @param p2 Second point of line
 * @param pen CVPen information
*/
inline
void DrawLine(cv::Mat& img, cv::Point p1, cv::Point p2, CVPen const& pen)
{
    cv::line(img, p1, p2, pen.mColor, pen.mThickness, pen.mLineType);
}

/**
 * @brief Draw vector of point on image
 * @tparam PointType Type of point
 * @param img An image to drawn on
 * @param pointList List of drawing point
 * @param color Color of point
 * @param offset Offset position
*/
template<class PointType>
void DrawPoints(cv::Mat& img, std::vector<PointType> const& pointList, cv::Vec4b const& color, cv::Point const& offset = cv::Point())
{
    if (img.empty()) return;
    if (img.type() == CV_8UC1)
    {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }
    if (img.type() != CV_8UC3) return;

    auto color3 = cv::Vec3b(color[0], color[1], color[2]);
    if (color[3])
    {
        //double const alpha = std::min(std::max(0, color[3]), 100) / 100.0; //cast the alpha to rang 0-1
        //double const beta = 1.0 - alpha;

        for (auto const& p : pointList)
        {
            cv::Point tmp = cv::Point((int)(p.x + offset.x + 0.5), (int)(p.y + offset.y + 0.5));
            if (tmp.x >= 0 && tmp.x < img.cols && tmp.y >= 0 && tmp.y < img.rows)
            {
                auto& c = img.at<cv::Vec3b>(tmp);
                c[0] = c[0] & color3[0];
                c[1] = c[1] & color3[1];
                c[2] = c[2] & color3[2];
            }
        }
    }
    else
    {
        for (auto const& p : pointList)
        {
            cv::Point tmp = cv::Point((int)(p.x + offset.x + 0.5), (int)(p.y + offset.y + 0.5));
            if (tmp.x >= 0 && tmp.x < img.cols && tmp.y >= 0 && tmp.y < img.rows)
                img.at<cv::Vec3b>(tmp) = color3;
        }
    }
}

/**
 * @brief Draw vector of point on image
 * @tparam PointType Type of point
 * @param img An image to drawn on
 * @param pointList List of drawing point
 * @param color Color of point
 * @param offset Offset position
*/
template<class PointType>
void DrawPoints(cv::Mat& img, std::vector<PointType> const& pointList, cv::Vec3b const& color, cv::Point const& offset = cv::Point())
{
    if (img.empty()) return;
    if (img.type() == CV_8UC1)
    {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }
    if (img.type() != CV_8UC3) return;
    for (auto const& p : pointList)
    {
        cv::Point tmp = cv::Point((int)(p.x + offset.x + 0.5), (int)(p.y + offset.y + 0.5));
        if (tmp.x >= 0 && tmp.x < img.cols && tmp.y >= 0 && tmp.y < img.rows)
            img.at<cv::Vec3b>(tmp) = color;
    }
}


/**@}*/ //end of group drawing
}