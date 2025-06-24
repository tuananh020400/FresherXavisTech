#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtInspection.h"
#include "xvtCV/xvtRange.h"
#include "xvtCV/xvtRansac.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace xvt {
//! @addtogroup Shape
//! @{

/**
 * @brief Line structure.
 * 
 * A line is represent by an equation: wx*x+wy*y+wr=0 (ax+by+c=0).
 * This class supports to handle all related line functions.
*/
class XVT_EXPORTS Line
{
public:
    Line() = default;

    /**
     * @brief Constructs a line from an angle and a distance.
     * @param angle Angle between the normal vector of the line and the x-axis in radians.
     * @param distance Distance from the origin to the line.
    */
    Line(double angle, double distance);

    /**
     * @brief Constructs a line from the coefficients of the line equation ax + by + c = 0.
     * @param a Coefficient a.
     * @param b Coefficient b.
     * @param c Coefficient c.
    */
    Line(double a, double b, double c);

    /**
     * @brief Constructs a line from two points or one point and the line direction.
     * @param p1 First point on the line.
     * @param p2 Second point or direction vector if isDirectVector is true.
     * @param isDirectVector If true, p2 is treated as a direction vector.
    */
    Line(cv::Point2d p1, cv::Point2d p2, bool isDirectVector = false);

    /**
     * @brief Computes the Root Mean Square Error (RMSE) of the line with respect to a set of points.
     * @tparam T Type of point data: int, float, double.
     * @param points Vector of points.
     * @return RMSE value.
    */
    template<typename T>
    auto GetRMSE(std::vector<cv::Point_<T>>const& points)const->double;

    /**
     * @brief Computes the RMSE of the line with respect to a single point.
     * @param p Point.
     * @return RMSE value.
    */
    auto GetRMSE(cv::Point2d p)const->double;

    /**
     * @brief Computes the intersection point of the line with another line.
     * @param l Another line.
     * @return Intersection point.
    */
    auto GetIntersectPoint(Line const& l) const->cv::Point2d;

    /**
     * @brief Get the distance d from a point to line. 
     * 
     * d>0: point in the clockwise direction of normalize vector
     * d<0: point in the counter-clockwise direction of normalize vector
     * clockwise direction is rotation from positive y to positive x direction.
     * 
     * @param p A point
     * @return Distance from p to the line
    */
    auto GetDistance(cv::Point2d p = cv::Point2d(0, 0))const->double;

    /**
     * @brief Get angle between current line and a vector
     * 
     * angle > 0 means line is before the vector 'vec' in the clockwise direction
     * angle < 0 means line is after the vector 'vec' in the clockwise direction
     * clockwise direction is rotation from positive y to positive x direction.
     * 
     * @param vec A vector
     * @return Angle in radian
    */
    auto GetAngle(cv::Point2d vec = (cv::Point2d(1, 0)))const->double;

    /**
     * @brief Gets the point on the line closest to a given point.
     * @param p Point.
     * @return Closest point on the line.
    */
    auto GetPoint(cv::Point2d p)const->cv::Point2d;

    /**
     * @brief Gets the point on the line for a given x-coordinate.
     * @param p Point with x-coordinate.
     * @return Point on the line.
    */
    auto GetPointX(cv::Point2d p)const->cv::Point2d;

    /**
     * @brief Gets the point on the line for a given y-coordinate.
     * @param p Point with y-coordinate.
     * @return Point on the line.
    */
    auto GetPointY(cv::Point2d p)const->cv::Point2d;

    /**
     * @brief Gets the normal vector of the line.
     * @return Normal vector.
    */
    auto GetNormalVector()const->cv::Point2d;

    /**
     * @brief Sets the normal vector of the line.
     * @param vec Normal vector.
    */
    auto SetNormalVector(cv::Point2d vec) & ->void;

    /**
     * @brief Gets the normalized vector along the line direction.
     * @return Normalized direction vector.
    */
    auto GetNormalizeVector() const->cv::Point2d const;

    /**
     * @brief Sets the normalized vector along the line direction.
     * @param vec Normalized direction vector.
    */
    auto SetNormalizeVector(cv::Point2d vec) & ->void;

    /**
     * @brief Gets a point on the line that is closest to (0,0).
     * @return Point on the line.
    */
    auto GetPoint0() const->cv::Point2d const;

    /**
     * @brief Sets a point on the line.
     * @param p Point on the line.
    */
    auto SetPoint0(cv::Point2d p) & ->void;

    /**
     * @brief Comparator two lines
     * @param other Other line
     * @return true if two lines are the same
    */
    bool operator==(xvt::Line const& other);

    /**
     * @brief Shift the line by a given vector
     * @param p Shift vector.
     * @return
    */
    auto operator+=(cv::Point2d const& p) & ->xvt::Line&;

    /**
     * @brief Shift the line by a given vector
     * @param p Shift vector.
     * @return 
    */
    auto operator+(cv::Point2d const& p) const->xvt::Line;

    /**
     * @brief Transforms the line using a transformation matrix.
     * @param m Transformation matrix.
    */
    virtual
    auto Transform(cv::Mat const& m) & ->void;

    /**
     * @brief Shifts the line by a given vector.
     * @param p Shift vector.
    */
    virtual
    auto Transform(cv::Point2d const& p) & ->void;

    /**
     * @brief Draws the line on an image.
     * @param img input Image.
     * @param pen Pen for drawing.
     * @param offset Offset point.
     * @param drawDetail Whether to draw line information details.
    */
    void Draw(cv::Mat& img, xvt::CVPen pen, cv::Point2d offset = cv::Point2d(), bool drawDetail = false) const;

public:
protected:
    /// x coffecient
    double m_wx = 0;
    /// y coffecient
    double m_wy = 0;
    /// distance coffecient
    double m_wr = 0;

};

/**
* @brief Class representing the result of a line detection or inspection.
*
* This class extends both `Line` and `InspectionResult`, providing methods
* for transforming the line, calculating RMSE, drawing the result, and exporting
* data to CSV.
*/
class XVT_EXPORTS LineResult :
      public Line
    , public InspectionResult
{
public:
    /**
     * @brief Default constructor.
    */
    LineResult() = default;

    /**
     * @brief Constructor with result and message.
     * @param result Inspection result.
     * @param msg Message related to the result.
    */
    LineResult(xvt::EResult result, std::string const& msg) : Line(), InspectionResult(result, msg) {};

    /**
     * @brief Constructor with angle and distance.
     * @param angle Angle of the line.
     * @param distance Distance from the origin.
    */
    LineResult(double angle, double distance) : Line(angle, distance), InspectionResult() {};

    /**
     * @brief Constructor with two points defining the line.
     * @param p1 First point.
     * @param p2 Second point.
     * @param isDirectVector whether p2 is direct vector or normal point.
     * @see Line::line(cv::Point, cv::Point, bool)
    */
    LineResult(cv::Point const& p1, cv::Point const& p2, bool isDirectVector = false) : Line(p1, p2, isDirectVector), InspectionResult() {};

    /**
     * @brief Copy constructor.
     * @param l Line to copy.
    */
    LineResult(Line const& l) : Line(l), InspectionResult() {};

    /**
     * @brief Move constructor.
     * @param l Line to move.
    */
    LineResult(Line&& l) : Line(std::move(l)), InspectionResult() {};

    /**
     * @brief Constructor from an InspectionResult.
     * @param res InspectionResult to copy.
    */
    LineResult(InspectionResult const& res) : Line(), InspectionResult(res) {};

    /**
     * @brief Move constructor from an InspectionResult.
     * @param res InspectionResult to move.
    */
    LineResult(InspectionResult&& res) : Line(), InspectionResult(std::move(res)) {};

    /**
     * @brief Transforms the line based on the transformation matrix m.
     * @param m Transformation matrix.
    */
    virtual auto Transform(cv::Mat const& m) & ->void override;

    /**
     * @brief Shifts the line by a vector.
     * @param p Shift vector.
    */
    virtual auto Transform(cv::Point2d const& p) & ->void override;

    /**
     * @brief Gets the RMSE of the line result based on stored points.
     * @return RMSE value.
    */
    auto GetRMSE() const->double;

    /**
     * @brief Gets the RMSE of the line result relative to a specific point.
     * @param p Point to compute RMSE against.
     * @return RMSE value.
    */
    auto GetRMSE(cv::Point2d p)const->double { return Line::GetRMSE(p); }

    /**
     * @brief Gets the RMSE of the line result relative to a set of points.
     * @param points Points to compute RMSE against.
     * @return RMSE value.
    */
    auto GetRMSE(xvt::VecPoint points)const->double { return Line::GetRMSE(points); }

    /**
     * @brief Draws the line result on an image.
     * @param img Image to draw on.
     * @param offSetPoint Offset point for drawing.
     * @param pen Pen for drawing.
    */
    void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    /**
     * @brief Exports the line result to CSV format.
     * @param out CSV output stream.
     * @param prefix Prefix for CSV data.
     * @param isResursive Whether to include recursive data.
    */
    void GetCSVData(CSVOutput& out, std::string prefix = "", bool isResursive = true) const override;

public:

};

/**
 * @brief Class for detecting line
 *
 * This class extends the `xvt::Ransac` base class and provides methods
 * to fit lines using different approaches and detect lines in images.
*/
class XVT_EXPORTS LineDetector :public xvt::Ransac<LineResult>
{
public:

    /**
     * @brief Default constructor.
    */
    LineDetector() : RansacType(2, 1.0, 5) {}

    /**
     * @brief Fits a line to a set of points using Least Squares Estimation (LSE).
     * @param points Vector of points.
     * @return Fitted line.
    */
    static
    auto FitLineLSQE(std::vector<cv::Point>const& points)->xvt::Line;

    /**
     * @brief Fits a line to a set of points using Least Squares Estimation (LSE) with float precision.
     * @param points Vector of points.
     * @return Fitted line.
    */
    static
    auto FitLineLSQE(std::vector<cv::Point2f>const& points)->xvt::Line;

    /**
     * @brief Fits a line to a set of points using Least Squares Estimation (LSE) with double precision.
     * @param points Vector of points.
     * @return Fitted line.
    */
    static
    auto FitLineLSQE(std::vector<cv::Point2d>const& points)->xvt::Line;

    /**
     * @brief Fits a line to a set of points using OpenCV's line fitting.
     * @param points Vector of points.
     * @param disType Distance type for fitting.
     * @param param Parameter for the distance function.
     * @param reps Sufficient accuracy for the radius (distance between the coordinate origin and the line).
     * @param aeps Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.
     * @return Fitted line.
     * @see cv::fitLine
    */
    static
    auto FitLineOpenCV(std::vector<cv::Point>const& points
                           , cv::DistanceTypes disType = cv::DistanceTypes::DIST_L2
                           , double param = 0, double reps = 0.01, double aeps = 0.01
        )->xvt::Line;


    /**
     * @brief Finds a line in the image
     * 
     * Determining if it is horizontal or vertical based on width and height.
     * Uses Sobel edge detection if `mUseEdge` is true.
     * 
     * @param img Input image.
     * @return Detected line result.
    */
    auto FindLine(cv::Mat const& img)const->xvt::LineResult;

    /**
     * @brief Finds a line in the image, optionally determining if it is horizontal.
     * Uses Sobel edge detection if `mUseEdge` is true.
     * 
     * @param img Input image.
     * @param isHorLine If true, determines if the line is horizontal.
     * @return Detected line result.
    */
    auto FindLine(cv::Mat const& img, bool isHorLine = true)const->xvt::LineResult;

    /**
     * @brief Finds a horizontal line in the image using Sobel y image if `mUseEdge` is true.
     * @param img Input image.
     * @return Detected horizontal line result.
    */
    auto FindHorizontalLine(cv::Mat const& img)const->xvt::LineResult;

    /**
     * @brief Finds a vertical line in the image using Sobel x image if `mUseEdge` is true.
     * @param img Input image.
     * @return Detected vertical line result.
    */
    auto FindVerticalLine(cv::Mat const& img)const->xvt::LineResult;

    /**
     * @brief Fits a line to a set of points using Least Squares Estimation (LSE).
     * @param points Vector of points.
     * @return Model result.
     */
    auto FitLSQE(xvt::VecPoint const& points) const->ModelResultType override;

    /**
     * @brief Checks if the model result is valid.
     * @param res Model result.
     * @return True if the model result is valid, otherwise false.
    */
    auto IsValidModel(ModelResultType const& res) const->bool override;

public:
    /// Apply sobel on the image to find the line
    bool  mUseEdge = true;
    /// Threshold value (it can be edge value when useEdge is true)
    float mThreshold = 0;
    /// From left-right, top-down direction, whether line is between black to white or white/black region
    bool  mIsBlack2White = true;
    /// Range of interested line angle. it is optional
    xvt::Rangef mValidAngle = xvt::Rangef(0, 0, false);
    /// Range of interested error. it is optional
    xvt::Rangef mValidRMSE = xvt::Rangef(0, 0, false);
};

inline
auto Line::GetNormalVector()const->cv::Point2d
{
    return cv::Point2d(m_wx, m_wy);
}

inline
auto Line::SetNormalVector(cv::Point2d vec) & -> void
{
    double l = cv::norm(vec);
    if (l > 0 && l != 1)
    {
        vec /= l;
    }

    m_wx = vec.x;
    m_wy = vec.y;
}

inline
auto Line::GetNormalizeVector()const->cv::Point2d const
{
    return cv::Point2d(-m_wy, m_wx);
}

inline
auto Line::SetNormalizeVector(cv::Point2d vec) & ->void
{
    auto l = cv::norm(vec);
    if (l > 0 && l != 1)
    {
        vec /= l;
    }
    m_wx = vec.y;
    m_wy = -vec.x;
}

inline
auto Line::GetPoint0()const->cv::Point2d const
{
    return GetPoint(cv::Point2d(0, 0));
}

inline
auto Line::SetPoint0(cv::Point2d p) & ->void
{
    m_wr = -m_wx * p.x - m_wy * p.y;
}

inline
bool Line::operator==(xvt::Line const& other)
{
    auto v1 = GetNormalizeVector();
    auto v2 = other.GetNormalizeVector();
    auto p0 = GetPoint0();
    auto dir = abs(abs(v1.ddot(v2)) - 1);
    auto dis = abs(other.GetDistance(p0));
    return  (dir < 2 * DBL_EPSILON) && (dis < DBL_EPSILON);
}

inline
auto Line::operator+=(cv::Point2d const& p) & -> xvt::Line&
{
    this->Transform(p);
    return *this;
}

inline
auto Line::operator+(cv::Point2d const& p) const-> xvt::Line
{
    xvt::Line l = { *this };
    l.Transform(p);
    return l;
}

template<typename T>
auto Line::GetRMSE(std::vector<cv::Point_<T>>const& points)const->double
{
    double rms = 0.0;
    size_t size = points.size();
    if (size > 0)
    {
        for (int i = 0; i < size; i++)
        {
            double d = GetDistance(points[i]);
            rms += d * d;
        }
        rms = sqrt(rms / size);
    }
    return rms;
}

inline
auto LineResult::GetRMSE() const ->double
{
    return Line::GetRMSE(mPoints);
}

XVT_EXPORTS
auto Transform(Line const& l, cv::Point2d const& p)->xvt::Line;

XVT_EXPORTS
auto Transform(Line&& l, cv::Point2d const& m)->xvt::Line;

XVT_EXPORTS
auto Transform(Line const& l, cv::Mat const& m)->xvt::Line;

XVT_EXPORTS
auto Transform(Line&& l, cv::Mat const& m)->xvt::Line;

XVT_EXPORTS
auto Transform(LineResult const& l, cv::Mat const& m)->xvt::LineResult;

XVT_EXPORTS
auto Transform(LineResult&& l, cv::Mat const& m)->xvt::LineResult;

XVT_EXPORTS
auto Transform(LineResult const& l, cv::Point2d const& p)->xvt::Line;

XVT_EXPORTS
auto Transform(LineResult&& l, cv::Point2d const& m)->xvt::Line;

XVT_EXPORTS
auto Intersect(xvt::Line const& l, cv::Rect const& rect)->std::vector<cv::Point2f>;

//! @} end of group Shape

}

