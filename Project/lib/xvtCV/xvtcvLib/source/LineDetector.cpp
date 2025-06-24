#include "xvtCV/LineDetector.h"
#include <xvtCV/ColorDefine.h>
#include <xvtCV/Utils.h>

#ifdef _DEBUG
//Show the debuglog
//#define _DEBUG_LOG_ 
#endif //_DEBUG
#include <xvtCV/Drawing.h>

namespace xvt {
/*
*       y   /
*       |  /
*       | /
*       |/alpha
*       /---------
*      /|
*     / |O--------x

    Line equation: l(angle,r) =  x*sin(angle) - y*cos(angle) - r = 0
        Angle is the angle between the line and the vertical (Ox) axis.
        R is the distance from the original to the line.
        Vector t(cos(mAngle), sin(mAngle)) is the normalized vector that colinear to the line
    The e(xi) = l(angle,r)^2
    E = sum(e(xi) ^ 2)
    take the differentiate E with respect to each coefficient angle, r = 0:
                          2*[(sum(x*y)-sum(x)*sum(y)]
    tan(angle)=---------------------------------------------------------
                [sum(x^2) - sum(x)*sum(x) - sum(y^2) + sum(y)*sum*(y)]
    sum(x)*sin(angle) + sum(y)*cos(angle) + r = 0
    Therefore:
    ax = sum(x)/n
    ay = sum(y)/2
    M(ax,ay) will belong to the line.

*/
template<typename T>
auto FitLineLSQE(std::vector<cv::Point_<T>>const& points)->xvt::Line
{
    xvt::Line l;

    double sumX = 0;
    double sumX2 = 0;
    double sumY = 0;
    double sumY2 = 0;
    double sumXY = 0;
    size_t n = points.size();
    if (n < 1) return l;

    for (auto const& p : points)
    {
        sumX += p.x;
        sumY += p.y;
        sumX2 += p.x * (double)p.x;
        sumY2 += p.y * (double)p.y;
        sumXY += p.x * (double)p.y;
    }

    sumX /= n;
    sumY /= n;

    sumX2 /= n;
    sumY2 /= n;

    sumXY /= n;

    double deltaX = sumX2 - sumX * sumX;
    double deltaY = sumY2 - sumY * sumY;
    double deltaXY = sumXY - sumX * sumY;

    auto angle = atan2(2 * deltaXY, deltaX - deltaY) / 2;
    auto distance = (sumX * cos(angle) + sumY * sin(angle));

    l.SetNormalizeVector(cv::Point2d(cos(angle), sin(angle)));
    l.SetPoint0(cv::Point2d(sumX, sumY));
    return l;
}

Line::Line(double angle, double distance)
{
    m_wx = cos(angle);
    m_wy = sin(angle);
    m_wr = -distance;
}

Line::Line(double a, double b, double c) :m_wx{ a }, m_wy{ b }, m_wr{ c }
{
    auto l = sqrt(a * a + b * b);
    if (l > 0 && l != 1)
    {
        m_wx = a / l;
        m_wy = b / l;
        m_wr = c / l;
    }
}

Line::Line(cv::Point2d p1, cv::Point2d p2, bool isDirectVector)
{
    if (!isDirectVector && p1 != p2)
    {
        cv::Point2d v = p2 - p1;
        auto l = cv::norm(v);

        p1 = cv::Point2d(p1 + p2) / 2.0;
        p2 = v / l;
    }

    SetNormalizeVector(p2);
    SetPoint0(p1);
}

auto Line::GetRMSE(cv::Point2d p) const -> double
{
    return abs(GetDistance(p));
}

auto Line::GetIntersectPoint(Line const& l)const->cv::Point2d
{
    cv::Point2d p(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

    double det  =  m_wx * l.m_wy - m_wy * l.m_wx;
    double detx = -m_wr * l.m_wy + m_wy * l.m_wr;
    double dety = -m_wx * l.m_wr + m_wr * l.m_wx;

    if (abs(det) > FLT_EPSILON * 10.0)
    {
        p.x = detx / det;
        p.y = dety / det;
    }

    return p;
}

auto Line::GetDistance(cv::Point2d p)const -> double
{
    return  m_wx * p.x + m_wy * p.y + m_wr;
}

auto Line::GetAngle(cv::Point2d vec)const -> double
{
    auto angleVec = atan2(vec.y, vec.x);
    auto line = GetNormalizeVector();
    auto angle = atan2(line.y, line.x);
    if (line.ddot(vec) < 0)
    {
        angle = atan2(-line.y, -line.x);
    }

    return angle - angleVec;
}

auto Line::GetPoint(cv::Point2d p) const -> cv::Point2d
{
    auto angle = xvt::Rad2Deg(GetAngle());
    if ((angle > 45 && angle < 135) || (angle < -45 && angle > -135))
    {
        p = GetPointY(p);
    }
    else
    {
        p = GetPointX(p);
    }

    return p;
}

auto Line::GetPointX(cv::Point2d p) const -> cv::Point2d
{
    if (abs(m_wy) > DBL_EPSILON)
        p.y = (m_wx * p.x + m_wr) / -m_wy;
    else
        p.x = m_wr * m_wx;

    return p;
}

auto Line::GetPointY(cv::Point2d p) const -> cv::Point2d
{
    if (abs(m_wx) > DBL_EPSILON)
        p.x = (m_wy * p.y + m_wr) / -m_wx;
    else
        p.y = m_wr * m_wy;

    return p;
}

auto Line::Transform(cv::Mat const& m)& -> void
{
    if (m.rows != 2 && m.rows != 3) return;
    
    cv::Mat mt;
    if (m.rows == 2)
    {
        mt = cv::Mat::eye(3, 3, CV_64FC1);
        m.row(0).copyTo(mt.row(0));
        m.row(1).copyTo(mt.row(1));
    }
    else
    {
        mt = m.clone();
    }

    mt = mt.inv().t();
    
    auto a = mt.at<double>(0, 0) * m_wx + mt.at<double>(0, 1) * m_wy + mt.at<double>(0, 2)* m_wr;
    auto b = mt.at<double>(1, 0) * m_wx + mt.at<double>(1, 1) * m_wy + mt.at<double>(1, 2)* m_wr;
    auto c = mt.at<double>(2, 0) * m_wx + mt.at<double>(2, 1) * m_wy + mt.at<double>(2, 2)* m_wr;

    *this = Line(a, b, c);
}

auto Line::Transform(cv::Point2d const& p) & ->void
{
    m_wr -= m_wx * p.x + m_wy * p.y;
}

void Line::Draw(cv::Mat& img, xvt::CVPen pen, cv::Point2d offset, bool drawDetail) const
{
    auto normalizeVec = GetNormalVector();
    auto point0 = GetPoint0();
    if (img.empty() || normalizeVec == cv::Point2d()) return;
    auto l = cv::norm(cv::Point(img.cols, img.rows));

    auto p1 = GetPoint(cv::Point2d(-l, -l));
    auto p2 = GetPoint(cv::Point2d(l, l));

    cv::line(img, p1 + offset, p2 + offset, pen.mColor, pen.mThickness, pen.mLineType);

    if (drawDetail)
    {
        auto p0 = point0 + offset;
        cv::line(img, p0, p0 + GetNormalVector() * 10, COLOR_CV_ALICEBLUE);
        cv::line(img, p0, p0 + GetNormalizeVector() * 10, COLOR_CV_YELLOW);

        pen.mColor = COLOR_CV_ORANGE;
        xvt::DrawPlusSign(img, p0, pen, 4);
    }
}

auto LineDetector::FitLineLSQE(std::vector<cv::Point>const& points)->xvt::Line
{
    return xvt::FitLineLSQE<int>(points);
}

auto LineDetector::FitLineLSQE(std::vector<cv::Point2f>const& points)->xvt::Line
{
    return xvt::FitLineLSQE<float>(points);
}

auto LineDetector::FitLineLSQE(std::vector<cv::Point2d>const& points)->xvt::Line
{
    return xvt::FitLineLSQE<double>(points);
}

auto LineDetector::FitLineOpenCV(std::vector<cv::Point>const& points
                                 , cv::DistanceTypes disType
                                 , double param, double reps, double aeps
) -> xvt::Line
{
    xvt::Line l2;
    if (points.size() > 2)
    {
        cv::Vec4f l;
        cv::fitLine(points, l, disType, param, reps, aeps);

        l2 = xvt::Line(cv::Point2d(l[2], l[3]), cv::Point2d(l[0], l[1]), true);
    }
    return l2;
}

auto LineDetector::FindLine(cv::Mat const& src) const -> xvt::LineResult
{
    xvt::LineResult line(EResult::ER, "Invalid image");
    cv::Mat img;
    if (xvt::Convert8Bits(src, img, false)) return line;
    auto ratio = (float)img.cols / img.rows;
    return FindLine(img, ratio > 1.0f);
}

auto LineDetector::FindLine(cv::Mat const& src, bool isHorLine) const -> xvt::LineResult
{
    xvt::LineResult line(EResult::ER, "Invalid image");

    cv::Mat img;
    if (xvt::Convert8Bits(src, img, false)) return line;
    if (isHorLine)
    {
        line = FindHorizontalLine(img);
    }
    else
    {
        line = FindVerticalLine(img);
    }

    return line;
}

auto LineDetector::FindHorizontalLine(cv::Mat const& src) const -> xvt::LineResult
{
    xvt::LineResult line(EResult::ER, "Invalid image");

    cv::Mat img;
    if (xvt::Convert8Bits(src, img, false)) return line;
    if (mUseEdge)
    {
        cv::Mat gauKernel = cv::getGaussianKernel(5, -1, CV_32F);
        cv::Mat gauMat = gauKernel * gauKernel.t();

        cv::Mat sobKernelY;
        cv::Sobel(gauMat, sobKernelY, CV_32F, 0, 1, 3);
        cv::Mat sobelImg;
        cv::filter2D(img, sobelImg, CV_32F, sobKernelY);

        int w = img.cols;
        xvt::VecPoint points;
        points.reserve(w);
        if (mIsBlack2White)
        {
            for (int c = 0; c < w; c++)
            {
                cv::Mat coldata = sobelImg.col(c);
                auto start = coldata.begin<float>();
                auto end = coldata.end<float>();
                auto minPtr = std::min_element(start, end);
                if (minPtr != end && *minPtr < -mThreshold)
                {
                    points.emplace_back(c, std::distance(start, minPtr));
                }
            }
        }
        else
        {
            for (int c = 0; c < w; c++)
            {
                cv::Mat coldata = sobelImg.col(c);
                auto start = coldata.begin<float>();
                auto end = coldata.end<float>();
                auto maxPtr = std::max_element(start, end);
                if (maxPtr != end && *maxPtr > mThreshold)
                {
                    points.emplace_back(c, std::distance(start, maxPtr));
                }
            }
        }

        if (points.size() < sobelImg.cols / 10)
        {
            line.SetResult(EResult::ER);
            line.SetMsg("Number of candidate point is too small");
        }
        else
        {
            line = xvt::LineDetector::FitLineOpenCV(points);
            line.SetResult(EResult::OK);
            line.mPoints = std::move(points);
            auto a = xvt::Rad2Deg(line.GetAngle());
            auto e = line.GetRMSE();
            line.SetResult(mValidAngle(a) && mValidRMSE(e));
        }
    }
    else
    {
        line(EResult::ER, "not implement");
    }
    return line;
}

auto LineDetector::FindVerticalLine(cv::Mat const& src) const -> xvt::LineResult
{
    xvt::LineResult line(EResult::ER, "Invalid image");

    cv::Mat img;
    if (xvt::Convert8Bits(src, img, false)) return line;
    line(EResult::ER, "not implement");
    if (mUseEdge)
    {
        cv::Mat gauKernel = cv::getGaussianKernel(5, -1, CV_32F);
        cv::Mat gauMat = gauKernel * gauKernel.t();

        cv::Mat sobKernelX;
        cv::Sobel(gauMat, sobKernelX, CV_32F, 1, 0, 3);
        cv::Mat sobelImg;
        cv::filter2D(img, sobelImg, CV_32F, sobKernelX);

        int h = img.rows;
        xvt::VecPoint points;
        points.reserve(h);
        if (mIsBlack2White)
        {
            for (int r = 0; r < h; r++)
            {
                cv::Mat coldata = sobelImg.row(r);
                auto start = coldata.begin<float>();
                auto end = coldata.end<float>();
                auto minPtr = std::min_element(start, end);
                if (minPtr != end && *minPtr < -mThreshold)
                {
                    points.emplace_back(std::distance(start, minPtr), r);
                }
            }
        }
        else
        {
            for (int r = 0; r < h; r++)
            {
                cv::Mat rowdata = sobelImg.row(r);
                auto start = rowdata.begin<float>();
                auto end = rowdata.end<float>();
                auto maxPtr = std::max_element(start, end);
                if (maxPtr != end && *maxPtr > mThreshold)
                {
                    points.emplace_back(std::distance(start, maxPtr), r);
                }
            }
        }

        if (points.size() < sobelImg.cols / 10)
        {
            line.SetResult(EResult::ER);
            line.SetMsg("Number of candidate point is too small");
        }
        else
        {
            line = xvt::LineDetector::FitLineOpenCV(points);
            line.SetResult(EResult::OK);
            line.mPoints = std::move(points);
            auto a = xvt::Rad2Deg(line.GetAngle());
            auto e = line.GetRMSE();
            line.SetResult(mValidAngle(a) && mValidRMSE(e));
        }
    }
    else
    {
        line(EResult::ER, "not implement");
    }
    return line;
}

auto LineDetector::FitLSQE(xvt::VecPoint const& points) const -> ModelResultType
{
    auto res = ModelResultType();
    xvt::Line& l = res;
    l = FitLineLSQE(points);
    res.SetResult(1);
    return res;
}

auto LineDetector::IsValidModel(ModelResultType const& res) const -> bool
{
    return true;
}

void LineResult::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    if (xvt::ConvertRGB(img, img, false)) return;
    auto roi = GetROI();
    offSetPoint += roi.tl();
    InspectionResult::DrawResult(img, offSetPoint, pen);
    Line::Draw(img, pen, offSetPoint);
}

auto LineResult::Transform(cv::Mat const& m)&->void
{
    if (!m.empty())
    {
        Line::Transform(m);
        InspectionResult::Transform(m);
    }
}

auto LineResult::Transform(cv::Point2d const& p)&->void
{
    Line::Transform(p);
    cv::Mat m = cv::Mat::eye(2, 3, CV_64FC1);
    m.at<double>(0, 2) = p.x;
    m.at<double>(1, 2) = p.x;
    InspectionResult::Transform(m);
}

auto LineResult::GetCSVData(CSVOutput& out, std::string prefix, bool isResursive) const -> void
{
    auto rmse = GetRMSE();

    out.emplace_back(prefix + "a", xvt::ToString(m_wx, 1));
    out.emplace_back(prefix + "b", xvt::ToString(m_wy, 1));
    out.emplace_back(prefix + "c", xvt::ToString(m_wr, 1));
    out.emplace_back(prefix + "e", xvt::ToString(rmse, 2));
}

auto Transform(Line const& l, cv::Point2d const& p) -> xvt::Line
{
    Line l2{ l };
    l2.Transform(p);
    return l2;
}

auto Transform(Line&& l, cv::Point2d const& p) -> xvt::Line
{
    l.Transform(p);
    return std::move(l);
}

auto Transform(Line const& l, cv::Mat const& m)->xvt::Line
{
    Line l2{ l };
    l2.Transform(m);
    return l2;
}

auto Transform(Line&& l, cv::Mat const& m)->xvt::Line
{
    l.Transform(m);
    return std::move(l);
}

auto Transform(LineResult const& l, cv::Mat const& m)->xvt::LineResult
{
    LineResult l2 = l;
    l2.Transform(m);
    return l2;
}

auto Transform(LineResult&& l, cv::Mat const& m)->xvt::LineResult
{
    l.Transform(m);
    return std::move(l);
}

auto Transform(LineResult const& l, cv::Point2d const& p) -> xvt::Line
{
    LineResult l2 = l;
    l2.Transform(p);
    return l2;
}

auto Transform(LineResult&& l, cv::Point2d const& p) -> xvt::Line
{
    l.Transform(p);
    return std::move(l);
}

auto Intersect(xvt::Line const& l, cv::Rect const& rect) -> std::vector<cv::Point2f>
{
    auto res = std::vector<cv::Point2f>();
    if (rect.empty()) return res;

    int const& w = rect.width;
    int const& h = rect.height;

    auto p00 = rect.tl();
    auto p11 = rect.br();
    auto p10 = cv::Point2f(p11.x, p00.y);
    auto p01 = cv::Point2f(p00.x, p11.y);

    auto d00 = l.GetDistance(p00);
    auto d10 = l.GetDistance(p10);
    auto d11 = l.GetDistance(p11);
    auto d01 = l.GetDistance(p01);

    if (d00 * d10 <= 0)
    {
        if (0 == d00) res.push_back(p00);
        else if (d10 != 0)res.push_back(l.GetIntersectPoint(xvt::Line(p00, p10)));
    }

    if (d10 * d11 <= 0)
    {
        if (0 == d10) res.push_back(p10);
        else if (d11 != 0) res.push_back(l.GetIntersectPoint(xvt::Line(p11, p10)));
    }

    if (d11 * d01 <= 0)
    {
        if (0 == d11) res.push_back(p11);
        else if (d01 != 0) res.push_back(l.GetIntersectPoint(xvt::Line(p11, p01)));
    }

    if (d01 * d00 <= 0)
    {
        if (0 == d01) res.push_back(p01);
        else if (d00 != 0) res.push_back(l.GetIntersectPoint(xvt::Line(p01, p00)));
    }
    return res;
}

}//namespace xvt