#include "xvtCV/CircleDetector.h"
#include "xvtCV/Utils.h"
#include "xvtCV/Drawing.h"
#include <time.h>
#include <random>

#if _DEBUG
//#define DEBUG_RANSAC
#endif

namespace xvt {

CircleDetector::CircleDetector() :RansacType(3, 1.0, 5)
{
    mRadiusRange  = Ranged(0, 1);
    mMaxCoverRate = 0.5;
}

CircleDetector::CircleDetector(Ranged radiusRange) : RansacType(3, 1.0, 5)
{
    mRadiusRange = radiusRange;
}

bool CircleDetector::IsOnCircle(double centerX, double centerY, double r, double x, double y, double tolerance)
{
    bool rtnValue = false;
    // Compare radius of circle with distance of its center from given point 
    double cal = Distance(cv::Point2d(x, y), cv::Point2d(centerX, centerY));
    rtnValue = (cal >= (r - tolerance)) && (cal <= (r + tolerance));
    return rtnValue;
}

auto CircleDetector::Fit(const std::vector<cv::Point>& pointList
                         , std::vector<cv::Point>& rChoosenPointList
) const->ModelResultType
{
    ModelResultType rtnvl;
    
    switch (mFitMethod)
    {
    case FittingMethod::LEAST_SQUARE:
        rtnvl = FitLSQE(pointList);
        break;
    case FittingMethod::RANSAC:
        rtnvl = FitRANSAC(pointList, rChoosenPointList);
        break;
    default:
        break;
    }

    return rtnvl;
}

auto CircleDetector::IsValidModel(ModelResultType const& m) const -> bool
{
    auto res = mRadiusRange(m.mRadius);
    return res;
}

auto CircleDetector::FitLSQE(xvt::VecPoint const& points) const -> ModelResultType
{
    cv::Point2f c;
    double r;
    double e;
    auto isOK = FitLeastSquare(points, c, r, e);
    ModelResultType res(c, r);
    if(isOK)
    {
        res(EResult::OK,"");
    }
    else
    {
        res(EResult::ER, "LSQ can not find the circle");
    }
    return res;
}

//---------------------------------------------------------------------
// Fits a circle to a given set of points. There must be at least 2 points
// The circle equation is of the form: (x-xc)^2 + (y-yc)^2 = r^2
// Returns true if there is a fit, false in case no circles can be fit
// rPr: Predict radius
// rPe: Predict error
// refer: Least-Squares Circle Fit, Randy Bullock
bool CircleDetector::FitLeastSquare(const std::vector<cv::Point>& pointList, cv::Point2f& rCenterPoint, double& rPr, double& rPe)
{
    bool rtnVal = true;
    rPe = std::numeric_limits<double>::max();
    size_t N = pointList.size();
    if (N < 3)
    {
        rtnVal = false;
    }
    else
    {
        double xAvg = 0;
        double yAvg = 0;

        for (int i = 0; i < N; i++)
        {
            xAvg += pointList[i].x;
            yAvg += pointList[i].y;
        } //end-for

        xAvg /= N;
        yAvg /= N;

        double Suu = 0;
        double Suv = 0;
        double Svv = 0;
        double Suuu = 0;
        double Suvv = 0;
        double Svvv = 0;
        double Svuu = 0;

        for (int i = 0; i < N; i++)
        {
            double u = pointList[i].x - xAvg;
            double v = pointList[i].y - yAvg;

            Suu += u * u;
            Suv += u * v;
            Svv += v * v;
            Suuu += u * u * u;
            Suvv += u * v * v;
            Svvv += v * v * v;
            Svuu += v * u * u;
        } //end-for

          // Now, we solve for the following linear system of equations
          // Av = b, where v = (uc, vc) is the center of the circle
          //
          // |Suu  Suv| |uc| = |b1|
          // |Suv  Svv| |vc| = |b2|
          //
          // where b1 = 0.5*(Suuu+Suvv) and b2 = 0.5*(Svvv+Svuu)
          //
        double detA = Suu * Svv - Suv * Suv;
        if (0 == detA)
        {
            rtnVal = false;
        }
        else
        {
            double b1 = 0.5 * (Suuu + Suvv);
            double b2 = 0.5 * (Svvv + Svuu);

            double uc = (Svv * b1 - Suv * b2) / detA;
            double vc = (Suu * b2 - Suv * b1) / detA;

            double R = sqrt(uc * uc + vc * vc + (Suu + Svv) / N);
            double pxc = uc + xAvg;
            double pyc = vc + yAvg;

            // Compute mean square error
            rPe = ComputeRMSE(pointList, pxc, pyc, R);
            rCenterPoint.x = static_cast<float>(pxc);
            rCenterPoint.y = static_cast<float>(pyc);
            rPr = R;
        }
    }//end not if (N < 3)
    return rtnVal;
}

auto CircleDetector::ComputeRMSE(const std::vector<cv::Point>& pointList, double centerX, double centerY, double R)->double
{
    double error = 0;
    size_t N = pointList.size();
    double maxError = 0;
    double pe = 0;
    for (int i = 0; i < N; i++)
    {
        double dx = pointList[i].x - centerX;
        double dy = pointList[i].y - centerY;
        double d = sqrt(dx * dx + dy * dy) - R;
        error += d * d;
        if (error > maxError)
        {
            maxError = error;
        }
        else
        {
            //Do nothing
        }
    } //end-for

    if (N > 0)
    {
        pe = sqrt(error / N);
    }
    else
    {
        pe = std::numeric_limits<double>::infinity();
    }

#ifdef _DEBUG
    /*	std::cout << "RMSError"<< std::endl;
        std::cout << "\tTong So diem: " << N << std::endl;
        std::cout << "\tTam: " << centerX << ", " << centerY << std::endl;
        std::cout << "\tSai So: " << pe << std::endl;
        std::cout << "\tSai So Max: " << maxError << std::endl;*/
#endif // _DEBUG
    return pe;
}

bool Circle::operator==(xvt::Circle const& other)
{
    return mRadius == other.mRadius && mCenter == other.mCenter;
}

auto Circle::operator+=(cv::Point2d const& p) & -> xvt::Circle&
{
    mCenter += p;
    return *this;
}

auto Circle::operator+(cv::Point2d const& p) const -> xvt::Circle
{
    return xvt::Circle(mCenter+p, mRadius);
}

auto Circle::operator+=(double const& r) & ->xvt::Circle&
{
    mRadius += r;
    return *this;
}

auto Circle::operator+(double const& r) const->xvt::Circle
{
    return xvt::Circle(mCenter, mRadius + r);
}

auto Circle::GetDistance(cv::Point2d p) const -> double
{
    auto v = p - mCenter;
    return cv::norm(v) - mRadius;
}

void Circle::Draw(cv::Mat& img, xvt::CVPen pen, cv::Point2d offset, bool drawCenter) const
{
    if(!xvt::ConvertRGB(img, img, false))
    {
        auto c = mCenter + offset;
        auto c2 = cv::Point(std::round(c.x), std::round(c.y));
        if (mRadius > 0)
        {
            cv::circle(img, c2, std::round(mRadius), pen.mColor, pen.mThickness, pen.mLineType);
        }

        if (drawCenter)
        {
            xvt::DrawPlusSign(img, c2, pen, 4 * pen.mThickness);
        }
    }
}

void CircleResult::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    Circle::Draw(img, pen, offSetPoint, true);
    InspectionResult::DrawResult(img, offSetPoint, pen);
}

}
