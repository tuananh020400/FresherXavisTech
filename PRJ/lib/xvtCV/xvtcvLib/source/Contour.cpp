#include "xvtCV/Contour.h"
#include "opencv2/imgproc.hpp"

//template class  cv::Point_<float>;
namespace xvt {
Contour::Contour()
    : mArea{ 0 }
    , mBoundaryRect{ cv::Rect() }
    , mRotatedBoundaryRect{ cv::RotatedRect() }
    , mCenter{ cv::Point2f() }
{
    SetAllCalculatedFlag(false);
    mBoundaryPointList = {};
    mRefAngle = 90;
    mSquareTolerance = 0.01f;
}

Contour::Contour(std::vector<cv::Point2f> const& pointList)
    : Contour()
{
    mBoundaryPointList = pointList;
}

Contour::Contour(std::vector<cv::Point2f>&& pointList)
    : Contour()
{
    mBoundaryPointList = std::move(pointList);
}

Contour::Contour(std::vector<cv::Point> const& pointList)
    : Contour()
{
    std::vector<cv::Point2f> points(pointList.begin(), pointList.end());
    mBoundaryPointList = std::move(points);
}

void Contour::SetAllCalculatedFlag(bool flag)
{
    mIsCenterCalculated = flag;
    mIsAreaCalculated = flag;
    mIsBoundaryRectCalculated = flag;
    mIsRotatedBoundaryRectCalculated = flag;
    misRotatedAngleCalculated = flag;
}

void Contour::SetBoundaryPoints(std::vector<cv::Point2f> const& pList)
{
    SetAllCalculatedFlag(false);
    mBoundaryPointList = pList;
}

std::vector<cv::Point2f> const& Contour::GetBoundaryPoint2fList() const&
{
    return mBoundaryPointList;
}

std::vector<cv::Point> Contour::GetBoundaryPointList() const&
{
    std::vector<cv::Point> boundaryPointList{};
    rsize_t size = mBoundaryPointList.size();
    for (int i = 0; i < size; i++)
    {
        cv::Point p = cv::Point((int)round(mBoundaryPointList[i].x), (int)round(mBoundaryPointList[i].y));
        boundaryPointList.push_back(p);
    }
    return boundaryPointList;
}

void Contour::SetBoundaryPoints(std::vector<cv::Point2f>&& pList)
{
    SetAllCalculatedFlag(false);
    mBoundaryPointList = std::move(pList);
}

void Contour::SetBoundaryPoints(std::vector<cv::Point> const& pList)
{
    SetAllCalculatedFlag(false);
    std::vector<cv::Point2f> tmp(pList.begin(), pList.end());
    mBoundaryPointList = std::move(tmp);
}

std::vector<cv::Point2f> Contour::GetBoundaryPoint2fList()&&
{
    return std::move(mBoundaryPointList);
}

double Contour::GetArea()
{
    if (!mIsAreaCalculated)
    {
        if (mBoundaryPointList.size())
        {
            mArea = cv::contourArea(mBoundaryPointList);
        }
        else
        {
            mArea = 0;
        }
        mIsAreaCalculated = true;
    }
    else
    {

    }

    return mArea;
}

double Contour::GetRotatedAngle()
{
    if (!misRotatedAngleCalculated)
    {
        cv::Point2f pts[4];
        // Calculated angle
        mRotatedAngle = 0;
        cv::RotatedRect rotatedRect = GetMinBoundaryRectangle();
        // get 4 points of rotatedRect
        rotatedRect.points(pts);
        // get length of two edges
        float h = static_cast<float>(cv::norm(pts[0] - pts[1]));
        float w = static_cast<float>(cv::norm(pts[2] - pts[1]));
        if (h > 0 && w > 0)
        {
            float ratio = w / h;
            if (abs(1 - ratio) <= mSquareTolerance)
            {
                // rectangle is a square
                mRefAngle = 45;
                mRotatedAngle = rotatedRect.angle >= -45 ? rotatedRect.angle : 90 + rotatedRect.angle;
            }
            else
            {
                // rectangle is not a square
                mRefAngle = 90;
                // w > h
                if (ratio > 1)
                {
                    mRotatedAngle = rotatedRect.angle;
                }
                // h > w
                else
                {
                    mRotatedAngle = 90.0 + rotatedRect.angle;

                }
            }
        }
        else
        {
            //do nothing 
        }
        misRotatedAngleCalculated = true;
    }
    return mRotatedAngle;
}

cv::Rect Contour::GetBoundaryRectangle()
{
    if (!mIsBoundaryRectCalculated)
    {
        mBoundaryRect = cv::boundingRect(mBoundaryPointList);
        mIsBoundaryRectCalculated = true;
    }
    else
    {

    }
    return mBoundaryRect;
}

cv::RotatedRect Contour::GetMinBoundaryRectangle()
{
    if (!mIsRotatedBoundaryRectCalculated)
    {
        if (!mBoundaryPointList.empty())
        {
            mRotatedBoundaryRect = cv::minAreaRect(mBoundaryPointList);
        }
        else
        {
            mRotatedBoundaryRect = cv::RotatedRect();
        }
        mIsRotatedBoundaryRectCalculated = true;
    }
    else
    {

    }
    return mRotatedBoundaryRect;
}

cv::Point2f Contour::GetCenter()
{
    if (!mIsCenterCalculated)
    {
        if (!mBoundaryPointList.empty())
        {
            cv::Moments mm = cv::moments(mBoundaryPointList);
            if (0 == mm.m00)
            {
                mCenter = cv::Point2f();
            }
            else
            {
                mCenter.x = static_cast<float>(mm.m10 / mm.m00);
                mCenter.y = static_cast<float>(mm.m01 / mm.m00);
            }
        }
        else
        {
            mCenter = cv::Point2f();
        }
        mIsCenterCalculated = true;
    }
    else
    {

    }
    return mCenter;

}

bool Contour::FindMaxContour(cv::Mat const& binImg, Contour& rMaxContour)
{
    bool rtnVl = false;
    //Contour rMaxContour = Contour();
    if (!binImg.empty() && CV_8UC1 == binImg.type() )
    {
        std::vector<std::vector<cv::Point>> contourList{};
        cv::findContours(binImg, contourList, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        auto maxContour = std::max_element(contourList.begin(), contourList.end(), [](auto const& c1, auto const& c2)
                                           {
                                               double s1 = cv::contourArea(c1);
                                               double s2 = cv::contourArea(c2);
                                               return (s1 < s2);
                                           });

        if (maxContour != contourList.end())
        {
            rMaxContour.SetBoundaryPoints(*maxContour);
            rtnVl = true;
        }
        else
        {
        }
    }
    else
    {
    }

    return rtnVl;
}

void Contour::Draw(cv::Mat& inImg, CVPen pen, cv::Point2f offSet) const
{
    if (!inImg.empty() && mBoundaryPointList.size() > 0)
    {
        std::vector<cv::Point> offsetPointList{};
        std::transform(mBoundaryPointList.begin(), mBoundaryPointList.end(),
                       std::back_inserter(offsetPointList),
                       [offSet](const cv::Point2f& p) { return cv::Point(static_cast<int>(round(p.x + offSet.x)), static_cast<int>(round(p.y + offSet.y))); });
        if (pen.mThickness < 0)
        {
            std::vector<std::vector<cv::Point>> pointsToDrawList{ offsetPointList };
            cv::fillPoly(inImg, pointsToDrawList, pen.mColor, pen.mLineType);
        }
        else
        {
            cv::polylines(inImg, offsetPointList, true, pen.mColor, pen.mThickness, pen.mLineType);
        }
    }
    else
    {

    }
}

void Contour::DrawContour(cv::Mat& inImg, CVPen pen, cv::Point2f offSet) const
{
    if (!inImg.empty() && mBoundaryPointList.size() > 0)
    {
        std::vector<cv::Point> offsetPointList{};
        std::transform(mBoundaryPointList.begin(), mBoundaryPointList.end(),
                       std::back_inserter(offsetPointList),
                       [offSet](const cv::Point2f& p) { return cv::Point(static_cast<int>(round(p.x + offSet.x)), static_cast<int>(round(p.y + offSet.y))); });
        std::vector<std::vector<cv::Point>> pointsToDrawList{ offsetPointList };
        cv::drawContours(inImg, pointsToDrawList, -1, pen.mColor, pen.mThickness, pen.mLineType);
    }
    else
    {

    }
}

double GetRotatedAngle(std::vector<cv::Point> const& ptList, cv::Point2d& mainAxis, cv::Point2d& center)
{
    double angle = 0.0;
    size_t sz = ptList.size();

    if (sz > 2)
    {
        cv::Mat data_pts = cv::Mat(sz, 2, CV_64F);
        for (size_t i = 0; i < sz; i++)
        {
            data_pts.at<double>(i, 0) = ptList[i].x;
            data_pts.at<double>(i, 1) = ptList[i].y;
        }

        //Perform PCA analysis
        cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

        mainAxis = cv::Point2d(pca_analysis.eigenvectors.at<double>(0, 0), pca_analysis.eigenvectors.at<double>(0, 1));

        //Store the center of the object
        center = cv::Point2d(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));

        angle = atan2(mainAxis.y, mainAxis.x);
    }
    else if (sz > 1)
    {
        mainAxis = ptList[1] - ptList[0];
        double l = sqrt(mainAxis.x * mainAxis.x + mainAxis.y * mainAxis.y);
        if (l != 0) mainAxis /= l;
        angle = atan2(mainAxis.y, mainAxis.x);
        center = (ptList[1] + ptList[0]) / 2.0;
    }
    else if (sz > 0)
    {
        angle = 0.0;
        mainAxis = cv::Point2d();
        center = ptList[0];
    }

    return angle;
}

bool FindClosestPointOnContour(const std::vector<cv::Point>& contour, cv::Point const& pt, int& index)
{
    index = -1;
    if (contour.empty()) return false;
    cv::Point tmp = contour[0] - pt;
    double minDistance = sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
    index = 0;
    int contourSize = (int)contour.size();
    for (int i = 1; i < contourSize; i++)
    {
        tmp = contour[i] - pt;
        double distance = sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
        if (minDistance > distance)
        {
            minDistance = distance;
            index = i;
        }
    }

    return true;
}

auto FindCornerPointsIdx(std::vector<cv::Point> const& contour) -> std::vector<int>
{
    std::vector<int> result;
    bool finalResult = !contour.empty();
    if (finalResult)
    {
        auto batRect = cv::boundingRect(contour);
        const cv::Point refTL = batRect.tl();
        const cv::Point refTR = cv::Point(batRect.br().x, batRect.y);
        const cv::Point refBR = batRect.br();
        const cv::Point refBL = cv::Point(batRect.x, batRect.br().y);

        int contourSize = contour.size();
        int idxTL = 0;
        int idxTR = 0;
        int idxBR = 0;
        int idxBL = 0;
        double minTLDis = DBL_MAX;
        double minTRDis = DBL_MAX;
        double minBRDis = DBL_MAX;
        double minBLDis = DBL_MAX;
        for (int i = 0; i < contourSize; i++)
        {
            cv::Point tmptl = contour[i] - refTL;
            cv::Point tmptr = contour[i] - refTR;
            cv::Point tmpbr = contour[i] - refBR;
            cv::Point tmpbl = contour[i] - refBL;
            double dtl = sqrt(tmptl.x * tmptl.x + tmptl.y * tmptl.y);
            double dtr = sqrt(tmptr.x * tmptr.x + tmptr.y * tmptr.y);
            double dbr = sqrt(tmpbr.x * tmpbr.x + tmpbr.y * tmpbr.y);
            double dbl = sqrt(tmpbl.x * tmpbl.x + tmpbl.y * tmpbl.y);

            if (minTLDis > dtl)
            {
                minTLDis = dtl;
                idxTL = i;
            }

            if (minTRDis > dtr)
            {
                minTRDis = dtr;
                idxTR = i;
            }

            if (minBRDis > dbr)
            {
                minBRDis = dbr;
                idxBR = i;
            }

            if (minBLDis > dbl)
            {
                minBLDis = dbl;
                idxBL = i;
            }
        }

        result = {idxTL, idxTR, idxBR, idxBL};
    }
    return result;
}

auto FindCornerPoints(std::vector<cv::Point> const& contour) -> std::vector<cv::Point>
{
    std::vector<cv::Point> finalResult;
    auto result = FindCornerPointsIdx(contour);
    for(auto idx : result)
    {
        finalResult.emplace_back(contour[idx]);
    }
    return finalResult;
}

auto FindCornerPoints(std::vector<cv::Point> const& contour, cv::Point& tl, cv::Point& tr, cv::Point& br, cv::Point& bl) -> bool
{
    auto result = FindCornerPointsIdx(contour);
    if(result.size() == 4)
    {
        tl = contour[result[0]];
        tr = contour[result[1]];
        br = contour[result[2]];
        bl = contour[result[3]];
        return true;
    }

    return false;
}

}
