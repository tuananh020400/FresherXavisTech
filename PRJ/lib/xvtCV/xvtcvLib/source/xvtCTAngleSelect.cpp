#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "xvtCV/Peak.h"
#include "xvtCV/Utils.h"
#include "xvtCV/CircleDetector.h"
#include "xvtCV/xvtCTAngleSelect.h"
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/xvtIntensityAnalyzer.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/LineDetector.h"

namespace xvt {
namespace ct {

void writeVecToFile(const std::vector<float>const& intensity, const std::string& filename)
{
    std::ofstream outputFile(filename);
    if (outputFile.is_open())
    {
        for (const auto& value : intensity)
        {
            outputFile << value << std::endl;
        }
        outputFile.close();
        std::cout << "Intensity data has been written to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

void AngleSelector::GetCenter(cv::Mat const& img, cv::Point2f center) const
{
    cv::Mat imgThres;
    if (img.empty()) {
        return;
    }
    xvt::Convert8Bits(img, imgThres);
    xvt::threshold::Threshold(imgThres, imgThres, mCenterThreshold, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(imgThres, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point cen_img(img.cols / 2, img.rows / 2);

    std::vector<cv::Point> u_centers;
    std::vector<cv::Point> l_centers;

    int mMidPoint = img.rows / 2;
    int mRadPoint = img.rows * 0.4;

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);

        if (area > 3) {
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 != 0) {
                cv::Point p;
                p.x = mu.m10 / mu.m00;
                p.y = mu.m01 / mu.m00;

                if (p.y > mMidPoint && cv::norm(p - cen_img) < mRadPoint) {
                    l_centers.push_back(p);
                }
                else if (p.y < mMidPoint && cv::norm(p - cen_img) < mRadPoint) {
                    u_centers.push_back(p);
                }
            }
        }
    }

    xvt::Line resultLine1 = xvt::LineDetector::FitLineLSQE(l_centers);
    xvt::Line resultLine2 = xvt::LineDetector::FitLineLSQE(u_centers);
    cv::Point tmpCenter = resultLine1.GetIntersectPoint(resultLine2);

    std::vector<cv::Point> u_centers_nor;
    std::vector<cv::Point> l_centers_nor;

    //Take the points located within a 30-degree angle around the line
    double sin15 = sin(15.0 * CV_PI / 180.0);
    for (const cv::Point& p : l_centers) {
        double h = resultLine1.GetDistance(cv::Point2d(p));
        double l = cv::norm(p - tmpCenter);
        if (l > 0) {
            if (h / l < sin15) {
                l_centers_nor.push_back(p);
            }
        }
    }
    xvt::Line resultLine1Nor = xvt::LineDetector::FitLineLSQE(l_centers_nor);

    for (const cv::Point& p : u_centers) {
        double h = resultLine1.GetDistance(cv::Point2d(p));
        double l = cv::norm(p - tmpCenter);
        if (l > 0) {
            if (h / l < sin15) {
                u_centers_nor.push_back(p);
            }
        }
    }
    xvt::Line resultLine2Nor = xvt::LineDetector::FitLineLSQE(u_centers_nor);

    center = resultLine1Nor.GetIntersectPoint(resultLine2Nor);
}

auto AngleSelector::GetAngle(cv::Mat const& img, AngleSelectorResult& res) const-> void
{
    std::vector<float>& varianceVec = res.mVarianceVec;
    std::vector<float>& angles = res.mAngleList;

    cv::Mat dst;
    if (!xvt::Convert8Bits(img, dst, false))
    {
        cv::GaussianBlur(dst, dst, cv::Size(3, 3), 0);
        auto radius = std::min(std::min((int)res.mRadius, dst.cols), dst.rows)*0.9;
        auto minR = radius * 0.2;

        for (float angle = mAngleStart; angle < mAngleEnd; angle += mAngleStep)
        {
            auto radian = xvt::Deg2Rad(angle);
            cv::Point2f vec(std::cos(radian), std::sin(radian));

            auto  points    = xvt::GetPoints(dst, res.mCenter+ vec*minR, res.mCenter + vec * radius, mRadiusStep);

            auto  intensity = xvt::GetIntensity((cv::Mat1b const&)dst, points);
            float variance  = xvt::GetVariance(intensity);
            varianceVec.push_back(variance);
        }

        if(mVarianceBlurSize%2>0)
            cv::blur(varianceVec, varianceVec, cv::Size(mVarianceBlurSize, 1));

        xvt::FindPeaks peakFinder(xvt::PeakType::Peak, xvt::PeakFindingMethod::Prominence, 100, false);
        peakFinder.Process(varianceVec);

        std::vector<xvt::Peak> peaks = peakFinder.GetPeakResult(mVarianceThreshold);

        std::sort(peaks.begin(), peaks.end(), xvt::GreaterProminence);

        int n = std::min(mAngleNo, (int)peaks.size());
        if (n > 0)
        {
            if(mAngleStep==1.0f)
            {
                for (int i = 0; i < n; i++)
                {
                    angles.push_back(peaks[i].index);
                }
            }
            else
            {
                for (int i = 0; i < n; i++)
                {
                    angles.push_back(peaks[i].index*mAngleStep);
                }
            }

            std::sort(angles.begin(), angles.end());
        }
    }

}

auto AngleSelector::GetSliceImage(Volume const& volume, int zSliceIdx, float r) const-> AngleSelectorResult
{
    AngleSelectorResult res;
    res.SetResult(EResult::OK);

    if (volume.size() < zSliceIdx) return res;

    cv::Mat const&  inImg = volume[zSliceIdx];
    cv::Mat &      resImg = res.mSliceImage;

    res.mCenter = cv::Point2f(inImg.size().width / 2, inImg.size().height / 2);
    if(r<0)
    {
        auto c = GetCircle(inImg, mCircleThreshold);
        if (c.IsOK())
        {
            res.mRadius = c.mRadius;
            res.mCenter = c.mCenter;
            r = res.mRadius * 1.05;
        }
        else
        {
            res.AddMsg("Can not find the center!");
            res.mRadius = std::min(inImg.size().width / 2, inImg.size().height / 2);
            r = res.mRadius;
        }
    }
    else
    {
        res.mRadius = r;
    }

    GetAngle(inImg, res);

    if (!res.mAngleList.empty())
    {
        resImg = GetSliceMPR(volume
                                  , res.mCenter
                                  , res.mAngleList
                                  , r
                                  , mAvgNo
                                  , SliceBlurType::ParrallelLine
                                  , mAvgStep
        );
    }

    res.mAvgNo = mAvgNo;
    return res;
}

void AngleSelectorResult::DrawResult(cv::Mat& drawImg, cv::Point offSetPoint, CVPen pen) const
{
    if (xvt::ConvertRGB(drawImg, drawImg, false))return;

    pen.mLineType = cv::LINE_AA;

    pen.mColor = cv::Scalar(0, 255, 255);

    cv::circle(drawImg, mCenter, mRadius, cv::Scalar(0, 255, 0), 2, pen.mLineType);
    for (auto const& angle : mAngleList)
    {
        auto p = mCenter + cv::Point2f(mRadius * cos(xvt::Deg2Rad(angle)), mRadius * sin(xvt::Deg2Rad(angle)));
        cv::line(drawImg, mCenter, p, cv::Scalar(0, 255, 0), 2, pen.mLineType);

        auto p1 = mCenter + cv::Point2f(mRadius * cos(xvt::Deg2Rad(angle + mAvgNo)), mRadius * sin(xvt::Deg2Rad(angle + mAvgNo)));
        auto p2 = mCenter + cv::Point2f(mRadius * cos(xvt::Deg2Rad(angle - mAvgNo)), mRadius * sin(xvt::Deg2Rad(angle - mAvgNo)));
        cv::line(drawImg, mCenter, p1, COLOR_CV_PINK, 1, pen.mLineType);
        cv::line(drawImg, mCenter, p2, COLOR_CV_PINK, 1, pen.mLineType);
    }

    xvt::DrawPlusSign(drawImg, mCenter, pen, 5);

    if(!mVarianceVec.empty())
    {
        auto rect = xvt::CreateROI(0, drawImg.rows * 0.95, drawImg.rows, drawImg.cols, drawImg.size());
        xvt::Drawing drawTool;
        drawTool.yTopOrigin = false;
        drawTool.thickness = pen.mThickness;
        auto img2 = drawImg(rect);
        drawTool.Plot(img2, rect.x, rect.width, mVarianceVec, false);
        
        VecFloat normVar;
        cv::normalize(mVarianceVec, normVar, mRadius, mRadius*1.08, cv::NormTypes::NORM_MINMAX);
        VecPoint2f tmpPoints;
        float angle = 0.0;
        float angleStrep = 360.0/ mVarianceVec.size();
        for (auto s : normVar)
        {
            auto p = mCenter + cv::Point2f(s* cos(xvt::Deg2Rad(angle)), s * sin(xvt::Deg2Rad(angle)));
            tmpPoints.push_back(p);
            angle += angleStrep;
        }
        for (int i = 1; i < tmpPoints.size(); i++)
        {
            cv::line(drawImg, tmpPoints[i - 1], tmpPoints[i], drawTool.color, pen.mThickness, pen.mLineType);
        }
    }
}

void AngleSelectorResult::DrawResult(cv::Mat& zImg, cv::Mat& sideImg, int zSliceIdx) const
{
    if (!xvt::ConvertRGB(zImg, zImg, false))
    {
        xvt::CVPen pen;
        DrawResult(zImg);
    }

    if (sideImg.empty()) xvt::ConvertRGB(mSliceImage, sideImg, true);
    if (!sideImg.empty())
    {
        cv::line(sideImg, cv::Point(0, zSliceIdx), cv::Point(sideImg.cols, zSliceIdx), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
}

auto GetCircle(cv::Mat const& img, int thd) -> xvt::CircleResult
{
    auto res = xvt::CircleResult();
    res.SetResult(EResult::ER);
    cv::Mat binImg;
    if (xvt::Convert8Bits(img, binImg)) return res;

    xvt::threshold::Threshold(binImg, binImg, thd, 255, cv::THRESH_BINARY);

    xvt::VecPoint ptns;
    cv::findNonZero(binImg, ptns);

    if (ptns.size() > 3)
    {
        cv::Point2f c3;
        double r3;
        double e3;

        xvt::CircleDetector cirDectect;
        cirDectect.FitLeastSquare(ptns, c3, r3, e3);

        res.mRadius = r3;
        res.mCenter = c3;

        res.SetResult(EResult::OK);
    }

#ifdef _DEBUG
    cv::Mat drawImg2;
    xvt::ConvertRGB(binImg, drawImg2);
    res.Draw(drawImg2, CVPen(COLOR_CV_RED, 2));
#endif // _DEBUG

    return res;
}

template<typename _T>
auto GetSlice(Volume const& img, xvt::VecPoint2f const& points)->cv::Mat
{
    int h = img.size();
    int w = points.size();
    auto type = img[0].type();

    cv::Mat resultImage = cv::Mat::zeros(h, w, type);
    assert(resultImage.elemSize1() == sizeof(_T));

    #pragma omp parallel for
    for (int r = 0; r < h; r++)
    {
        auto iten1 = GetIntensity((cv::Mat_<_T> const&)img[r], points, true);
        for (int c = 0; c < w; c++)
        {
            resultImage.at<_T>(r, c) = iten1[c];
        }
    }

    return resultImage;
}

auto GetSliceImage(Volume const& volume, xvt::VecPoint2f const& points)->cv::Mat
{
    if (volume.empty() || points.empty()) return cv::Mat();

    auto type = volume[0].type();

    cv::Mat resultImage;

    switch (type)
    {
        case CV_8UC1:
            resultImage = GetSlice<uchar>(volume, points);
            break;
        case CV_16UC1:
            resultImage = GetSlice<ushort>(volume, points);
            break;
        case CV_32FC1:
            resultImage = GetSlice<float>(volume, points);
            break;
        default:
            break;
    }

    return resultImage;
}

auto GetSliceImage(Volume const& volume, xvt::Line const& l) -> cv::Mat
{
    cv::Mat res;
    if (volume.empty() || volume[0].empty()) return res;
    int w = volume[0].cols;
    int h = volume[0].rows;
    cv::Rect rect = cv::Rect(0, 0, w, h);
    auto pnts = xvt::Intersect(l, rect);
    if (pnts.size() > 1)
    {
        auto points1 = xvt::GetPoints(volume[0], pnts[0], pnts[1], 1.0f);
        res =  GetSliceImage(volume, points1);
    }

    return res;
}

auto GetSliceImage(Volume const& volume
                   , cv::Point2f center
                   , std::vector<float> angles
                   , int   size
                   , float scale
)->cv::Mat
{
    //xvt::ScopeTimer t("GetSlice");
    if (volume.empty() || size < 1 || angles.size() < 1) return cv::Mat();

    int w = volume[0].cols;
    int h = volume[0].rows;
    int minSize = std::min(std::min(w - center.x, center.x), std::min(h - center.y, center.y));
    if(size<=0)
    {
        size = minSize;
    }

    auto angle1 = angles[0];
    auto angle2 = angles.size() > 1 ? angles[0] : angle1 + 180.0;

    auto points1 = xvt::GetPoints(volume[0], center, angle1, size, scale, true);
    auto points2 = xvt::GetPoints(volume[0], center, angle2, size, scale, true);
    if (!points1.empty())
        std::reverse(points1.begin(), points1.end());
    if (!points2.empty())
        points1.insert(points1.end(), points2.begin(), points2.end());

    return GetSliceImage(volume, points1);
}

auto GetSliceImage(Volume const& volume
                   , cv::Point2f center
                   , float       angle
                   , int         size
                   , float       scale
                   , bool        isHalf
)->cv::Mat
{
    if (volume.empty() || size < 1) return {};

    auto  lineVec = cv::Point2f(cos(xvt::Deg2Rad(angle)), sin(xvt::Deg2Rad(angle)));
    if(!isHalf) center -= size * lineVec;
    return GetSliceImage(volume, center, lineVec, isHalf ? size : size * 2);
}

auto GetSliceImage(Volume const& volume
                   , cv::Point2f start
                   , cv::Point2f dir
                   , int         size
                   , float       scale
) -> cv::Mat
{
    auto points1 = xvt::GetPoints(volume[0], start, dir, size, scale, true);
    return  GetSliceImage(volume, points1);
}

auto GetSliceMPRAngle(Volume const& volume
                      , cv::Point2f center
                      , float       angle
                      , int         size
                      , float       d
                      , float       step
                      , bool        cvt28bits
                      , bool        isHalf
)->cv::Mat
{
    if (volume.empty() || size < 1) return {};

    cv::Mat1f sumImg;
    if (d == 0 || step == 0)
    {
        sumImg = GetSliceImage(volume, center, angle, size, isHalf);
    }
    else
    {
        int k = int(d / step) * 2 + 1;
        VecMat list(k, cv::Mat());
        angle -= step * (int(k / 2));

        #pragma omp parallel for
        for (int i = 0; i < k; i++)
        {
            list[i] = GetSliceImage(volume, center, angle + i * step, size, 1.0f, isHalf);
        }

        sumImg = cv::Mat1f::zeros(list[0].size());

        for (auto& m : list)
        {
            bool isAdd = !m.empty() && m.cols == sumImg.cols && m.rows == sumImg.rows;
            assert(isAdd);
            if (isAdd)
            {
                sumImg += m;
            }
        }

        if (!list.empty())
            sumImg /= (float)list.size();
    }

    if (cvt28bits)
    {
        xvt::Convert8Bits(sumImg, sumImg);
    }
    return sumImg;
}

auto GetSliceMPRLine(Volume const& volume
                      , cv::Point2f start
                      , cv::Point2f dir
                      , int         size
                      , float       d
                      , float       step
                      , bool        cvt28bits
) -> cv::Mat
{
    //xvt::ScopeTimer t("GetSlice parrallel");
    cv::Mat1f sumImg;
    auto l = cv::norm(dir);
    if (l == 0 || size < 1) return sumImg;

    if (d == 0 || step == 0)
    {
        sumImg = GetSliceImage(volume, start, dir, size);
    }
    else
    {
        dir /= l;

        auto n = cv::Point2f(dir.y, -dir.x) * step;

        d = abs(d);
        step = abs(step);

        int k = int(d / step) * 2 + 1;
        VecMat list(k, cv::Mat());
        start -= n * (int(k/2));
        #pragma omp parallel for
        for (int i = 0; i < k; i ++)
        {
            list[i] = GetSliceImage(volume, start + i * n, dir, size);
        }

        sumImg = cv::Mat1f::zeros(list[0].size());

        for(auto & m:list)
        {
            bool isAdd = !m.empty() && m.cols == sumImg.cols && m.rows == sumImg.rows;
            assert(isAdd);
            if(isAdd)
            {
                sumImg += m;
            }
        }

        if (!list.empty())
            sumImg /= (float)list.size();
    }

    if (cvt28bits)
    {
        xvt::Convert8Bits(sumImg, sumImg);
    }
    return sumImg;
}

auto GetSliceMPR(Volume const& volume
                      , cv::Point2f center
                      , float angle
                      , int size
                      , float blurSize
                      , SliceBlurType blurType
                      , float blurStep
                      , bool cvt28bits
)->cv::Mat
{
    if (volume.empty() || size < 1) return {};
    auto radAngle = xvt::Deg2Rad(angle);
    auto  dir = cv::Point2f(cos(radAngle), sin(radAngle));

    cv::Mat res;
    switch (blurType)
    {
        case xvt::ct::SliceBlurType::Angle:
        {
            res = GetSliceMPRAngle(volume, center, angle, size, blurSize, blurStep, cvt28bits, false);
            break;
        }
        case xvt::ct::SliceBlurType::ParrallelLine:
        {
            auto  start = center - size * dir;
            res = GetSliceMPRLine(volume, start, dir, size * 2, blurSize, blurStep, cvt28bits);
            break;
        }
        default:
            break;
    }

    return res;
}

auto GetSliceMPR(Volume const&        volume
                 , cv::Point2f        center
                 , std::vector<float> angleList
                 , int                size
                 , float              blurSize
                 , SliceBlurType      blurType
                 , float              blurStep
                 , bool               cvt28bits
) -> cv::Mat
{
    //xvt::ScopeTimer t("GetSlice Angle");
    cv::Mat resImg;
    if (size < 1 || angleList.empty()) return resImg;

    auto angle1 = angleList[1];
    auto angle2 = angleList.size() > 1 ? angleList[0] : angle1 + 180.0;

    if (volume.empty() || size < 1) return {};
    auto radAngle1 = xvt::Deg2Rad(angle1);
    auto radAngle2 = xvt::Deg2Rad(angle2);

    auto  dir1 = cv::Point2f(cos(radAngle1), sin(radAngle1));
    auto  dir2 = cv::Point2f(cos(radAngle2), sin(radAngle2));

    cv::Mat img1, img2;
    switch (blurType)
    {
        case xvt::ct::SliceBlurType::Angle:
        {
            img1 = GetSliceMPRAngle(volume, center, angle1, size, blurSize, blurStep, false, true);
            img2 = GetSliceMPRAngle(volume, center, angle2, size, blurSize, blurStep, false, true);
            break;
        }
        case xvt::ct::SliceBlurType::ParrallelLine:
        {
            img1 = GetSliceMPRLine(volume, center, dir1, size, blurSize, blurStep, false);
            img2 = GetSliceMPRLine(volume, center, dir2, size, blurSize, blurStep, false);
            break;
        }
        default:
            break;
    }

    if(!img1.empty() && !img2.empty())
    {
        cv::flip(img2, img2, 1);
        cv::hconcat(img2, img1, resImg);

        if (cvt28bits)
        {
            xvt::Convert8Bits(resImg, resImg);
        }
    }

    return resImg;
}

Slicer::Slicer(cv::Point2f c, float angle, int size, float blurSize, float blurStep, SliceBlurType blurType, bool cvt8bit):
    mCenter{ c }, mAngle{ angle }, mSize{ size }, mBlurSize{ blurSize }, mBlurStep{ blurStep }, mIsConvert8bit{ cvt8bit }, mBlurType{blurType}
{
}

auto Slicer::GetSlice(Volume const& volume) const -> cv::Mat
{
    return xvt::ct::GetSliceMPR(volume, mCenter, mAngle, mSize, mBlurSize, mBlurType, mBlurStep, mIsConvert8bit);
}

auto Slicer::GetSlice(Volume const& volume, float angle) & -> cv::Mat
{
    mAngle = angle;
    return xvt::ct::GetSliceMPR(volume, mCenter, mAngle, mSize, mBlurSize, mBlurType, mBlurStep, mIsConvert8bit);
}

void Slicer::Draw(cv::Mat& img, CVPen pen) const
{
    if (xvt::ConvertRGB(img, img, false)) return;

    xvt::DrawPlusSign(img, mCenter, pen);

    auto radAngle = xvt::Deg2Rad(mAngle);
    auto dir   = cv::Point2f(cos(radAngle), sin(radAngle));
    auto start = mCenter + dir * mSize;
    auto end   = mCenter - dir * mSize;

    cv::line(img, start, end, COLOR_CV_YELLOW, pen.mThickness, cv::LineTypes::LINE_AA);

    switch (mBlurType)
    {
        case xvt::ct::SliceBlurType::Angle:
        {
            auto p1 = mSize*cv::Point2f(cos(xvt::Deg2Rad(mAngle + mBlurSize)), sin(xvt::Deg2Rad(mAngle + mBlurSize)));
            auto p2 = mSize*cv::Point2f(cos(xvt::Deg2Rad(mAngle - mBlurSize)), sin(xvt::Deg2Rad(mAngle - mBlurSize)));

            cv::line(img, mCenter - p1, mCenter + p1, COLOR_CV_PINK, pen.mThickness, cv::LineTypes::LINE_AA);
            cv::line(img, mCenter - p2, mCenter + p2, COLOR_CV_PINK, pen.mThickness, cv::LineTypes::LINE_AA);
            break;
        }
        case xvt::ct::SliceBlurType::ParrallelLine:
        {
            auto n = cv::Point2f(-dir.y, dir.x)* mBlurSize;
            cv::line(img, start - n, end - n, COLOR_CV_CYAN  , pen.mThickness, cv::LineTypes::LINE_AA);
            cv::line(img, start + n, end + n, COLOR_CV_CYAN  , pen.mThickness, cv::LineTypes::LINE_AA);
            break;
        }
        default:
            break;
    }
}

}//End namspace ct
}//End namspace xvt

