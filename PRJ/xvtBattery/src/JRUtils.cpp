#include "xvtBattery/JRUtils.h"

namespace xvt {
namespace battery {

cv::Point FlipYPoint(cv::Point p1, const cv::Size &size)
{
    return cv::Point(size.width - p1.x - 1, p1.y);
}

cv::Point FlipXPoint(cv::Point p1, const cv::Size &size)
{
    return cv::Point(p1.x, size.height - p1.y - 1);
}

cv::Point TransposePoint(cv::Point p1)
{
    return cv::Point(p1.y, p1.x);
}

void FlipXPoints(std::vector<cv::Point> &vtPoints, const cv::Size &size)
{
    for (auto &p : vtPoints)
        p = FlipXPoint(p, size);
}

void FlipYPoints(std::vector<cv::Point> &vtPoints, const cv::Size &size)
{
    for (auto &p : vtPoints)
        p = FlipYPoint(p, size);
}

void TransposePoints(std::vector<cv::Point> &vtPoints)
{
    for (auto &p : vtPoints)
        p = TransposePoint(p);
}

cv::Point TranformPoint(cv::Point p, const cv::Size &size)
{
    cv::Point pDst;
    pDst = TransposePoint(p);
    pDst = FlipYPoint(pDst, cv::Size(size.height, size.width));
    return pDst;
}

cv::Point TranformPointInvert(cv::Point p, const cv::Size &size)
{
    cv::Point pDst;
    pDst = FlipYPoint(p, size);
    pDst = TransposePoint(pDst);
    return pDst;
}

std::vector<cv::Point> TranformPointsInvert(std::vector<cv::Point> vtPoints, const cv::Size &size, cv::Point offset)
{
    std::vector<cv::Point> vtpDst;

    if (!vtPoints.empty() && !size.empty())
    {
        if (offset.x != 0 && offset.y != 0)
            vtpDst = xvt::Transform(vtPoints, offset);

        for (auto &p : vtpDst)
        {
            p = TranformPointInvert(p, size);
        }
    }

    return vtpDst;
}

cv::Rect TranformRect(cv::Rect r, const cv::Size &size)
{
    cv::Point tl = r.tl();
    cv::Point br = r.br();
    cv::Point new_tl = TranformPoint(tl, size);
    cv::Point new_br = TranformPoint(br, size);

    tl = cv::Point(new_br.x, new_tl.y);
    br = cv::Point(new_tl.x, new_br.y);

    return cv::Rect(tl, br);
}

cv::Rect TranformRectInvert(cv::Rect r, const cv::Size &size)
{
    cv::Point tl = r.tl();
    cv::Point br = r.br();
    cv::Point new_tl = TranformPointInvert(tl, size);
    cv::Point new_br = TranformPointInvert(br, size);

    tl = cv::Point(new_tl.x, new_br.y);
    br = cv::Point(new_br.x, new_tl.y);

    return cv::Rect(tl, br);
}

bool BatteryPositionIsLeft(cv::Mat src, cv::Rect r)
{
    bool batpos = false;
    if (!src.empty())
    {
        cv::Mat sumImg;
        cv::Scalar sumMatL = cv::sum(src(cv::Rect(r.x, r.y, r.width / 2, r.height)));
        cv::Scalar sumMatR = cv::sum(src(cv::Rect(r.x + r.width / 2, r.y, r.width / 2, r.height)));
        if (sumMatL[0] > sumMatR[0])
        {
            batpos = true;
        }
    }

    return batpos;
}
}
}