#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Contour.h"
#include "xvtCV/Utils.h"
#include "xvtCV/xvtInspection.h"
#include "xvtCV/xvtCSV.h"
#include "opencv2/imgproc.hpp"

namespace xvt {

InspectionResult::InspectionResult(xvt::EResult result, std::string const& msg)
{
    SetResult(result);
    SetMsg(msg);
    mDateTime = xvt::GetCurrentTime();
}

auto InspectionResult::GetResultStr() const -> std::string
{
    std::string res = IInspectionResult::GetResultStr();
    CSVOutput resData;
    GetCSVData(resData, "", false);
    if (!resData.empty())
    {
        res += ": " + GetKeyValueStr(resData);
    }
    return res;
}

void InspectionResult::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    if (!mEnable) return;
    if (img.empty()) return;
    if (img.channels() == 1)
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    auto color = GetResultColor();
    auto roi = GetROI() + offSetPoint;
    if (mShowRoi || !IsOK()) cv::rectangle(img, roi, color, pen.mThickness, pen.mLineType);
    cv::Mat roiImg = roi.empty() ? img : xvt::GetImage(img, roi);
    xvt::DrawPoints(roiImg, mPoints, cv::Vec3b(color[0], color[1], color[2]));
}

auto InspectionResult::DrawResultStr(cv::Mat& image
                                     , std::string const& name
                                     , CVPen const& pen
                                     , cv::Point const& pos
                                     , bool isDrawOKResult
) const->cv::Point
{
    //XVT_MESUARE_PERFORMANCE();
    auto tmpPen = pen;
    tmpPen.mColor = GetResultColor();
    const int tabSize = tmpPen.GetTabSize();
    auto tmpPos = pos;
    if (isDrawOKResult || !IsOK())
    {
        auto restText = GetResultStr();
        tmpPos.y += tmpPen.mSpace;
        auto firstEndPos = restText.find('\n');
        auto firstLine = restText.substr(0, firstEndPos);
        tmpPos = DrawText(image, !name.empty() ? name + ": " + firstLine : firstLine, tmpPos, tmpPen);

        if (firstEndPos != std::string::npos)
        {
            auto subResult = restText.substr(firstEndPos + 1);
            tmpPos.x = pos.x + tabSize;
            tmpPos.y += tmpPen.mSpace;
            tmpPos = DrawText(image, subResult, tmpPos, tmpPen);
        }
    }

    tmpPos = cv::Point(pos.x + tabSize, tmpPos.y);
    tmpPos = DrawMsg(image, tmpPen, tmpPos);

    tmpPos.x = pos.x;
    return tmpPos;
}

auto InspectionResult::DrawMsg(cv::Mat& image, CVPen const& pen, cv::Point const& pos) const->cv::Point
{
    //XVT_MESUARE_PERFORMANCE();
    cv::Point startingPoint = pos;
    auto msg = GetMsg();

    if (!msg.empty())
    {
        startingPoint.y += pen.mSpace;
        startingPoint = DrawText(image, msg, startingPoint, pen);
    }

    return cv::Point(pos.x, startingPoint.y);
}

bool InspectionResult::Save(std::wstring const& path, std::wstring const& imgName, bool isNewFile) const
{
    bool rtn = false;
    auto ext = xvt::GetFileExtension(path);

    if (ext == L".csv")
    {
        rtn = CSV::Save(path, imgName, isNewFile);
    }
    return rtn;
}

auto InspectionResult::Transform(cv::Mat const& m) & ->void
{
    if (!m.empty() && !mPoints.empty())
    {
        mPoints = xvt::Transform(mPoints, m);
    }
}

auto InspectionResult::GetCornerPoint() const->std::array<cv::Point, 4>
{
    auto tmp = std::array<cv::Point, 4>();
    FindCornerPoints(mPoints, tmp[0], tmp[1], tmp[2], tmp[3]);
    return tmp;
}

auto InspectionResult::GetCornerPointIdx() const->std::vector<int>
{
    return FindCornerPointsIdx(mPoints);
}

}