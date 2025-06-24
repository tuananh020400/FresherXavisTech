#include "xvtCV/xvtIntensityAnalyzer.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Utils.h"
#include "xvtCV/xvtConvert.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
namespace xvt {

template<typename T>
auto GetIntensity(const cv::Mat_<T>& image, const std::vector<cv::Point2f> const& points, bool isKeepOuter=false) -> std::vector<T>
{
    std::vector<T> intensityVec;
    cv::Rect rect(0, 0, image.cols - 1, image.rows - 1);
    if (rect.empty()) return intensityVec;
    intensityVec.reserve(points.size());
    for (const auto& p : points)
    {
        if (rect.contains(p))
        {
            cv::Point p00(p.x, p.y);
            cv::Point p01(p00.x + 1, p00.y);
            cv::Point p10(p00.x, p00.y + 1);
            cv::Point p11(p00.x + 1, p00.y + 1);

            double wx = p.x - (double)p00.x;
            double wy = p.y - (double)p00.y;

            double I00 = image(p00) * (1.f - wx) * (1.f - wy);
            double I01 = image(p01) * (1.f - wx) * wy;
            double I10 = image(p10) * wx * (1.f - wy);
            double I11 = image(p11) * wx * wy;

            intensityVec.push_back(I00 + I10 + I01 + I11);
        }
        else if(isKeepOuter)
        {
            intensityVec.push_back(0);
        }
    }

    //cv::Mat tmp(1, (int)intensityVec.size(), CV_32FC1, intensityVec.data());
    return intensityVec;
}

auto GetIntensity(const cv::Mat1b& image, const std::vector<cv::Point2f> const& points, bool isKeepOuter) -> std::vector<uchar>
{
    return GetIntensity<uchar>(image, points, isKeepOuter);
}

auto GetIntensity(const cv::Mat1w& image, const std::vector<cv::Point2f> const& points, bool isKeepOuter) -> std::vector<ushort>
{
    return  GetIntensity<ushort>(image, points, isKeepOuter);
}

auto GetIntensity(const cv::Mat1f& image, std::vector<cv::Point2f> const& points, bool isKeepOuter) -> std::vector<float>
{
    return GetIntensity<float>(image, points, isKeepOuter);
}

auto GetPoints(cv::Mat const& img, cv::Point2f start, cv::Point2f dir, int size, float scale, bool isKeepOuter)->std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;

    cv::Rect rect(0, 0, img.cols - 1, img.rows - 1);
    double l = cv::norm(dir);
    if (l > DBL_EPSILON && size > 1)
    {
        scale = abs(scale);
        dir /= l;
        dir *= scale;

        points.reserve(size + 1);
        for (int i = 0; i < size; i++)
        {
            auto p = (start + i * dir);
            if (rect.contains(p) || isKeepOuter)
            {
                points.push_back(p);
            }
        }
    }

    return points;
}

auto GetPoints(cv::Mat const& img, cv::Point2f start, cv::Point2f end, float scale, bool isKeepOuter)->std::vector<cv::Point2f>
{
    auto v = end - start;
    int size = cv::norm(v);

    return GetPoints(img, start, v, size, scale, isKeepOuter);
}

auto GetPoints(cv::Mat const& img, cv::Point2f center, float angle, int size, float scale, bool isKeepOuter)->std::vector<cv::Point2f>
{
    float angle_radian = xvt::Deg2Rad(angle);
    cv::Point2f vec(std::cos(angle_radian), std::sin(angle_radian));

    return GetPoints(img, center, vec, size, scale, isKeepOuter);
}

auto IntensityAnalyzer::Load(const std::wstring& settingPath) & ->bool
{
    std::ifstream infile(settingPath);
    std::string line;
    bool rtn = infile.is_open();
    if (rtn)
    {
        xvt::VecRect tmpRois;
        try
        {
            while (std::getline(infile, line))
            {
                if (line.empty() || line[0] == '#' || line[0] == '/')
                {
                    continue;
                }
                else
                {
                    std::vector<int> roiValues;
                    std::istringstream iss(line);
                    std::string token;
                    cv::Rect rect;
                    while (std::getline(iss, token, ','))
                    {
                        std::string valueStr = xvt::TrimSpace(token);
                        if (!valueStr.empty())
                        {
                            int value = std::stoi(valueStr);
                            roiValues.push_back(value);
                        }
                    }

                    if (roiValues.size() == 4)
                    {
                        tmpRois.emplace_back(roiValues[0], roiValues[1], roiValues[2], roiValues[3]);
                    }
                }
            }

            std::swap(tmpRois, mRoiList);
        }
        catch(std::exception& ex)
        {
            rtn = false;
        }
    }

    mEnable = rtn;
    return rtn;
}

auto IntensityAnalyzer::Inspect(cv::Mat const& inImg) const-> std::unique_ptr<IInspectionResult>
{
    ScopeTimer t("ITA");

    auto result = std::make_unique<IntensityAnalyzerResult>();
    result->mEnable = mEnable;
    if (!mEnable) return result;

    if (!inImg.empty())
    {
        result->SetResult(EResult::OK);
        bool res = true;

        // Loop through each ROI
        for (const auto& r : mRoiList)
        {
            cv::Rect roi = xvt::CreateROI(r.x, r.y, r.width, r.height, inImg.size());
            RoiInfo t;
            t.mSetRoi = r;
            t.mUseRoi = roi;
            if (!roi.empty())
            {
                cv::Mat src = inImg(roi);
                cv::Mat roiImg;
                bool isConvertOk = true;
                if (mIsUse8Bit)
                {
                    isConvertOk = !xvt::Convert8Bits(src, roiImg, false);
                }
                else
                {
                    roiImg = src;
                }

                if (isConvertOk)
                {
                    double minIntensity, maxIntensity;
                    cv::minMaxLoc(roiImg, &minIntensity, &maxIntensity);

                    // Calculate standard deviation
                    cv::Scalar mean, stddev;
                    cv::meanStdDev(roiImg, mean, stddev);
                    double stdIntensity = stddev[0];
                    double meanIntensity = mean[0];

                    t.mAvg = meanIntensity;
                    t.mMin = minIntensity;
                    t.mMax = maxIntensity;
                    t.mStd = stdIntensity;
                }
            }
            else
            {
                res = false;
            }
            result->mRois.push_back(t);
        }

            if (!res)
            {
                result->Combine(EResult::NG, "Analysis Intensity: There is empty ROI");
            }
    }
    else
    {
        result->Combine(EResult::ER,"Analysis Intensity: Image Empty");
        // Handle the case when inImg is empty
    }

    result->mProTime = t.Stop().count();
    return result;
}

auto IntensityAnalyzer::Inspect(std::vector<cv::Mat> const& imgList) const->std::unique_ptr<IInspectionResult>
{
    return std::make_unique<InspectionResult>(EResult::ER, "Not implemented");
}

auto IntensityAnalyzer::Save(std::wstring const& path)const->bool
{
    return false;
}

auto IntensityAnalyzer::Clone() const -> std::unique_ptr<IInspection>
{
    return std::make_unique<IntensityAnalyzer>(*this);
}

auto IntensityAnalyzerResult::GetResultStr() const -> std::string
{
    std::string tmp = IInspectionResult::GetResultStr() + "\n";
    CSVOutput data;
    GetCSVData(data, "", true);

    if (!data.empty())
    {
        tmp += data[0].first + ":" + xvt::ToString(data[0].second);
        for (size_t i = 1, size = data.size(); i < size; i++)
        {
            tmp += (i % 4 != 0) ? ", " : "\n";
            tmp += data[i].first + ":" + xvt::ToString(data[i].second);
        }
    }

    return tmp;
}

void IntensityAnalyzerResult::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty()|| !mEnable) return;

    if (!xvt::ConvertRGB(img, img, false))
    {
        pen.mColor = GetResultColor();
        for (const auto& r : mRois)
        {
            cv::rectangle(img, r.mSetRoi + offSetPoint, pen.mColor, pen.mThickness);
        }
    }
}

auto IntensityAnalyzerResult::GetCSVData(VecKeyValueStr& out, std::string prefix, bool isRecursive) const -> void
{
    if (mRois.empty() || !isRecursive) return;

    size_t targetCapacity = out.size() + mRois.size() * 2;
    if (out.capacity() < targetCapacity)
        out.reserve(targetCapacity);

    for (const auto& mRoi : mRois)
    {
        mRoi.GetCSVData(out, prefix, isRecursive);
    }
}

auto RoiInfo::GetCSVData(VecKeyValueStr& out, std::string prefix, bool isRecursive) const -> void
{
    out.emplace_back("avg", xvt::ToString(mAvg, 2));
    out.emplace_back("min", xvt::ToString(mMin, 2));
    out.emplace_back("max", xvt::ToString(mMax, 2));
    out.emplace_back("std", xvt::ToString(mStd, 2));
}

}