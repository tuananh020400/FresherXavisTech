#include "xvtCV/ImageQualityMeasure.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Peak.h"
#include <numeric>
#include <fstream>

namespace xvt {

LinePairs::LinePairs()
{
    mStep = 10;
    mMinProminance = 0.0f;
    mMinDistance = 0.0f;
    mLineColor = false;
    mNumOfLinePairs = 1;
}

auto FilterPeak(std::vector<xvt::Peak>& listPeak, int NumofLp, float offset)
{
    if (!listPeak.empty())
    {
        std::partial_sort(listPeak.begin(), listPeak.begin() + NumofLp, listPeak.end(), [](auto& a, auto& b)
            {
                return a.value > b.value;
            });
        float meanValPeak = std::accumulate(listPeak.begin(), listPeak.begin() + NumofLp, 0.0, [](double acc, auto& p)
            {return acc + p.value; }) / NumofLp;

        listPeak.erase(std::remove_if(listPeak.begin(), listPeak.end(), [&meanValPeak, &offset](auto& p)
            {return p.value < meanValPeak * (1.0f - offset); }), listPeak.end());
        std::sort(listPeak.begin(), listPeak.end(), [](auto& p1, auto& p2) {return p1.index < p2.index; });
    }
}

auto LinePairs::CalculateLinePairContrast(const cv::Mat& inputImage) const->std::unique_ptr<LinePairsResult>
{
    auto result = std::make_unique<LinePairsResult>();
    result->SetResult(EResult::OK);
    int col = inputImage.cols;
    int row = inputImage.rows;
    for (int i = 0; i < col; i += mStep)
    {
        std::vector<float> listData = inputImage(cv::Range::all(), cv::Range(i, i + 1));
        if (!listData.empty())
        {
            cv::GaussianBlur(listData, listData, cv::Size(5, 5), 0.0);
            xvt::FindPeaks peak;
            peak = xvt::FindPeaks(xvt::PeakType::Peak, xvt::PeakFindingMethod::Prominence);
            peak.Process(listData);
            std::vector<xvt::Peak> listPeak = peak.GetPeakResult(mMinProminance, mMinDistance);
            xvt::FindPeaks valley;
            valley = xvt::FindPeaks(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
            valley.Process(listData);
            std::vector<xvt::Peak> listValley = valley.GetPeakResult(mMinProminance, mMinDistance);

            if (!listPeak.empty() && !listValley.empty())
            {

                FilterPeak(listPeak, mNumOfLinePairs, mOffset);
                int sizePeak = listPeak.size();
                int idx1 = listPeak[0].index;
                int idx2 = listPeak[sizePeak - 1].index;
                listValley.erase(std::remove_if(listValley.begin(), listValley.end(), [&idx1, &idx2](auto& p) {return p.index < idx1 || p.index > idx2; }), listValley.end());
                int sizeValley = listValley.size();

                // Calculate mean value of all peaks
                double meanValPeak = std::accumulate(listPeak.begin(), listPeak.end(), 0.0, [](double acc, auto& p)
                    {return acc + p.value; }) / sizePeak;
                // Calculate mean value of all valleys
                double meanValValley = std::accumulate(listValley.begin(), listValley.end(), 0.0, [](double acc, auto& p)
                    {return acc + p.value; }) / sizeValley;

                // Calculate contrast C = (Imax - Imin) /  (Imax + Imin)
                double contrast = (meanValPeak - meanValValley) / (meanValPeak + meanValValley);
                result->mListContrast.push_back(contrast);

                // Find start idx 
                float meanStart = (listPeak[0].value + listValley[0].value) / 2.0f;
                int startS = std::abs(listPeak[0].index - (listValley[0].index - listPeak[0].index));
                int startE = listPeak[0].index;

                if (startS > startE) { std::swap(startS, startE); }
                auto itStart = std::min_element(listData.begin() + startS, listData.begin() + startE, [&meanStart]
                (auto& a, auto& b) {return std::abs(a - meanStart) < std::abs(b - meanStart); });

                int idxStart = std::distance(listData.begin(), itStart);


                float meanEnd = (listPeak[sizePeak - 1].value + listValley[sizeValley - 1].value) / 2.0f;
                int endS = listValley[sizeValley - 1].index;
                int endE = listPeak[sizePeak - 1].index;
                if (endS > endE) { std::swap(endS, endE); }
                auto itEnd = std::min_element(listData.begin() + endS, listData.begin() + endE, [&meanEnd]
                (auto& a, auto& b) {return std::abs(a - meanEnd) < std::abs(b - meanEnd); });

                int idxEnd = std::distance(listData.begin(), itEnd);

                double lpMM = float(mNumOfLinePairs - 0.5) / static_cast<float>(idxEnd - idxStart);
                result->mListLinePair.push_back(lpMM);
            }
            else
            {
                result->SetResult(EResult::ER);
                result->SetMsg("Peak list or valley list is empty!");
            }
        }
        else
        {
            result->SetResult(EResult::ER);
            result->SetMsg("Data intensity is column !" + std::to_string(i) + "is empty!");
        }
    }
    return result;
}

auto LinePairs::Inspect(cv::Mat const& inImg) const->std::unique_ptr<IInspectionResult>
{
    ScopeTimer t("Time");
    auto result = std::make_unique<LinePairsResult>();
    if (mNumOfLinePairs <= 0) {
        result->SetResult(EResult::ER);
        result->SetMsg("Number of linepair is negative. It must be greater zero!");
        return result;
    }
    else
    {
        if (inImg.empty())
        {
            result->SetResult(EResult::ER);
            result->SetMsg("Image is empty!");
            return result;
        }
        else
        {
            cv::Mat inputImg;
            if (inImg.channels() == 3)
            {
                cv::cvtColor(inImg, inputImg, cv::COLOR_BGR2GRAY);
            }
            else {
                if(inImg.channels() == 1)  inputImg = inImg;
                else
                {
                    result->SetResult(EResult::ER);
                    result->SetMsg("Do not support image type!");
                    return result;
                }
               
            }
            if (mLineColor) { cv::bitwise_not(inputImg, inputImg); }
            result = LinePairs::CalculateLinePairContrast(inputImg);
        }
    }
    result->mProTime = t.Stop().count();
    return result;
}

auto LinePairs::Inspect(std::vector<cv::Mat> const& imgList) const->std::unique_ptr<IInspectionResult>
{
    return std::make_unique<InspectionResult>(EResult::ER, "Not implemented");
}

auto LinePairs::Save(std::wstring const& path)const->bool
{
    return false;
}

auto LinePairs::Load(const std::wstring& settingPath) & -> bool
{
    return false;
}

auto LinePairs::Clone() const -> std::unique_ptr<IInspection>
{
    return std::make_unique<LinePairs>(*this);
}

LinePairsResult::LinePairsResult()
{
    mListContrast = {};
    mListLinePair = {};
    mPixelSize = 1;
}

auto LinePairsResult::Convert2LinePairPerMM() const->std::vector<double>
{
    std::vector<double> result(mListLinePair.size());
    double pxSize = mPixelSize;
    if (!mListLinePair.empty())
    {
        std::transform(mListLinePair.begin(), mListLinePair.end(), result.begin(), [&pxSize](auto& lp) {return lp / pxSize; });
    }
    return result;
}

auto LinePairsResult::GenerateCSVResult(const std::string& filePath) const->void
{
    if (mListLinePair.size() == mListContrast.size())
    {
        std::ofstream file(filePath);
        if (file.is_open())
        {
            file << "Contrast,LinePair/px\n";
            int size = mListContrast.size();
            for (size_t i = 0; i < size; ++i) {
                file << mListContrast[i] << "," << mListLinePair[i] << "\n";
            }

            // Close the file
            file.close();
            std::cout << "Data saved to " << filePath << std::endl;
        }
        else
        {
            std::cout << "Fail to open file!!" << std::endl;
        }
    }
}
}