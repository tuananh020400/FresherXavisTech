#include "xvtBattery/BatteryInspectorCathode.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/AStar.h"
#include "xvtCV/Peak.h"
#include "xvtCV/Utils.h"
#include <xvtCV/ColorDefine.h>
#include <CvPlot/cvplot.h>

namespace xvt {
namespace battery {

auto BatteryInspectorCathode::Inspect(cv::Mat const &inImg, cv::Rect roi, std::vector<PoleInfo> &poles) const -> BatteryInspectorCathodeResult
{
    auto ispResult = BatteryInspectorCathodeResult();

    ispResult.mEnable = mEnable;

    if (!ispResult.mEnable)
    {
        ispResult(EResult::UC, "Disabled!");
        return ispResult;
    }

    cv::Rect subRoi = CreateROI(roi.tl(), roi.size(), inImg.size());
    cv::Mat src;
    ispResult.SetROI(subRoi);
    if (!xvt::Convert8Bits(inImg, src, false) && !subRoi.empty())
    {
        // 5: Applied Local Thresholding for determine the ending Point of Anode Poles
        cv::Mat imgROI4_0, imgEnhanced;
        cv::bilateralFilter(src(subRoi), imgROI4_0, 5, 55, 5);

        auto cathodeLineResult = FindCathodeLine(imgROI4_0, ispResult.mPoints);

        if (cathodeLineResult.IsOK())
        {
            imgEnhanced = imgROI4_0;
            if (mIsApplyEnhanced)
            {
                cv::Ptr<cv::CLAHE> claheTool4Cathode = cv::createCLAHE(15, cv::Size(75, imgEnhanced.rows));
                claheTool4Cathode->apply(imgEnhanced, imgEnhanced);
            }

            auto cathodeXPosResult = FindCathodeXPosition(imgEnhanced, ispResult.mPoints, ispResult.mContour, poles);
            ispResult.CombineResult(&cathodeXPosResult);
        }
        else
        {
            ispResult.CombineResult(&cathodeLineResult);
        }
    }
    else
    {
        ispResult(EResult::ER, "Inspect: Image type is not supported!");
    }

    return ispResult;
}

auto BatteryInspectorCathode::FindCathodeLine(const cv::Mat &srcImg, VecPoint &lstCathodeLine) const -> InspectionResult
{
    InspectionResult finalResult;

    const int minImageHeight = 5;
    if (!srcImg.empty() && srcImg.rows > minImageHeight)
    {
        bool isUseAutoAlgorithm = (mCathodeLineThresholdInner == 0 && mCathodeLineThresholdMiddle == 0 && mCathodeLineThresholdOuter == 0);

        if (mEnableAutoMode || isUseAutoAlgorithm)
        {
            finalResult = FindCathodeLineByAStar(srcImg, lstCathodeLine, GetJR().mCenterNeglectionWidth, GetJR().mNoRegion);
        }
        else
        {
            VecDouble cathodeLine;
            finalResult = FindCathodeLineByPeak(srcImg, cathodeLine);

            lstCathodeLine.reserve(srcImg.cols);
            for (int pIdx = 0; pIdx < srcImg.cols; pIdx++)
            {
                int index = std::min<int>(cathodeLine.size() - 1, pIdx / mCathodeLineWindowSize);
                lstCathodeLine.emplace_back(pIdx, cathodeLine[index]);
            }
        }
    }
    else
    {
        finalResult(EResult::ER, "FindCathodeLine: Image Empty");
    }

    return finalResult;
}

auto BatteryInspectorCathode::FindCathodeLineByAStar(const cv::Mat &srcImg,
                                                     VecPoint &lstCathodeLine,
                                                     int centerNeglectionWidth,
                                                     int region) const -> InspectionResult
{
    InspectionResult finalResult;

    if (!srcImg.empty())
    {
        lstCathodeLine.reserve(srcImg.cols);

        //local histogram equalization
        //cv::Mat localEnhanceCathodeLine = LocalHistogramEqualization(srcImg, region); //LOCAL_HISTOGRAM_REDUCE
        cv::Mat localEnhanceCathodeLine = srcImg.clone();
        int borderMode = cv::BORDER_REFLECT;                                          //border types for blur, soble
        int filterSize = 7;
        double sigma = 1.0;
        cv::Mat dstImg = DiagonalFilter(localEnhanceCathodeLine, filterSize, sigma, borderMode);

        // start - end column of neglection area
        int startNeglectCol = (centerNeglectionWidth > 30 && dstImg.cols > 50) ? (dstImg.cols - centerNeglectionWidth) >> 1 : dstImg.cols;

        cv::Rect leftSideROI = cv::Rect(0, 0, startNeglectCol, dstImg.rows);
        VecPoint leftCathodeLine = xvt::FindPathByAStar(dstImg(leftSideROI), 30);
        lstCathodeLine.insert(lstCathodeLine.begin(), leftCathodeLine.begin(), leftCathodeLine.end());

        int endNeglectCol = (dstImg.cols + centerNeglectionWidth) >> 1;
        if (endNeglectCol - startNeglectCol > 0)
        {
            cv::Rect righttSideROI = cv::Rect(endNeglectCol, 0, dstImg.cols - endNeglectCol, dstImg.rows);
            VecPoint rightCathodeLine = xvt::FindPathByAStar(dstImg(righttSideROI), 30);

            float alpha = 0.0;
            for (int i = startNeglectCol; i < endNeglectCol; i++)
            {
                alpha = (i - startNeglectCol) / (float)(endNeglectCol - startNeglectCol);
                lstCathodeLine.emplace_back(i, leftCathodeLine.back().y * (1 - alpha) + alpha * rightCathodeLine.front().y);
            }

            for (const auto &p : rightCathodeLine)
            {
                lstCathodeLine.emplace_back(p.x + endNeglectCol, p.y);
            }
        }
        finalResult(EResult::OK, "");
    }
    else
    {
        finalResult(EResult::ER, "FindCathodeLineByAStar: Image Empty");
    }

    return finalResult;
}

auto BatteryInspectorCathode::FindCathodeLineByPeak(const cv::Mat &srcImg, VecDouble &lstCathodeLine) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!srcImg.empty())
    {
        cv::Mat src;
        int kSize = 5;
        cv::medianBlur(srcImg, src, kSize);

        cv::Size imgSize = src.size();
        int wSize = std::min(std::max(mCathodeLineWindowSize, 5), imgSize.width);
        int wIdx = 0;
        cv::Mat tmpImg = cv::Mat();
        float localThresh = 5.0;
        int idx = 0;
        cv::Mat prominenceImage = cv::Mat();

        while (wIdx + wSize < imgSize.width)
        {
            idx = lstCathodeLine.size() > 0 ? lstCathodeLine.back() : 0;
            int regionIdx = GetRegionIndex(wIdx, 6, imgSize.width, GetJR().mCenterNeglectionWidth);

            if (regionIdx > 0)
            {
                switch (regionIdx)
                {
                case 1:
                    localThresh = mCathodeLineThresholdOuter;
                    break;
                case 2:
                    localThresh = mCathodeLineThresholdMiddle;
                    break;
                case 3:
                    localThresh = mCathodeLineThresholdInner;
                    break;
                default:
                    localThresh = mCathodeLineThresholdMiddle;
                    break;
                }
                cv::Rect subROI = cv::Rect(wIdx - wSize, 0, wSize, imgSize.height);
                if (true == RefineROI(subROI, imgSize))
                {
                    tmpImg = src(subROI);
                    cv::Mat1f reduceImg = cv::Mat();
                    cv::reduce(tmpImg, reduceImg, 1, cv::REDUCE_AVG, CV_32F); //Reduce to one row.
                    VecFloat reduceVec = cv::Mat_<float>(reduceImg);
                    reduceImg.release();

                    VecFloat list = reduceVec;
                    int vecSize = reduceVec.size() - 1;

                    for (int i = 2; i < vecSize - 1; i++)
                    {
                        reduceVec[i] = 0.1 * list[i - 2] + 0.25 * list[i - 1] + 0.3 * list[i] + 0.25 * list[i + 1] + 0.1 * list[i + 2];
                    }
                    // extern 2 element this will not skip 2 last element in filter
                    reduceVec[0] = reduceVec[2];
                    reduceVec[1] = reduceVec[2];
                    reduceVec[vecSize] = reduceVec[vecSize - 2];
                    reduceVec[vecSize - 1] = reduceVec[vecSize - 2];

                    VecFloat diffVec;
                    diffVec.emplace_back(0);
                    for (int i = 1; i < vecSize; i++)
                    {
                        float diff = reduceVec[i + 1] - reduceVec[i - 1];
                        diff = diff > 0 ? 0 : diff;
                        diff = -pow(diff, 2);
                        diffVec.emplace_back(diff); //  > 0 ? 0 : diff
                    }
                    diffVec.emplace_back(0);

                    FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
                    findPeaks.Process(diffVec);
                    VecPeak peaks = findPeaks.GetPeakResult(localThresh);

                    if (peaks.size() > 0)
                    {
                        idx = peaks.back().index;
                    }
                }
                else
                {
                    //ROI Refinement failed
                }
            }

            lstCathodeLine.emplace_back(idx);
            wIdx += wSize;
        }

        int sampleWindow = 5;
        if (lstCathodeLine.size() > sampleWindow)
        {
            lstCathodeLine.emplace_back(lstCathodeLine.back());

            int borderNeglction = 2;
            int neglectionWidth = borderNeglction * wSize;
            // start - end column of neglection area
            int startNeglectCol = (imgSize.width - GetJR().mCenterNeglectionWidth) >> 1;
            int endNeglectCol = (imgSize.width + GetJR().mCenterNeglectionWidth) >> 1;
            int rightEndBounder = imgSize.width - neglectionWidth;

            AverageCathodeLineResult<double> avgCathodeLineResult =
                FindAverageCathodeLine(lstCathodeLine, neglectionWidth, startNeglectCol, endNeglectCol, rightEndBounder, 11, wSize);

            if (avgCathodeLineResult.success)
            {
                RefineCathodeLine(avgCathodeLineResult, lstCathodeLine, wSize, startNeglectCol, endNeglectCol);
            }
            else
            {
                /* Do nothing*/
            }
            finalResult(EResult::OK, "");
        }
        else
        {
            finalResult(EResult::ER, "FindCathodeLineByPeak: Can't find the cathode line");
        }
    }
    else
    {
        finalResult(EResult::ER, "FindCathodeLineByPeak: Image Empty");
    }

    return finalResult;
}

struct DataDistribution
{
    double min = 0.0;
    double max = 0.0;
    double average = 0.0;
    double stdDev = 0.0;
    double mean = 0.0;

    // Calculate the properties of the data distribution
    void calculate(const VecDouble &data)
    {
        if (!data.empty())
        {
            min = data[0];
            max = data[0];
            double sum = 0.0;
            for (auto d : data)
            {
                if (d < min)
                {
                    min = d;
                }
                if (d > max)
                {
                    max = d;
                }
                sum += d;
            }
            average = sum / data.size();

            std::tie(mean, stdDev) = MeanStdDevLite(data);
        }
    }
};

auto BatteryInspectorCathode::GetPoleInfo(const VecPoint &vtLine,
                                          VecPeak &peaks,
                                          std::vector<PoleInfo> &poles,
                                          Rangei &disRange,
                                          int cols) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!peaks.empty())
    {
        int numberOfLeftPoles = 0;
        int numberOfRightPoles = 0;
        int centerLeft = (cols - GetJR().mCenterNeglectionWidth) / 2;
        int centerRight = (cols + GetJR().mCenterNeglectionWidth) / 2;
        int boundaryLeft = 0;
        int boundaryRight = cols;
        int distanceThd = (disRange.GetLower() + disRange.GetUpper()) / 4;
        DataDistribution disData, prominenceData;

        // Remove the poles close to the outer boundary
        boundaryLeft += distanceThd + 1;
        boundaryRight -= distanceThd + 1;

        // Remove the poles close to the inner boundary
        if (GetJR().mCenterNeglectionWidth > 0)
        {
            centerLeft -= distanceThd;
            centerRight += distanceThd;
        }

        VecDouble vtDisData, vtProminenceData;
        for (int i = peaks.size() - 1; i >= 0; i--)
        {
            if ((peaks[i].index > centerLeft && peaks[i].index < centerRight) || peaks[i].index < boundaryLeft || peaks[i].index > boundaryRight)
            {
                peaks.erase(peaks.begin() + i);
            }
            else
            {
                if (peaks[i].index <= centerLeft)
                {
                    numberOfLeftPoles += 1;
                }
                else
                {
                    //pole->index >= centerRight
                    numberOfRightPoles += 1;
                }
                PoleInfo tmpPole(peaks[i],cv::Point(), vtLine[peaks[i].index], PoleType::Real);
                poles.insert(poles.begin(), tmpPole);
                vtProminenceData.emplace_back(abs(peaks[i].prominence));
                if (i > 0)
                    vtDisData.emplace_back(abs(peaks[i].index - peaks[i - 1].index));
            }
        }
        prominenceData.calculate(vtProminenceData);
        disData.calculate(vtDisData);

        std::string tmpProminence = "Prominence: min=" + xvt::ToString(prominenceData.min, 2) + ", max=" + xvt::ToString(prominenceData.max, 2) +
                                    ", avg=" + xvt::ToString(prominenceData.average, 2) + ", stdDev=" + xvt::ToString(prominenceData.stdDev, 2) +
                                    ", mean=" + xvt::ToString(prominenceData.mean, 2);
        std::string tmpDistance = "Pole Distance: min=" + xvt::ToString(disData.min, 2) + ", max=" + xvt::ToString(disData.max, 2) +
                                  ", avg=" + xvt::ToString(disData.average, 2) + ", stdDev=" + xvt::ToString(disData.stdDev, 2) +
                                  ", mean=" + xvt::ToString(disData.mean, 2);

        finalResult(EResult::OK, "");
        finalResult.CombineMsg(tmpProminence);
        finalResult.CombineMsg(tmpDistance);
        finalResult.CombineMsg("No. Peak:" + xvt::ToString(peaks.size(), 0));
    }
    else
    {
        finalResult(EResult::ER, "FindCathodeXPosition: Can't find the x position poles");
    }
    return finalResult;
}

#define DEBUG_PLOT
auto BatteryInspectorCathode::FindCathodeXPosition(const cv::Mat &src,
                                                   VecPoint &vtLine,
                                                   VecPoint &vtContour,
                                                   std::vector<PoleInfo> &poles) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!src.empty() && !vtLine.empty())
    {
        if (poles.size() > 0)
            poles.clear();

        // 6: From Anode ending lines, select a sub-ROI 4.0 (mask) for detecting poles position
        cv::Mat tmpDst; // = src.clone();
        cv::Mat mask = cv::Mat::zeros(src.size(), src.type());
        int kSize = 5;

        cv::Size iSize = src.size();

        VecPoint outerPoints;
        outerPoints.emplace_back(0, 0);
        outerPoints.emplace_back(iSize.width - 1, 0);

        vtContour = refineMaskingContour(vtLine, iSize.width, 0, GetPole().mPoleHeight, outerPoints);

        if (vtContour.size() > iSize.width)
        {
            cv::drawContours(mask, std::vector<VecPoint>(1, vtContour), -1, cv::Scalar::all(255), -1);
            // 7: Implement the CLAHE with rectangular size (35x image.height)
            tmpDst = cv::Mat::zeros(iSize, CV_8UC1);
            src.copyTo(tmpDst, mask);

            VecFloat reduceVec = ReduceBlackBackgroundMat(tmpDst, 0, kSize);
            Rangei distanceRange = GetPole().mPolesDistanceRange;
            VecPeak polePos;
            if (GetPole().mPolesProminenceThreshold == 0)
            {
                polePos = FindCathodeXPositionAuto(reduceVec, distanceRange, GetJR().mCenterNeglectionWidth);
            }
            else
            {
                polePos = FindCathodeXPositionManual(reduceVec, GetPole().mPolesProminenceThreshold, distanceRange.GetLower());
            }

            
            // vtLine, polePos, poles, distanceRange, src.cols
            finalResult = GetPoleInfo(vtLine, polePos, poles, distanceRange, src.cols);
#ifdef DEBUG_PLOT
            cv::Mat mat(600, 1200, CV_8UC3);
            mat.setTo(cv::Scalar(255, 255, 255));

            CvPlot::Axes axes = CvPlot::makePlotAxes();
            axes.enableHorizontalGrid();
            axes.enableVerticalGrid();
            axes.title("ROI");

            axes.create<CvPlot::Series>(reduceVec, "-r");
            for (auto pole : polePos) {
                axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ { (float)(pole.index), reduceVec[pole.index] }, { (float)(pole.index), reduceVec[pole.index] }}, "-og");
            }

            int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
            axes.setMargins(borderLeft, borderRight, borderTop, borderBottom);
            axes.setXLim(std::pair<int, int>(0, reduceVec.size()));
            axes.setXTight(true);
            axes.render(mat);

#endif //DEBUG_PLOT
        }
        else
        {
            finalResult(EResult::ER, "FindCathodeXPosition: Not found any contour");
        }
    }
    else
    {
        finalResult(EResult::ER, "FindCathodeXPosition: Image Empty");
    }
    return finalResult;
}

void BatteryInspectorCathodeResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    if (!mPoints.empty())
        xvt::DrawPoints(img, mPoints, cv::Vec3b(0, 0, 200), offSetPoint);

    /*if (!GetROI().empty())
        cv::rectangle(img, GetROI(), cv::Scalar(120, 120, 255), 2);*/
}

auto BatteryInspectorCathodeResult::DrawResultStr(cv::Mat &image,
                                                  std::string const &name,
                                                  CVPen const &pen,
                                                  cv::Point const &offset,
                                                  bool isDrawOKResult) const -> cv::Point
{
    auto tmpPos = InspectionResult::DrawResultStr(image, name, pen, offset, isDrawOKResult);

    return cv::Point(offset.x, tmpPos.y);
}

auto BatteryInspectorCathodeResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
}

} // namespace battery
} // namespace xvt