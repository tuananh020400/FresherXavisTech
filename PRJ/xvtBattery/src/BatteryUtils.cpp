#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/Utils.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/Contour.h"
#include "xvtCV/Peak.h"
#include "xvtCV/xvtConvert.h"
#include "xvtCV/ScopeTimer.h"
#include <iomanip>
#include <ctime>
#include <execution>
#include <fstream>
#include <locale>
#include <codecvt>

#define _DEBUG_POLE_LEANING     (1)
#define _DEBUG_FIND_TURN_POINT  (_DEBUG && 0)

namespace xvt {
namespace battery {

int CheckRegion(int xCord, int regions, int imgWidth, int midWidth)
{
    if (regions % 2 == 1 || regions < 2)
        return 0;

    float midWidthRegionStart = ((imgWidth - midWidth) / 2);
    float midWidthRegionEnd = ((imgWidth + midWidth) / 2);

    float nRegionIn1Side = regions / 2;
    if (xCord >= midWidthRegionStart && xCord <= midWidthRegionEnd)
    {
        //std::cout << "pixel out of bound" << std::endl;
        return 0;
    }

    float pixRegionWidth = midWidthRegionStart / nRegionIn1Side;
    if (pixRegionWidth <= 0)
        return 0;

    int rIdx = 0;
    if (xCord < midWidthRegionStart)
    {
        rIdx = xCord / pixRegionWidth;
    }
    else
    {
        rIdx = nRegionIn1Side - ((xCord - midWidthRegionEnd) / pixRegionWidth);
    }
    rIdx++;
    if (rIdx < 1)
    {
        rIdx = 1;
    }
    if (rIdx > nRegionIn1Side)
    {
        rIdx = nRegionIn1Side;
    }
    return rIdx;
}

cv::Point drawText(cv::Mat &resImg, cv::Point const &pos, std::string const &str, cv::Scalar const &color, float fontScale, float fontWeight)
{
    cv::Point nextPos = pos;
    if (!resImg.empty() && !str.empty())
    {
        auto txtSize = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, fontScale, fontWeight, 0);
        cv::putText(resImg, str, pos, cv::FONT_HERSHEY_SIMPLEX, fontScale, color, fontWeight);
        nextPos.x += txtSize.width;
    }
    return nextPos;
}

cv::Point drawText(cv::Mat &resImg,
                   cv::Point const &startPos,
                   std::vector<std::pair<std::string, cv::Scalar>> const &textList,
                   float fontScale,
                   float fontWeight)
{
    //Draw the pole number
    cv::Point nextPos = startPos;
    for (auto &i : textList)
    {
        //Draw the pole position infor
        nextPos = drawText(resImg, nextPos, i.first, i.second, fontScale, fontWeight);
    }

    return nextPos;
}

VecFloat GetLUT(int a, int b)
{
    int minValue = std::min(std::max(std::min(a, b), 0), 255);
    int maxValue = std::min(std::max(std::max(a, b), 0), 255);

    VecFloat LUT(256, 0);
    std::fill(LUT.begin() + maxValue, LUT.end(), 255);
    double scale = 255.0 / (double)(maxValue - minValue);
    for (int k = minValue + 1; k < maxValue; k++)
    {
        LUT[k] = (cv::saturate_cast<float>((k - minValue) * scale));
    }
    return LUT;
}

cv::Mat DiagonalFilter(const cv::Mat &src, int ksize, double sigmaX, int borderMode)
{
    cv::Mat dst = cv::Mat();
    if (!src.empty())
    {
        int ddepth = CV_32F;
        dst = cv::Mat::zeros(src.size(), ddepth);
        float scaleSobel = 1.0; // / 120;
        cv::Point anchor = cv::Point(-1, -1);
        cv::Mat gaussianKenel1 = cv::getGaussianKernel(ksize, sigmaX, ddepth);
        cv::Mat gaussianKenelFlip;
        cv::rotate(gaussianKenel1, gaussianKenelFlip, cv::ROTATE_90_CLOCKWISE);
        cv::Mat gaussianKenel = gaussianKenel1 * gaussianKenelFlip; //Gaussian_filter(ksize, sigmaX, ddepth);
        double gaussianSum = cv::sum(gaussianKenel)[0];
        VecFloat vtGaussianKernel = cv::Mat_<float>(gaussianKenel1);
        double delta = 0;
        float sqr2 = std::sqrt(2);
        //find D1, D2, D3
        cv::Mat D1, D2, D3;
        {
            int kSobelSize = 3;
            // D1
            cv::Mat sumImg;
            cv::Sobel(gaussianKenel, D1, ddepth, 0, 1, kSobelSize, scaleSobel, 0.0, cv::BORDER_REPLICATE);
            cv::threshold(D1, sumImg, 0, 255, cv::THRESH_TOZERO);
            double sumVal = cv::sum(sumImg)[0];
            D1 *= (1.0 / sumVal);
            cv::Mat1f kernel = (cv::Mat_<float>(kSobelSize, kSobelSize) << 1.0, sqr2, 0.0, sqr2, 0.0, -sqr2, 0.0, -sqr2, -1.0);
            // D2
            int r = ksize / 2;
            cv::Mat gaussianMakeBorder;
            cv::copyMakeBorder(gaussianKenel, gaussianMakeBorder, r, r, r, r, cv::BORDER_CONSTANT, cv::Scalar(0));

            filter2D(gaussianMakeBorder, D2, ddepth, kernel, anchor, delta, cv::BORDER_REPLICATE);
            D2 = D2(cv::Rect(r, r, ksize, ksize));
            D2.convertTo(D2, CV_32S, 100000);  // Convert to integer with a scaling of 1000000
            D2.convertTo(D2, CV_32F, 0.00001); // Convert back to double with a scaling of 0.000001

            // D3
            cv::threshold(D2, sumImg, 0, 255, cv::THRESH_TOZERO);
            sumVal = cv::sum(sumImg)[0];
            D2 *= (1.0 / sumVal);

            cv::flip(D2, D3, 1);
        }

        cv::Mat W1, W2, W3;
        cv::filter2D(src, W1, ddepth, D1, anchor, delta, cv::BORDER_DEFAULT);
        cv::filter2D(src, W2, ddepth, D2, anchor, delta, cv::BORDER_DEFAULT);
        cv::filter2D(src, W3, ddepth, D3, anchor, delta, cv::BORDER_DEFAULT);
        W2 = abs(W2);
        W3 = abs(W3);
        for (int i = 0; i < dst.cols; i++)
        {
            for (int j = 0; j < dst.rows; j++)
            {
                if (W1.at<float>(j, i) > 0)
                    dst.at<float>(j, i) = std::max<float>({W1.at<float>(j, i), W2.at<float>(j, i), W3.at<float>(j, i)});
                /*else
                        dst.at<float>(j, i) = W1.at<float>(j, i);*/
            }
        }
        //cv::threshold(dst, dst, 0, 255, cv::THRESH_TOZERO);

        //dst.convertTo(dst, src.type());
#ifdef DEBUG_FIND_LINE_AUTO
        //convertImg32FTo8U(dst, drawImage);
        //saveOutputImage("_DiagonalFilter", drawImage);
        //saveOutputImage("_DiagonalFilter_Histogram", getHistogramImg(drawImage));
#endif
    }
    else
    {
        // Do nothing
    }

    return dst;
}

cv::Mat LocalHistogramEqualization(const cv::Mat &src, const int region)
{
    cv::Mat dst;
    if (!src.empty())
    {
        dst = src.clone();
        //check region
        // region >= 1 && region < src.cols or (10)

        // find the LUT of each region
        std::vector<VecFloat> vtLUT;
        // Calculate histograms of arrays of images
        int histSize = 256;
        float range[] = {0, 256}; //the upper boundary is exclusive
        const float *histRange[] = {range};
        cv::Mat gray_hist;
        bool uniform = true, accumulate = false;
        int localMinTh, localMaxTh;
#ifdef DEBUG_FIND_LINE_AUTO
        cv::Mat drawImg = dst.clone();
        cv::cvtColor(drawImg, drawImg, cv::COLOR_GRAY2BGR);
#endif
        for (int i = 0; i < region; i++)
        {
            cv::Rect regionROI = cv::Rect(i * (dst.cols / region), 0, floor(dst.cols / region), dst.rows);
            cv::Mat lutImg = dst(regionROI).clone();
            cv::calcHist(&lutImg, 1, 0, cv::Mat(), gray_hist, 1, &histSize, histRange, uniform, accumulate);

            // find min threshold
            int ddepth = CV_32F;
            cv::Mat1f reduceRowImg;
            cv::reduce(lutImg, reduceRowImg, 1, cv::REDUCE_AVG, ddepth); //Reduce to one row.
            //cv::threshold(sobelImg, sobelImg, 0, 255, cv::THRESH_TOZERO);
            VecFloat rowsum = cv::Mat_<float>(reduceRowImg);
            auto minMaxRowList = std::minmax_element(rowsum.begin(), rowsum.end(), [](const float &lsh, const float &rsh) { return lsh < rsh; });
            localMinTh = round(*minMaxRowList.first);

            // find max threshold
            VecFloat columnsum;
            cv::reduce(lutImg, columnsum, 0, cv::ReduceTypes::REDUCE_AVG, ddepth);
            cv::medianBlur(columnsum, columnsum, 5);
            auto minMaxColList =
                std::minmax_element(columnsum.begin(), columnsum.end(), [](const float &lsh, const float &rsh) { return lsh < rsh; });
            /*if (i > 0 && i < (int)floor(region - 1) / 2 - 1 || i >(int)floor((region - 1) / 2) + 1 && i < region - 1)
                {
                    float ratioRange = 0.3;
                    localMaxTh = round(*minMaxRowList.first * (1 - ratioRange) + *minMaxRowList.second * ratioRange);
                }
                else*/
            {
                localMaxTh = round(*minMaxColList.second);
            }
#ifdef DEBUG_FIND_LINE_AUTO
            cv::Mat hisImg = histDisplay(gray_hist, "", localMinTh, localMaxTh);
            cv::resize(hisImg, hisImg, regionROI.size(), cv::INTER_LINEAR);
            hisImg.copyTo(drawImg(regionROI));
            cv::rectangle(drawImg, regionROI, COLOR_CV_BLUE);
            cv::putText(drawImg,
                        "R: " + std::to_string(i + 1),
                        cv::Point(regionROI.tl() + cv::Point(10, 10)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.3,
                        {0, 0, 255});
            cv::putText(drawImg,
                        "MinTh: " + std::to_string(localMinTh),
                        cv::Point(regionROI.tl() + cv::Point(10, 30)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.3,
                        {0, 0, 255});
            cv::putText(drawImg,
                        "MaxTh: " + std::to_string(localMaxTh),
                        cv::Point(regionROI.tl() + cv::Point(10, 50)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.3,
                        {0, 0, 255});
#endif
            localMinTh = (localMinTh >= localMaxTh) ? localMaxTh : localMinTh;
            vtLUT.push_back(GetLUT(localMinTh, localMaxTh));
            gray_hist.release();
        }

        //check vector lut size
        if (!vtLUT.empty())
        {
            // apply the LUT
            int haftRegionWidth = floor((dst.cols / (float)region) / 2.0);
            float alpha;
            size_t lutIdx = 0;
            int leftLimitIdex = haftRegionWidth;
            int rightLimitIdex = floor(dst.cols - (haftRegionWidth + dst.cols % region + region));

            for (int i = 0; i < dst.cols; i++)
            {
                if (i <= leftLimitIdex)
                {
                    lutIdx = 0;
                    alpha = 0;
                }
                else if (i >= rightLimitIdex)
                {
                    lutIdx = region - 2;
                    alpha = 1;
                }
                else
                {
                    alpha = ((i - haftRegionWidth) % (haftRegionWidth * 2)) / (haftRegionWidth * 2.0);
                    lutIdx = std::min<int>(floor((i - haftRegionWidth) / (haftRegionWidth * 2.0)), region - 2);
                }

                for (int j = 0; j < dst.rows; j++)
                {
                    dst.at<uchar>(j, i) = round(vtLUT[lutIdx][dst.at<uchar>(j, i)] * (1 - alpha) + alpha * vtLUT[lutIdx + 1][dst.at<uchar>(j, i)]);
                }
            }

#ifdef DEBUG_FIND_LINE_AUTO
            //saveOutputImage("_Original_Image", src);
            //saveOutputImage("_LocalEH", dst);
            //saveOutputImage("_LocalEH", drawImg);
            //Utils::show_window("_LocalEHhisImg", drawImg);
#endif // DEBUG_FIND_LINE_AUTO
        }
    }
    return dst;
}

VecPeak FindCathodeXPositionManual(const VecFloat &reduceVec, const float &minProminence, const float &polesMinDistance)
{

    // Determine the pole position from accumulation graph - STEP 5
    FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);

    findPeaks.Process(reduceVec);
    VecPeak polePos = findPeaks.GetPeakResult(minProminence, polesMinDistance);

    return polePos;
}

int findMode(VecInt numVector)
{
    int mode = -1;
    if (!numVector.empty())
    {
        size_t n = *std::max_element(numVector.begin(), numVector.end());
        VecInt histogram(n + 1, 0);
        for (int i = 0; i < numVector.size(); ++i)
            ++histogram[numVector[i]];
        mode = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
    }
    return mode;
}

VecPeak getPeakResult(FindPeaks &findPeaks,
                                VecFloat reduceVec,
                                float prominenceTh,
                                float &minDis,
                                float &maxDis,
                                float alpha = 0.1)
{
    VecPeak peakResult;

    if (!reduceVec.empty())
    {
        double mean = 0.0, stdDev = 0.0;
        findPeaks.Process(reduceVec);
        peakResult = findPeaks.GetPeakResult(prominenceTh);
        if (!peakResult.empty())
        {
            VecInt vtDistance;
            vtDistance.reserve(peakResult.size() - 1);
            for (int i = 1; i < peakResult.size(); i++)
            {
                vtDistance.push_back(abs(peakResult[i].index - peakResult[i - 1].index));
            }

            if (!vtDistance.empty())
            {
                int mode = findMode(vtDistance);
                if (mode > 0)
                {
                    float beta = 0.3;
                    int minRange = mode - round(mode * beta);
                    int maxRange = mode + round(mode * beta);
                    for (int i = vtDistance.size() - 1; i >= 0; i--)
                    {
                        if (vtDistance[i] > maxRange || vtDistance[i] < minRange)
                            vtDistance.erase(vtDistance.begin() + i);
                    }
                }
            }

            if (!vtDistance.empty())
            {
                std::tie(mean, stdDev) = MeanStdDevLite(vtDistance);

                float minStdDev = 1.0;
                if (stdDev > minStdDev)
                {
                    minDis = mean - alpha * stdDev;
                    maxDis = mean + alpha * stdDev;
                }
                else
                {
                    minDis = *std::min_element(vtDistance.begin(), vtDistance.end()) - alpha * stdDev;
                    maxDis = *std::max_element(vtDistance.begin(), vtDistance.end()) + alpha * stdDev;
                }

                peakResult = findPeaks.GetPeakResult(prominenceTh, minDis);
            }
        }
    }

    return peakResult;
}

VecPeak FindCathodeXPositionAuto(const VecFloat &signal, Rangei &polesDistanceRange, const int &centerNeglectionWidth)
{
    VecPeak vtXPositions;
    int centerWidthNeglection = std::max<int>(signal.size() / 20, std::min<int>(signal.size() / 2, centerNeglectionWidth));
    if (!signal.empty())
    {
        const size_t size = signal.size();
        const int midIdx = std::floor(size / 2.0);
        const float ratioCenter = 0.7;
        float haftCenterNeglectionWidth = centerWidthNeglection / 2.0;
        const int centerLeftIdx = std::max<int>(0, midIdx - ratioCenter * haftCenterNeglectionWidth);
        const int centerRightIdx = std::min<int>(size - 1, midIdx + ratioCenter * haftCenterNeglectionWidth);
        const double alpha = 3.0;

        const VecFloat leftReduceVec(signal.begin(), signal.begin() + centerLeftIdx);
        const VecFloat rightReduceVec(signal.begin() + centerRightIdx, signal.end());
        const VecFloat centerNeglectReduceVec(signal.begin() + centerLeftIdx, signal.begin() + centerRightIdx);

        // Determine the pole position from accumulation graph
        FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
        findPeaks.Process(centerNeglectReduceVec);
        auto peakResult = findPeaks.GetPeakResult(1);

        VecFloat vtProminence, vtDistance;
        vtProminence.reserve(peakResult.size());
        for (const auto& it : peakResult)
        {
            vtProminence.push_back(abs(it.prominence));
        }

        if (!vtProminence.empty() && !rightReduceVec.empty() && !leftReduceVec.empty())
        {
            double mean, stdDev;
            std::tie(mean, stdDev) = MeanStdDevLite(vtProminence);
            vtProminence.clear();
#ifdef DEBUG_FIND_CATHODE_X_POSITION_AUTO
            int imageHeight = 200;
            cv::Mat ImgMat = cv::Mat::zeros(cv::Size(size, imageHeight), CV_8U);
            cv::Mat debugMat = cv::Mat::zeros(ImgMat.size(), CV_8UC3);
            cv::Rect centerROI = cv::Rect(centerLeftIdx, 0, centerRightIdx - centerLeftIdx, imageHeight);
            cv::Point textPos(centerROI.width / 3, 15);
            Drawing drawTool;
            drawTool.padding_Y = 15;
            drawTool.yTopOrigin = false;
            drawTool.color = COLOR_CV_YELLOW;
            cv::Mat drawPeakMat = DrawPeakInfo(ImgMat(centerROI), centerNeglectReduceVec, peakResult, drawTool, 0);
            drawTool.color = COLOR_CV_RED;
            drawTool.Plot(drawPeakMat, "m: " + std::to_string(mean), textPos, 1, true);
            drawTool.Plot(drawPeakMat, "s: " + std::to_string(stdDev), textPos + cv::Point(0, 15), 1, true);
            drawPeakMat.copyTo(debugMat(centerROI));
#endif // DEBUG_FIND_CATHODE_X_POSITION_AUTO
            float prominenceTh = mean;
            float leftMinPolesDistance = 0.0, leftMaxPolesDistance = 0.0;
            auto leftPeakResult = getPeakResult(findPeaks, leftReduceVec, prominenceTh, leftMinPolesDistance, leftMaxPolesDistance, alpha);

            float rightMinPolesDistance = 0.0, rightMaxPolesDistance = 0.0;
            auto rightPeakResult = getPeakResult(findPeaks, rightReduceVec, prominenceTh, rightMinPolesDistance, rightMaxPolesDistance, alpha);

            if (leftPeakResult.size() != 0 && rightPeakResult.size() != 0 && leftMinPolesDistance > 0 && rightMinPolesDistance > 0 &&
                leftMaxPolesDistance > 0 && rightMaxPolesDistance > 0)
            {
                vtXPositions.reserve(leftPeakResult.size() + rightPeakResult.size());
                vtXPositions.insert(vtXPositions.begin(), leftPeakResult.begin(), leftPeakResult.end());

                for (int i = 0; i < rightPeakResult.size(); i++)
                {
                    auto xPos = rightPeakResult[i];
                    xPos.index += centerRightIdx;
                    vtXPositions.push_back(std::move(xPos));
                }

#ifdef DEBUG_FIND_CATHODE_X_POSITION_AUTO
                drawTool.color = COLOR_CV_YELLOW;
                cv::Rect leftROI = cv::Rect(0, 0, centerLeftIdx, imageHeight);
                drawPeakMat = drawPeakInfo(ImgMat(leftROI), leftReduceVec, leftPeakResult, drawTool, 0);
                textPos = cv::Size(leftROI.width / 2, 15);
                drawTool.color = COLOR_CV_RED;
                drawTool.Plot(drawPeakMat, "mimDis: " + std::to_string(leftMinPolesDistance), textPos, 1, true);
                drawTool.Plot(drawPeakMat, "maxDis: " + std::to_string(leftMaxPolesDistance), textPos + cv::Point(0, 15), 1, true);
                drawPeakMat.copyTo(debugMat(leftROI));
                drawTool.color = COLOR_CV_YELLOW;
                cv::Rect rightROI = cv::Rect(centerRightIdx, 0, size - centerRightIdx, imageHeight);
                drawPeakMat = drawPeakInfo(ImgMat(rightROI), rightReduceVec, rightPeakResult, drawTool, 0);
                textPos = cv::Size(rightROI.width / 2, 15);
                drawTool.color = COLOR_CV_RED;
                drawTool.Plot(drawPeakMat, "mimDis: " + std::to_string(rightMinPolesDistance), textPos, 1, true);
                drawTool.Plot(drawPeakMat, "maxDis: " + std::to_string(rightMaxPolesDistance), textPos + cv::Point(0, 15), 1, true);
                drawPeakMat.copyTo(debugMat(rightROI));
                drawPeakMat.release();
                cv::rectangle(debugMat, centerROI, COLOR_CV_RED, 2);
                cv::line(debugMat,
                         cv::Point(midIdx - haftCenterNeglectionWidth),
                         cv::Point(midIdx - haftCenterNeglectionWidth, imageHeight),
                         COLOR_CV_GREEN,
                         2);
                cv::line(debugMat,
                         cv::Point(midIdx + haftCenterNeglectionWidth),
                         cv::Point(midIdx + haftCenterNeglectionWidth, imageHeight),
                         COLOR_CV_GREEN,
                         2);
                show_window("Find Cathode X Position Auto", debugMat);
#endif // DEBUG_FIND_CATHODE_X_POSITION_AUTO
                polesDistanceRange =
                    Rangei((leftMinPolesDistance + rightMinPolesDistance) / 2.0, (leftMaxPolesDistance + rightMaxPolesDistance) / 2.0);
            }
        }
    }
    return vtXPositions;
}

int FindClosestPoint(const VecPoint &vtDst, const int xValue, int xAfter, const int maxRange)
{
    int vIdx = -1;
    int d = 0;
    int minDis = maxRange;
    int endIdx = vtDst.size();
    int lowerValue = xValue - maxRange;
    int upperValue = xValue + maxRange;

    for (int i = 0; i < endIdx; i++)
    {
        float x = vtDst[i].x;
        if (lowerValue > x)
        {
            continue;
        }
        else if (upperValue < x)
        {
            break;
        }
        else
        {
            d = abs(x - xValue);
            if (d < minDis)
            {
                minDis = d;
                vIdx = i;
            }
        }
    }

    vIdx = (vIdx >= 0 && (minDis > abs(vtDst[vIdx].x - xAfter))) ? -1 : vIdx;
    return vIdx;
}

int FindClosestPoint(const VecInt &vtDst, const int xValue, int xAfter, const int maxRange)
{
    if (vtDst.empty())
        return -1;

    int vIdx = -1;
    int d = 0;
    int minDis = INT_MAX;
    int endIdx = vtDst.size();
    int lowerValue = xValue - maxRange;
    int upperValue = xValue + maxRange;

    for (int i = 0; i < endIdx; i++)
    {
        if (lowerValue > vtDst[i])
        {
            continue;
        }
        else if (upperValue < vtDst[i])
            break;
        else
        {
            d = abs(vtDst[i] - xValue);
            if (d < minDis)
            {
                minDis = d;
                vIdx = i;
            }
        }
    }

    vIdx = (vIdx >= 0 && (minDis > abs(vtDst[vIdx] - xAfter))) ? -1 : vIdx;
    return vIdx;
}

VecFloat ReduceBlackBackgroundMat(const cv::Mat &src, int dim, unsigned int kSize)
{
    VecFloat reduceVec;
    if (!src.empty())
    {
        size_t vtSize = dim ? src.rows : src.cols;
        reduceVec.reserve(vtSize);
        // Calculate the cummulative function
        for (int i = 0; i < vtSize; i++)
        {
            float sum = dim ? cv::sum(src.row(i))[0] : cv::sum(src.col(i))[0];
            int nNonBlack = dim ? cv::countNonZero(src.row(i)) : cv::countNonZero(src.col(i));
            reduceVec.push_back(nNonBlack == 0 ? 0 : (sum / (float)nNonBlack));
        }

        // Apply the Smoothing filter
        cv::GaussianBlur(reduceVec, reduceVec, cv::Size(kSize, 1), 0);
    }
    return reduceVec;
}


int GetRegionIndex(int xCord, int regions, int imgWidth, int midWidth)
{

    if (regions % 2 == 1 || regions < 2)
        return 0;
    float midWidthRegionStart = ((imgWidth - midWidth) / 2);
    float midWidthRegionEnd = ((imgWidth + midWidth) / 2);

    float nRegionIn1Side = regions / 2;
    if (xCord >= midWidthRegionStart && xCord <= midWidthRegionEnd)
    {
        //std::cout << "pixel out of bound" << std::endl;
        return 0;
    }


    float pixRegionWidth = midWidthRegionStart / nRegionIn1Side;
    if (pixRegionWidth <= 0)
        return 0;

    int rIdx = 0;
    if (xCord < midWidthRegionStart)
    {
        rIdx = xCord / pixRegionWidth;
    }
    else
    {
        rIdx = nRegionIn1Side - ((xCord - midWidthRegionEnd) / pixRegionWidth);
    }
    rIdx++;
    if (rIdx < 1)
    {
        rIdx = 1;
    }
    if (rIdx > nRegionIn1Side)
    {
        rIdx = nRegionIn1Side;
    }
    return rIdx;
}

int FindRefLine(cv::Mat &blurImg, int leaningThreshold)
{
    int yCathode = 0;
    if (!blurImg.empty())
    {
        cv::Mat sumImg = cv::Mat();
        cv::reduce(blurImg, sumImg, 1, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
        cv::blur(sumImg, sumImg, cv::Size(1, 11), cv::Point(-1, -1), cv::BorderTypes::BORDER_DEFAULT);

        VecFloat reduceVec = cv::Mat_<float>(sumImg);
        int tmpSize = static_cast<__int64>(reduceVec.size()) - 1;

        VecFloat columnsumDiff(tmpSize + 1, 0);
        //for (int i = 1; i < tmpSize; i++)
        //{
        //    columnsumDiff[i] = ((static_cast<double>(reduceVec[static_cast<__int64>(i) + 1]) - reduceVec[static_cast<__int64>(i) - 1]) / 2.0);
        //    ;
        //}
        cv::Sobel(reduceVec, columnsumDiff, -1, 1, 0, 3);

        FindPeaks findPeaks;
        findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
        findPeaks.Process(columnsumDiff);
        VecPeak peakList = findPeaks.GetPeakResult(0.5);
        yCathode = peakList.empty() ? tmpSize / 2.0 : peakList.back().index;

#if _DEBUG_POLE_LEANING
        cv::Mat blurImgDraw = blurImg.clone();
        int rows = reduceVec.size();
        VecInt yList(rows, 0);
        for (int i = 0; i < rows; i++)
            yList[i] = i;

        Drawing drawTool;
        drawTool.yTopOrigin = true;
        drawTool.thickness = 1;
        drawTool.Plot(blurImgDraw, reduceVec, yList);

        drawTool.color = cv::Scalar(255, 0, 255);
        drawTool.Plot(blurImgDraw, columnsumDiff, yList);

        cv::line(blurImgDraw, cv::Point(0, yCathode), cv::Point(blurImgDraw.cols, yCathode), COLOR_CV_BLUE);

        //show_window("Vertical", blurImgDraw);
        //cv::imwrite(Utils::RemoveExtension(imageName) + "Vertical.jpg", blurImgDraw);
#endif //_DEBUG_POLE_LEANING
    }
    return yCathode;
}

XVT_EXPORTS std::string GetStrCurrentTime()
{
    // Get the current time
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm localTime;
    localtime_s(&localTime, &currentTime);

    // Format the time (including milliseconds)
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << localTime.tm_mon + 1;    // Month (1-12)
    oss << "/";
    oss << std::setw(2) << std::setfill('0') << localTime.tm_mday;       // Day of the month
    oss << " ";
    oss << std::setw(2) << std::setfill('0') << localTime.tm_hour;       // Hour
    oss << ":";
    oss << std::setw(2) << std::setfill('0') << localTime.tm_min;        // Minute
    oss << ":";
    oss << std::setw(2) << std::setfill('0') << localTime.tm_sec;        // Second
    //oss << ".";

    // Get milliseconds
    auto duration = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;

    // Append milliseconds to the formatted time
    //oss << std::setw(3) << std::setfill('0') << milliseconds.count();

    return oss.str();
}

auto FindLeaningPos(VecFloat const& signal, PeakType type, float upperValueThd)->int
{
    bool isFound = false;
    int leftIdx = 0;
    FindPeaks findPeaks;
    findPeaks = FindPeaks(type, PeakFindingMethod::Prominence, 0, true);
    findPeaks.Process(signal);
    VecPeak peakList = findPeaks.GetPeakResult(1);
    auto comPareFunc = type == PeakType::Valley ? xvt::GreaterProminence : xvt::SmallerProminence;
    if (peakList.size() > 5)
    {
        auto maxExtrema = std::max_element(peakList.begin(), peakList.end(), comPareFunc);
        float maxPro = 5;
        if (maxExtrema != peakList.begin())
        {
            auto maxValleyLeft = std::max_element(peakList.begin(), maxExtrema, comPareFunc);
            maxPro = std::max(abs(maxValleyLeft->prominence), maxPro);
        }

        if (maxExtrema != peakList.end() - 1)
        {
            auto maxValleyRight = std::max_element(maxExtrema + 1, peakList.end(), comPareFunc);
            maxPro = std::max(abs(maxValleyRight->prominence), maxPro);
        }

		isFound = abs(maxExtrema->prominence) > maxPro * 2 && abs(maxExtrema->value) > upperValueThd;
        if (isFound) leftIdx = maxExtrema->index;
    }

    if (peakList.size() == 1)
    {
        isFound = true;
        leftIdx = peakList[0].index;
    }

    int meanIdx = 0;
    int thdIdx = signal.size();

    if (!isFound)
    {
        for (int i = 0; i < signal.size(); i++)
        {
            float c = abs(signal[i]);
            if (c < upperValueThd)
            {
                thdIdx = i;
                break;
            }
        }

        VecFloat leftRegionDiff = VecFloat(signal.begin(), signal.begin() + thdIdx);
        findPeaks.SetPeakType(PeakType::Peak == type ? PeakType::Valley : PeakType::Peak);
        findPeaks.Process(leftRegionDiff);
        auto valleyList = findPeaks.GetPeakResult(0.5);
        leftIdx = valleyList.empty() ? 0 : valleyList.begin()->index;
    }

#if _DEBUG_POLE_LEANING
    int n = signal.size();
    VecInt xList(signal.size(), 0);
    for (int i = 0; i < signal.size(); i++)
        xList[i] = i;
    cv::Mat drawImg = cv::Mat::zeros(100, signal.size(), CV_8UC3);

    Drawing drawTool;
    drawTool.yTopOrigin = false;
    drawTool.thickness = 1;
    drawTool.color = cv::Scalar(0, 255, 255);

    cv::line(drawImg, cv::Point(leftIdx, 0), cv::Point(leftIdx, drawImg.rows), COLOR_CV_BLUE);

    drawTool.thickness = 1;
    drawTool.color = cv::Scalar(255, 0, 255);
    drawTool.Plot(drawImg, xList, signal, true);

    drawTool.color = COLOR_CV_GREEN;
    drawTool.Plot(drawImg, VecFloat{ 0, (float)n }, VecFloat{ upperValueThd, upperValueThd }, false);

    drawTool.color = cv::Scalar(87, 2, 89);
    drawTool.Plot(drawImg, std::to_string(signal[leftIdx]), cv::Point(leftIdx, signal[leftIdx]));

#endif //_DEBUG_POLE_LEANING
    return leftIdx;
}

bool CheckPoleLeaningAuto(const cv::Mat &image, int windowSize, int &leftx, int &rightx)
{
    const double minStdv = 0.6;
    cv::Mat src;
    if (windowSize > 0)
    {
        int filterSize = std::max(windowSize % 2 == 0 ? windowSize + 1 : windowSize, 5);
        cv::medianBlur(image, src, filterSize);
        //cv::GaussianBlur(image, src, cv::Size(filterSize, filterSize), 0);
    }
    else
    {
        src = image;
    }

    cv::Mat claheimage;
    cv::normalize(src, claheimage, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Rect poleLeaningROI = cv::Rect(0, 0, claheimage.cols, claheimage.rows);
    poleLeaningROI.y = FindRefLine(claheimage, 0);
    poleLeaningROI.height -= poleLeaningROI.y;
    claheimage = claheimage(poleLeaningROI);

    int leftIdx = 0;
    int rightIdx = claheimage.cols - 1;
    cv::Scalar m = cv::Scalar(0);
    cv::Scalar stdDev = cv::Scalar(0);
    float upperValueThd = 255;
    float lowerValueThd = 0.0;
    bool isCheckLeaning = poleLeaningROI.height > 10;

    int yCathode = 0;
    if (isCheckLeaning)
    {
        VecFloat columnsum;
        cv::reduce(claheimage, columnsum, 0, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
        cv::GaussianBlur(columnsum, columnsum, cv::Size(11, 1), 0, 0, cv::BorderTypes::BORDER_DEFAULT);
        int numbercolumns = columnsum.size();
        VecFloat columnsumDiff;

        constexpr int region = 12;
        int limit4LeftSide = std::min(numbercolumns / 4, std::max(80, (numbercolumns / region)));
        int limit4RightSide = numbercolumns - limit4LeftSide;
        cv::Sobel(columnsum, columnsumDiff, -1, 1, 0, 3);

        cv::meanStdDev(columnsumDiff, m, stdDev);

        float stdThd = std::max(stdDev[0], minStdv);
        upperValueThd = 15;


        //auto leftSignal = VecFloat(columnsumDiff.begin(), columnsumDiff.begin() + limit4LeftSide);
        leftIdx  = FindLeaningPos(VecFloat(columnsumDiff.begin(), columnsumDiff.begin() + limit4LeftSide), PeakType::Valley, upperValueThd);
        rightIdx = FindLeaningPos(VecFloat(columnsumDiff.rbegin(), columnsumDiff.rbegin() + limit4LeftSide), PeakType::Peak, upperValueThd);
        rightIdx = columnsumDiff.size() - 1 - rightIdx;

#if _DEBUG_POLE_LEANING
        VecInt xList(numbercolumns);
        for (int i = 0; i < numbercolumns; i++)
            xList[i] = i;
        cv::Mat drawImg = image.clone();

        Drawing drawTool;
        drawTool.yTopOrigin = false;
        drawTool.thickness = 1;
        drawTool.color = cv::Scalar(0, 255, 255);
        drawTool.Plot(drawImg, xList, columnsum);
        //drawTool.color = cv::Scalar(218,223,19);
        //drawTool.plot(drawImg, xList, out,false);

        cv::line(drawImg, cv::Point(limit4LeftSide, 0), cv::Point(limit4LeftSide, drawImg.rows), COLOR_CV_RED);
        cv::line(drawImg, cv::Point(limit4RightSide, 0), cv::Point(limit4RightSide, drawImg.rows), COLOR_CV_RED);

        cv::line(drawImg, cv::Point(leftIdx, 0), cv::Point(leftIdx, drawImg.rows), COLOR_CV_BLUE);
        cv::line(drawImg, cv::Point(rightIdx, 0), cv::Point(rightIdx, drawImg.rows), COLOR_CV_BLUE);

        cv::line(drawImg, cv::Point(0, poleLeaningROI.y), cv::Point(drawImg.cols, poleLeaningROI.y), COLOR_CV_BLUE);

        drawTool.thickness = 1;
        drawTool.color = cv::Scalar(255, 0, 255);
        drawTool.Plot(drawImg, xList, columnsumDiff, true);
        drawTool.color = cv::Scalar(0, 0, 255);
        drawTool.Plot(drawImg, std::vector<double>{0, (double)numbercolumns}, std::vector<double>{m[0], m[0]}, false);
        drawTool.color = COLOR_CV_GREEN;
        drawTool.Plot(drawImg, VecFloat{0, (float)numbercolumns}, VecFloat{upperValueThd, upperValueThd}, false);
        drawTool.Plot(drawImg, VecFloat{0, (float)numbercolumns}, VecFloat{lowerValueThd, lowerValueThd}, false);

        drawTool.color = cv::Scalar(87, 2, 89);
        drawTool.Plot(drawImg, std::to_string(columnsumDiff[leftIdx]), cv::Point(leftIdx, columnsumDiff[leftIdx]));
        drawTool.Plot(drawImg, std::to_string(columnsumDiff[rightIdx]), cv::Point(rightIdx, columnsumDiff[rightIdx]));

        cv::Point textPos(drawImg.cols / 2, drawImg.rows / 3);
        drawTool.color = COLOR_CV_RED;
        drawTool.Plot(drawImg, "s: " + std::to_string(stdDev[0]), textPos, 1, true);
        drawTool.Plot(drawImg, "m: " + std::to_string(m[0]), textPos + cv::Point(0, 15), 1, true);

        //show_window("Leaning", drawImg);
        //cv::imwrite(RemoveExtension(imageName) + "Leaning.jpg", drawImg);
#endif //_DEBUG_POLE_LEANING
    }

    leftx = leftIdx + poleLeaningROI.x;
    rightx = rightIdx + poleLeaningROI.x;

    return isCheckLeaning;
}

bool CheckPoleLeaningManual(const cv::Mat &image, int leaningThreshold, int &leftx, int &rightx)
{
    int minPoleLeanningHeight = image.rows > 30 ? image.rows / 3 : 20;

    cv::Rect poleLeaningROI;
    poleLeaningROI.x = 0;
    poleLeaningROI.y += image.rows / 3;
    poleLeaningROI.width = image.cols;
    poleLeaningROI.height = image.rows - poleLeaningROI.y;
    RefineROI(poleLeaningROI, image.size());

    int leftIdx = 0;
    int rightIdx = poleLeaningROI.width;

    int upperValueThd = leaningThreshold;
    bool isCheckLeaning = !poleLeaningROI.empty() && (upperValueThd < 255) && (upperValueThd > 0) && (poleLeaningROI.height > minPoleLeanningHeight);

    if (isCheckLeaning)
    {
        int filterSize = 11;
        cv::Mat claheimage;

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(15, cv::Size(75, poleLeaningROI.height));
        clahe->apply(image(poleLeaningROI), claheimage);

        VecFloat columnsum;
        cv::reduce(claheimage, columnsum, 0, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
        cv::GaussianBlur(columnsum, columnsum, cv::Size(filterSize, 1), 0, 0, cv::BorderTypes::BORDER_DEFAULT);

        int numbercolumns = columnsum.size();
        VecFloat columnsumDiff(numbercolumns, 0);
        //VecFloat columnsumDiff2(numbercolumns, 0);

        constexpr int region = 12;
        int limit4RightSide = numbercolumns * (region - 1) / region;
        int limit4LeftSide = (numbercolumns / region);

        //Check right to middle
        for (int i = numbercolumns - 3; i > limit4RightSide; i--)
        {
            float current = columnsum[i];
            float next = columnsum[i - 2];
            if (current > upperValueThd && next < upperValueThd)
            {
                rightIdx = i;
                break;
            }
        }
        //Check left to middle
        for (int i = 0; i < limit4LeftSide; i++)
        {
            float current = columnsum[i];
            float next = columnsum[i + 2];
            if (current > upperValueThd && next < upperValueThd)
            {
                leftIdx = i;
                break;
            }
        }

#if _DEBUG_POLE_LEANING
        VecInt xList(numbercolumns);
        for (int i = 0; i < numbercolumns; i++)
            xList[i] = i;

        Drawing drawTool;
        drawTool.yTopOrigin = false;
        drawTool.thickness = 1;
        drawTool.Plot(claheimage, xList, columnsum);

        drawTool.color = COLOR_CV_GREEN;
        drawTool.Plot(claheimage, VecInt{0, numbercolumns}, VecInt{upperValueThd, upperValueThd}, false);

        cv::line(claheimage, cv::Point(limit4LeftSide, 0), cv::Point(limit4LeftSide, claheimage.rows), COLOR_CV_RED);
        cv::line(claheimage, cv::Point(limit4RightSide, 0), cv::Point(limit4RightSide, claheimage.rows), COLOR_CV_RED);

        cv::line(claheimage, cv::Point(leftIdx, 0), cv::Point(leftIdx, claheimage.rows), COLOR_CV_BLUE);
        cv::line(claheimage, cv::Point(rightIdx, 0), cv::Point(rightIdx, claheimage.rows), COLOR_CV_BLUE);

        //show_window("Leaning", claheimage);
        //cv::imwrite(Utils::RemoveExtension(imageName) + "Leaning.jpg", claheimage);
#endif //_DEBUG_POLE_LEANING
    }

    leftx = leftIdx + poleLeaningROI.x;
    rightx = rightIdx + poleLeaningROI.x;

    return isCheckLeaning;
}

// jrRoi la battery roi da duoc edit jrRoi.y=battery.y + jr.first
// If leaningThreshold >= 255 disable the checking leaning.
// If 0 < leaningThreshold < 255 check by manual threshold.
// If leaningThreshold < 0 check by auto threshold.
bool CheckPoleLeaning(const cv::Mat &image, int poleRegionHeight, int leaningThreshold, double &leftx, double &rightx)
{
    int borderWidth = round(image.cols * BORDER_WIDTH_RATIO);
    cv::Rect poleROI;
    poleROI.x = borderWidth;
    poleROI.y = 2 * borderWidth;
    poleROI.width = image.cols - (2 * borderWidth);
    poleROI.height = image.rows - poleROI.y;
    RefineROI(poleROI, image.size());

    int leftIdx = 0;
    int rightIdx = poleROI.width;

    bool isCheckLeaning = (leaningThreshold < 255) && !poleROI.empty();
    bool isUseManualMethod = (leaningThreshold < 255) && (leaningThreshold > 0);
    if (isCheckLeaning)
    {
        if (isUseManualMethod) //Threshold method
        {
            isCheckLeaning = CheckPoleLeaningManual(image(poleROI), leaningThreshold, leftIdx, rightIdx);
        }
        else //auto method
        {
            isCheckLeaning = CheckPoleLeaningAuto(image(poleROI), 11, leftIdx, rightIdx);
        }
    }

    leftx = leftIdx + poleROI.x;
    rightx = rightIdx + poleROI.x;

#if _DEBUG_POLE_LEANING
    cv::Mat drawImg;
    cv::cvtColor(image, drawImg, cv::ColorConversionCodes::COLOR_GRAY2BGR);
    cv::rectangle(drawImg, poleROI, COLOR_CV_GREEN, 2);
    cv::line(drawImg, cv::Point(leftx, poleROI.y), cv::Point(leftx, poleROI.br().y), COLOR_CV_BLUE, 2);
    cv::line(drawImg, cv::Point(rightx, poleROI.y), cv::Point(rightx, poleROI.br().y), COLOR_CV_BLUE, 2);
    //show_window("Leaning Finnal", drawImg);
    //cv::imwrite(RemoveExtension(imageName) + "LeaningFinnal.jpg", drawImg);
#endif //_DEBUG_POLE_LEANING
    return isCheckLeaning;
}

VecPoint refineMaskingContour(const VecPoint& inContour,
                                            int imageWidth,
                                            int bottomLineShift,
                                            int TopLineShift,
                                            VecPoint outerPoint)
{
    cv::Point extBottom =
        *std::max_element(inContour.begin(), inContour.end(), [](const cv::Point &lhs, const cv::Point &rhs) { return lhs.y < rhs.y; });

    int nAssessPnts = 20;
    int nAssessPntsSide = nAssessPnts / 2;
    int contourSize = inContour.size();
    cv::Point botLPnt, botRPnt;
    int lPIdx = 0, rPIdx = 0;
    VecInt vtLPIdx, vtRPIdx;
    VecPoint outContour;
    int pIdx = 0;
    //Flatting Left outter line
    VecPoint LeftOuterPnts;
    VecPoint vtLeftBotLine;
    VecPoint vtRightBotLine;

    int cPIdx = 0;
    //Find the bottom Center point
    for (auto pnt = inContour.begin(); pnt != inContour.end(); ++pnt)
    {
        if (pnt->x >= imageWidth / 2)
        {
            //CenterPnts.push_back(*pnt);
            cPIdx = pIdx;
            break;
        }
        pIdx++;
    }
    pIdx = 0;

    //Find left tuning point
    for (int i = cPIdx - nAssessPntsSide; i > nAssessPntsSide && inContour[i].y != 0; i--)
    {
        if (inContour[i].x == outerPoint[0].x)
        {
            LeftOuterPnts.push_back(inContour[i]);
            vtLPIdx.push_back(i);
            if (inContour[i].y != 0)
            {
                lPIdx = i;
                break;
            }
        }
    }
    if (LeftOuterPnts.size() > 0)
    {
        std::vector<size_t> lIdx(LeftOuterPnts.size());
        std::iota(lIdx.begin(), lIdx.end(), 0);
        std::stable_sort(lIdx.begin(), lIdx.end(), [&LeftOuterPnts](size_t left, size_t right) {
            return LeftOuterPnts[left].y > LeftOuterPnts[right].y;
        });

        lPIdx = vtLPIdx[lIdx[0]];
        for (int i = 0; i < LeftOuterPnts[lIdx[0]].x; i++)
        {
            vtLeftBotLine.push_back(cv::Point(i, LeftOuterPnts[lIdx[0]].y));
        }
    }

    //Flatting Right outter line
    pIdx = 0;
    VecPoint RightOuterPnts;
    for (auto pnt = inContour.begin(); pnt != inContour.end(); ++pnt)
    {
        if (pnt->x == outerPoint[1].x)
        {
            RightOuterPnts.push_back(*pnt);
            vtRPIdx.push_back(pIdx);
            if (pnt->y != 0)
            {
                rPIdx = pIdx;
                break;
            }
        }
        pIdx++;
    }

    if (RightOuterPnts.size() > 0)
    {
        for (int i = RightOuterPnts[0].x; i < imageWidth; i++)
        {
            vtRightBotLine.push_back(cv::Point(i, RightOuterPnts[0].y));
        }
    }

    VecPoint vtBottomLine;
    if (lPIdx < rPIdx)
    {
        vtBottomLine = VecPoint(inContour.begin() + lPIdx, inContour.begin() + rPIdx);
        vtBottomLine.insert(vtBottomLine.begin(), vtLeftBotLine.begin(), vtLeftBotLine.end());
        vtBottomLine.insert(vtBottomLine.end(), vtRightBotLine.begin(), vtRightBotLine.end());
        VecPoint vtTopLine(vtBottomLine);
        std::sort(vtTopLine.begin(), vtTopLine.end(), [](auto &left, auto &right) { return left.x > right.x; });
        std::for_each(vtBottomLine.begin(), vtBottomLine.end(), [&](cv::Point &p) { p.y -= bottomLineShift; });
        std::for_each(vtTopLine.begin(), vtTopLine.end(), [&](cv::Point &p) { p.y -= TopLineShift; });
        //std::for_each(vtTopLine.begin(), vtTopLine.end(), [](cv::Point& p) { p.y -= 30.0; });
        //std::for_each(vtBottomLine.begin(), vtBottomLine.end(), [](cv::Point& p) { p.y -= 10.0; });
        vtBottomLine.insert(vtBottomLine.begin(), vtTopLine.begin(), vtTopLine.end());
    }

    return vtBottomLine;
}

VecFloat CheckTiltedLine(const VecPoint &line, int radiusPole)
{
    VecFloat resultLine;
    if (!line.empty()) 
    {
        VecPoint copyLine = line;
        std::vector <float> derivativeLine;
        std::reverse(copyLine.begin(), copyLine.end());
        for (int i = 0; i < copyLine.size() - 1; i++)
        {
            float diffValue = float(-copyLine[i].x) + float(copyLine[i + 1].x);
            derivativeLine.push_back(diffValue);
        }

        VecFloat countLine(derivativeLine.size(), 0.0);
        int count = 0;
        for (int i = 0; i < derivativeLine.size(); i++)
        {
            if (derivativeLine[i] > 0.0)
            {
                if (count < 0)
                {
                    if (abs(count) > radiusPole)
                    {
                        countLine[i - 1] = count;
                    }
                    count = 0;
                }
                if (count >= 0) {
                    count = count + derivativeLine[i];
                }
            }
            if (derivativeLine[i] < 0.0)
            {
                if (count > 0)
                {
                    if (abs(count) > radiusPole)
                    {
                        countLine[i - 1] = count;
                    }
                    count = -1;
                }
                if (count <= 0)
                {
                    count = count + derivativeLine[i];
                }
            }
            if (i == derivativeLine.size() - 1)
            {
                countLine[i] = count;
            }
        }

        resultLine = countLine;
    }
    return resultLine;
}

auto AverageSliceImage(const std::vector<cv::Mat>& imgs) -> cv::Mat
{
    cv::Mat avgImg;
    if(!imgs.empty())
    {
        auto size = imgs[0].size();
        auto type = imgs[0].type();
        if(!size.empty())
        {
            avgImg = cv::Mat::zeros(size, CV_32F);
            int count = 0;
            for(auto img : imgs)
            {
                if(img.size() == size
                    && img.type() == type)
                {
                    count++;
                    avgImg += img;
                }
            }

            if(count != 0) avgImg /= count;

            avgImg.convertTo(avgImg, type);
        }
    }
    return avgImg;
}

auto AverageSliceImage(const cv::Mat& imgs, int noSlice) -> cv::Mat
{
    cv::Mat avgImg;
    if(!imgs.empty())
    {
        noSlice = xvt::SaturateCast<int>(noSlice, 1, 12);
        cv::Size imgSize(imgs.cols, imgs.rows / noSlice);
        auto type = imgs.type();
        if(!imgSize.empty() && noSlice > 1)
        {
            avgImg = cv::Mat::zeros(imgSize, CV_32F);
            int count = 0;
            for(int i = 0; i < noSlice; i++)
            {
                cv::Rect imgROI = CreateROI(cv::Point(0, imgSize.height*i), imgSize, imgs.size());
                if(!imgROI.empty() && imgROI.size() == imgSize)
                {
                    count++;
                    avgImg += imgs(imgROI);
                }
            }

            if(count != 0) avgImg /= count;

            avgImg.convertTo(avgImg, type);
        }
        else
        {
            avgImg = imgs.clone();
        }
    }
    return avgImg;
}

auto GetSliceImages(const cv::Mat& imgs, int noSlice) -> std::vector<cv::Mat>
{
    std::vector<cv::Mat> sliceImgs;
    if(!imgs.empty())
    {
        noSlice = xvt::SaturateCast<int>(noSlice, 1, 12);
        cv::Size imgSize(imgs.cols, imgs.rows / noSlice);
        if(!imgSize.empty() && noSlice > 1)
        {
            for(int i = 0; i < noSlice; i++)
            {
                cv::Rect imgROI = CreateROI(cv::Point(0, imgSize.height*i), imgSize, imgs.size());
                if(!imgROI.empty() && imgROI.size() == imgSize)
                {
                    sliceImgs.emplace_back(imgs(imgROI));
                }
            }
        }
        else
        {
            sliceImgs.emplace_back(imgs);
        }
    }
    return sliceImgs;
}

auto GetMeanProminece(xvt::FindPeaks & findPeaks)
{
    auto listPeaks = findPeaks.GetPeaks();
    int count = 0;
    float mean = 0.0;
    for (auto const& p : listPeaks)
    {
        if (p.prominence > 0)
        {
            mean += p.prominence;
            count++;
        }
    }

    return count>0? mean /= (float)count:0;
}

auto GetPeakAndValley(const VecFloat& signal, float prominence, int distance, int ksize) -> VecPeak
{
    VecPeak result;
    if(!signal.empty())
    {
        VecFloat smoothVec;
        if (ksize > 1)
            cv::GaussianBlur(signal, smoothVec, cv::Size(ksize, 1), 0.0, 0.0, cv::BORDER_REFLECT);
        else
            smoothVec = signal;

        for(int i = 0; i < smoothVec.size(); i++)
        {
            Peak p;
            p.index = i;
            p.value = smoothVec[i];
            result.push_back(p);
        }
        // Determine the pole position from accumulation graph
        xvt::FindPeaks findPeaks = xvt::FindPeaks(xvt::PeakType::Peak, xvt::PeakFindingMethod::Prominence);
        findPeaks.Process(smoothVec);
        auto proThd = prominence;
        if (prominence < 0)
        {
            proThd = GetMeanProminece(findPeaks) * abs(prominence);
        }

        auto peakResult = findPeaks.GetPeakResult(proThd, distance);
        for(auto p: peakResult)
            result[p.index].prominence = p.prominence;

        xvt::FindPeaks findValleys = xvt::FindPeaks(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
        findValleys.Process(smoothVec);

        if (prominence < 0)
        {
            proThd = GetMeanProminece(findValleys) * abs(prominence);
        }
        auto valleyResult = findValleys.GetPeakResult(proThd, distance);
        for(auto v: valleyResult)
            result[v.index].prominence = v.prominence;
    }
    return result;
}

auto GetPeakAndValleyImage(const cv::Mat& src, float prominence, int distance, int ksize) -> cv::Mat
{
    cv::Mat result;
    if(!src.empty())
    {
        result = cv::Mat::zeros(src.size(), CV_32FC1);
        cv::Size srcSize = src.size();
        for(int i = 0; i < srcSize.height; i++)
        {
            // Convert cv::Mat to vector
            std::vector<float> dataVector;
            src.row(i).reshape(1, 1).copyTo(dataVector);
            auto pvResult = GetPeakAndValley(dataVector, prominence, distance, ksize);
            for(auto v: pvResult)
                if(v.prominence != 0)
                    result.at<float>(cv::Point(v.index, i)) = v.value*v.prominence/abs(v.prominence);
        }
    }
    return result;
}

auto CalculateCTF(cv::Mat const& src, double prominence, int distance, int ksize) -> double
{
    double ctfValue = 0.0;
    if(!src.empty())
    {
        VecFloat reduceVec2;
        cv::reduce(src, reduceVec2, 0, cv::REDUCE_AVG, CV_32F);
        auto pvValue = GetPeakAndValley(reduceVec2, prominence, distance);
        int preIdx = 0;
        VecDouble ctfLst;
        
        for(auto p : pvValue) 
        {
            if(p.prominence != 0)
            {
                double ctf = 0.0;
                if(preIdx != p.index && (p.value + pvValue[preIdx].value) != 0)
                {
                    ctf = abs(p.value - pvValue[preIdx].value) / (p.value + pvValue[preIdx].value);
                }
                ctfLst.push_back(ctf);
                preIdx = p.index;
            }
        }

        if(!ctfLst.empty())
        {
            ctfValue = (std::accumulate(ctfLst.begin(), ctfLst.end(), 0.0)/ctfLst.size())*100;
        }

    }
    return ctfValue;
}

auto FindBatteryByTurnPoint(cv::Mat const& img, int thd, std::array<cv::Point, 4>& corners) -> InspectionResult
{
    InspectionResult result;
    if (!img.empty() && img.rows > 20)
    {
        cv::Mat blurImg;
        cv::medianBlur(img, blurImg, 9);
        cv::Mat sobelImg = xvt::Sobel(blurImg);
        cv::Rect lftROI = xvt::CreateROI(0, 0, img.cols * 0.25, img.rows, img.size());
        cv::Rect rgtROI = xvt::CreateROI(img.cols * 0.75, 0, img.cols * 0.25, img.rows, img.size());
        VecPoint leftCountours, rightCountours;
        bool leftTurn, rightTurn;
        std::tie(corners[0], corners[3], leftCountours, leftTurn) = FindTurnPoint(blurImg(lftROI), thd, true, img.rows/4);
        std::tie(corners[1], corners[2], rightCountours, rightTurn) = FindTurnPoint(blurImg(rgtROI), thd, false, img.rows/4);
        corners[1] += rgtROI.tl();
        corners[2] += rgtROI.tl();

        if(leftTurn && rightTurn)
        {
            result.SetResult(EResult::OK);
        }
        else
        {
            result(EResult::ER, "FindBattery: Can't find corner point of battery!");
        }
        result.mPoints = std::move(leftCountours);
        rightCountours = xvt::Transform(rightCountours, rgtROI.tl());

        result.mPoints.insert(result.mPoints.end(), rightCountours.begin(), rightCountours.end());
    }
    else
    {
        result(EResult::ER, "FindBattery: Invalid Image!");
    }
    return result;
}

auto FindTurnPoint(cv::Mat const& img, int thd, bool isLeft, int min_length) -> std::tuple<cv::Point, cv::Point, VecPoint, bool>
{
    cv::Point p1;
    cv::Point p2;
    cv::Point p3;
    VecPoint contour;
    bool isTurn = false;
    if (!img.empty())
    {
        contour = FindMaxContour(img, thd, false, CompareRectArea, cv::CHAIN_APPROX_NONE);
        auto rect = cv::boundingRect(contour);
		min_length = std::max(rect.height / 2, 20);
        if(!contour.empty())
        {
            double eps = 3;
            VecPoint aproxcontour;
            cv::approxPolyDP(contour, aproxcontour, eps, false);

            if(isLeft)
            {
                std::reverse(aproxcontour.begin(), aproxcontour.end());
            }

            int startidx = 0;
            int endidx = (int)aproxcontour.size() - 1;
            for (int i = startidx; i <= endidx; i++)
            {
               
                p2 = aproxcontour[i];
                if (i == startidx)
                {
                    p1 = aproxcontour[endidx];
                }
                else
                {
                    p1 = aproxcontour[i - 1];
                }

                if (i == endidx)
                {
                    p3 = aproxcontour[startidx];
                }
                else
                {
                    p3 = aproxcontour[i + 1];
                }
                cv::Point p1Vec = p1 - p2;
                cv::Point p2Vec = p2 - p3;
                auto len_v = cv::norm(p1Vec);
                double l = 0.0;
                if (len_v != 0)
                {
                    l = p2Vec.cross(p1Vec);
                }
                else
                {
                    l = cv::norm(p1Vec);
                }
                isTurn = (isLeft ? l < 0 : l > 0) && p1Vec.y > min_length;
                if (isTurn)
                {
                    break;
                }
            }

#if _DEBUG_FIND_TURN_POINT
            cv::Mat drawImg;
            xvt::ConvertRGB(img, drawImg);
            xvt::DrawPoints(drawImg, contour, COLOR_CV_RED);
            xvt::DrawPoints(drawImg, aproxcontour, COLOR_CV_CYAN);
            xvt::DrawPlusSign(drawImg, p1, CVPen(COLOR_CV_ORANGE), 5);
            xvt::DrawPlusSign(drawImg, p2, CVPen(COLOR_CV_YELLOW), 5);
            xvt::DrawPlusSign(drawImg, p3, CVPen(COLOR_CV_BLUE), 5);
#endif // _DEBUG_FIND_TURN_POINT
        }

    }
    return std::make_tuple(p2, p1, contour, isTurn);
}

}
}