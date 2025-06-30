#include "JRBatteryInspect.h"
#include "BatteryUtils.h"
#include "CVPen.h"
#include "ColorDefine.h"
#include "Peak.h"
#include "Utils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace xvt
{
namespace battery
{

//Find Cathode Line
std::vector<cv::Point> JRBatteryInspect::FindAnodeLine(const cv::Mat &imgROI, cv::Point offsetAnode, int region)
{
    std::vector<cv::Point> catLine;
    if (!imgROI.empty() && imgROI.rows > mMinImageHeight)
    {
        catLine.reserve(imgROI.cols);
        //local histogram equalization
        cv::Mat localEnhanceCathodeLine = LocalHistogramEqualization(imgROI, region); //LOCAL_HISTOGRAM_REDUCE
        cv::Mat dstImg = DiagonalFilter(localEnhanceCathodeLine, mFilterSize, mSigma, mBorderMode);
        catLine = Dijkstra_Algorithm(dstImg);
        for (auto &p : catLine)
            p += offsetAnode;
    }
    return catLine;
}
//Find Start Point Cathode
cv::Point JRBatteryInspect::FindStartPointCathode(const cv::Mat &src, cv::Rect &midSubROI, cv::Rect &subROI)
{
    cv::Point catPos{};
    if (!src.empty() && RefineROI(midSubROI, src.size()))
    {
        std::vector<float> tempReduceVec;
        cv::reduce(src(midSubROI), tempReduceVec, 0, cv::REDUCE_AVG);
        auto maxPos = std::min_element(tempReduceVec.begin(), tempReduceVec.end());
        catPos.x = midSubROI.tl().x + std::distance(tempReduceVec.begin(), maxPos);

        cv::Mat thdImg, reduceThresholdImg;
        cv::threshold(src(midSubROI), thdImg, 0, 1, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        cv::reduce(thdImg, reduceThresholdImg, 1, cv::REDUCE_SUM, CV_32FC1);
        std::vector<float> tempreduceThresholdVec = cv::Mat_<float>(reduceThresholdImg.col(0));
        int thresholdCathode = (midSubROI.width < mThresholdCathodeDefault) ? midSubROI.width : mThresholdCathodeDefault;
        auto firstPos = std::find_if(tempreduceThresholdVec.begin(), tempreduceThresholdVec.end(), [&thresholdCathode](const float &t) {
            return t > thresholdCathode;
        });
        catPos.y = midSubROI.tl().y + std::distance(tempreduceThresholdVec.begin(), firstPos);
    }
    return catPos;
}

//Apply mask for original image
cv::Mat JRBatteryInspect::ContourMaskFilter(const cv::Mat &imgROI, std::vector<cv::Point> cathodeLine, cv::Rect &subROI)
{
    cv::Mat tmpDst;

    if (!imgROI.empty() && !cathodeLine.empty())
    {
        std::vector<cv::Point> outerPoints;
        outerPoints.push_back(cv::Point(0, 0));
        outerPoints.push_back(cv::Point(subROI.width - 1, 0));
        std::vector<cv::Point> vtRoi4ContourAll = refineMaskingContour(cathodeLine, subROI.width, 0, mPoleHeight, outerPoints);

        tmpDst = cv::Mat::zeros(imgROI.size(), CV_8UC1);
        if (!vtRoi4ContourAll.empty())
        {
            cv::Mat mask = cv::Mat::zeros(imgROI.size(), imgROI.type());
            drawContours(mask, std::vector<std::vector<cv::Point>>(1, vtRoi4ContourAll), -1, cv::Scalar::all(255), -1);
            imgROI.copyTo(tmpDst, mask);
        }
    }

    return tmpDst;
}

JRBatteryInspectResult JRBatteryInspect::Inspect(const cv::Mat &src, cv::Mat &res, cv::Rect subROI)
{
    JRBatteryInspectResult ouput;

    if (src.empty())
        return ouput;
    else
    {
        cv::cvtColor(src, res, cv::COLOR_GRAY2BGR);
    }
    xvt::RefineROI(subROI, src.size());
    cv::Mat imgROI = src(subROI).clone();

    //Find Anode Line
    std::vector<cv::Point> catLine = FindAnodeLine(imgROI, cv::Point());

    //Image ROI after using a filtering mask
    cv::Mat imgROIContour = ContourMaskFilter(imgROI, catLine, subROI);
    std::vector<float> reduceVec = reduceBlackBackgroundMat(imgROIContour, 0, mKSize);


    // Determine the pole position from accumulation graph
    FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
    findPeaks.Process(reduceVec);
    auto peakResult = findPeaks.GetPeakResult(mProminenceThreshold, mProminenceDistanceThreshold);

    std::vector<cv::Point> lstMidPos;
    for (auto &peak : peakResult)
    {
        lstMidPos.push_back(catLine[peak.index] + subROI.tl());
    }

    //Find List Start Points Cathode
    std::vector<cv::Point> lstCathodePos;
    for (int i = 1; i < lstMidPos.size(); i++)
    {
        cv::Rect midSubROI(lstMidPos[i - 1] + cv::Point(mOffset, 0),
                           cv::Size(abs(lstMidPos[i].x - lstMidPos[i - 1].x) - 2 * mOffset, abs(subROI.br().y - lstMidPos[i - 1].y)));
        xvt::RefineROI(midSubROI, src.size());
        lstCathodePos.push_back(FindStartPointCathode(src, midSubROI, subROI));
    }

    std::vector<cv::Point> lstAnodePos = lstMidPos;
    std::vector<cv::Point> lstAnodePosBegin;

    lstAnodePosBegin.emplace_back(lstAnodePos.begin()->x, lstCathodePos.begin()->y);
    for (int i = 1; i < lstAnodePos.size(); i++)
    {
        lstAnodePosBegin.emplace_back(lstAnodePos[i].x, lstCathodePos[i - 1].y);
    }
    assert(lstAnodePosBegin.size() == lstAnodePos.size());

    for (int i = 0; i < lstAnodePosBegin.size(); i++)
    {
        cv::Rect anodeSubROI(lstAnodePos[i] - cv::Point(mSizeW, 0), cv::Size(1 + 2 * mOffset, abs(lstAnodePosBegin[i].y - lstAnodePos[i].y)));
        if (xvt::RefineROI(anodeSubROI, src.size()))
        {
            double th = cv::sum(src(anodeSubROI))[0] / anodeSubROI.area() + mOffsetThreshold;

            for (int j = lstAnodePosBegin[i].y; j > subROI.tl().y; j--)
            {
                if (src.at<uchar>(cv::Point(lstAnodePosBegin[i].x, j)) > th && src.at<uchar>(cv::Point(lstAnodePosBegin[i].x, j - 1)) > th &&
                    src.at<uchar>(cv::Point(lstAnodePosBegin[i].x, j - 2)) > th && src.at<uchar>(cv::Point(lstAnodePosBegin[i].x, j - 3)) > th &&
                    src.at<uchar>(cv::Point(lstAnodePosBegin[i].x, j - 4)) > th)
                {
                    lstAnodePos[i].y = j;
                    break;
                }
            }
        }
        ouput.lstAnodeLenght.push_back(abs(lstAnodePosBegin[i].y - lstAnodePos[i].y));
    }

    //Draw Result
    DrawResults(res, lstAnodePos, lstCathodePos, subROI);
    ouput.lstPoint = lstCathodePos;
    ouput.lstPoint.insert(ouput.lstPoint.end(), lstAnodePos.begin(), lstAnodePos.end());

    std::sort(ouput.lstPoint.begin(), ouput.lstPoint.end(), [](const cv::Point &p1, const cv::Point &p2) { return p1.x < p2.x; });
    return ouput;
}

//Draw Result
void JRBatteryInspect::DrawResults(cv::Mat &img,
                                   const std::vector<cv::Point> endAnodePos,
                                   const std::vector<cv::Point> endCathodePos,
                                   cv::Rect subROI)
{
    //Define parameters
    int mEndDashLine = -1;
    cv::Scalar mCathodeColor = COLOR_CV_RED;
    cv::Scalar mAnodeColor = COLOR_CV_CYAN;

    if (img.empty())
        return;
    //if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
    //    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    if (!subROI.empty())
    {
        mEndDashLine = subROI.br().y;
        //Draw ROI Inspection
        cv::rectangle(img, subROI, COLOR_CV_ORANGE, 3);
    }
    else
    {
        mEndDashLine = img.rows;
    }
    //Draw result Cathode
    xvt::CVPen pen;
    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    pen.mFontScale = 0.5;
    pen.mColor = mCathodeColor;
    pen.mThickness = 1;
    for (int i = 0; i < endCathodePos.size(); i++)
    {
        xvt::DrawText(img, std::to_string((i + 1) * 2), endCathodePos[i], TextAlign::MIDDLE_CENTER, pen, cv::Point(0, -20));
        xvt::Drawing::DrawDashedLine(img, cv::Point(endCathodePos[i].x, mEndDashLine), endCathodePos[i], mCathodeColor, 1, "", 10);
        // Draw the point on the image
        cv::circle(img, endCathodePos[i], 2, pen.mColor, 3);
    }

    //Draw result Anode
    pen.mColor = mAnodeColor;
    for (int i = 0; i < endAnodePos.size(); i++)
    {
        xvt::DrawText(img, std::to_string((i * 2) + 1), endAnodePos[i], TextAlign::MIDDLE_CENTER, pen, cv::Point(0, -20));
        xvt::Drawing::DrawDashedLine(img, cv::Point(endAnodePos[i].x, mEndDashLine), endAnodePos[i], mAnodeColor, 1, "", 10);
        cv::circle(img, endAnodePos[i], 2, pen.mColor, 3);
    }
}
} // namespace battery
} // namespace xvt