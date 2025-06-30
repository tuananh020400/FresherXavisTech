#include "xvtBattery/CylinderBatteryUpper.h"
#include "xvtCV/Polyfit.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/Contour.h"
#include <xvtCV/Peak.h>
#include <xvtCV/Utils.h>
#include <numeric>

using namespace xvt::threshold;
#ifdef _DEBUG
#define DEBUG_POLE
#endif // _DEBUG
namespace xvt {
    namespace battery {


CylinderBatteryUpper::CylinderBatteryUpper() : CylinderBatteryBase()
{
    mInspectingItems.SetUpperInspectDefail();
}

int CylinderBatteryUpper::InspectBeading(const cv::Mat& Img, cv::Mat& resImage, cv::Rect batteryROI, VecDouble& BIresult)
{
    /*
    *                             .
    *                        .
    *                   .A0
    *               .
    *               .
    *              .L1
    *               .
    *                  .A1
    *                        .
    *                          .P1
    *                        .
    *                  .A2
    *               .
    *               .
    *               .
    */
    //For Left Point References
    cv::Point A0, A1, A2, P1, P2, L1;
    //For Right Point references
    cv::Point A3, A4, A5, P3, P4, L2;
    A0 = A1 = A2 = P1 = P2 = L1 = cv::Point(-1, -1);
    A3 = A4 = A5 = P3 = P4 = L2 = cv::Point(-1, -1);

    int yA2_real = batteryROI.y;

    cv::Mat grayImg;
    cv::threshold(Img(batteryROI), grayImg, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY_INV);
    cv::medianBlur(grayImg, grayImg, 9);
    cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
    cv::morphologyEx(grayImg, grayImg, cv::MORPH_CLOSE, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);

    auto bounderContour = FindMaxContour(grayImg, 0, false, CompareArea, cv::CHAIN_APPROX_NONE);

    if (bounderContour.size() < 100) return yA2_real;
    int idxTopLeft, idxBottomLeft, idxTopRight, idxBottomRight;
    std::pair<int, int> leftTurningIdx, rightTurningIdx;
    idxTopLeft = getPointOnContourId(bounderContour, cv::Point(0, 0));
    idxBottomLeft = getPointOnContourId(bounderContour, cv::Point(0, batteryROI.height));
    idxTopRight = getPointOnContourId(bounderContour, cv::Point(batteryROI.width, 0));
    idxBottomRight = getPointOnContourId(bounderContour, cv::Point(batteryROI.width, batteryROI.height));

    if (idxTopLeft == -1 || idxBottomLeft == -1 || idxTopRight == -1 || idxBottomRight == -1) { return yA2_real; }
    if (idxTopLeft > idxTopRight)
    {
        idxTopRight = bounderContour.size() - 1;
    }

    VecFloat difVecLeft, difVecRight;
    for (int it = idxTopLeft; it < idxBottomLeft; it++)
    {
        difVecLeft.push_back(bounderContour[it].x);
    }
    for (int it = idxBottomRight; it < idxTopRight; it++)
    {
        difVecRight.push_back(bounderContour[it].x);
    }

    if (difVecLeft.empty() || difVecRight.empty()) { return yA2_real; }

    cv::blur(difVecLeft, difVecLeft, cv::Size(3, 1));
    cv::blur(difVecRight, difVecRight, cv::Size(3, 1));

    FindPeaks findPeaks;
    findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
    findPeaks.Process(difVecLeft);
    std::vector<Peak> PeakLeft = findPeaks.GetPeakResult(1, mBeadingHeightMin, 0.8);

    findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
    findPeaks.Process(difVecRight);
    std::vector<Peak> PeakRight = findPeaks.GetPeakResult(1, mBeadingHeightMin, 0.8);

    if (PeakRight.empty() || PeakLeft.empty())
        return yA2_real;

    Peak maxPeakLeft = *std::max_element(PeakLeft.begin(), PeakLeft.end(),
        [](const Peak& lhs, const Peak& rhs) {
            return abs(lhs.prominence) < abs(rhs.prominence);
        });

    Peak maxPeakRight = *std::max_element(PeakRight.begin(), PeakRight.end(),
        [](const Peak& lhs, const Peak& rhs) {
            return abs(lhs.prominence) < abs(rhs.prominence);
        });

    int idxP1;
    idxP1 = maxPeakLeft.index + idxTopLeft;
    int idxP2;
    idxP2 = maxPeakRight.index + idxBottomRight;
    P1 = bounderContour[idxP1];
    P2 = bounderContour[maxPeakRight.index + idxBottomRight];

    if (mDisplayMode == DisplayMode::DALL)
    {
        cv::Mat drawingContour;
        cv::cvtColor(grayImg, drawingContour, cv::COLOR_GRAY2BGR);
        cv::drawContours(drawingContour, std::vector<VecPoint>{bounderContour}, 0, cv::Scalar(255, 0, 0), 1);

        cv::circle(drawingContour, P1, 2, COLOR_CV_RED, -1);
        cv::putText(drawingContour, "P1", cv::Point(P1.x + 2, P1.y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0, 0, 255 });

        cv::circle(drawingContour, P2, 2, COLOR_CV_RED, -1);
        cv::putText(drawingContour, "P2", cv::Point(P2.x + 2, P2.y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0, 0, 255 });

        cv::circle(drawingContour, bounderContour[idxTopLeft], 2, COLOR_CV_RED, -1);
        cv::circle(drawingContour, bounderContour[idxBottomLeft], 2, COLOR_CV_RED, -1);
        cv::circle(drawingContour, bounderContour[idxTopRight], 2, COLOR_CV_RED, -1);
        cv::circle(drawingContour, bounderContour[idxBottomRight], 2, COLOR_CV_RED, -1);

        namedWindow("drawing Beading Contour", cv::WINDOW_NORMAL);
        show_window("drawing Beading Contour", drawingContour);
    }
    if ((P1.x < bounderContour[idxTopLeft].x) || (P1.x < bounderContour[idxBottomLeft].x)
        || (P2.x > bounderContour[idxTopRight].x) || (P1.x > batteryROI.width / 2) || (P2.x < batteryROI.width / 2))
    {
        return yA2_real;
    }
    if (true == mIsCheckBeading)
    {
        if ((P1.x > (batteryROI.width - P2.x) * 1.2) || ((batteryROI.width - P2.x) > P1.x * 1.2)
            || (abs(P1.y - P2.y) > tan(5.0 * CV_PI / 180.0) * abs(P1.x - P2.x)))
        {
            return yA2_real;
        }
    }

    //Find Point in sub-Roi 3.1
    L1 = *std::min_element(bounderContour.begin() + idxTopLeft, bounderContour.begin() + idxP1,
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.x < rhs.x;
        });

    //Find Point in sub-Roi 3.2 in right

    L2 = *std::max_element(bounderContour.begin() + idxP2, bounderContour.begin() + idxTopRight,
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.x < rhs.x;
        });

    //find maximumpoint on the left

    if (mD1StartPosition >= 1)
    {
        A1 = P1;
        A2 = P1;
    }
    else
    {
        int grooveWidth = abs(P1.x - L1.x) * (1 - mD1StartPosition);
        for (int idx = idxP1; idx > idxTopLeft; idx--) {
            if (P1.x - bounderContour[idx].x >= grooveWidth) {
                A1 = bounderContour[idx];
                break;
            }
        }

        for (int idx = idxP1; idx < idxBottomLeft; idx++) {
            if (P1.x - bounderContour[idx].x >= grooveWidth) {
                A2 = bounderContour[idx];
                break;
            }
        }
    }

    for (int i = 0; i < A1.y; i++) {
        if (grayImg.at<uchar>(i, A1.x) == 255) {
            A0 = cv::Point(A1.x, i);
            break;
        }
    }

    //find maximumpoint on the right
    if (mD1StartPosition >= 1)
    {
        A4 = P2;
        A5 = P2;
    }
    else
    {
        int grooveWidth = abs(P2.x - L2.x) * (1 - mD1StartPosition);
        for (int idx = idxP2; idx < idxTopRight; idx++) {
            if (bounderContour[idx].x - P2.x >= grooveWidth) {
                A4 = bounderContour[idx];
                break;
            }
        }

        for (int idx = idxP2; idx > idxBottomRight; idx--) {
            if (bounderContour[idx].x - P2.x >= grooveWidth) {
                A5 = bounderContour[idx];
                break;
            }
        }
    }

    for (int i = 0; i < A4.y; i++) {
        if (grayImg.at<uchar>(i, A1.x) == 255) {
            A3 = cv::Point(A4.x, i);
            break;
        }
    }
    grayImg.release();

    if (A0.x == -1 || A2.x == -1 || A1.y == A2.y || A3.x == -1 || A4.x == -1 || A5.y == A4.y)
    {
        return yA2_real;
    }

    // A0
    cv::Point Offset = batteryROI.tl();
    A0 = A0 + Offset;
    A1 = A1 + Offset;
    A2 = A2 + Offset;
    P1 = P1 + Offset;
    P2 = P2 + Offset;
    L1 = L1 + Offset;

    //A3 = A3 + Offset;
    //A4 = A4 + Offset;
    A5 = A5 + Offset;
    //P3 = P3 + Offset;
    //P4 = P4 + Offset;
    //L2 = L2 + Offset;

    yA2_real = (A2.y + A5.y) / 2;

    assert(BIresult.size() == 5);
    int padding = 25;
    int length = 50;

    // 2: The inner diameter of the plant distance between P1 and P2
    if (mInspectingItems.INNER_DIAMETER) {
        BIresult[1] = abs(P1.x - P2.x) * mPixelSize;
        Drawing::DrawDimention(resImage, P1, P2, BIresult[1], cv::Scalar(0, 0, 0), 3, padding, length);
    }

    // 6: Maximum outer diameter bettery width
    if (mInspectingItems.OUTER_DIAMETER) {
        BIresult[2] = batteryROI.width * mPixelSize;

        cv::Point start = Offset - cv::Point(0, length);
        cv::Point end = start + cv::Point(batteryROI.width, 0);

        Drawing::DrawDimention(resImage, start, end, BIresult[2], cv::Scalar(18, 118, 219), 3, padding, length);
    }

    // 7: The depth of the groove = P1 - L1
    if (mInspectingItems.GROOVE_DEPTH) {
        BIresult[3] = (P1.x - L1.x) * mPixelSize;
        cv::Point start(L1.x, Offset.y + length / 3);
        cv::Point end(P1.x, Offset.y + length / 3);

        Drawing::DrawDimention(resImage, start, end, BIresult[3], cv::Scalar(255, 0, 255), 3, padding, length / 2);
        cv::line(resImage, end, P1, cv::Scalar(255, 0, 255), 2, cv::LINE_4);
    }

    // 8: Distance between upper and lower edges
    if (mInspectingItems.GROOVE_HEIGHT) {
        BIresult[4] = (A2.y - A1.y) * mPixelSize;
        Drawing::DrawDimention(resImage, A1, A2, BIresult[4], cv::Scalar(0, 0, 255), 3, padding, length / 2);
    }

    // 1: Thickness of top cover d = A0 - A1
    if (mInspectingItems.COVER_HEIGHT) {
        BIresult[0] = abs(A1.y - A0.y) * mPixelSize;
        Drawing::DrawDimention(resImage, A0, A1, BIresult[0], cv::Scalar(153, 210, 252), 3, padding, length / 2);
    }

    return yA2_real;
}

ERR_CODE CylinderBatteryUpper::FindCathodeXPosition(const cv::Mat& src, VecInt& lstPolePosXAll)
{
    if (!src.empty())
    {
        if (lstPolePosXAll.size() > 0) lstPolePosXAll.clear();

        int kSize = 5;
        int centerLeft = (src.cols - mCenterNeglectionWidth) / 2;
        int centerRight = (src.cols + mCenterNeglectionWidth) / 2;
        int boundaryLeft = 0;
        int boundaryRight = src.cols;
        VecFloat reduceVec = ReduceBlackBackgroundMat(src, 0, kSize);
        Rangei distanceRange = mPolesDistanceRange;
        std::vector<Peak> polePos;
        if (mEnableAutoMode || mPolesProminenceThreshold == 0)
        {
            polePos = FindCathodeXPositionAuto(reduceVec, distanceRange, mCenterNeglectionWidth);
        }
        else
        {
            polePos = FindCathodeXPositionManual(reduceVec, mPolesProminenceThreshold, distanceRange.GetLower());
        }

        int numberOfLeftPoles = 0;
        int numberOfRightPoles = 0;
        if (!polePos.empty())
        {
            if (mDisplayMode == DisplayMode::DALL)
            {
                Drawing drawTool;
                drawTool.padding_Y = 15;
                drawTool.yTopOrigin = false;
                drawTool.color = COLOR_CV_YELLOW;
                cv::Mat poleImg = DrawPeakInfo(src, reduceVec, polePos, drawTool);
                drawTool.ShowImg("Find Pole by Prominence", poleImg);
                poleImg.release();
            }

            int distanceThd = (distanceRange.GetLower() + distanceRange.GetUpper()) / 4;

            // Remove the poles close to the outer boundary
            boundaryLeft += distanceThd + 1;
            boundaryRight -= distanceThd + 1;

            // Remove the poles close to the inner boundary
            centerLeft -= distanceThd;
            centerRight += distanceThd;


            for (auto pole = polePos.begin(); pole != polePos.end();) {
                if ((pole->index > centerLeft && pole->index < centerRight) || pole->index < boundaryLeft || pole->index > boundaryRight) {
                    pole = polePos.erase(pole);
                }
                else {
                    if (pole->index <= centerLeft) {
                        numberOfLeftPoles += 1;
                    }
                    else {
                        //pole->index >= centerRight
                        numberOfRightPoles += 1;
                    }
                    lstPolePosXAll.push_back(pole->index);
                    pole++;
                }
            }
        }

        if (numberOfLeftPoles < 3 || numberOfRightPoles < 3
            || (mIsCheckPoleNo && (numberOfLeftPoles < mOneSidePoleNumber / 2
                || numberOfRightPoles < mOneSidePoleNumber / 2))
            )
        {
            return ERR_CODE::errPoleXDetection3;
        }
    }
    else
    {
        return ERR_CODE::errPoleXDetection;
    }
    return ERR_CODE::OK;
}


VecDouble CylinderBatteryUpper::FindCathodeLineByPeak(const cv::Mat& srcImg)
{
    VecDouble lstCathodeLine{};
    const int minImageHeight = 5;

    if (!srcImg.empty() && srcImg.rows > minImageHeight) {

        cv::Mat src;
        cv::medianBlur(srcImg, src, 3);

        cv::Size imgSize = src.size();
        int wSize = std::min(std::max(mCathodeLineWindowSize, 5), imgSize.width);
        int wIdx = 0;
        cv::Mat tmpImg = cv::Mat();
        float localThresh = 5.0;
        int idx = 0;
        cv::Mat prominenceImage = cv::Mat();
        if (mDisplayMode == DisplayMode::DALL) { prominenceImage = cv::Mat::zeros(src.size(), CV_8UC3); }

#ifdef _DEBUG_FIND_ANODE_X_POS
        cv::Mat drawImg = src.clone();
        cv::cvtColor(drawImg, drawImg, cv::COLOR_GRAY2BGR);
#endif

        while (wIdx + wSize < imgSize.width)
        {
            idx = lstCathodeLine.size() > 0 ? lstCathodeLine.back() : 0;
            int regionIdx = GetRegionIndex(wIdx, 6, imgSize.width, mCenterNeglectionWidth);

            if (regionIdx > 0)
            {
                switch (regionIdx)
                {
                case 1:
                    localThresh = mCathodeLineThredsholdOuter;
                    break;
                case 2:
                    localThresh = mCathodeLineThredsholdMiddle;
                    break;
                case 3:
                    localThresh = mCathodeLineThredsholdInner;
                    break;
                default:
                    localThresh = mCathodeLineThredsholdMiddle;
                    break;
                }
                cv::Rect subROI = cv::Rect(wIdx, 0, wSize, imgSize.height);
                if (true == RefineROI(subROI, imgSize))
                {
                    tmpImg = src(subROI);
                    cv::Mat1f reduceImg = cv::Mat();
                    cv::reduce(tmpImg, reduceImg, 1, cv::REDUCE_AVG, CV_32F); //Reduce to one row.
                    VecFloat reduceVec = cv::Mat_<float>(reduceImg);
                    reduceImg.release();

                    VecFloat list = reduceVec;
                    int vecSize = reduceVec.size() - 1;

                    for (int i = 2; i < reduceVec.size() - 2; i++)
                    {
                        reduceVec[i] = 0.1 * list[i - 2] + 0.25 * list[i - 1] + 0.3 * list[i] + 0.25 * list[i + 1] + 0.1 * list[i + 2];
                    }
                    // extern 2 element this will not skip 2 last element in filter
                    reduceVec[0] = reduceVec[2];
                    reduceVec[1] = reduceVec[2];
                    reduceVec[vecSize] = reduceVec[vecSize - 2];
                    reduceVec[vecSize - 1] = reduceVec[vecSize - 2];

                    VecFloat diffVec;
                    diffVec.push_back(0);
                    for (int i = 1; i < vecSize; i++) {
                        float diff = reduceVec[i + 1] - reduceVec[i - 1];
                        diff = diff > 0 ? 0 : diff;
                        diff = -pow(diff, 2);
                        diffVec.push_back(diff); //  > 0 ? 0 : diff
                    }
                    diffVec.push_back(0);

                    FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
                    findPeaks.Process(diffVec);
                    std::vector<Peak> peaks = findPeaks.GetPeakResult(localThresh);

                    if (peaks.size() > 0) {
                        idx = peaks.back().index;

                        if (mDisplayMode == DisplayMode::DALL)
                            std::for_each(peaks.begin(), peaks.end(), [&prominenceImage, fontScale = 0.25, wIdx](auto& p) {
                            cv::Point p1(wIdx, p.index);
                            std::ostringstream strProminence;
                            strProminence << std::fixed << std::setprecision(1) << abs(p.prominence);

                            std::string msg = strProminence.str();
                            cv::Size txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, fontScale, 1, 0);
                            prominenceImage.at<cv::Vec3b>(p1.y, p1.x) = { 0, 255, 0 };
                            cv::putText(prominenceImage, msg, cv::Point(p1.x - txtSize.width / 2, p1.y - 2), cv::FONT_HERSHEY_SIMPLEX, fontScale, { 0, 0, 255 });
                        });
                    }
                }
                else
                {
                    //ROI Refinement failed
                }
            }

            lstCathodeLine.push_back(idx);
            wIdx += wSize;
        }

        if (mDisplayMode == DisplayMode::DALL)
        {
            cv::namedWindow("Prominence Image", cv::WINDOW_NORMAL);
            show_window("Prominence Image", prominenceImage);
        }

        int sampleWindow = 5;
        if (lstCathodeLine.size() > sampleWindow)
        {
            lstCathodeLine.push_back(lstCathodeLine.back());

            int borderNeglction = 2;
            int neglectionWidth = borderNeglction * wSize;
            // start - end column of neglection area
            int startNeglectCol = (imgSize.width - mCenterNeglectionWidth) >> 1;
            int endNeglectCol = (imgSize.width + mCenterNeglectionWidth) >> 1;
            int rightEndBounder = imgSize.width - neglectionWidth;

            AverageCathodeLineResult<double> avgCathodeLineResult =
                FindAverageCathodeLine(lstCathodeLine, neglectionWidth, startNeglectCol, endNeglectCol, rightEndBounder, 11, wSize);

#ifdef _DEBUG_FIND_ANODE_X_POS
            // draw sth
            cv::line(drawImg, cv::Point((drawImg.cols - mCenterNeglectionWidth) / 2, 0), cv::Point((drawImg.cols - mCenterNeglectionWidth) / 2, drawImg.rows), cv::Scalar(0, 255, 0));
            cv::line(drawImg, cv::Point((drawImg.cols + mCenterNeglectionWidth) / 2, 0), cv::Point((drawImg.cols + mCenterNeglectionWidth) / 2, drawImg.rows), cv::Scalar(0, 255, 0));
            int wIdxDraw = 0;
            while (wIdxDraw + wSize <= src.size().width && avgCathodeLineResult.success)
            {
                cv::line(drawImg, cv::Point(wIdxDraw, avgCathodeLineResult.localAverageCathodeLine[wIdxDraw / wSize]), cv::Point(wIdxDraw + wSize, avgCathodeLineResult.localAverageCathodeLine[wIdxDraw / wSize]), cv::Scalar(0, 255, 0));
                wIdxDraw += wSize;
            }
#endif

            if (avgCathodeLineResult.success)
            {
#ifdef CATHODE_LINE_TRACING
                std::pair<int, int> minErrIdx =
                    FindCathodeLineStartingPoint(avgCathodeLineResult.localAverageCathodeLineLeft, avgCathodeLineResult.localAverageCathodeLineRight, lstCathodeLine, 3);
#endif

                RefineCathodeLine(avgCathodeLineResult, lstCathodeLine, wSize, startNeglectCol, endNeglectCol);

#ifdef CATHODE_LINE_TRACING
                TracingCathodeLine(src, wSize, 20, lstCathodeLine, minErrIdx, mCenterNeglectionWidth, 1);
#endif

#ifdef _DEBUG_FIND_ANODE_X_POS
                int X = 0;
                cv::Mat drawCathodeLineImg = srcImg.clone();
                cv::Mat enhanced = src.clone();
                cv::cvtColor(drawCathodeLineImg, drawCathodeLineImg, cv::COLOR_GRAY2BGR);
                cv::cvtColor(enhanced, enhanced, cv::COLOR_GRAY2BGR);
                while (X + wSize <= srcImg.cols)
                {
                    cv::line(enhanced, cv::Point(X, lstCathodeLine[X / wSize]), cv::Point(X + wSize, lstCathodeLine[X / wSize]), cv::Scalar(0, 0, 255), 1);
                    cv::line(drawCathodeLineImg, cv::Point(X, lstCathodeLine[X / wSize]), cv::Point(X + wSize, lstCathodeLine[X / wSize]), cv::Scalar(0, 0, 255), 1);
                    X += wSize;
                }
#endif

            }
            else { /* Do nothing*/ }
        }
        else { /* Do nothing*/ }
    }
    else { /* Do nothing*/ }

    return lstCathodeLine;
}

//#define _DEBUG_FIND_ANODE_X_POS
ERR_CODE CylinderBatteryUpper::FindCathodeLine(const cv::Mat& srcImg, VecPoint& lstCathodeLine, std::string& descript)
{
    lstCathodeLine.clear();
    descript = "";
    if (!srcImg.empty())
    {
        bool isUseAutoAlgorithm = (mCathodeLineThredsholdInner == 0
            && mCathodeLineThredsholdMiddle == 0
            && mCathodeLineThredsholdOuter == 0);

        if (mEnableAutoMode || isUseAutoAlgorithm)
        {
            lstCathodeLine = findCathodeLineByAStar(srcImg, mCenterNeglectionWidth);
        }
        else
        {
            VecDouble cathodeLine{};
            cathodeLine = FindCathodeLineByPeak(srcImg);
            int sampleWindow = 5;

            if (cathodeLine.size() < sampleWindow) {
                descript = "Cannot find suitable cathode line, please check Pole Region Height or Cathode line parameter in setting";
                return ERR_CODE::errPoleCathodeLine3;
            }

            int wSize = mCathodeLineWindowSize, lIdx = 0;

            float gaussSigma = 1;
            VecDouble vecSmoothedCath;
            vecSmoothedCath = gaussSmoothen(cathodeLine, gaussSigma, sampleWindow);
            if (vecSmoothedCath.size() <= 0) {
                descript = "Cannot find suitable cathode line, please check Pole Region Height or Cathode line parameter in setting";
                return ERR_CODE::errPoleCathodeLine4;
            }

            if (mDisplayMode == DisplayMode::DALL)
            {
                cv::Mat centerTestMat;
                cv::cvtColor(srcImg, centerTestMat, cv::COLOR_GRAY2BGR);
                for (int cathIdx = 0; cathIdx < vecSmoothedCath.size(); cathIdx++) {
                    cv::line(centerTestMat, cv::Point(lIdx, vecSmoothedCath[cathIdx]), cv::Point(lIdx + wSize, vecSmoothedCath[cathIdx]), cv::Scalar(0, 0, 250));
                    cv::putText(centerTestMat, std::to_string(cathIdx + 1), cv::Point(lIdx + 1, vecSmoothedCath[cathIdx] + 15), 1, 0.8, cv::Scalar(255, 0, 0));
                    lIdx += wSize;
                }
                show_window("cathodeLine img", centerTestMat);
                centerTestMat.release();
            }

            lstCathodeLine.reserve(srcImg.cols);
            for (int pIdx = 0; pIdx < srcImg.cols; pIdx++) {
                lstCathodeLine.push_back(cv::Point(pIdx, vecSmoothedCath[pIdx / wSize]));
            }
        }

        if (lstCathodeLine.size() < srcImg.cols) {
            descript = "Cannot generate enough sample of cathode line, please check Pole Region Height or Cathode line parameter in setting";
            return ERR_CODE::errPoleCathodeLine5;
        }
    }

    return ERR_CODE::OK;
}

/// <summary>
/// Find the top reference line
/// </summary>
/// <param name="image">JR image (ROI image)</param>
/// <param name="BatInsp">Setting</param>
/// <returns>top and bottom y position</returns>
std::pair<int, int> CylinderBatteryUpper::FindTopReferenceLine(const cv::Mat& image)
{
    // Sub ROI for eliminate the border part of battery
    std::pair<int, int> resultPair = std::pair<int, int>(-1, -1);
    int subRoiX = JR_ROIX;
    int imgHeight = image.rows;
    int imgWidth = image.cols;
    cv::Rect roi1 = cv::Rect(0, 0, imgWidth, imgHeight);
    RefineROI(roi1, image.size());
    cv::Mat imageroi1 = image(roi1).clone();

    if (!imageroi1.empty())
    {
        //int tmpXOffset = borderPadding + subRoiX;
        cv::Rect ROI(roi1.x + subRoiX, roi1.y, roi1.width - subRoiX * 2, roi1.height);
        if (true == RefineROI(ROI, image.size()))
        {
            cv::Mat grayImage = image(ROI).clone();

            VecInt thresholds = ThresholdOtsuMulti(grayImage);
            cv::Mat binaryImage = cv::Mat();
            cv::threshold(grayImage, binaryImage, thresholds[1], 255, cv::THRESH_BINARY);
            int morph_size = 2;
            cv::Mat element = cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(2 * morph_size + 1,
                    2 * morph_size + 1),
                cv::Point(morph_size,
                    morph_size));
            cv::morphologyEx(binaryImage, binaryImage, cv::MORPH_OPEN, element);
            std::vector<VecPoint> contours{};
            std::vector<cv::Vec4i> hierarchy{};
            findContours(binaryImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

            if (contours.size() > 0)
            {
                int largestContourIndex = getMaxAreaContourId(contours, 5);
                if (largestContourIndex == -1) return std::pair<int, int>(-1, -1);

                cv::Moments shapeMoments = cv::moments(contours[largestContourIndex]);

                cv::Point center(shapeMoments.m10 / shapeMoments.m00, shapeMoments.m01 / shapeMoments.m00);

                // Binary thresholding automatically based on histogram    
                int jrTop = 0;
                int jrBot = 0;
                int windows = 20;
                int centerIdx = 0;
                int centerIdy = 0;

                cv::Rect boundary = cv::boundingRect(contours[largestContourIndex]);
                if (center.x >= windows && (center.x + windows) < binaryImage.cols) {
                    centerIdx = center.x;
                    centerIdy = center.y;
                }
                else {
                    if ((boundary.x + boundary.width / 2) > windows && (boundary.x + boundary.width / 2 + windows) < binaryImage.cols)
                        centerIdx = boundary.x + boundary.width / 2;
                    centerIdy = boundary.y + boundary.height / 2;
                }
                if (!(centerIdx < windows || centerIdx + windows >= binaryImage.cols))
                {
                    cv::Rect rect = cv::Rect(centerIdx - windows, boundary.y, windows * 2, boundary.height);
                    if (true == RefineROI(rect, binaryImage.size()))
                    {
                        cv::Mat imageRoi = binaryImage(rect).clone();
                        cv::Mat imageReduced = cv::Mat();
                        imageRoi /= 255;
                        cv::reduce(imageRoi, imageReduced, 1, cv::REDUCE_SUM, CV_32FC1);
                        VecFloat reduceVec = cv::Mat_<float>(imageReduced);
                        int reduceVecSize = reduceVec.size();
                        for (int i = std::abs(centerIdy - boundary.y); i >= 0; i--)
                        {
                            if (reduceVec[i] < windows) { jrTop = i + 1; break; }
                        }

                        if (jrTop > 0)
                            for (int i = jrTop; i < reduceVecSize; i++)
                            {
                                if (reduceVec[i] == 0) { jrBot = i; break; }
                            }
                        if (jrBot == 0) {
                            jrBot = reduceVec.size() - 1;
                        }
                        if (jrTop + rect.y == 0) {
                            jrTop = centerIdy / 2;
                        }
                        if (mDisplayMode == DisplayMode::DALL) {
                            cv::circle(grayImage, center, 4, cv::Scalar(0), -1, 8, 0);

                            for (int i = 0; i < contours.size(); i++)
                            {
                                if (contours[i].size() > 10) {
                                    drawContours(grayImage, contours, i, cv::Scalar(255, 0, 0), 3);
                                }
                            }

                            cv::cvtColor(grayImage, grayImage, cv::COLOR_GRAY2BGR);

                            std::string msg = "JR Top";
                            cv::Size txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
                            cv::Point pJRTop(centerIdx, jrTop + rect.y);
                            cv::circle(grayImage, pJRTop, 2, COLOR_CV_RED, -1);
                            cv::putText(grayImage, msg, cv::Point(pJRTop.x - txtSize.width / 2, pJRTop.y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0, 0, 255 });
                            cv::Point pJRBottom(centerIdx, jrBot + rect.y);
                            msg = "JR Bottom";
                            txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
                            cv::circle(grayImage, pJRBottom, 2, COLOR_CV_RED, -1);
                            cv::putText(grayImage, msg, cv::Point(pJRBottom.x - txtSize.width / 2, pJRBottom.y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0, 0, 255 });
                            namedWindow("Finding JR_binaryImage", cv::WINDOW_NORMAL);
                            show_window("Finding JR_binaryImage", grayImage);
                            //cv::waitKey();
                        }

                        resultPair = std::make_pair(jrTop + rect.y + ROI.y, jrBot + rect.y + ROI.y);
                        imageRoi.release();
                        imageReduced.release();
                    }
                    else
                    {
                        //Failed Refinement rect
                    }
                }
                else
                {
                    //centerIdx out of range
                }
            }
            else
            {
                //image ROI (cut x) contourSize ==0
            }
            grayImage.release();
            binaryImage.release();
        }
    }
    else
    {
        // imageroi1 is empty
    }

    imageroi1.release();

    return resultPair;
}

cv::Rect CylinderBatteryUpper::FindBatteryROI(const cv::Mat& image)
{
    cv::Rect batteryROI = {};

    cv::Rect ROI(mRoi);
    if (!image.empty() && RefineROI(ROI, image.size()))
    {
        //===================Find contour of the battery======================//
        cv::Mat grayImg = cv::Mat();
        if (mEnableAutoMode || mThreshold == 0) {
            cv::threshold(image(ROI), grayImg, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);
        }
        else {
            cv::threshold(image(ROI), grayImg, mThreshold, 255, cv::THRESH_BINARY_INV);
        }
        cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
        cv::morphologyEx(grayImg, grayImg, cv::MORPH_OPEN, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);

        auto contour = FindMaxContour(grayImg, CompareArea, cv::CHAIN_APPROX_NONE);
        if (contour.size() > 10)
        {
            cv::Rect boundRect = cv::boundingRect(contour);

            batteryROI = cv::Rect(ROI.tl() + boundRect.tl(), boundRect.size());

            if (mDisplayMode == DisplayMode::DALL)
            {
                cv::Mat drawingContour;
                cv::cvtColor(image(ROI), drawingContour, cv::COLOR_GRAY2BGR);
                cv::polylines(drawingContour, contour, true, cv::Scalar(255, 0, 0), 4);

                cv::rectangle(drawingContour, boundRect, cv::Scalar(0, 0, 255), 4);

                namedWindow("Binary ROI1 battery", cv::WINDOW_NORMAL);
                show_window("Binary ROI1 battery", grayImg);

                namedWindow("ROI1 battery contour", cv::WINDOW_NORMAL);
                show_window("ROI1 battery contour", drawingContour);
            }
        }
        else
        {
            // the found contour is small
        }

        grayImg.release();
    }
    else
    {
        // Image is empty or roi is empty
    }
    
    return batteryROI;
}

ERR_CODE CylinderBatteryUpper::FindAllPolesPos(const cv::Mat& Img, cv::Rect& poleRegionROI, VecInt& lstPolePosXAll, VecInt& anodePos, VecInt& cathodePos, std::string& descript, bool isExtraROI)
{
    //Extra battery roi for fix black boundary pole when CLAHE image
    int borderWidth = isExtraROI ? round(poleRegionROI.width * 0.02) : 0;
    cv::Rect batteryROIExtra = cv::Rect(poleRegionROI.x - borderWidth, poleRegionROI.y, poleRegionROI.width + borderWidth * 2, poleRegionROI.height);
    RefineROI(batteryROIExtra, Img.size());
    borderWidth = poleRegionROI.x - batteryROIExtra.x;
    cv::Mat src = Img(batteryROIExtra).clone();

    if (src.empty())
    {
        descript = "Cannot find suitable cathode line, please check Cathode line parameter in setting";
        return ERR_CODE::errPoleCathodeLine2;
    }

    if (lstPolePosXAll.size() > 0) lstPolePosXAll.clear();
    if (anodePos.size() > 0) anodePos.clear();
    if (cathodePos.size() > 0) cathodePos.clear();

    // 5: Applied Local Thresholding for determine the ending Point of Anode Poles
    cv::Mat imgROI4_0, imgCLAHE;
    cv::bilateralFilter(src, imgROI4_0, 5, 55, 5);

    cv::Ptr<cv::CLAHE> claheTool4Cathode = cv::createCLAHE(15, cv::Size(75, imgROI4_0.rows));
    claheTool4Cathode->apply(imgROI4_0, imgCLAHE);
    // Use poleRegionROI as caculation
    cv::Rect batteryROI_0 = cv::Rect(borderWidth, 0, poleRegionROI.width, poleRegionROI.height);

    imgROI4_0 = imgROI4_0(batteryROI_0).clone();
    imgCLAHE = imgCLAHE(batteryROI_0).clone();
    src = src(batteryROI_0).clone();

    cv::Size imgSize = imgROI4_0.size();
    VecPoint vecAllCath;

    ERR_CODE errFindCathodeLine = FindCathodeLine(src, vecAllCath, descript);
    if (errFindCathodeLine != ERR_CODE::OK)
    {
        return errFindCathodeLine;
    }

    // 6: From Anode ending lines, select a sub-ROI 4.0 (mask) for detecting poles position
    cv::Mat tmpDst = src.clone();
    cv::Mat mask = cv::Mat::zeros(tmpDst.size(), tmpDst.type());

    VecPoint outerPoints;
    outerPoints.push_back(cv::Point(0, 0));
    outerPoints.push_back(cv::Point(imgSize.width - 1, 0));

    int poleHeight = (mValidCathode2CaseRange.GetLower() / mPixelSize + mValidCathode2CaseRange.GetUpper() / mPixelSize) / 2;
    poleHeight = std::max(30, std::min(poleHeight, 60));

    VecPoint vtRoi4ContourAll = refineMaskingContour(vecAllCath, imgSize.width, 0, poleHeight, outerPoints);

    if (vtRoi4ContourAll.size() < imgSize.width) {
        descript = "Generated sample of cathode line is not correct, please check Cathode line parameter in setting";
        return ERR_CODE::errPoleCathodeLine6;
    }
    drawContours(mask, std::vector<VecPoint>(1, vtRoi4ContourAll), -1, cv::Scalar::all(255), -1);
    // 7: Implement the CLAHE with rectangular size (35x image.height)
    tmpDst = cv::Mat::zeros(imgSize, CV_8UC1);
    imgCLAHE.copyTo(tmpDst, mask);
    if (mDisplayMode == DisplayMode::DALL)
    {
        show_window("masked img", tmpDst);
        show_window("Binarized Anode ending line", imgROI4_0);
    }

    // 8: Select the sub-ROI 3.1 for pole analysis
    // 9: Find the cumulative function vertically 
    // 10: The position of each pole is defined as the lowest extreme point in each period (dark poles)
    //std::pair<VecInt, VecDouble>polePositions = findPolePosition(claheImage);
    ERR_CODE errFindXCathode = FindCathodeXPosition(tmpDst, lstPolePosXAll);
    if (errFindXCathode != ERR_CODE::OK)
    {
        descript = "Cannot detect enough pole, please check Poles Height or Pole Detection Threshold parameter in setting or wrong input image";
        return errFindXCathode;
    }

    tmpDst.release();

    ////=============== Find Cathode Y position =================
    VecPoint CathodePnts;
    std::for_each(lstPolePosXAll.begin(), lstPolePosXAll.end(), [&vecAllCath, &CathodePnts, &cathodePos](const float& pnt) {CathodePnts.push_back(vecAllCath[pnt]); cathodePos.push_back(vecAllCath[pnt].y); });
    
    if(mInspectingItems.ANODE_TO_CATHODE_LENGTH || mInspectingItems.ANODE_TO_CASE_GAP)
    {
        ////=============== Find Anode Y position =================
        cv::Mat anodeDebugImage;
        /*if (mDisplayMode == DisplayMode::DALL) { */cv::cvtColor(imgCLAHE, anodeDebugImage, cv::COLOR_GRAY2BGR); /*}*/
        cv::Mat erasePoleImage = anodeDebugImage.clone();

        std::vector<listPole> listAllPoles;
        ERR_CODE errFindAnode = FindAnode(imgCLAHE, anodePos, CathodePnts, listAllPoles, anodeDebugImage);
        refineAnodePos(listAllPoles, anodePos, cathodePos, lstPolePosXAll, imgCLAHE, anodeDebugImage);

        if (errFindAnode != ERR_CODE::OK)
        {
            descript = "Cannot detect anode y position, please check Anode Thresold Inner, Midle, Outer parameter in setting or wrong input image";
            return errFindAnode;
        }

        if (mDisplayMode == DisplayMode::DALL)
        {
            cv::cvtColor(imgCLAHE, imgCLAHE, cv::COLOR_GRAY2BGR);
            show_window("Roi4_Result", imgCLAHE);
            show_window("Find Anode Y Cord Upper", anodeDebugImage);
        }

        VecInt listindexRemove;
        VecInt lstPolePosXAllBeforeRemove = lstPolePosXAll;
        if (mEnableAutoMode || mSkipPolesDistance == 0) {
            std::pair<int, int> result = findUnstableRegion(lstPolePosXAllBeforeRemove, lstPolePosXAll, anodePos, cathodePos, listindexRemove, imgSize.width, mNumberRemovePoleAuto, erasePoleImage);
            poleRegionROI.x += result.first;
            if (result.second == 0) {
                result.second = poleRegionROI.width;
            }
            poleRegionROI.width = result.second - result.first;
            //startingPoleRegion = startingPoleRegion + poleRegionROI.x;
            for (int i = 0; i < lstPolePosXAll.size(); i++) lstPolePosXAll[i] -= result.first;
        }

        anodeDebugImage.release();

#if USE_POLE_REFINEMENT
        // 13: Refine incorrect measurement by using symmetry property
        ERR_CODE resRefinement = PoleLengthRefinement(lstPolePosXAll, anodePos, cathodePos, imgSize.width, descript, anodeDebugImage(cv::Rect(0, 0, anodeDebugImage.cols / 2, anodeDebugImage.rows)));
        if (resRefinement != ERR_CODE::OK) {
            return resRefinement;
        }
#endif // USE_POLE_LENGTH_REFINE
    }
    else
    {
        anodePos.clear();
        anodePos.insert(anodePos.end(),cathodePos.size(), 0);
    }

    return ERR_CODE::OK;
}

//Waiting for implementation
int CylinderBatteryUpper::FindAnodeByKmean(const cv::Mat& inputImg, cv::Point startPoint, int defaultVal, int stepHorizontal, int stepVertical, cv::Mat& drawOuput) {
    int outputYAnode = defaultVal;
    bool isValidStartPoint = (startPoint.x > 0 && startPoint.x < inputImg.cols) && (startPoint.y > 0 && startPoint.y < inputImg.rows);
    if (!inputImg.empty() && isValidStartPoint && (stepHorizontal > 0 && stepVertical > 0)) {
        cv::Mat sumAverageImage = cv::Mat::zeros(inputImg.size(), 0);
        cv::Mat sumAverageImageColr;
        cv::Mat inputImgColr;
        // Create Sum average Matrix
        for (int row = inputImg.rows - 1; row >= 0; row--) {
            for (int col = 0; col < inputImg.cols; col++) {
                int colAverage = inputImg.at<uchar>(row, col);
                if (row >= stepVertical) {
                    for (int i = 1; i < stepVertical; i++) {
                        colAverage = colAverage + inputImg.at<uchar>(row - i, col);
                    }
                }
                else {
                    colAverage = colAverage * stepVertical;
                }
                colAverage = colAverage / stepVertical;
                sumAverageImage.at<uchar>(row, col) = colAverage;
            }
        }
        VecPoint listMinimumPoint;
        VecPoint listMinimumIntensity;

        listMinimumPoint.push_back(startPoint);
        listMinimumIntensity.push_back(cv::Point(sumAverageImage.at<uchar>(startPoint), sumAverageImage.at<uchar>(startPoint)));
        cv::Point startingForLoop = cv::Point(startPoint.x, startPoint.y - 1);
        //Find all minimum points from startpoint which window 1x7
        cv::Point localMinimumPoint = startingForLoop;
        int minimumPointIntensity = sumAverageImage.at<uchar>(startingForLoop);
        for (int i = startingForLoop.y; i >= 0; i--) {
            bool flagUpdated = false;
            for (int j = -stepHorizontal; j < stepHorizontal; j++) {
                if (localMinimumPoint.x + j < 0) {
                    localMinimumPoint.x = -j;
                }
                else if (localMinimumPoint.x + j >= inputImg.cols) {
                    localMinimumPoint.x = -j + inputImg.cols;
                }
                else {
                    if (sumAverageImage.at<uchar>(i, localMinimumPoint.x + j) < minimumPointIntensity) {
                        minimumPointIntensity = sumAverageImage.at<uchar>(i, localMinimumPoint.x + j);
                        localMinimumPoint = cv::Point(localMinimumPoint.x + j, i);
                        flagUpdated = true;
                    }
                }
            }
            if (false == flagUpdated) {
                minimumPointIntensity = sumAverageImage.at<uchar>(i, localMinimumPoint.x);
                localMinimumPoint = cv::Point(localMinimumPoint.x, i);
            }
            cv::circle(drawOuput, localMinimumPoint, 1, cv::Scalar(0, 0, 255));
            cv::Point intensityPoint = cv::Point(sumAverageImage.at<uchar>(localMinimumPoint), sumAverageImage.at<uchar>(localMinimumPoint));
            listMinimumPoint.push_back(localMinimumPoint);
            listMinimumIntensity.push_back(intensityPoint);
        }
        int pos = -1;
        if (listMinimumIntensity.size() > 1 && listMinimumPoint.size() > 1) {
            std::reverse(listMinimumIntensity.begin(), listMinimumIntensity.end());
            std::reverse(listMinimumPoint.begin(), listMinimumPoint.end());

            // Run Kmean  algorithm
            cv::Mat labellist;
            cv::Mat centers;
            cv::TermCriteria tc;
            cv::kmeans(listMinimumIntensity, 2, labellist, tc, cv::TermCriteria::MAX_ITER, cv::KMEANS_RANDOM_CENTERS, centers);
            int sumLabel0 = 0;
            int sumLabel1 = 0;
            int countLabel0 = 0;
            int countLabel1 = 0;
            for (int i = 0; i < labellist.rows; i++) {
                if (0 == labellist.at<int>(i, 0)) {
                    sumLabel0 = sumLabel0 + listMinimumIntensity[i].x;
                    countLabel0 = countLabel0 + 1;
                }
                else {
                    sumLabel1 = sumLabel1 + listMinimumIntensity[i].x;
                    countLabel1 = countLabel1 + 1;
                }
            }
            int averageLabel0 = sumLabel0 / countLabel0;
            int averageLabel1 = sumLabel1 / countLabel1;
            if (averageLabel0 > averageLabel1) {
                for (int i = 0; i < labellist.rows; i++) {
                    if (1 == labellist.at<int>(i, 0)) {
                        labellist.at<int>(i, 0) = 0;
                    }
                    else {
                        labellist.at<int>(i, 0) = 1;
                    }
                }
            }
            for (int i = 1; i < labellist.rows - 1; i++) {
                if (i > 1 && i < labellist.rows - 2) {
                    if ((1 == labellist.at<int>(i, 0) - labellist.at<int>(i - 1, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i - 2, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i + 1, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i + 2, 0))
                        ) {
                        labellist.at<int>(i, 0) = labellist.at<int>(i - 1, 0);
                    }
                }
                else if (1 == i)
                {
                    if ((1 == labellist.at<int>(i, 0) - labellist.at<int>(i - 1, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i + 1, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i + 2, 0))
                        ) {
                        labellist.at<int>(i, 0) = labellist.at<int>(i - 1, 0);
                    }
                }
                else {
                    if ((1 == labellist.at<int>(i, 0) - labellist.at<int>(i - 1, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i - 2, 0))
                        && (1 == labellist.at<int>(i, 0) - labellist.at<int>(i + 1, 0))
                        ) {
                        labellist.at<int>(i, 0) = labellist.at<int>(i - 1, 0);
                    }
                }
            }

            for (int i = labellist.rows - 2; i > 1; i--) {
                if (1 == std::abs(labellist.at<int>(i, 0) - labellist.at<int>(i - 1, 0))
                    && 1 == std::abs(labellist.at<int>(i + 1, 0) - labellist.at<int>(i - 1, 0))
                    && 1 == std::abs(labellist.at<int>(i, 0) - labellist.at<int>(i - 2, 0))) {
                    pos = i;
                    break;

                }
            }
            cv::Mat labelgrayImage;
            labellist.convertTo(labelgrayImage, CV_8U);
            labelgrayImage = labelgrayImage * 255;
            cv::cvtColor(labelgrayImage, labelgrayImage, cv::COLOR_GRAY2BGR);
            cv::Rect roi = cv::Rect(startPoint.x, startPoint.y - listMinimumIntensity.size() + 1, labelgrayImage.cols, labelgrayImage.rows);
            labelgrayImage.copyTo(drawOuput(roi));
        }
        else {
            return outputYAnode;
        }
        if (-1 != pos) {
            outputYAnode = listMinimumPoint[pos].y;
            cv::circle(inputImgColr, listMinimumPoint[pos], 3, cv::Scalar(255, 0, 0));
            if (!drawOuput.empty()) {
                cv::circle(drawOuput, listMinimumPoint[pos], 3, cv::Scalar(255, 0, 0));
            }
        }
    }
    else {
        return outputYAnode;
    }
    return outputYAnode;
}

ERR_CODE CylinderBatteryUpper::FindAnode(const cv::Mat& inputImg, VecInt& vtAnodePos, VecPoint const& vtStartPoints, std::vector<listPole>& listAllPoles, cv::Mat& drawOuput) {
    if (inputImg.empty() || vtStartPoints.empty())
        return ERR_CODE::errPoleAnodeLine;

    vtAnodePos.clear();
    vtAnodePos.reserve(vtStartPoints.size());
    int anodeYPos;
    int stepHorizontal = 3;
    int stepVertical = 5;
    double aThreshold = 0.0;
    int moveAlow = 3;
    anodeAlgoType algoType = mEnableAutoMode ? anodeAlgoType::Edge : anodeAlgoType::LineTracing;
    if (mAnodeThresholdInner == 0 && mAnodeThresholdMiddle == 0 && mAnodeThresholdOuter == 0) {
        algoType = anodeAlgoType::Edge;
    }

    for (int poleIdx = 0; poleIdx < vtStartPoints.size(); poleIdx++)
    {
        std::vector<pointValue> listMinimumIntensity;
        listPole t{};
        cv::Point ending;
        switch (algoType)
        {
        case anodeAlgoType::LineTracing:
        {
            int regionIdx = CheckRegion(vtStartPoints[poleIdx].x, 6, inputImg.cols, mCenterNeglectionWidth);
            switch (regionIdx)
            {
            case 1:
                aThreshold = mAnodeThresholdOuter;
                break;
            case 2:
                aThreshold = mAnodeThresholdMiddle;
                break;
            case 3:
                aThreshold = mAnodeThresholdInner;
                break;
            default:
                aThreshold = mAnodeThresholdMiddle;
            }

            // 12: For each pole position, select the sub-Roi area (ROI_P1)
            // containing only Anode this pole (based on poles position in step and Anode end point in step)
            //BORDER CHECK
            int isBorder = 0;
            if (poleIdx == 0 /*|| poleIdx == 1 || poleIdx == 2*/)
            {
                isBorder = -1;
            }
            else if (poleIdx == vtStartPoints.size() - 1 /*|| poleIdx == lstPoleSize-2 || poleIdx == lstPoleSize -3*/)
            {
                isBorder = 1;
            }

            bool isRestrictMove = (regionIdx == 1);

            anodeYPos = findAnodeByLineTracing(inputImg, vtStartPoints[poleIdx], vtStartPoints[poleIdx].y, stepHorizontal, stepVertical, aThreshold * 10, drawOuput,
                isBorder, mPolesDistanceRange.GetLower(), isRestrictMove, moveAlow);
        }
        break;
        case anodeAlgoType::Edge:
            anodeYPos = findAnodeAutoByEdge(inputImg, vtStartPoints[poleIdx], vtStartPoints[poleIdx].y, stepHorizontal, stepVertical, drawOuput, listMinimumIntensity, ending);
            t = { listMinimumIntensity, vtStartPoints[poleIdx], ending };
            listAllPoles.push_back(t);
            break;
        case anodeAlgoType::Kmean:
        default:
            anodeYPos = vtStartPoints[poleIdx].y;
            break;
        }

        anodeYPos = std::max(0, std::min(anodeYPos, vtStartPoints[poleIdx].y));
        vtAnodePos.push_back(anodeYPos);
    }
    return ERR_CODE::OK;
}

ERR_CODE CylinderBatteryUpper::Inspection2(const cv::Mat& inImg, BatteryInspectionResult& BIresult)
{
    //-----end of pre-processing ------------------------------------------------------------------------
    cv::Mat src;
    auto res = Convert8Bits(inImg, src);
    if (res)
    {
        BIresult.Description = "Image empty or not support!";
        return res == 1 ? ERR_CODE::errImageEmpty : ERR_CODE::errImageFormat;
    }

    mGamma.Apply(src, src);

    if (mPixelSize == 0) {
        BIresult.Description = "Pixel size cannot set to 0";
        return ERR_CODE::errPixelSize;
    }

    // check predefined setting 
    if (false == RefineROI(mRoi, cv::Size(src.cols, src.rows)))
    {
        BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errROIRefinement;
    }

    const cv::Rect settingROI(mRoi);
    std::string FinalDecision;

    std::vector<std::string> warnMsg;

    cv::Rect batteryROI = FindBatteryROI(src);

    if (batteryROI.width > 0 && batteryROI.height > 0)
    {
        //Check the width of battery is in the range of masterJigCellSize
        if (!mValidCellWidthRange.IsInRange(round_f(batteryROI.width * mPixelSize, 2)))
        {
            BIresult.Description = "Error: Width of battery is out of range 2 : " + std::to_string(batteryROI.width * mPixelSize);
            return ERR_CODE::errWidthBattery;
        }
    }
    else
    {
        BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }
    //======================================Beading Inspection====================================//
    /*vtInspectingItem[0] = false;
    vtInspectingItem[1] = false;
    vtInspectingItem[5] = false;
    vtInspectingItem[6] = false;*/
    // beadingInsResult[0]: Thickness of top cover
    // beadingInsResult[1]: The inner diameter of the plant
    // beadingInsResult[2]: Maximum outer diameter
    // beadingInsResult[3]: The depth of the groove
    // beadingInsResult[4]: Distance between upper and lower edges
    if (batteryROI.height < mBeadingHeightMin) {
        BIresult.Description = "Cannot inspect the Beading, please check again Min Beading Height, Outer Roi or input image";
        return ERR_CODE::errBeadingArea;
    }
    cv::cvtColor(src, BIresult.resImg, cv::COLOR_GRAY2BGR);
    int yA2;
    VecDouble beadingInsResult = { -1,-1,-1,-1,-1 };
    if (mInspectingItems.CHECK_BEADING)
    {
        //inspect beading and get case reference line (yA2)
        yA2 = InspectBeading(src, BIresult.resImg, batteryROI, beadingInsResult);

        //The ROI from setting should contain enough pole information (at least 200 rows from yA2);
        if (yA2 - batteryROI.y < mBeadingHeightMin) {
            BIresult.Description = "Cannot inspect the Beading, please check again Min Beading Height, Outer Roi or input image";
            return ERR_CODE::errBeadingInsp;
        }
    }
    else
    {
        yA2 = batteryROI.y;
    }
    
    if (yA2 >= (batteryROI.y + batteryROI.height)) {
        BIresult.Description = "The pole region height is out of Image ROI, please check Pole Region Height parameter in setting";
        return ERR_CODE::errReferenceCase;
    }

    // 1: From battery?s boundary found in section 1- step 4 and Point A2 found in section 2 ? step 4,
    // select batteryROI_4 as the left image.
    //cv::Rect batteryROI_4(batteryROI.x, yA2, batteryROI.width, mPoleRegionHeight);
    //RoiRefinement(batteryROI_4, Img.size());

    //2022-05-30 Nhan change checkpole learning before check JRROI to get correct PoleRegion avoid detect pole at boundary fail
    bool poleRegionAuto = false;
    /*if (mEnableAutoMode || !mInspectingItems.FIND_JR_ROI) {
        JR_ROIX = 0;
    }

    if (mEnableAutoMode || JR_ROIY == 0 || !mInspectingItems.FIND_JR_ROI) {
        JR_ROIY = 15;
    }*/
    if (mEnableAutoMode || mPoleRegionHeight == 0) {
        if ((batteryROI.height + batteryROI.y) > yA2) {
            mPoleRegionHeight = ((batteryROI.height + batteryROI.y) - yA2) * 0.75;
            poleRegionAuto = true;
        }
    }

    // 4: Find the shift of inner Poles (Pole leaning)
    cv::Rect poleLeaningROI = batteryROI;
    poleLeaningROI.y = yA2 + JR_ROIY;
    poleLeaningROI.height = mPoleRegionHeight;
    double LeftBorder = 0;
    double RightBorder = poleLeaningROI.width;
    if (mInspectingItems.CHECK_LEANING)
    {
#ifdef DEBUG_SHOW_ROI
        cv::rectangle(res, batteryROI, cv::Scalar(0, 255, 255));
        cv::rectangle(res, poleLeaningROI, cv::Scalar(0, 0, 255));
        cv::putText(res, "PL", cv::Point(poleLeaningROI.x + poleLeaningROI.width + 10, poleLeaningROI.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
#endif
        if (false == RefineROI(poleLeaningROI, src.size()))
        {
            BIresult.Description = "PoleLeaning ROI Refinement failed";
            return ERR_CODE::errROIRefinement;
        }
        
        int leaningThreshold = mEnableAutoMode ? 0 : mPoleLeaningThreshold;

        if (poleLeaningROI.empty() || !CheckPoleLeaning(src(poleLeaningROI), mPoleRegionHeight, leaningThreshold, LeftBorder, RightBorder))
        {
            warnMsg.push_back("! Detection Pole Leaning is disable");
        }
    }

    //Roi for get clahe in full width image, otherwise, the most outer pole will be affected by the bounder.
    cv::Rect poleRegionROI;
    if (mInspectingItems.FIND_JR_ROI)
    {
        // JRROI
        int maxLeaningDistance = std::max(LeftBorder, poleLeaningROI.width - RightBorder);
        cv::Rect JRROI = cv::Rect();
        JRROI.x = maxLeaningDistance + poleLeaningROI.x;
        JRROI.y = poleLeaningROI.y;
        JRROI.width = poleLeaningROI.width - (maxLeaningDistance * 2);
        JRROI.height = mPoleRegionHeight;

        if (false == RefineROI(JRROI, src.size()))
        {
            BIresult.Description = "JRROI Refinement failed";
            return ERR_CODE::errROIRefinement;
        }

        // 2: Finding the Jelly Roll (J/R) area
        std::pair<int, int> jrPos = FindTopReferenceLine(src(JRROI));
        if (jrPos.first < 0)
        {
            BIresult.Description = "Cannot identify the top blank area of pole in battery (for reference line), please check again JR Setting";
            return ERR_CODE::errJRFinding;
        }

        poleRegionROI.y = JRROI.y + jrPos.first;
        poleRegionROI.height = mPoleRegionHeight;
        int poleHeight = mValidCathode2AnodeRange.GetUpper() / mPixelSize;
        poleHeight = std::max(30, std::min(poleHeight, 80));
        if ((mEnableAutoMode || poleRegionAuto) && jrPos.second > 0 && jrPos.second - jrPos.first > 10) {
            if (jrPos.second + JRROI.y > poleRegionROI.y) {
                int region = ((jrPos.second + JRROI.y - poleRegionROI.y) + poleHeight) * 2;
                int region2 = batteryROI.y + batteryROI.height - poleRegionROI.y;
                region = std::min(region, region2);
                poleRegionROI.height = region;
                mPoleRegionHeight = poleRegionROI.height;
            }

        }
        
    }
    else
    {
        poleRegionROI = poleLeaningROI;
    }

    poleRegionROI.x = poleLeaningROI.x + LeftBorder + JR_ROIX;
    poleRegionROI.width = RightBorder - LeftBorder - 2 * JR_ROIX;

    if (false == RefineROI(poleRegionROI, src.size()))
    {
        BIresult.Description = "PoleRegion ROI Refinement failed";
        return ERR_CODE::errROIRefinement;
    }

    if (mInspectingItems.CHECK_LEANING)
    {
        //Draw pole leaning position
        cv::line(BIresult.resImg, cv::Point(poleLeaningROI.x + LeftBorder, poleRegionROI.y), cv::Point(poleLeaningROI.x + LeftBorder, poleRegionROI.br().y), cv::Scalar(255, 0, 255));
        cv::line(BIresult.resImg, cv::Point(poleLeaningROI.x + RightBorder, poleRegionROI.y), cv::Point(poleLeaningROI.x + RightBorder, poleRegionROI.br().y), cv::Scalar(255, 0, 255));
    }

#ifdef DEBUG_SHOW_ROI

    cv::putText(res, "JR", cv::Point(JRROI.x + JRROI.width + 10, JRROI.y + JRROI.height / 3), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);
    //Extra battery roi for fix black boundary pole when CLAHE image
    int borderWidth = round(poleRegionROI.width * 0.02);
    cv::Rect batteryROIExtra = cv::Rect(poleRegionROI.x - borderWidth, poleRegionROI.y, poleRegionROI.width + borderWidth * 2, poleRegionROI.height);
    cv::rectangle(res, batteryROIExtra, cv::Scalar(0, 255, 0));
    cv::putText(res, "CL", cv::Point(batteryROIExtra.x + batteryROIExtra.width + 10, batteryROIExtra.y + batteryROIExtra.height / 1.5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
#endif

    // find all poles position
    VecInt lstPolePosXAll, anodePos, cathodePos;
    std::string descript = "";
    float maxA = -1, maxB = -1;
    float minA = -1, minB = -1;
    float averageA = 0.0f, averageB = 0.0f;
    std::vector<bool> listAno2CathodDecision, listAno2CaseDecision, listCathode2CaseDecision;
    //pixel distance
    VecDouble listAnode2CathodeDistance;
    //pixel distance
    VecDouble listAnode2CaseDistance;

    VecDouble listCathode2CaseDistance;
    //pixel distance to the battery ROI
    VecDouble listPolePos;
    VecDouble sCathode2CaseMeas;
    bool isAnode2CaseOK = true;
    bool isCathode2CaseOK = true;
    bool isAnode2CathodeOK = true;
    // poleLeftIdx is number of pole in left side
    //Check disappear poles
    std::vector<std::pair<int, int>> lstPolePositionErr;
    int nPoleLeft = 0;
    int nPoleRight = 0;
    //For calculate the anode to case distance
    const int caseLine = yA2 + mCaseLineOffset;

    if (mInspectingItems.ANODE_TO_CATHODE_LENGTH
        || mInspectingItems.CATHODE_TO_CASE_GAP
        || mInspectingItems.ANODE_TO_CASE_GAP)
    {   
        ERR_CODE err = FindAllPolesPos(src, poleRegionROI, lstPolePosXAll, anodePos, cathodePos, descript);
        // check error when find cathode position
        if (err != ERR_CODE::OK && !descript.empty())
        {
            BIresult.Description = descript;
            return err;
        }
        else
        {
            warnMsg.push_back(descript);
        }

        //===============================================making Decisions==================================================//
        
        VecInt leftXList;
        VecInt leftAnodeList;
        VecInt leftCathodeList;
        VecInt rightXList;
        VecInt rightAnodeList;
        VecInt rightCathodeList;
        int borderLeft = mEnableAutoMode ? 0 : mSkipPolesDistance;
        int borderRight = poleRegionROI.width - borderLeft;
        int centerLeft = (poleRegionROI.width - mCenterNeglectionWidth) / 2;
        int centerRight = (poleRegionROI.width + mCenterNeglectionWidth) / 2;
        for (auto pole = 0; pole < lstPolePosXAll.size(); pole++) {
            if (lstPolePosXAll[pole] >= borderLeft
                && lstPolePosXAll[pole] <= centerLeft) {
                nPoleLeft++;
                leftXList.push_back(lstPolePosXAll[pole]);
                leftAnodeList.push_back(anodePos[pole]);
                leftCathodeList.push_back(cathodePos[pole]);
                if (mIsCheckPoleNo && nPoleLeft >= mOneSidePoleNumber) {
                    break;
                }
            }
        }

        int poleRightIdx = lstPolePosXAll.size() - 1;
        for (auto pole = poleRightIdx; pole >= 0; pole--) {
            if (lstPolePosXAll[pole] >= centerRight
                && lstPolePosXAll[pole] <= borderRight) {
                nPoleRight++;
                rightXList.push_back(lstPolePosXAll[pole]);
                rightAnodeList.push_back(anodePos[pole]);
                rightCathodeList.push_back(cathodePos[pole]);
                if (mIsCheckPoleNo && (nPoleRight) >= mOneSidePoleNumber)
                    break;
            }
        }

        anodePos.clear();
        cathodePos.clear();
        lstPolePosXAll.clear();
        anodePos = leftAnodeList;
        anodePos.insert(anodePos.end(), rightAnodeList.rbegin(), rightAnodeList.rend());
        cathodePos = leftCathodeList;
        cathodePos.insert(cathodePos.end(), rightCathodeList.rbegin(), rightCathodeList.rend());
        lstPolePosXAll = leftXList;
        lstPolePosXAll.insert(lstPolePosXAll.end(), rightXList.rbegin(), rightXList.rend());

        const int detectedPoleNumb = lstPolePosXAll.size();

        // For preventing anode position out of range
        std::for_each(anodePos.begin(), anodePos.end(), [](int& ap) {ap = ap < 0 ? 0 : ap; });

        //Can xem xet
        VecDouble lstAnodeDouble;
        std::for_each(anodePos.begin(), anodePos.end(), [&lstAnodeDouble](int& ap) { lstAnodeDouble.push_back(ap); });
        cv::blur(lstAnodeDouble, lstAnodeDouble, cv::Size(5, 1));
        //lstAnodeDouble = weightedAvgSmoothen(lstAnodeDouble);
        anodePos.clear();
        std::for_each(lstAnodeDouble.begin(), lstAnodeDouble.end(), [&anodePos](double& ap) { anodePos.push_back(ap); });

        assert(cathodePos.size() == detectedPoleNumb && anodePos.size() == detectedPoleNumb);
        
        int decimal_places = 2;
        
        for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
        {
            //Update the original position on the Image
            auto mAnode = cv::Point(poleRegionROI.x + lstPolePosXAll[poleIdx], poleRegionROI.y + anodePos[poleIdx]);
            auto mCathode = cv::Point(poleRegionROI.x + lstPolePosXAll[poleIdx], poleRegionROI.y + cathodePos[poleIdx]);

            //Get the distance from the pole to the left battery roi
            listPolePos.push_back(abs(mAnode.x - batteryROI.x) * mPixelSize);

            //Get the anode to case result
            double mAnode2CasePixel = abs(mAnode.y - caseLine) + mAnode2CaseOffset;
            double mAnode2Case_mm = mAnode2CasePixel * mPixelSize;
            auto mAnode2CaseResult = !mInspectingItems.ANODE_TO_CASE_GAP || mValidAnode2CaseRange.IsInRange(round_f(mAnode2Case_mm, decimal_places));

            //Get the anode to cathode result
            double mAnode2CathodePixel = abs(mAnode.y - mCathode.y) + mCathode2AnodeOffset;
            double mAnode2Cathode_mm = mAnode2CathodePixel * mPixelSize;
            auto mAnode2CathodeResult = !mInspectingItems.ANODE_TO_CATHODE_LENGTH || mValidCathode2AnodeRange.IsInRange(round_f(mAnode2Cathode_mm, decimal_places));

            //Get the anode to cathode result
			double mCathode2CasePixel = abs(mCathode.y - caseLine) + mCathode2CaseOffset;
			double mCathode2Case_mm = mCathode2CasePixel * mPixelSize;
			auto mCathode2CaseResult = !mInspectingItems.CATHODE_TO_CASE_GAP || mValidCathode2CaseRange.IsInRange(round_f(mCathode2Case_mm, decimal_places));

            isAnode2CaseOK &= mAnode2CaseResult;
            isAnode2CathodeOK &= mAnode2CathodeResult;
            isCathode2CaseOK &= mCathode2CaseResult;

            listAno2CaseDecision.push_back(mAnode2CaseResult);
            listAno2CathodDecision.push_back(mAnode2CathodeResult);
            listCathode2CaseDecision.push_back(mCathode2CaseResult);

            listAnode2CaseDistance.push_back(mAnode2Case_mm);
            listAnode2CathodeDistance.push_back(mAnode2Cathode_mm);
            listCathode2CaseDistance.push_back(mCathode2Case_mm);

            BIresult.vtAnodes.push_back(mAnode);
            BIresult.vtCathodes.push_back(mCathode);
        }

        if (mInspectingItems.CHECK_ERASE_POLE) {
            int boundaryList[4] = { borderLeft, centerLeft, centerRight, borderRight };
            CheckErasedPole(leftXList, rightXList, lstPolePositionErr, mPolesDistanceRange, boundaryList, mIsCheckPoleNo);
        }
        
        for (auto& pairX : lstPolePositionErr)
        {
            pairX.first += poleRegionROI.x;
            pairX.second += poleRegionROI.x;
        }

        // Plot the skip pole region
        cv::line(BIresult.resImg, cv::Point(poleRegionROI.x + borderLeft, poleRegionROI.y), cv::Point(poleRegionROI.x + borderLeft, poleRegionROI.y + mPoleRegionHeight), CV_RGB(0xc2, 0xa6, 0x09));
        cv::line(BIresult.resImg, cv::Point(poleRegionROI.x + poleRegionROI.width - borderLeft, poleRegionROI.y), cv::Point(poleRegionROI.x + poleRegionROI.width - borderLeft, poleRegionROI.y + mPoleRegionHeight), CV_RGB(0xc2, 0xa6, 0x09));
    
        //Draw the pole result
        plotPoleWithRect(BIresult.resImg
            , poleRegionROI
            , mCenterNeglectionWidth
            , mSkipPolesDistance
            , nPoleLeft
            , mLineType
            , ((int)mDisplayMode & (int)DisplayMode::GRID)
            , BIresult.vtAnodes, BIresult.vtCathodes
            , listAno2CathodDecision, listAno2CaseDecision, listCathode2CaseDecision
            , lstPolePositionErr
        );

      auto nA = listAnode2CaseDistance.size();
        if (nA != 0) {
            maxA = *max_element(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
            minA = *min_element(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
            averageA = std::accumulate(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end(), 0.0) / nA;
        }

        auto nB = listAnode2CathodeDistance.size();
        if (nB != 0) {
            maxB = *max_element(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
            minB = *min_element(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
            averageB = std::accumulate(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end(), 0.0) / nB;
        }

        for (int pIdx = 0; pIdx < nA; pIdx++)
        {
            sCathode2CaseMeas.push_back(listAnode2CaseDistance[pIdx] + listAnode2CathodeDistance[pIdx]);
        }

        //Print reference case line
        if (mInspectingItems.CATHODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CASE_VARIATION)
        {
            cv::line(BIresult.resImg, cv::Point(poleRegionROI.x, caseLine), cv::Point(poleRegionROI.x + poleRegionROI.width, caseLine), cv::Scalar(0, 255, 0), 3);
        }
    }
    //===============================================PLoting results=================================================//
    int fontWeight = 2;

    //Draw setting ROI
    cv::rectangle(BIresult.resImg, settingROI, cv::Scalar(0, 255, 255), 3);

    //Draw leaning max distance setting
    if (mInspectingItems.CHECK_LEANING
        && mLeaningDistanceMin > 0)
    {
        cv::line(BIresult.resImg, cv::Point(batteryROI.x + mLeaningDistanceMin, poleRegionROI.y - 10), cv::Point(batteryROI.x + mLeaningDistanceMin, poleRegionROI.y + 30), CV_RGB(116, 33, 165), 2);
        cv::line(BIresult.resImg, cv::Point(batteryROI.br().x - mLeaningDistanceMin, poleRegionROI.y - 10), cv::Point(batteryROI.br().x - mLeaningDistanceMin, poleRegionROI.y + 30), CV_RGB(116, 33, 165), 2);
    }

    std::string str = " ,maxB=" + std::to_string(maxB) + " ,minB=" + std::to_string(minB) + " ,averageB=" + std::to_string(averageB);
    BIresult.sData += str.c_str();
    BIresult.yA2 = yA2;
    // Beading Inspection
    BIresult.corverHeight = beadingInsResult[0];
    BIresult.corverDiameter = beadingInsResult[1];
    BIresult.outerDiameter = beadingInsResult[2];
    BIresult.grooveDepth = beadingInsResult[3];
    BIresult.grooveHeight = beadingInsResult[4];
    BIresult.nLeftPole = nPoleLeft;

    BIresult.minCathode2Anode = minB;
    BIresult.maxCathode2Anode = maxB;
    BIresult.avgCathode2Anode = averageB;
    BIresult.minAnode2Case = minA;
    BIresult.maxAnode2Case = maxA;
    BIresult.avgAnode2Case = averageA;

    BIresult.minCathode2Case = (minA + minB);
    BIresult.maxCathode2Case = (maxA + maxB);
    BIresult.avgCathode2Case = (averageA + averageB);

    BIresult.sCathode2Anode = std::move(listAnode2CathodeDistance);
    BIresult.sAnode2Case = std::move(listAnode2CaseDistance);
    BIresult.sCathode2Case = std::move(listCathode2CaseDistance);
    BIresult.sXPos = std::move(listPolePos);
    BIresult.vtAno2CathodDecision = std::move(listAno2CathodDecision);
    BIresult.vtAno2CaseDecision = std::move(listAno2CaseDecision);
    BIresult.vtCathode2CaseDecision = std::move(listCathode2CaseDecision);
    BIresult.outerROI = batteryROI;
    BIresult.Anodes2CaseDecision = isAnode2CaseOK ? "OK" : "NG";
    BIresult.Anode2CathodeDecision = isAnode2CathodeOK ? "OK" : "NG";
    BIresult.Cathode2CaseDecision = isCathode2CaseOK ? "OK" : "NG";
    FinalDecision = (isAnode2CaseOK && isAnode2CathodeOK && isCathode2CaseOK) ? "OK" : "NG";

    std::vector<std::string> errMsg;
    if (mInspectingItems.CHECK_LEANING
        && mLeaningDistanceMin > 0
        && ((LeftBorder - batteryROI.x) > mLeaningDistanceMin || (batteryROI.br().x - RightBorder) > mLeaningDistanceMin))
    {
        FinalDecision = "NG";
        errMsg.push_back("Leaning Distance NG");
    }

    if (mInspectingItems.CHECK_ERASE_POLE && (lstPolePositionErr.size() > 0
        || (mIsCheckPoleNo && (nPoleLeft < mOneSidePoleNumber || nPoleRight < mOneSidePoleNumber)))
        && (mInspectingItems.ANODE_TO_CATHODE_LENGTH || mInspectingItems.ANODE_TO_CASE_GAP))
    {
        FinalDecision = "NG";
        errMsg.push_back("Number of detected poles is not sufficient");
    }

    if (BIresult.Anode2CathodeDecision == "NG"
        && mInspectingItems.ANODE_TO_CATHODE_LENGTH) {
        FinalDecision = "NG";
        errMsg.push_back("Anode to Cathode Distance NG");
    }

    if (BIresult.Anodes2CaseDecision == "NG"
        && mInspectingItems.ANODE_TO_CASE_GAP) {
        FinalDecision = "NG";
        errMsg.push_back("Anode to Case Distance NG");
    }

    if (BIresult.Cathode2CaseDecision == "NG"
        && mInspectingItems.CATHODE_TO_CASE_GAP) {
        FinalDecision = "NG";
        errMsg.push_back("Cathode to Case Distance NG");
    }

    if ((int)mDisplayMode & (int)DisplayMode::TEXT
        && (mInspectingItems.ANODE_TO_CATHODE_LENGTH || mInspectingItems.CATHODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CASE_GAP))
    {
        //Draw the result infor
        drawPoleTextResult(this, BIresult);
    }
    
    if ((mInspectingItems.ANODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CATHODE_LENGTH)
        && mInspectingItems.ANODE_TO_CASE_VARIATION)
    {
        int tmpPosX = BIresult.resImg.cols / 2.8;
        int tmpPosY = mTextPosition.y + 150;
        bool isNGA2Case = abs(BIresult.maxAnode2Case - BIresult.minAnode2Case) > mVariationAnode2Case;
        std::string isNGA2CaseStr = isNGA2Case ? "NG" : "OK";
        cv::putText(BIresult.resImg, "Anode2Case Variation : " + isNGA2CaseStr, cv::Point(tmpPosX, tmpPosY), cv::FONT_HERSHEY_SIMPLEX, 1, isNGA2Case ? COLOR_CV_RED : cv::Scalar(255, 255, 0), 2);
        std::ostringstream tmpStrMax;
        tmpStrMax << std::fixed << std::setprecision(3) << BIresult.maxAnode2Case;
        cv::putText(BIresult.resImg, "Max : " + tmpStrMax.str() + " mm", cv::Point(tmpPosX, tmpPosY + 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
        std::ostringstream tmpStrMin;
        tmpStrMin << std::fixed << std::setprecision(3) << BIresult.minAnode2Case;
        cv::putText(BIresult.resImg, "Min : " + tmpStrMin.str() + " mm", cv::Point(tmpPosX, tmpPosY + 30 * 2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);
        BIresult.Anode2CaseVariationDecision = isNGA2Case ? "NG" : "OK";
        if (isNGA2Case) {
            FinalDecision = "NG";
            // errMsg.push_back("Anode to Case Variation NG");
        }
    }

    if (mInspectingItems.CHECK_BEADING)
    {
        if (mInspectingItems.COVER_HEIGHT)
        {
            BIresult.BeadingCoverDiameterDecision = mValidCoverHeightRange.IsInRange(round_f(BIresult.corverHeight, 2)) ? "OK" : "NG";
        
            if (BIresult.BeadingCoverDiameterDecision == "NG")
            {
                FinalDecision = "NG";
                errMsg.push_back("Beading Cover Height NG");
            }
        }

        if (mInspectingItems.GROOVE_HEIGHT)
        {
            BIresult.BeadingGrooveHeightDecision = mValidGrooveHeightRange.IsInRange(round_f(BIresult.grooveHeight, 2)) ? "OK" : "NG";
        
            if (BIresult.BeadingGrooveHeightDecision == "NG")
            {
                FinalDecision = "NG";
                errMsg.push_back("Beading Groove Height NG");
            }
        }

        if (mInspectingItems.GROOVE_DEPTH)
        {
            BIresult.BeadingGrooveDepthDecision = mValidGrooveDepthRange.IsInRange(round_f(BIresult.grooveDepth, 2)) ? "OK" : "NG";
        
            if (BIresult.BeadingGrooveDepthDecision == "NG")
            {
                FinalDecision = "NG";
                errMsg.push_back("Beading Groove Depth NG");
            }
        }

        if (mInspectingItems.INNER_DIAMETER)
        {
            BIresult.BeadingInnerDiameterDecision = mValidInnerDiameterRange.IsInRange(round_f(BIresult.corverDiameter, 2)) ? "OK" : "NG";
        
            if (BIresult.BeadingInnerDiameterDecision == "NG")
            {
                FinalDecision = "NG";
                errMsg.push_back("Beading Inner Diameter NG");
            }
        }
    }

    for (int i = 0; i < errMsg.size(); i++) {
        cv::putText(BIresult.resImg, errMsg[i], cv::Point(BIresult.resImg.cols / 2.8, 150 + i * 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), fontWeight);
    }

#ifdef _XVT_SIMULATOR_
    /*if (mDisplayMode == 2)
    {
        int start = errMsg.size();
        for (int i = 0; i < warnMsg.size(); i++) {
            cv::putText(res, warnMsg[i], cv::Point(res.cols / 2.8, 150 + (start + i) * 30), cv::FONT_HERSHEY_SIMPLEX, 1, CV_ORANGE, fontWeight);
        }
    }*/
#endif
    cv::putText(BIresult.resImg, "Result: " + FinalDecision, cv::Point(BIresult.resImg.cols / 2.5, 100), cv::FONT_HERSHEY_SIMPLEX, 2,
        FinalDecision == "OK" ? cv::Scalar(245, 164, 66) : cv::Scalar(0, 0, 255), fontWeight);

    BIresult.finalDecision = FinalDecision;
    return ERR_CODE::OK;
}
}
}