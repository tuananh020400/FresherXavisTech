#include "xvtBattery/CylinderBatteryLower.h"
#include "xvtBattery/CylinderUtils.h"
#include "xvtCV/Polyfit.h"
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/Utils.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/Contour.h"
#include "xvtCV/Peak.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <numeric>
#include <iomanip>
constexpr double MAX_CATHODE_LINE_ERROR = 20.0;

using namespace xvt::threshold;

namespace xvt {
namespace battery {

CylinderBatteryLower::CylinderBatteryLower() : CylinderBatteryBase()
{
    mTextPosition = cv::Point(0, 200);
    mInspectingItems.SetLowerInspectDefail();
}

ERR_CODE CylinderBatteryLower::FindCathodeXPosition(const cv::Mat& src, VecInt& lstPolePosXAll) {
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
        if (mPolesProminenceThreshold == 0 || mEnableAutoMode)
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
                show_window("Find Pole by Prominence", poleImg);
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
            || (mIsCheckPoleNo && (numberOfLeftPoles < mOneSidePoleNumber / 2 || numberOfRightPoles < mOneSidePoleNumber / 2))
            )
        {
            return ERR_CODE::errPoleXDetection4;
        }
    }
    else
    {
        return ERR_CODE::errPoleXDetection;
    }
    return ERR_CODE::OK;
}

cv::Rect CylinderBatteryLower::FindBatteryROI(const cv::Mat& image) {

    cv::Rect ROI = mRoi;
    cv::Rect batteryROI = {};

    if (!image.empty() && RefineROI(ROI, image.size()))
    {
        cv::Mat grayImg = cv::Mat();
        if (mDirection == 1) {
            cv::Point topLeft(ROI.x, ROI.y);
            cv::Point bottomRight(ROI.x + ROI.width, ROI.y + ROI.height);

            cv::Point center(image.cols / 2, image.rows / 2);

            float pX = center.x - (topLeft.x - center.x);
            float pY = center.y - (topLeft.y - center.y);
            topLeft = cv::Point(pX, pY);

            pX = center.x - (bottomRight.x - center.x);
            pY = center.y - (bottomRight.y - center.y);
            bottomRight = cv::Point(pX, pY);

            ROI = cv::Rect(topLeft, bottomRight);
        }

        if (mEnableAutoMode || mThreshold == 0)
        {
            cv::threshold(image(ROI), grayImg, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);
        }
        else
        {
            cv::threshold(image(ROI), grayImg, mThreshold, 255, cv::THRESH_BINARY_INV);
        }
        /* Morphology Transformations: close
        * Useful to remove small holes*/
        cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
        cv::morphologyEx(grayImg, grayImg, cv::MORPH_OPEN, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);

        // Finding contour of battery region
        auto contour = FindMaxContour(grayImg, CompareArea, cv::CHAIN_APPROX_NONE);
        if (contour.size() > 10)
        {
            //===================Select only the battery area======================//
            /* Find boundary rectangle of the contour.
            * The boundary width is corresponding battery‘s maximum outer diameter*/
            cv::Rect boundingrect = cv::boundingRect(contour);
            int idxTopLeft = getPointOnContourId(contour, boundingrect.tl());
            int idxTopRight = getPointOnContourId(contour, cv::Point(boundingrect.br().x, boundingrect.tl().y));
            int maxTop = std::max((contour)[idxTopLeft].y, (contour)[idxTopRight].y);

            batteryROI = cv::Rect(ROI.x + boundingrect.tl().x, ROI.y + maxTop, boundingrect.width, boundingrect.br().y - maxTop);

            //===================Debugging======================//
            if (mDisplayMode == DisplayMode::DALL) {
                cv::Mat drawingContour;
                cv::cvtColor(image(ROI), drawingContour, cv::COLOR_GRAY2BGR);

                cv::polylines(drawingContour, contour, true, cv::Scalar(255, 0, 0), 4);

                cv::Rect boundary(boundingrect.tl().x, maxTop, batteryROI.width, batteryROI.height);
                cv::rectangle(drawingContour, boundary, cv::Scalar(0, 0, 255), 4);

                namedWindow("Binary ROI1 battery", cv::WINDOW_NORMAL);
                show_window("Binary ROI1 battery", grayImg);

                namedWindow("ROI1 battery contour", cv::WINDOW_NORMAL);
                show_window("ROI1 battery contour", drawingContour);
            }
        }
        else
        {
            // Cannot find the contour 
        }
        grayImg.release();
    }
    else
    {
        // 
    }
    return batteryROI;

}

int CylinderBatteryLower::LineRefinement(VecPoint& lstPolePosAll, int widthROI)
{
    VecPoint lstPoleLeft, lstPoleRight;
    VecInt xLeft, xRight;

    for (auto it = 0; it < lstPolePosAll.size(); it++)
    {
        if (lstPolePosAll[it].x < widthROI / 2) {
            lstPoleLeft.push_back(lstPolePosAll[it]);
            xLeft.push_back(lstPolePosAll[it].x);
        }
        else
        {
            lstPoleRight.push_back(cv::Point(widthROI - lstPolePosAll[it].x, lstPolePosAll[it].y));
            xRight.push_back(lstPolePosAll[it].x);
        }
    }
    std::sort(lstPoleRight.begin(), lstPoleRight.end(), [](auto& left, auto& right)
        {
            return left.x < right.x;
        });

    VecPoint PosXAll(lstPoleLeft.begin(), lstPoleLeft.end());
    PosXAll.insert(PosXAll.end(), lstPoleRight.begin(), lstPoleRight.end());
    std::sort(PosXAll.begin(), PosXAll.end(), [](auto& left, auto& right)
        {
            return left.x < right.x;
        });
    std::sort(xRight.begin(), xRight.end(), [](auto& left, auto& right)
        {
            return left < right;
        });

    int MaxDis = 14;
    PosXAll.clear();
    int matchIdx = 0;
    for (auto it = 0; it < xLeft.size(); it++)
    {
        float xNext = it + 1 < xLeft.size() ? xLeft[it + 1] : xLeft[it] + MaxDis;
        matchIdx = FindClosestPoint(xRight, xLeft[it], xNext, MaxDis);
        if (matchIdx < 0) {
            PosXAll.push_back(cv::Point(xLeft[it], 0));
        }
        else {
            xRight.erase(xRight.begin() + matchIdx);
        }
    }
    for (int it = 0; it < xRight.size(); it++)
    {
        PosXAll.push_back(cv::Point(xRight[it], 0));
    }
    /*for (int i = PosXAll.size() - 1; i > 0;)
    {
        VecPoint::iterator it = PosXAll.begin() + i;
        if (it->x - (it - 1)->x < MaxDis)
        {
            i = i - 2;
            PosXAll.erase(PosXAll.begin() + (i + 2));
            PosXAll.erase(PosXAll.begin() + (i + 1));
        }
        else
            i--;
    }*/

    VecDouble coeffRs;
    VecDouble coeffLs;
    double error = 0;
    int order = 5;
    float pInlier = 0.4;

    if (lstPoleLeft.size() <= order || lstPoleRight.size() <= order) {
        return -1;
    }

    bool rtnR = Polyfit::LSQFit(lstPoleRight, order, coeffRs) < MAX_CATHODE_LINE_ERROR;

    float sumLeft = 0.0;

    int nL = lstPoleLeft.size();
    std::for_each(lstPoleLeft.begin(), lstPoleLeft.end(), [&sumLeft](cv::Point& p) {sumLeft += p.y; });
    int avgL = sumLeft / nL;
    std::for_each(lstPoleLeft.begin(), lstPoleLeft.end(), [&avgL](cv::Point& p) {p.y -= avgL; });
    bool rtnL = Polyfit::LSQFit(lstPoleLeft, order, coeffLs) < MAX_CATHODE_LINE_ERROR;
    std::for_each(lstPoleLeft.begin(), lstPoleLeft.end(), [&avgL](cv::Point& p) {p.y += avgL; });

    int leftOriginalIndex = lstPoleLeft.size();
    for (int it = 0; it < PosXAll.size(); it++)
    {
        //auto result1 = std::find(lstPoleLeft.begin(), lstPoleLeft.end(), PosXAll[it]);
        auto result1 = std::find_if(lstPoleLeft.begin(), lstPoleLeft.end(), [&PosXAll, &it](const cv::Point& p) { return p.x == PosXAll[it].x; });

        if (result1 != lstPoleLeft.end())
        {
            int indexL = std::distance(lstPoleLeft.begin(), result1);

            float xs = lstPoleLeft[indexL].x;
            float ys = Polyfit::f(coeffRs, xs);

            VecPoint::iterator lsIter;
            if (indexL >= lstPoleRight.size()) lsIter = lstPoleRight.end();
            else lsIter = lstPoleRight.begin() + indexL;

            lstPoleRight.insert(lsIter, cv::Point(xs, ys));
        }
        else
        {
            //auto result2 = std::find(lstPoleRight.begin(), lstPoleRight.end(), PosXAll[it]);
            auto result2 = std::find_if(lstPoleRight.begin(), lstPoleRight.end(), [&widthROI, &PosXAll, &it](const cv::Point& p) { return p.x == (widthROI - PosXAll[it].x); });
            int indexR = std::distance(lstPoleRight.begin(), result2);

            float xs = lstPoleRight[indexR].x;
            float ys = Polyfit::f(coeffLs, xs) + avgL;

            lstPoleLeft.insert(lstPoleLeft.begin() + leftOriginalIndex, cv::Point(xs, ys));
        }
    }

    if (lstPoleLeft.size() != lstPoleRight.size())
        return -1;

    VecInt listError;

    for (int i = 0; i < lstPoleLeft.size(); i++)
    {
        listError.push_back(lstPoleLeft[i].y - lstPoleRight[i].y);
    }

    VecInt listEIndex;
    std::vector<bool> bErrorDiff;
    int MaxDiff = 20;
    int NumDiff = listError.size();
    for (int i = 0; i < NumDiff; i++)
    {
        if (std::abs(listError[i]) > MaxDiff)
        {
            bErrorDiff.push_back(false);
            listEIndex.push_back(i);
        }
        else
            bErrorDiff.push_back(true);
    }

    if (listEIndex.size() > 0)
    {
        for (int i = 0; i < listEIndex.size(); i++)
        {
            int idex = listEIndex[i];
            int refEs;
            int dLIndex = -1;
            int dRIndex = -1;
            for (int j = idex + 1; j < bErrorDiff.size(); j++)
            {
                if (bErrorDiff[j] == true)
                {
                    dRIndex = j - idex;
                    break;
                }
            }
            for (int j = idex - 1; j >= 0; j--)
            {
                if (bErrorDiff[j] == true)
                {
                    dLIndex = idex - j;
                    break;
                }
            }

            if (dLIndex != -1)
            {
                if (dRIndex != -1)
                {
                    if (dRIndex > dLIndex * 2)
                    {
                        refEs = idex - dLIndex;
                    }
                    else
                        refEs = dRIndex + idex;
                    }
                else
                    refEs = idex - dLIndex;
                }
            else
                refEs = dRIndex + idex;

            if (std::abs(lstPoleLeft[idex].y - lstPoleLeft[refEs].y) > std::abs(lstPoleRight[idex].y - lstPoleRight[refEs].y))
            {
                auto result1 = std::find(lstPolePosAll.begin(), lstPolePosAll.end(), lstPoleLeft[idex]);
                if (result1 != lstPolePosAll.end())
                {
                    int realIndex = std::distance(lstPolePosAll.begin(), result1);
                    lstPolePosAll[realIndex].y = lstPoleRight[idex].y;
                }
            }
            else
            {
                int PosX = widthROI - lstPoleRight[idex].x;

                auto result1 = std::find(lstPolePosAll.begin(), lstPolePosAll.end(), cv::Point(PosX, lstPoleRight[idex].y));
                if (result1 != lstPolePosAll.end())
                {
                    int realIndex = std::distance(lstPolePosAll.begin(), result1);
                    lstPolePosAll[realIndex].y = lstPoleLeft[idex].y;
                }
            }
        }
    }

    return 0;
}

//finding starting point width and ending
int CylinderBatteryLower::FindCenterPin(const cv::Mat& rotatedImg, cv::Rect batteryROI, int cp_startingcenter, int thresh_horiwidth, int thresh_verticalwidth, int thresh_st_checkending, int threshending, cv::Point& pointleft, cv::Point& pointright, cv::Point& pointtop) {

    cv::Rect batteryROI_10(batteryROI.x + 150, rotatedImg.rows - 200, batteryROI.width - 300, 200);
    if (!RefineROI(batteryROI_10, rotatedImg.size()))
    {
        return -1;
    }

    cv::Mat imageroi = rotatedImg(batteryROI_10);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(55, cv::Size(12, 44));
    cv::Mat claheimage;
    cv::Mat enhancedImage;
    clahe->apply(imageroi, claheimage);
    cv::threshold(claheimage, claheimage, 50, 255, cv::THRESH_BINARY);
#ifdef checkingRoilower
    cv::namedWindow("CLAHE", cv::WINDOW_GUI_NORMAL);
    show_window("CLAHE", rotatedImg);
#endif // checkingRoiupper
    cv::Mat columnsum;
    cv::reduce(claheimage, columnsum, 0, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
    int numbercolumns = columnsum.cols;
    VecPoint listpoint, listpoint2;
    cv::Mat cloneimage = cv::Mat::zeros(imageroi.size(), 0);

    //finding all vertical line include center pin and pad
    for (int i = 1; i < numbercolumns - 1; i++) {
        float current = columnsum.at<float>(i);
        float previous = columnsum.at<float>(i - 1);
        float next = columnsum.at<float>(i + 1);
        int total = (current + previous + next) / 3;
        if (total < cp_startingcenter) {
            listpoint.push_back(cv::Point(i, imageroi.rows));
        }
    }
    if (listpoint.size() == 0) {
        return 0;
    }
    else {
        for (int i = 0; i < listpoint.size(); i++) {
            cv::circle(cloneimage, listpoint[i], 3, cv::Scalar(255, 255, 255), 1);
        }
        std::vector<VecPoint > contours;
        std::vector< cv::Vec4i > hierarchy;
        cv::findContours(cloneimage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        int contourSize = contours.size();
        bool for1 = true;
        bool for2 = true;
        for (int i = imageroi.cols / 2; i > 0 && for1 == true; i--) {
            for (int j = 0; j < contours.size() && for1 == true; j++) {
                cv::Rect rect = cv::boundingRect(contours[j]);
                if (rect.x == i) {
                    listpoint2.push_back(cv::Point(rect.x + batteryROI.x + 150, rotatedImg.rows));
                    for1 = false;
                }
            }
        }
        for (int i = imageroi.cols / 2; i < imageroi.cols && for2 == true; i++) {
            for (int j = 0; j < contours.size() && for2 == true; j++) {
                cv::Rect rect = cv::boundingRect(contours[j]);
                if (rect.x == i) {
                    listpoint2.push_back(cv::Point(rect.x + rect.width + batteryROI.x + 150, rotatedImg.rows));
                    for2 = false;
                }
            }
        }
        if (listpoint2.size() < 2) {
            return 0;
        }
        else {
            int starting = 0;
            int ending = 0;
            if (listpoint2[0].x > listpoint2[1].x) {
                starting = listpoint2[1].x - 30;
                ending = listpoint2[0].x + 30;
            }
            else {
                starting = listpoint2[0].x - 30;
                ending = listpoint2[1].x + 30;
            }

            //finding horizotal line widthness
            cv::Rect batteryROI_12(starting, batteryROI.y, ending - starting, 300);
            if (!RefineROI(batteryROI_12, rotatedImg.size()))
            {
                return -1;
            }

            cv::Mat imageroiPIN = rotatedImg(batteryROI_12);
            cv::Mat claheimagePIN;
            cv::equalizeHist(imageroiPIN, claheimagePIN);
            cv::Mat rowsumPIN;
            cv::reduce(claheimagePIN, rowsumPIN, 1, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
            int numberrowssize = rowsumPIN.rows;
            cv::Point rowpos;
            for (int i = rowsumPIN.rows - 1; i > 1; i--)
            {
                float current = rowsumPIN.at<float>(i);
                float next = rowsumPIN.at<float>(i - 1);
                if (current > thresh_horiwidth) {
                    rowpos = cv::Point(0, i + batteryROI.y);
                    break;
                }
            }
            cv::Rect batteryROI_13(starting, rowpos.y - 20, ending - starting, 40);
            if (!RefineROI(batteryROI_13, rotatedImg.size()))
            {
                return -1;
            }

            cv::Mat imageroiPIN1 = rotatedImg(batteryROI_13);
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(25, cv::Size(8, 25));
            cv::Mat claheimage3;
            clahe->apply(imageroiPIN1, claheimage3);
            cv::Mat colss;
            cv::reduce(claheimage3, colss, 0, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
            cv::Mat colorpin3;
            cv::cvtColor(imageroiPIN1, colorpin3, cv::COLOR_GRAY2BGR);
            int ncols = colss.cols;
            cv::Point colsleftpos, colsrightpos;
            //Finding left pos on horizontal widthness

            for (int i = (ncols - 1) / 2; i > 1; i--)
            {
                float current = colss.at<float>(i);
                float next = colss.at<float>(i - 1);
                if (current < thresh_verticalwidth) {
                    colsleftpos = cv::Point(i + starting, rowpos.y);
                    cv::line(colorpin3, cv::Point(i, 0), cv::Point(i, colorpin3.rows), cv::Scalar(255, 255, 0), 1);
                    break;
                }
            }
            //Finding right pos on horizontal widthness

            for (int i = (ncols - 1) / 2; i < (ncols - 1); i++)
            {
                float current = colss.at<float>(i);
                float next = colss.at<float>(i - 1);
                if (current < thresh_verticalwidth) {
                    cv::line(colorpin3, cv::Point(i, 0), cv::Point(i, colorpin3.rows), cv::Scalar(255, 255, 0), 1);
                    colsrightpos = cv::Point(i + starting, rowpos.y);
                    break;
                }
        }

            //Finding ending point
            cv::Rect batteryROI_14(colsleftpos.x, batteryROI.y + 5, colsrightpos.x - colsleftpos.x, rotatedImg.rows - batteryROI.y - rowpos.y);
            if (!RefineROI(batteryROI_14, rotatedImg.size()))
            {
                return -1;
            }

            cv::Mat imageroiPIN12 = rotatedImg(batteryROI_14).clone();
            cv::blur(imageroiPIN12, imageroiPIN12, cv::Size(3, 3));
            cv::threshold(imageroiPIN12, imageroiPIN12, 0, 255, cv::THRESH_OTSU);
            cv::Mat rowending;
            cv::reduce(imageroiPIN12, rowending, 1, cv::ReduceTypes::REDUCE_AVG, CV_32FC1);
            cv::Mat colorpin13;

            cv::cvtColor(imageroiPIN12, colorpin13, cv::COLOR_GRAY2BGR);
            int nrows = rowending.rows;
            int refpos = 0;
            cv::Point endingpoint = cv::Point(0, 0);


            for (int i = nrows - 1; i > 0; i--)
            {
                float current = rowending.at<float>(i);
                if (current > thresh_st_checkending) {
                    //cv::line(colorpin13, cv::Point(0, i), cv::Point(colorpin13.cols, i), cv::Scalar(255, 255, 0), 1);
                    refpos = i;
                    break;
                }
            }
            for (int i = refpos + 1; i > 0; i--)
            {
                float current = rowending.at<float>(i);
                if (current < threshending) {
                    //cv::line(colorpin13, cv::Point(0, i), cv::Point(colorpin13.cols, i), cv::Scalar(255, 0, 0), 1);
                    endingpoint = cv::Point((colsleftpos.x + colsrightpos.x) / 2, i + batteryROI.y);
                    break;
                }
            }
            //Result
            //Two points left and right
            pointleft = colsleftpos;
            pointright = colsrightpos;
            //Point for find depth
            pointtop = endingpoint;
#ifdef checkingRoilower
            cv::namedWindow("CheckROI111111153456323", cv::WINDOW_GUI_NORMAL);
            show_window("CheckROI111111153456323", colorpin13);
            cv::namedWindow("CheckROI123", cv::WINDOW_GUI_NORMAL);
            show_window("CheckROI123", rotatedImg(batteryROI_14));
            cv::namedWindow("CheckROI11123", cv::WINDOW_GUI_NORMAL);
            show_window("CheckROI11123", colorpin3);
            cv::waitKey(0);
#endif

            return 1;
    }
}


}
void CylinderBatteryLower::RecheckDisconectAnode(const cv::Mat& Roi4Res, cv::Mat& anodeDebugImage, const VecPoint& CathodePnts, VecInt& anodePos, const int& stepVertical)
{
    //Take the most appearance of pole height
    cv::Size imgSize = Roi4Res.size();
    VecFloat poleHeight;
    int range = 6;
    VecFloat poleHeightRange;
    VecInt idxListPoleRecheck;
    std::vector<std::pair<cv::Point, double>> listPointChecked;
    int numberPixelConcecutive = 5;
    int verticalHeightWindow = stepVertical + 1;
    if (CathodePnts.size() == anodePos.size() && anodePos.size() > 5 && CathodePnts.size() != 0)
    {
        for (int i = 0; i < CathodePnts.size(); i++)
        {
            poleHeight.push_back(std::abs(CathodePnts[i].y - anodePos[i]));
        }
    }
    else
    {
        return;
    }
    if (poleHeight.size() == 0)
    {
        return;
    }
    for (int i = 0; i < poleHeight.size(); i++)
    {
        for (int j = (-range); j < range; j++)
        {
            poleHeightRange.push_back(poleHeight[i] + j);
        }
    }
    if (poleHeightRange.size() == 0)
    {
        return;
    }
    //Finding most frequent
    std::sort(poleHeightRange.begin(), poleHeightRange.end(), [](const float& a, const float& b) {
        return a < b; });
    int max_count = 1, res = poleHeightRange[0], curr_count = 1;
    for (int i = 1; i < poleHeightRange.size(); i++) {
        if (poleHeightRange[i] == poleHeightRange[i - 1])
            curr_count++;
        else {
            if (curr_count > max_count) {
                max_count = curr_count;
                res = poleHeightRange[i - 1];
            }
            curr_count = 1;
        }
    }

    // If last element is most frequent
    if (curr_count > max_count)
    {
        max_count = curr_count;
        res = poleHeightRange[poleHeightRange.size() - 1];
    }

    //Recheck pole with poleheight smaller than res
    for (int i = 0; i < poleHeight.size(); i++)
    {
        if (poleHeight[i] < res / 1.5)
        {
            idxListPoleRecheck.push_back(i);
        }
    }
    //Finding new starting Point from ending disconnection
    for (int i = 0; i < idxListPoleRecheck.size(); i++)
    {
        int startingPos = anodePos[idxListPoleRecheck[i]] - 1;
        int count = 0;
        for (int j = startingPos; j > startingPos - res; j--)
        {
            int sum = 0;
            for (int k = 0; k < verticalHeightWindow; k++)
            {
                if (j - k >= 0)
                {
                    int intensity = Roi4Res.at<uchar>(j - k, CathodePnts[idxListPoleRecheck[i]].x);
                    sum = sum + intensity;
                }
                else
                {
                    break;
                }
            }
            sum = sum / verticalHeightWindow;
            int regionIdx = CheckRegion(CathodePnts[idxListPoleRecheck[i]].x, 6, imgSize.width, mCenterNeglectionWidth);
            double aThreshold;
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
            bool flag = true;
            if (sum <= (aThreshold * 10)) {
                for (int m = 0; m < numberPixelConcecutive; m++)
                {
                    int sumInten = 0;
                    bool flagBreak = false;
                    for (int k = 0; k < verticalHeightWindow; k++)
                    {
                        if (j - k - m >= 0) {
                            int intensity = Roi4Res.at<uchar>(j - k - m, CathodePnts[idxListPoleRecheck[i]].x);
                            sumInten = sumInten + intensity;
                        }
                        else
                        {
                            flagBreak = true;
                            break;
                        }
                    }
                    sumInten = sumInten / verticalHeightWindow;
                    if (sumInten > (aThreshold * 10) || flagBreak == true)
                    {
                        flag = false;
                        break;
                    }
                }
                if (flag == true)
                {
                    if (mDisplayMode == DisplayMode::DALL)
                    {
                        cv::circle(anodeDebugImage, cv::Point(CathodePnts[idxListPoleRecheck[i]].x, j), 1, cv::Scalar(0, 255, 125), 1);
                        cv::namedWindow("RecheckDisconnec", cv::WINDOW_GUI_EXPANDED);
                        show_window("RecheckDisconnec", anodeDebugImage);
                    }
                    listPointChecked.push_back(std::make_pair(cv::Point(CathodePnts[idxListPoleRecheck[i]].x, j), aThreshold));
                    break;
                }
            }
        }
    }
    //Apply Line tracing from disconecction point
    for (auto p : listPointChecked)
    {
        int regionIdx = CheckRegion(p.first.x, 6, imgSize.width, mCenterNeglectionWidth);
        //Line tracing for lower
        cv::Point startingPoint(p.first);

        //Check if pole is within outer region to restrict movement
        bool ifRestrict = false;
        if (regionIdx == 1) {
            ifRestrict = true;
        }

        //cv::Mat lookUpTable(1, 256, CV_8U);
        //uchar* p = lookUpTable.ptr();
        //for (int iGam = 0; iGam < 256; iGam++)
        //	p[iGam] = cv::saturate_cast<uchar>(pow(iGam / 255.0, 0.67) * 255.0);
        //cv::Mat gammaCorrect = imageROI4.clone();
        //LUT(imageROI4, lookUpTable, gammaCorrect);

        int anodeYPosVal = findAnodeByLineTracing(/*roiDraw,*/Roi4Res, startingPoint, startingPoint.y, 3, 7, p.second * 10, anodeDebugImage, 0, 0, ifRestrict, 3);
        for (int i = 0; i < CathodePnts.size(); i++)
        {
            if (CathodePnts[i].x == p.first.x)
            {
                anodePos[i] = anodeYPosVal;
                break;
            }
        }
        //End calculate local mean of anodeROIs
        ///////////////////////////////////////
        //End finding starting and ending point of Anodes
    }

    // Check Boundary tab poles
    CheckBoundaryTab(poleHeight, anodePos, res, 5);
}

/// <summary>
/// Applied to only UPPER images. Find the reference line (the bottom part of the case)
/// </summary>
/// <param name="image"> battery image</param>
/// <returns> Position of the reference line on Y axis. First: J/R start position, Second: J/R end position</returns>
std::pair<int, int> CylinderBatteryLower::FindTopReferenceLine(const cv::Mat& image) {

    // Sub ROI for eliminate the border part of battery
    int subRoiX = JR_ROIX;
    int subHeight = 80;
    std::pair<int, int> resultPair = std::pair<int, int>(-1, -1);

    cv::Rect ROI(subRoiX, 0, image.cols - subRoiX * 2, image.rows);
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

            if (centerIdx < windows || centerIdx + windows >= binaryImage.cols) {
                return std::pair<int, int>(-1, -1);
            }

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
                    for (int i = jrTop; i < reduceVec.size(); i++)
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
                        if (contours[i].size() > 100) {
                            drawContours(grayImage, contours, i, cv::Scalar(255, 0, 0), 3);
                        }
                    }

                    cv::cvtColor(grayImage, grayImage, cv::COLOR_GRAY2BGR);
                    cv::rectangle(grayImage, boundary, cv::Scalar(240, 176, 0));
                    cv::rectangle(grayImage, rect, cv::Scalar(160, 48, 112));

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
                imageRoi.release();
                imageReduced.release();
                resultPair = std::make_pair(jrTop + rect.y + ROI.y, jrBot + rect.y + ROI.y);
            }
            else
            {
                //Failed rect Refinement
            }
        }
        else
        {
            // Not found contour
        }
        binaryImage.release();
        grayImage.release();
    }
    else
    {
        //Failed ROI Refinement
    }

    return resultPair;
}

ERR_CODE CylinderBatteryLower::Inspection2(const cv::Mat& inImg, BatteryInspectionResult& BIresult)
{
    cv::Mat src;
    auto res = Convert8Bits(inImg, src);
    if (res)
    {
        BIresult.Description = "Image empty or not support!";
        return res == 1 ? ERR_CODE::errImageEmpty : ERR_CODE::errImageFormat;
    }

    //GammaCorrection(src, mGamma);
    mGamma.Apply(src, src);

    if (false == RefineROI(mRoi, cv::Size(src.cols, src.rows)))
    {
        BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errROIRefinement;
    }
    //-----end of pre-processing ------------------------------------------------------------------------

    std::string FinalDecision;

    cv::Mat rotatedImg;
    if (mDirection == 1) {
        cv::rotate(src, rotatedImg, cv::ROTATE_180);
    }
    else {
        rotatedImg = src.clone();
    }

    if (mPixelSize == 0) {
        BIresult.Description = "Pixel size cannot set to 0";
        return ERR_CODE::errPixelSize;
    }

    std::vector<std::string> warnMsg;

    cv::Rect batteryROI = FindBatteryROI(rotatedImg);
    if (batteryROI.width > 0 && batteryROI.height > 0)
    {
        //Check the width of battery is in the range of masterJigCellSize
        auto batteryWidth_mm = batteryROI.width * mPixelSize;

        if (!mValidCellWidthRange.IsInRange(round_f(batteryWidth_mm, 2)))
        {
            BIresult.Description = "Error: Width of battery is out of range 2 : " + std::to_string(batteryWidth_mm);;
            return ERR_CODE::errWidthBattery;
    }
}
    else
    {
        BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }

    //======================================Finding Pole Positions====================================//
    // 1: From battery?s boundary found in section 1- select batteryROI_4
    //int yA2 = batteryROI.y;
    int roiBtmYPosition = mRoi.br().y;
    if (mDirection == 1) {
        roiBtmYPosition = src.rows - mRoi.y;
    }

    if (batteryROI.y >= roiBtmYPosition) {
        BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }

    cv::cvtColor(rotatedImg, BIresult.resImg, cv::COLOR_GRAY2BGR);
    bool poleRegionAuto = false;

    /*if (mEnableAutoMode || !mInspectingItems.FIND_JR_ROI) {
        JR_ROIX = 20;
        JR_ROIY = 0;
    }*/

    if (mEnableAutoMode || mPoleRegionHeight == 0) {
        if ((batteryROI.height + batteryROI.y) < rotatedImg.rows) {
            mPoleRegionHeight = ((batteryROI.height)) * 0.5;
            poleRegionAuto = true;
        }
    }
    //2022-05-30 Nhan change checkpole learning before check JRROI to get correct PoleRegion avoid detect pole at boundary fail

    // 4: Find the shift of inner Poles (Pole leaning)
    cv::Rect poleLeaningROI = batteryROI;
    poleLeaningROI.y = batteryROI.y + JR_ROIY;
    poleLeaningROI.height = mPoleRegionHeight;
    double LeftBorder = 0;
    double RightBorder = poleLeaningROI.height;
    if (mInspectingItems.CHECK_LEANING)
    {
        if (false == RefineROI(poleLeaningROI, rotatedImg.size()))
        {
            BIresult.Description = "PoleLeaning ROI Refinement failed";
            return ERR_CODE::errROIRefinement;
        }
    #ifdef DEBUG_SHOW_ROI
        cv::rectangle(res, poleLeaningROI, cv::Scalar(0, 0, 255));
        cv::putText(res, "PL", cv::Point(poleLeaningROI.x + poleLeaningROI.width + 10, poleLeaningROI.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    #endif
        int leaningThreshold = mEnableAutoMode ? 0 : mPoleLeaningThreshold;
        if (!CheckPoleLeaning(rotatedImg(poleLeaningROI), mPoleRegionHeight, leaningThreshold, LeftBorder, RightBorder))
        {
            warnMsg.push_back("! Detection Pole Leaning is disable");
        }
    }

    cv::Rect poleRegionROI;
    if (mInspectingItems.FIND_JR_ROI)
    {
        // JRROI
        cv::Rect JRROI = poleLeaningROI;
        int maxLeaningDistance = std::max(LeftBorder, poleLeaningROI.width - RightBorder);
        JRROI.x = maxLeaningDistance + poleLeaningROI.x;
        JRROI.width = poleLeaningROI.width - (maxLeaningDistance * 2);
        JRROI.height = mPoleRegionHeight;
        if (false == RefineROI(JRROI, src.size()))
        {
            BIresult.Description = "JR ROI Refinement failed";
            return ERR_CODE::errROIRefinement;
        }

        // 2: Finding the Jelly Roll (J/R) area
        const std::pair<int, int> jrPos = FindTopReferenceLine(rotatedImg(JRROI));
        //cv::fillPoly(rotatedImg, TapContours, cv::Scalar(255));
        if (jrPos.first < 0) {
            BIresult.Description = "Cannot identify the top blank area of pole in battery (for reference line), please check again JR Setting";
            return ERR_CODE::errJRFinding;
        }

        //poleRegionROI.x = poleLeaningROI.x + LeftBorder + JR_ROIX;
        poleRegionROI.y = JRROI.y + jrPos.first;
        //poleRegionROI.width = RightBorder - LeftBorder - 2 * JR_ROIX;
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

    int poleRegionY = poleRegionROI.y;
    if (mInspectingItems.CHECK_LEANING)
    {
        //Draw pole leaning position
        cv::line(BIresult.resImg, cv::Point(poleLeaningROI.x + LeftBorder, poleRegionROI.y), cv::Point(poleLeaningROI.x + LeftBorder, poleRegionROI.br().y), cv::Scalar(255, 0, 255));
        cv::line(BIresult.resImg, cv::Point(poleLeaningROI.x + RightBorder, poleRegionROI.y), cv::Point(poleLeaningROI.x + RightBorder, poleRegionROI.br().y), cv::Scalar(255, 0, 255));
    }

    cv::Point pinLeftPoint = cv::Point(0, 0);
    cv::Point pinRightPoint = cv::Point(0, 0);
    cv::Point pinTopPoint = cv::Point(0, 0);
    if(mInspectingItems.CHECK_CENTER_PIN)
    {
        // 3: Checking pin center
        int centerPinRes = FindCenterPin(rotatedImg, batteryROI, 60, 200, 60, 200, 60, pinLeftPoint, pinRightPoint, pinTopPoint);

        if (centerPinRes < 0) {
            BIresult.Description = "Cannot find suitable area for inspecting Center Pin, please check again outer ROI Setting or input image";
            return ERR_CODE::errBeadingArea;
        }

        double cPinWidth = 0.0, cPinDepth = 0.0;
        if (pinLeftPoint.x != 0) {
            BIresult.isPinExist = true;
            cv::arrowedLine(BIresult.resImg, cv::Point(pinTopPoint.x, pinTopPoint.y), cv::Point(pinTopPoint.x, batteryROI.y), cv::Scalar(20, 220, 255), 4);
            cv::arrowedLine(BIresult.resImg, cv::Point(pinTopPoint.x, batteryROI.y), cv::Point(pinTopPoint.x, pinTopPoint.y), cv::Scalar(20, 220, 255), 4);
            cv::arrowedLine(BIresult.resImg, pinLeftPoint, pinRightPoint, cv::Scalar(0, 255, 0), 2);
            cv::arrowedLine(BIresult.resImg, pinRightPoint, pinLeftPoint, cv::Scalar(0, 255, 0), 2);
            int lineLength = 30;
            cv::line(BIresult.resImg, cv::Point(pinLeftPoint.x, pinLeftPoint.y - lineLength), cv::Point(pinLeftPoint.x, pinLeftPoint.y + lineLength), cv::Scalar(20, 220, 0), 2);
            cv::line(BIresult.resImg, cv::Point(pinRightPoint.x, pinRightPoint.y - lineLength), cv::Point(pinRightPoint.x, pinRightPoint.y + lineLength), cv::Scalar(20, 220, 0), 2);
            cPinWidth = abs(pinLeftPoint.x - pinRightPoint.x) * mPixelSize;
            cPinDepth = abs(pinTopPoint.y - batteryROI.y) * mPixelSize;
            if (pinTopPoint.x == 0 || cPinDepth == 0) {
                BIresult.Description = "Pin Depth can not be measured";
                return ERR_CODE::errCenterPin;
            }

            BIresult.centerPinDepth = cPinDepth;
            BIresult.centerPinWidth = cPinWidth;
        }
        else
        {
            BIresult.isPinExist = false;
            BIresult.centerPinDepth = -1;
            BIresult.centerPinWidth = -1;
        }
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
    float maxA = -1.0, maxB = -1.0;
    float minA = -1.0, minB = -1.0;
    float averageA = 0.0f, averageB = 0.0f;
    int nPoleRight = 0;
    int nPoleLeft = 0;
    int fontWeight = 2;

    VecDouble sCath2CaseMeas;
    //Position in the original image
    VecPoint listAnodeOri;
    //Position in the original image
    VecPoint listCathodeOri;
    //mm distance
    VecDouble listAnode2CathodeDistance;
    std::vector<bool> listAno2CathodDecision;
    //mm distance
    VecDouble listAnode2CaseDistance;
    std::vector<bool> listAno2CaseDecision;
    //mm distance
    VecDouble listCathode2CaseDistance;
    std::vector<bool> listCathode2CaseDecision;
    //pixel distance to the battery ROI
    VecDouble listPolePos;
    //Check disappear poles
    std::vector<std::pair<int, int>> lstPolePositionErr;
    ERR_CODE errBlackRegion = ERR_CODE::OK;

    bool isAnode2CaseOK = true;
    bool isAnode2CathodeOK = true;
    bool isCathode2CaseOK = true;
    if (mInspectingItems.ANODE_TO_CATHODE_LENGTH || mInspectingItems.CATHODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CASE_GAP)
    {
        ERR_CODE err = FindAllPolesPos(rotatedImg, poleRegionROI, lstPolePosXAll, anodePos, cathodePos, descript);

        // check error when find cathode position
        if (err != ERR_CODE::OK && !descript.empty())
        {
            BIresult.Description = descript;
            return err;
        }

        // check black cloud
        if (mInspectingItems.CHECK_BLACK_CLOUD)
        {
            VecPoint BlackRegion;
            cv::Rect PoleROI;
            PoleROI.x = poleRegionROI.x;
            PoleROI.y = batteryROI.y + mBlackCloudOffset;
            PoleROI.width = poleRegionROI.width;
            PoleROI.height = *std::max_element(cathodePos.begin(), cathodePos.end()) + (poleRegionROI.y - PoleROI.y);
            errBlackRegion = CheckBlackCloud(rotatedImg, PoleROI, BlackRegion, mBlackCloudThreshold, BIresult.blackCloudHeight, BIresult.blackCloudAreaRatio);
            if (errBlackRegion == ERR_CODE::errCheckBlackCloud)
            {
                BIresult.Description = "Error CheckBlackCloud";
                return errBlackRegion;
            }
            else
            {
                if (mBlackCloudCheckCondition ==  0)
                {
					errBlackRegion = (BIresult.blackCloudAreaRatio > mRatioAreaThreshold || BIresult.blackCloudHeight > mHeightThreshold) ? ERR_CODE::NG : ERR_CODE::OK;

					cv::Scalar color = errBlackRegion == ERR_CODE::OK ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
					if (BlackRegion.size()) cv::polylines(BIresult.resImg, BlackRegion, true, color, 2);
				}
                else {
					errBlackRegion = (BIresult.blackCloudAreaRatio > mRatioAreaThreshold && BIresult.blackCloudHeight > mHeightThreshold) ? ERR_CODE::NG : ERR_CODE::OK;

					cv::Scalar color = errBlackRegion == ERR_CODE::OK ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
					if (BlackRegion.size()) cv::polylines(BIresult.resImg, BlackRegion, true, color, 2);
				}
            }
        }
        //===============================================making Decisions==================================================//
        
        int poleIdx = 0;
        int missPoleIdx = 0;
       
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

        int lstPoleSize = lstPolePosXAll.size();

        // For preventing anode position out of range
        std::for_each(anodePos.begin(), anodePos.end(), [](int& ap) {ap = ap < 0 ? 0 : ap; });

        VecDouble lstAnodeDouble;
        std::for_each(anodePos.begin(), anodePos.end(), [&lstAnodeDouble](int& ap) {lstAnodeDouble.push_back(ap); });
        cv::blur(lstAnodeDouble, lstAnodeDouble, cv::Size(5, 1));
        //lstAnodeDouble = weightedAvgSmoothen(lstAnodeDouble);
        anodePos.clear();
        std::for_each(lstAnodeDouble.begin(), lstAnodeDouble.end(), [&anodePos](double& ap) {anodePos.push_back(ap); });

        assert(cathodePos.size() == lstPoleSize && anodePos.size() == lstPoleSize);

        //=============================================Making Decisions===================================
        float fontScale = 0.8;
        int textLineSpace = 25;

        //For calculate the anode to case distance
        const int caseLine = batteryROI.y + mCaseLineOffset;

        
        int decimal_places = 2;
        for (int poleIdx = 0; poleIdx < lstPoleSize; poleIdx++)
        {
            //Update the original position on the Image
            auto mAnode = cv::Point(poleRegionROI.x + lstPolePosXAll[poleIdx], poleRegionROI.y + anodePos[poleIdx]);
            auto mCathode = cv::Point(poleRegionROI.x + lstPolePosXAll[poleIdx], poleRegionROI.y + cathodePos[poleIdx]);

            //Get the distance from the pole to the left battery roi
            listPolePos.push_back(abs(mAnode.x - batteryROI.br().x) * mPixelSize);

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

            listAnodeOri.push_back(mAnode);
            listCathodeOri.push_back(mCathode);
        }

        if (mInspectingItems.CHECK_ERASE_POLE ) {
            int boundaryList[4] = { borderLeft, centerLeft, centerRight, borderRight };
            CheckErasedPole(leftXList, rightXList, lstPolePositionErr, mPolesDistanceRange, boundaryList, mIsCheckPoleNo);
        }

        for (auto& pairX : lstPolePositionErr) {
            pairX.first += poleRegionROI.x;
            pairX.second += poleRegionROI.x;
        }

        //Print reference case line
        if (mInspectingItems.CATHODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CASE_VARIATION)
        {
            //cv::line(BIresult.resImg, cv::Point(poleRegionROI.x, caseLine), cv::Point(poleRegionROI.x + poleRegionROI.width, caseLine), cv::Scalar(0, 255, 0), 3);
            cv::line(BIresult.resImg, cv::Point(poleRegionROI.x, caseLine), cv::Point(poleRegionROI.x + poleRegionROI.width, caseLine), cv::Scalar(0, 255, 0), 3);
        }

        // Plot the skip pole region
        cv::line(BIresult.resImg, cv::Point(poleRegionROI.x + borderLeft, poleRegionY), cv::Point(poleRegionROI.x + borderLeft, poleRegionY + mPoleRegionHeight), CV_RGB(0xc2, 0xa6, 0x09));
        cv::line(BIresult.resImg, cv::Point(poleRegionROI.x + poleRegionROI.width - borderLeft, poleRegionY), cv::Point(poleRegionROI.x + poleRegionROI.width - borderLeft, poleRegionY + mPoleRegionHeight), CV_RGB(0xc2, 0xa6, 0x09));
        
        //Draw the pole result
        plotPoleWithRect(BIresult.resImg
            , poleRegionROI
            , mCenterNeglectionWidth
            , mSkipPolesDistance
            , nPoleLeft
            , mLineType
            , ((int)mDisplayMode & (int)DisplayMode::GRID)
            , listAnodeOri, listCathodeOri
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
            sCath2CaseMeas.push_back(listAnode2CaseDistance[pIdx] + listAnode2CathodeDistance[pIdx]);
        }

        //===============================================PLoting results in the original direction=================================================//
        if (mDirection == 1) {
            //cv::rotate(res, res, cv::ROTATE_180);
            cv::Size imgSize = src.size();
            //cv::Point ImgCenter = cv::Point(src.cols / 2, src.rows / 2);
            listAnodeOri = rotate180vtPoint(listAnodeOri, imgSize);
            listCathodeOri = rotate180vtPoint(listCathodeOri, imgSize);
            std::reverse(sCath2CaseMeas.begin(), sCath2CaseMeas.end());
            std::reverse(listPolePos.begin(), listPolePos.end());
            std::reverse(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
            std::reverse(listAno2CaseDecision.begin(), listAno2CaseDecision.end());
            std::reverse(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
            std::reverse(listAno2CathodDecision.begin(), listAno2CathodDecision.end());
            std::reverse(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
            std::reverse(listCathode2CaseDecision.begin(), listCathode2CaseDecision.end());
        }
    }

    if (mDirection == 1) {
        cv::rotate(BIresult.resImg, BIresult.resImg, cv::ROTATE_180);
        // Update the rectangle coordinates after rotation
        batteryROI.x = BIresult.resImg.cols - batteryROI.x - batteryROI.width;
        batteryROI.y = BIresult.resImg.rows - batteryROI.y - batteryROI.height;
    }

    //Draw the setting ROI
    cv::Rect settingROI(mRoi);
    cv::rectangle(BIresult.resImg, settingROI, cv::Scalar(0, 255, 255), 3);
    //===============================================PLoting results in the 180 rotated image=================================================//

    std::string str = " ,maxB=" + std::to_string(maxB) + " ,minB=" + std::to_string(minB) + " ,averageB=" + std::to_string(averageB);
    BIresult.sData += str.c_str();
    BIresult.minCathode2Anode = minB;
    BIresult.maxCathode2Anode = maxB;
    BIresult.avgCathode2Anode = averageB;
    BIresult.minAnode2Case = minA;
    BIresult.maxAnode2Case = maxA;
    BIresult.avgAnode2Case = averageA;
    BIresult.minCathode2Case = (minA + minB);
    BIresult.maxCathode2Case = (maxA + maxB);
    BIresult.avgCathode2Case = (averageA + averageB);
    
    BIresult.nLeftPole = nPoleRight;
    BIresult.outerROI = batteryROI;
    BIresult.yA2 = (mDirection == 1) ? batteryROI.br().y : batteryROI.y;
    BIresult.sCathode2Anode = std::move(listAnode2CathodeDistance);
    BIresult.sAnode2Case = std::move(listAnode2CaseDistance);
    BIresult.sCathode2Case = std::move(listCathode2CaseDistance);
    BIresult.vtAnodes = std::move(listAnodeOri);
    BIresult.vtCathodes = std::move(listCathodeOri);
    BIresult.sXPos = std::move(listPolePos);
    BIresult.vtAno2CathodDecision = std::move(listAno2CathodDecision);
    BIresult.vtAno2CaseDecision = std::move(listAno2CaseDecision);
    BIresult.vtCathode2CaseDecision = std::move(listCathode2CaseDecision);
    BIresult.Anodes2CaseDecision = isAnode2CaseOK ? "OK" : "NG";
    BIresult.Anode2CathodeDecision = isAnode2CathodeOK ? "OK" : "NG";
    BIresult.Cathode2CaseDecision = isCathode2CaseOK ? "OK" : "NG";
    FinalDecision = (isAnode2CaseOK && isAnode2CathodeOK && isCathode2CaseOK) ? "OK" : "NG";

    std::vector<std::string> errMsg;
    //Draw the pole leaning limit
    if (mInspectingItems.CHECK_LEANING
        && mLeaningDistanceMin > 0)
    {
        cv::line(BIresult.resImg, cv::Point(batteryROI.x + mLeaningDistanceMin, poleRegionROI.br().y - 30), cv::Point(batteryROI.x + mLeaningDistanceMin, poleRegionROI.br().y + 10), CV_RGB(116, 33, 165), 2);
        cv::line(BIresult.resImg, cv::Point(batteryROI.br().x - mLeaningDistanceMin, poleRegionROI.br().y - 30), cv::Point(batteryROI.br().x - mLeaningDistanceMin, poleRegionROI.br().y + 10), CV_RGB(116, 33, 165), 2);
    }

    //Draw the battery pin information
    if ((pinLeftPoint.x > 0)
        && mInspectingItems.CHECK_CENTER_PIN)
    {
        cv::Point pntDeptText(pinTopPoint.x, pinTopPoint.y + 5);
        cv::Point pntWidthText(0.5 * (pinLeftPoint.x + pinRightPoint.x) + 20, pinLeftPoint.y + 10);

        if (mDirection == 1) {
            cv::Point center(BIresult.resImg.cols / 2, BIresult.resImg.rows / 2);

            float pX = center.x - (pntDeptText.x - center.x);
            float pY = center.y - (pntDeptText.y - center.y);
            pntDeptText = cv::Point(pX, pY);

            pX = center.x - (pntWidthText.x - center.x);
            pY = center.y - (pntWidthText.y - center.y);
            pntWidthText = cv::Point(pX, pY);
        }
        if (BIresult.isPinExist) {
            std::ostringstream strCPinW;
            strCPinW << std::fixed;
            strCPinW << std::setprecision(2);
            strCPinW << BIresult.centerPinWidth;
            cv::putText(BIresult.resImg, strCPinW.str(), pntWidthText, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 220, 0), 1);
            std::ostringstream strCPinD;
            strCPinD << std::fixed;
            strCPinD << std::setprecision(2);
            strCPinD << BIresult.centerPinDepth;
            cv::putText(BIresult.resImg, strCPinD.str(), pntDeptText, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(20, 220, 255), 1);
        }
    }

    if (mLeaningDistanceMin > 0
        && ((LeftBorder - batteryROI.x) > mLeaningDistanceMin || (batteryROI.br().x - RightBorder) > mLeaningDistanceMin)
        && mInspectingItems.CHECK_LEANING)
    {
        FinalDecision = "NG";
        errMsg.push_back("Leaning Distance NG");
    }

    if ( mInspectingItems.CHECK_ERASE_POLE && (lstPolePositionErr.size() > 0
        || (mIsCheckPoleNo && (nPoleLeft < mOneSidePoleNumber || nPoleRight < mOneSidePoleNumber)))
        && (mInspectingItems.ANODE_TO_CATHODE_LENGTH || mInspectingItems.ANODE_TO_CASE_GAP))
    {
        FinalDecision = "NG";
        errMsg.push_back("Number of detected poles is not sufficient");
    }

    if (BIresult.Anode2CathodeDecision == "NG"
        && mInspectingItems.ANODE_TO_CATHODE_LENGTH)
    {
        FinalDecision = "NG";
        errMsg.push_back("Anode to Cathode Distance NG");
    }

    if (BIresult.Anodes2CaseDecision == "NG"
        && mInspectingItems.ANODE_TO_CASE_GAP)
    {
        FinalDecision = "NG";
        errMsg.push_back("Anode to Case Distance NG");
    }

    if (BIresult.Cathode2CaseDecision == "NG"
        && mInspectingItems.CATHODE_TO_CASE_GAP)
    {
        FinalDecision = "NG";
        errMsg.push_back("Cathode to Case Distance NG");
    }

    if (((int)mDisplayMode & (int)DisplayMode::TEXT)
        && (mInspectingItems.ANODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CATHODE_LENGTH || mInspectingItems.CATHODE_TO_CASE_GAP))
    {
        //Draw the result infor
        drawPoleTextResult(this, BIresult);
    }

    if ((mInspectingItems.ANODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CATHODE_LENGTH)
        && mInspectingItems.ANODE_TO_CASE_VARIATION)
    {
        int tmpPosX = BIresult.resImg.cols / 2.8;
        int tmpPosY = mTextPosition.y + 200;
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

    if (mInspectingItems.CHECK_BLACK_CLOUD)
    {
        bool isNGBlackCloud = !(errBlackRegion == ERR_CODE::OK);
        BIresult.BlackCloudDecision = isNGBlackCloud ? "NG" : "OK";
        cv::Scalar blackCloudTextColor = isNGBlackCloud ? COLOR_CV_RED : cv::Scalar(255, 255, 0);
        int tmpPosX = BIresult.resImg.cols / 2.8;
        int tmpPosY = mTextPosition.y + 300;
        cv::putText(BIresult.resImg, "Black Cloud Detected : " + BIresult.BlackCloudDecision, cv::Point(tmpPosX, tmpPosY), cv::FONT_HERSHEY_SIMPLEX, 1, blackCloudTextColor, 2);
        std::ostringstream tmpStrHeight;
        tmpStrHeight << std::fixed << std::setprecision(0) << round(BIresult.blackCloudHeight);
        cv::putText(BIresult.resImg, "Height : " + tmpStrHeight.str() + " px", cv::Point(tmpPosX, tmpPosY + 30), cv::FONT_HERSHEY_SIMPLEX, 1, blackCloudTextColor, 2);
        std::ostringstream tmpStrAreaRatio;
        tmpStrAreaRatio << std::fixed << std::setprecision(3) << BIresult.blackCloudAreaRatio;
        cv::putText(BIresult.resImg, "Area Ratio : " + tmpStrAreaRatio.str(), cv::Point(tmpPosX, tmpPosY + 30 * 2), cv::FONT_HERSHEY_SIMPLEX, 1, blackCloudTextColor, 2);

        if (isNGBlackCloud)
        {
            FinalDecision = "NG";
            //errMsg.push_back("Black Cloud Detected");
        }
    }

    if( mInspectingItems.CHECK_CENTER_PIN && !BIresult.isPinExist)
    {
        FinalDecision = "NG";
        errMsg.push_back("The Pin not exist");
        //pGI->SetInspectionNGCode(NG_LowerCenterPin);
    }

    for (int i = 0; i < errMsg.size(); i++) {
        cv::putText(BIresult.resImg, errMsg[i], cv::Point(BIresult.resImg.cols / 2.8, 150 + i * 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), fontWeight);
    }

#ifdef _XVT_SIMULATOR_
    if (mDisplayMode == 2)
    {
        int start = errMsg.size();
        for (int i = 0; i < warnMsg.size(); i++) {
            cv::putText(res, warnMsg[i], cv::Point(res.cols / 2.8, 150 + (start + i) * 30), cv::FONT_HERSHEY_SIMPLEX, 1, CV_ORANGE, fontWeight);
        }
    }
#endif

    cv::putText(BIresult.resImg, "Result: " + FinalDecision, cv::Point(BIresult.resImg.cols / 2.5, 100), cv::FONT_HERSHEY_SIMPLEX, 2,
        FinalDecision == "OK" ? cv::Scalar(245, 164, 66) : cv::Scalar(0, 0, 255), fontWeight);

    if (mDisplayMode == DisplayMode::DALL)
    {
        show_window("Result", BIresult.resImg);
    }

    BIresult.finalDecision = FinalDecision;
    return  ERR_CODE::OK;
}

ERR_CODE CylinderBatteryLower::CheckBlackCloud(const cv::Mat& rotatedImg,
    cv::Rect& PoleROI, VecPoint& BlackRegion, int Thd, double& resultHeight, double& resultRatioArea)
{
    resultHeight = 0.0;
    resultRatioArea = 0.0;
    ERR_CODE iResult;
    if (!rotatedImg.empty() && !PoleROI.empty())
    {
        cv::Mat BlurImg, BinImg;
        RefineROI(PoleROI, rotatedImg.size());
        cv::blur(rotatedImg(PoleROI), BlurImg, cv::Size(3, 3));
        cv::threshold(BlurImg, BinImg, Thd, 255, cv::THRESH_BINARY_INV);
        std::vector<VecPoint> contour;
        cv::findContours(BinImg, contour, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if (!contour.empty())
        {
            auto Maxcontour = std::max_element(contour.begin(), contour.end(), [](auto const& c1, auto const& c2)
                {
                    double s1 = cv::contourArea(c1);
                    double s2 = cv::contourArea(c2);
                    return s1 < s2;
                });
            cv::Rect bounding_box = cv::boundingRect(*Maxcontour);
            resultHeight = bounding_box.height;
            double area_contour = cv::contourArea(*Maxcontour);
            resultRatioArea = area_contour / (double)(PoleROI.height * PoleROI.width);

            BlackRegion = std::move(*Maxcontour);
            for (auto& p : BlackRegion)
            {
                p += PoleROI.tl();
            }

            iResult = (resultRatioArea > mRatioAreaThreshold && resultHeight > mHeightThreshold) ? ERR_CODE::NG : ERR_CODE::OK;
        }
        else
        {
            iResult = ERR_CODE::OK;
        }
    }
    else
    {
        iResult = ERR_CODE::errCheckBlackCloud;
    }
    return iResult;
}
ERR_CODE CylinderBatteryLower::FindAllPolesPos(const cv::Mat& rotatedImg, cv::Rect& poleRegionROI, VecInt& lstPolePosXAll, VecInt& anodePos, VecInt& cathodePos, std::string& descript, bool isExtraROI)
{
    //Extra battery roi for fix black boundary pole when CLAHE image
    int borderPadding = isExtraROI ? round(poleRegionROI.width * 0.02) : 0;
    cv::Rect batteryROIExtra = cv::Rect(poleRegionROI.x - borderPadding, poleRegionROI.y, poleRegionROI.width + borderPadding * 2, poleRegionROI.height);
    RefineROI(batteryROIExtra, rotatedImg.size());
    cv::Mat src = rotatedImg(batteryROIExtra).clone();
    if (src.empty())
    {
        descript = "Cannot find suitable cathode line, please check Cathode line parameter in setting";
        return ERR_CODE::errPoleCathodeLine2;
    }

    if (lstPolePosXAll.size() > 0) lstPolePosXAll.clear();
    if (anodePos.size() > 0) anodePos.clear();
    if (cathodePos.size() > 0) cathodePos.clear();

    cv::Mat imgROI4_0, imgCLAHE;
    cv::bilateralFilter(src, imgROI4_0, 5, 55, 5);

    //Preprocess step for Anode tracking
    /*cv::bilateralFilter(rotatedImg(poleRegionROI), imageROI4, 5, 55, 5);
    cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 3), cv::Point(-1, -1));*/
    //cv::morphologyEx(imageROI4, imageROI4, cv::MORPH_ERODE, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);			
    //xv::LDR(imageROI4, imageROI4, 5);

    cv::Ptr<cv::CLAHE> claheTool4Cathode = cv::createCLAHE(15, cv::Size(75, imgROI4_0.rows));
    claheTool4Cathode->apply(imgROI4_0, imgCLAHE);

    // Use poleRegionROI as caculation
    cv::Rect batteryROI_0 = (batteryROIExtra.width == rotatedImg.cols && batteryROIExtra.height == rotatedImg.rows) ? batteryROIExtra : cv::Rect(borderPadding, 0, poleRegionROI.width, poleRegionROI.height);
    imgROI4_0 = imgROI4_0(batteryROI_0).clone();
    imgCLAHE = imgCLAHE(batteryROI_0).clone();
    src = src(batteryROI_0).clone();
    cv::Size imgSize = imgROI4_0.size();

    if (mDisplayMode == DisplayMode::DALL)
    {
        show_window("Anode Roi", ~imgCLAHE);
    }

    VecPoint vtRoi4ContourAll, vecAllCath;

    ERR_CODE errFindCathodeLine = FindCathodeLine(src, vecAllCath, descript);
    if (errFindCathodeLine != ERR_CODE::OK)
    {
        return errFindCathodeLine;
    }

    // 6: From Anode ending lines, select a sub-ROI 4.0 (mask) for detecting poles position
    cv::Mat mask = cv::Mat::zeros(imgROI4_0.size(), imgROI4_0.type());

    VecPoint outerPoints;
    outerPoints.push_back(cv::Point(0, 0));
    outerPoints.push_back(cv::Point(imgROI4_0.cols - 1, 0));
    int cathodeShiftness = 1;

    int poleHeight = (mValidCathode2AnodeRange.GetLower() / mPixelSize + mValidCathode2AnodeRange.GetUpper() / mPixelSize) / 2;
    poleHeight = std::max(30, std::min(poleHeight, 60));

    vtRoi4ContourAll = refineMaskingContour(vecAllCath, imgROI4_0.cols, cathodeShiftness, poleHeight, outerPoints);
    if (vtRoi4ContourAll.size() < imgSize.width) {
        descript = "Generated sample of cathode line is not correct, please check Cathode line parameter in setting";
        return ERR_CODE::errPoleCathodeLine6;
    }
    drawContours(mask, std::vector<VecPoint>(1, vtRoi4ContourAll), -1, cv::Scalar::all(255), -1);

    std::for_each(vtRoi4ContourAll.begin(), vtRoi4ContourAll.end(), [&cathodeShiftness](cv::Point& p) { p.y += cathodeShiftness; });

    //if (mDisplayMode == DisplayMode::DALL)
    //{
    //    Utils::show_window("Contrast enhanced", claheImage);
    //}
    cv::Mat tmpDst = cv::Mat::zeros(imgROI4_0.size(), CV_8UC1);
    imgCLAHE.copyTo(tmpDst, mask);

    if (mDisplayMode == DisplayMode::DALL) { show_window("masked img", tmpDst); }

    cv::Point extBottom = *std::max_element(vtRoi4ContourAll.begin(), vtRoi4ContourAll.end(),
        [](const cv::Point& lhs, const cv::Point& rhs) {
            return lhs.y < rhs.y;
        });

    // 8: Select the sub-ROI 3.1 for pole analysis
    // 9: Find the cumulative function vertically 
    // 10: The position of each pole is defined as the lowest extreme point in each period (dark poles)
    ERR_CODE errFindXCathode = FindCathodeXPosition(tmpDst, lstPolePosXAll);
    if (errFindXCathode != ERR_CODE::OK)
    {
        descript = "Cannot detect enough pole, please check Poles Height or Pole Detection Threshold parameter in setting or wrong input image";
        return errFindXCathode;
    }

    //Merge 2 list of pole in 2 side to a final list
    int lstPoleSize = lstPolePosXAll.size();
    if (lstPoleSize < 5 || (mIsCheckPoleNo && lstPoleSize < mOneSidePoleNumber / 2)) {
        descript = "Cannot detect any pole, please check input image or poleDetectionThreshold param in setting";
        return ERR_CODE::errPoleXDetection;
    }

    // ===============================Find cathode Y position        ==================================
    VecPoint CathodePnts;
    std::for_each(lstPolePosXAll.begin(), lstPolePosXAll.end(), [&vecAllCath, &CathodePnts, &cathodePos](const float& pnt) {CathodePnts.push_back(vecAllCath[pnt]); cathodePos.push_back(vecAllCath[pnt].y); });

    if (mInspectingItems.ANODE_TO_CASE_GAP || mInspectingItems.ANODE_TO_CATHODE_LENGTH)
    {
        //=============== Find Cathode Y position ===================
        VecInt missingPole;
        VecFloat roiMean;
        cv::Mat anodeDebugImage;
        cv::cvtColor(imgCLAHE, anodeDebugImage, cv::COLOR_GRAY2BGR);
        cv::Mat erasePoleImage = anodeDebugImage.clone();
        std::vector<listPole> listAllPoles;
        ERR_CODE errFindAthode = FindAnode(imgCLAHE, anodePos, CathodePnts, listAllPoles, anodeDebugImage);

        if (errFindAthode != ERR_CODE::OK)
        {
            descript = "Cannot detect anode y position, please check Anode Thresold Inner, Midle, Outer parameter in setting or wrong input image";
            return errFindAthode;
        }

        int offsetROI = 20;
        for (auto& p : CathodePnts)
        {
            //Calculate local mean of pole ROIs with height = pole length///
            int poleROIHeight = cathodePos.back();
            if (poleROIHeight != 0)
            {
                cv::Rect poleROI = cv::Rect(p.x - 1, offsetROI, 3, poleROIHeight);
                if (true == RefineROI(poleROI, imgROI4_0.size()))
                {
                    cv::Scalar roiMeanCalc = cv::mean(imgROI4_0(poleROI));
                    roiMean.push_back(roiMeanCalc[0]);
                }
                else
                {
                    roiMean.push_back(0);
                }
            }
            else roiMean.push_back(0);
        }

        if (mDisplayMode == DisplayMode::DALL) { cv::namedWindow("Find Anode Y Cord Upper", cv::WINDOW_NORMAL); show_window("Find Anode Y Cord Upper", anodeDebugImage); }


    #if 0 // DEBUG_DOMINANT_POLES
        //Check Tab Poles
        cv::Mat drawTabPoles = imageROI4.clone();
        cv::cvtColor(drawTabPoles, drawTabPoles, cv::COLOR_GRAY2BGR);
        std::vector<std::string> roiText;
        roiText = CylinderBatteryLower::debugTabPoles(drawTabPoles, roiMean, imageROI4.cols / 2, lstPolePosXAll, cathodePos, outFileName);
        //CylinderBatteryLower::write(roiText, "checkTabPoles.txt");
        //End Checking Tab Poles 
        /////////////////////////////////////////////
    #endif // DEBUG_DOMINANT_POLES

        VecInt listindexRemove;
        VecInt lstPolePosXAllBeforeRemove = lstPolePosXAll;

        // 13: Check tab appearence detected as poles
        if (roiMean.size() == cathodePos.size()) {
            VecInt deleteDominantPole = CheckDominantPoles(anodePos, cathodePos, roiMean, lstPolePosXAll, imgSize.width / 2);
            if (!deleteDominantPole.empty()) {
                for (int it = deleteDominantPole.size() - 1; it >= 0; it--) {

                    int index = deleteDominantPole[it];

                    anodePos.erase(anodePos.begin() + index);
                    cathodePos.erase(cathodePos.begin() + index);
                    lstPolePosXAll.erase(lstPolePosXAll.begin() + index);
                    listindexRemove.push_back(index);
                }
            }
        }
        if (/*mEnableAutoMode || mSkipPolesDistance == */0) {
            std::pair<int, int> result = findUnstableRegion(lstPolePosXAllBeforeRemove, lstPolePosXAll, anodePos, cathodePos, listindexRemove, imgCLAHE.cols, mNumberRemovePoleAuto, erasePoleImage);
            poleRegionROI.x += result.first;
            if (result.second == 0) {
                result.second = poleRegionROI.width;
            }
            poleRegionROI.width = result.second - result.first;
            //startingPoleRegion = startingPoleRegion + poleRegionROI.x;
            for (int i = 0; i < lstPolePosXAll.size(); i++) lstPolePosXAll[i] -= result.first;
        }
        //End checking tab appearence detected as poles
        // 
        ///////////////////////////////////////////////
        float meanAnode = cv::mean(anodePos)[0];
        cv::Rect subRoi5 = CreateROI(cv::Rect(0, 0, imgCLAHE.cols, meanAnode), imgCLAHE.size());

        if (mIsCheckPoleTAB == 1
            && !subRoi5.empty()
            && !lstPolePosXAll.empty()
            && !cathodePos.empty()
            && !anodePos.empty()
            && !imgCLAHE.empty())
        {
            std::vector<float> reduceVecRows;            
            cv::reduce(imgCLAHE(subRoi5), reduceVecRows, 0, cv::REDUCE_AVG);
            cv::blur(reduceVecRows, reduceVecRows, cv::Size(10, 1));
            if (!reduceVecRows.empty()) {
                std::vector<std::pair<int, int>> lstTab = FindTabs(reduceVecRows);
                /*for (auto p : lstTab) {
                    cv::line(anodeDebugImage, cv::Point(p.first, 0), cv::Point(p.first, anodeDebugImage.rows), cv::Scalar(0, 0, 255), 2);
                    cv::line(anodeDebugImage, cv::Point(p.second, 0), cv::Point(p.second, anodeDebugImage.rows), cv::Scalar(0, 0, 255), 2);
                }*/
                if (!lstTab.empty())
                {
                    std::vector<double>  anode2cathode_mm;
                    for (int i = 0; i < lstPolePosXAll.size(); i++) {
                        auto mAnode2Cathode_mm = abs(anodePos[i] - cathodePos[i]) * mPixelSize;
                        anode2cathode_mm.push_back(mAnode2Cathode_mm);
                    }

                    int paddingTab = 7;
                    std::vector<int> IdxpoleDelete;
                    for (int i = 0; i < lstPolePosXAll.size(); i++) {
                        for (auto p : lstTab) {
                            if (lstPolePosXAll[i] + paddingTab >= p.first
                                && lstPolePosXAll[i] - paddingTab <= p.second
                                && anode2cathode_mm[i] > mValidCathode2AnodeRange.GetUpper())
                            {
                                    IdxpoleDelete.push_back(i);
                            }                      
                        }                
                    }

                    if (IdxpoleDelete.size() != lstPolePosXAll.size() && !IdxpoleDelete.empty()) {
                        for (int it = IdxpoleDelete.size() - 1; it >= 0; it--) {
                            int index = IdxpoleDelete[it];

                            anodePos.erase(anodePos.begin() + index);
                            cathodePos.erase(cathodePos.begin() + index);
                            lstPolePosXAll.erase(lstPolePosXAll.begin() + index);
                        }
                    }
                }
            }
                
        }
        //Finding unstable region

        if (mDisplayMode == DisplayMode::DALL)
        {
            cv::cvtColor(imgCLAHE, imgCLAHE, cv::COLOR_GRAY2BGR);
            show_window("Roi4_Result", imgCLAHE);
        }

    #if USE_POLE_REFINEMENT
        // 14: Refine incorrect measurement by using symmetry property
        std::string strRefinementResult;
        ERR_CODE resRefinement = PoleLengthRefinement(lstPolePosXAll, anodePos, cathodePos, imgCLAHE.cols, strRefinementResult, cv::Mat());

        if (resRefinement != ERR_CODE::OK) {
            descript = strRefinementResult;
            return ERR_CODE::errPoleRefinement;
        }
    #endif // USE_POLE_LENGTH_REFINE
    }
    else
    {
        anodePos.clear();
        anodePos.insert(anodePos.begin(), cathodePos.size(), 0);
    }

    return ERR_CODE::OK;
}

std::vector<std::pair<int, int>> CylinderBatteryLower::FindTabs(const std::vector<float>& Signal)
{
    std::vector<std::pair<int, int>> listTab;
    if (Signal.empty())
    {
        return listTab;
    }
    int IdxleftEnd = (int)((Signal.size() - mCenterNeglectionWidth) >> 1);
    int IdxRightStart = (int)((Signal.size() + mCenterNeglectionWidth) >> 1);
    std::vector<float> valleyDiff(Signal.size(), 0);
    std::vector<float> peakDiff = valleyDiff;
    for (int i = 1; i < Signal.size(); i++)
    {
        if (i < IdxleftEnd || i > IdxRightStart)
        {
            float value = Signal[i] - Signal[i - 1];
            if (value > 0)
            {
                peakDiff[i] = value;
            }
            else
            {
                valleyDiff[i] = abs(value);
            }
        }          
    }
    int idxMid = Signal.size() / 2;
        
    SegmentMeans means = calc_mean_region(Signal, mCenterNeglectionWidth);
    // Find the iterator pointing to the maximum element
    auto maxLeftElementIterator = std::max_element(valleyDiff.begin(), valleyDiff.begin() + idxMid);
    int maxLeftBeginIndex = std::distance(valleyDiff.begin(), maxLeftElementIterator);
    auto maxLeftElementIteratorPeak = std::max_element(peakDiff.begin() + maxLeftBeginIndex, peakDiff.begin() + idxMid);
    int maxLeftEndIndex = std::distance(peakDiff.begin(), maxLeftElementIteratorPeak);
    if (maxLeftEndIndex - maxLeftBeginIndex > 8)
    {
        ERR_CODE isTableft = CheckIsTab(Signal, { maxLeftBeginIndex , maxLeftEndIndex }, means.meanLeftOuter, means.meanLeftMiddle, means.meanLeftInner);
        if (isTableft == ERR_CODE::OK)
        {
            listTab.emplace_back(maxLeftBeginIndex, maxLeftEndIndex);
        }
    }

    // Find the iterator pointing to the maximum element
    auto maxRightElementIterator = std::max_element(valleyDiff.begin() + idxMid, valleyDiff.end());
    int maxRightBeginIndex = std::distance(valleyDiff.begin(), maxRightElementIterator);
    auto maxRightElementIteratorPeak = std::max_element(peakDiff.begin() + maxRightBeginIndex, peakDiff.end());
    int maxRightEndIndex = std::distance(peakDiff.begin(), maxRightElementIteratorPeak);
    if (maxRightEndIndex - maxRightBeginIndex > 8)
    {
        ERR_CODE isTabRight = CheckIsTab(Signal, { maxRightBeginIndex , maxRightEndIndex }, means.meanRightOuter, means.meanRightMiddle, means.meanRightInner);
        if (isTabRight == ERR_CODE::OK) {
            listTab.emplace_back(maxRightBeginIndex, maxRightEndIndex);
        }
    }

    return listTab;
}

ERR_CODE CylinderBatteryLower::CheckIsTab(const std::vector<float>& Signal, const std::vector<int> PVIdx, float thresh_inner, float thresh_middle, float thresh_outer) {
    if (Signal.size() > 8 && !PVIdx.empty()) 
    {
        ERR_CODE result = ERR_CODE::OK;
        // Take 6px to the right if it is a valley
        int startExtend = 1;
        int endExtend = 7;
        
        for (auto x : PVIdx) 
        {
            int length = endExtend - startExtend;
            if (x + startExtend < 0 || x + endExtend > Signal.size() - 1 )
            {   
                startExtend = 0;
                endExtend = 0;
                length = 1;
            }          
            float mean = std::accumulate(Signal.begin() + x + startExtend, Signal.begin() + x + endExtend, 0.0) / length;
            int regionIdx = CheckRegion(x, 6, Signal.size(), mCenterNeglectionWidth);
            int Tabthreshold = 0;
            switch (regionIdx)
            {
            case 1:
                Tabthreshold = thresh_outer;
                break;
            case 2:
                Tabthreshold = thresh_middle;
                break;
            case 3:
                Tabthreshold = thresh_inner;
                break;
            default:
                Tabthreshold = thresh_middle;
                break;
            }
            if (mean > Tabthreshold) {
                result = ERR_CODE::NG;
            }
           // Take 6px to the left if it is a peak
            startExtend = -7;
            endExtend = -1;
        }
        return result;
    }
    else
    {
        return ERR_CODE::NG;
    }
}

ERR_CODE CylinderBatteryLower::FindAnode(const cv::Mat& inputImg, VecInt& vtAnodePos, VecPoint const& vtStartPoints, std::vector<listPole>& listAllPoles, cv::Mat& drawOuput) {
    if (inputImg.empty() || vtStartPoints.empty())
        return ERR_CODE::errPoleAnodeLine;

    vtAnodePos.clear();
    int anodeYPos;
    int stepHorizontal = 3;
    int stepVertical = 6;
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

            //Check if pole is within outer region to restrict movement
            bool isRestrictMove = (regionIdx == 1);

            //cv::Mat lookUpTable(1, 256, CV_8U);
            //uchar* p = lookUpTable.ptr();
            //for (int iGam = 0; iGam < 256; iGam++)
            //	p[iGam] = cv::saturate_cast<uchar>(pow(iGam / 255.0, 0.67) * 255.0);
            //cv::Mat gammaCorrect = imageROI4.clone();
            //LUT(imageROI4, lookUpTable, gammaCorrect);

            anodeYPos = findAnodeByLineTracing(inputImg, vtStartPoints[poleIdx], vtStartPoints[poleIdx].y, stepHorizontal, stepVertical, aThreshold * 10, drawOuput,
                0, 0, isRestrictMove, moveAlow);
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

    if (algoType == anodeAlgoType::LineTracing) {
        RecheckDisconectAnode(inputImg, drawOuput, vtStartPoints, vtAnodePos, stepVertical);
    }
    return  ERR_CODE::OK;
}

ERR_CODE CylinderBatteryLower::FindCathodeLine(const cv::Mat& srcImg, VecPoint& lstCathodeLine, std::string& descript)
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
            const int defaultWindowSize = 15;
            const int minWindowSize = 1;
            int wSize = mCathodeLineWindowSize < minWindowSize ? defaultWindowSize : mCathodeLineWindowSize;
            int lIdx = 0;
            VecDouble lstCathLine = CylinderBatteryLower::FindCathodeLineByPeak(srcImg);
            if (!CheckCathodeLine(lstCathLine)) {
                descript = "Cannot find suitable cathode line, please check ROI/Cathode line parameter in setting";
                return ERR_CODE::errPoleCathodeLine1;
            }
            VecPoint lstCathodePnts;
            for (int pIdx = 0; pIdx < lstCathLine.size(); pIdx++) {

                lstCathodePnts.push_back(cv::Point((pIdx + 0.5) * wSize, lstCathLine[pIdx]));
            }

#if USE_POLE_REFINEMENT
            CylinderBatteryLower::LineRefinement(lstCathodePnts, srcImg.cols);
#endif // USE_POLE_REFINEMENT

            VecDouble lstCathLineRefined;
            std::for_each(lstCathodePnts.begin(), lstCathodePnts.end(), [&lstCathLineRefined](cv::Point& p) {lstCathLineRefined.push_back(p.y); });

            VecDouble vecSmoothedCath;
            int gaussSigma = 1;
            int sampleWindow = 5;

            if (lstCathLineRefined.size() < sampleWindow) {
                descript = "Cannot find suitable cathode line, please check Cathode line parameter in setting";
                return ERR_CODE::errPoleCathodeLine3;
            }

            //Smoothing results
            vecSmoothedCath = gaussSmoothen(lstCathLineRefined, gaussSigma, sampleWindow);
            if ((vecSmoothedCath.size() < (srcImg.cols / wSize))) {
                descript = "Cannot find suitable cathode line, please check Cathode line parameter in setting";
                return ERR_CODE::errPoleCathodeLine4;
            }

            lstCathodeLine.reserve(srcImg.cols);
            for (int pIdx = 0; pIdx < srcImg.cols; pIdx++) {
                lstCathodeLine.push_back(cv::Point(pIdx, vecSmoothedCath[pIdx / wSize]));
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
        }
    }

    if (lstCathodeLine.size() < srcImg.cols) {
        descript = "Cannot generate enough sample of cathode line, please check Cathode line parameter in setting";
        return ERR_CODE::errPoleCathodeLine5;
    }

    return  ERR_CODE::OK;
}

VecDouble CylinderBatteryLower::FindCathodeLineByPeak(const cv::Mat& srcImage) {
    if (srcImage.empty() || srcImage.rows < 5) {
        return VecDouble();
    }
    cv::Mat src;
    cv::medianBlur(srcImage, src, 3);

    cv::Size imgSize = src.size();
    int wSize = mCathodeLineWindowSize;
    if (wSize <= 0) wSize = 5;
    if (wSize >= imgSize.width) wSize = imgSize.width - 1;
    int wIdx = 0;
    cv::Mat tmpImg, Hcumulative;
    VecDouble lstCathodeLine;
    int idx = 0;
    float localThresh = 5.0;// = 2.0;
    int leftmodeidx = -1;
    int rightmodeidx = -1;
    int indexleft = -1;
    int indexright = -1;
    int index = 0;
    bool center = false;;
    cv::Mat prominenceImage;
    if (mDisplayMode == DisplayMode::DALL) { prominenceImage = cv::Mat::zeros(src.size(), CV_8UC3); }

    while (wIdx + wSize < imgSize.width)
    {
        idx = lstCathodeLine.size() > 0 ? lstCathodeLine.back() : 0;
        int regionIdx = GetRegionIndex(wIdx, 6, imgSize.width, mCenterNeglectionWidth);
        if (regionIdx > 0) {
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
                cv::reduce(tmpImg, Hcumulative, 1, cv::REDUCE_AVG, CV_32FC1);

                VecFloat reduceVec = cv::Mat_<float>(Hcumulative);
                VecFloat list = reduceVec;
                int vecSize = reduceVec.size() - 1;
                for (int i = 2; i < reduceVec.size() - 2; i++) {
                    reduceVec[i] = 0.1 * list[i - 2] + 0.25 * list[i - 1] + 0.3 * list[i] + 0.25 * list[i + 1] + 0.1 * list[i + 2];
                }
                // extern 2 element this will not skip 2 last element in filter
                reduceVec[0] = reduceVec[2];
                reduceVec[1] = reduceVec[2];
                reduceVec[vecSize] = reduceVec[vecSize - 2];
                reduceVec[vecSize - 1] = reduceVec[vecSize - 2];

                VecFloat diffVec;

                diffVec.push_back(0);
                for (int i = 1; i < vecSize; i++)
                {
                    float diff = reduceVec[i + 1] - reduceVec[i - 1];
                    diff = diff > 0 ? 0 : diff;
                    diff = -pow(diff, 2);
                    diffVec.push_back(diff);
                }
                diffVec.push_back(0);

                FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
                findPeaks.Process(diffVec);
                std::vector<Peak> peaks = findPeaks.GetPeakResult(localThresh);

                if (!peaks.empty())
                {
                    idx = peaks.back().index;

                    const float percentageCalue = 0.3f;
                    auto currentPeak = peaks.back();
                    float correctValue = currentPeak.value - currentPeak.prominence * percentageCalue;

                    int /*topIdx = 0, */bottomIdx = imgSize.height - 1;
                    //find bottomIdx
                    for (int id = currentPeak.index + 1; id < imgSize.height; id++)
                    {
                        if (diffVec[id] > correctValue)
                        {
                            bottomIdx = id;
                            break;
                        }
                    }

                    //find topIdx
                    /*for (int id = currentPeak.index - 1; id > 0; id--)
                    {
                        if (diffVec[id] > correctValue)
                        {
                            topIdx = id;
                            break;
                        }
                    }*/

                    if (/*topidx != 0 &&*/ bottomIdx != imgSize.height - 1)
                    {
                        /*if(abs(topidx - idx) < abs(bottomidx - idx)) idx = topidx;
                        else idx = bottomidx;*/
                        idx = bottomIdx;
                    }

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
        }

        if (regionIdx <= 0 && rightmodeidx == -1 && leftmodeidx == -1)
        {
            leftmodeidx = idx;
            if (index > 1)
            {
                indexleft = index - 1;
            }
        }
        if (regionIdx > 0 && leftmodeidx != -1 && rightmodeidx == -1)
        {
            rightmodeidx = idx;
            indexright = index;
        }

        lstCathodeLine.push_back(idx);
        index++;
        wIdx += wSize;
    }


    if (lstCathodeLine.size() <= 0) {
        return VecDouble();
    }
    int poleHeight = (mValidCathode2AnodeRange.GetLower() / mPixelSize + mValidCathode2AnodeRange.GetUpper() / mPixelSize) / 2;
    poleHeight = std::max(30, std::min(poleHeight, 60));

    if (std::abs(leftmodeidx - rightmodeidx) >= poleHeight * 0.75)
    {
        if (indexleft > 0 && indexright < lstCathodeLine.size() - 1)
        {
            float left = std::abs(lstCathodeLine[indexleft - 1] - leftmodeidx);
            float right = std::abs(lstCathodeLine[indexright + 1] - rightmodeidx);
            for (int i = indexleft + 1; i < indexright; i++)
            {
                lstCathodeLine[i] = (left < right) ? leftmodeidx : rightmodeidx;
            }
        }
    }

    if (mDisplayMode == DisplayMode::DALL)
    {
        cv::namedWindow("Prominence Image", cv::WINDOW_NORMAL);
        show_window("Prominence Image", prominenceImage);
    }

    lstCathodeLine.push_back(lstCathodeLine.back());


    int borderNeglction = 2;
    int neglectionWidth = borderNeglction * wSize;
    // start - end column of neglection area
    int startNeglectCol = (imgSize.width - mCenterNeglectionWidth) >> 1;
    int endNeglectCol = (imgSize.width + mCenterNeglectionWidth) >> 1;
    int rightEndBounder = imgSize.width - neglectionWidth;


    AverageCathodeLineResult<double> avgCathodeLineResult =
        FindAverageCathodeLine(lstCathodeLine, neglectionWidth, startNeglectCol, endNeglectCol, rightEndBounder, 11, wSize);

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
        cv::Mat drawCathodeLineImg = srcImage.clone();
        cv::Mat enhanced = src.clone();
        cv::cvtColor(drawCathodeLineImg, drawCathodeLineImg, cv::COLOR_GRAY2BGR);
        cv::cvtColor(enhanced, enhanced, cv::COLOR_GRAY2BGR);
        while (X + wSize <= srcImage.cols)
        {
            cv::line(enhanced, cv::Point(X, lstCathodeLine[X / wSize]), cv::Point(X + wSize, lstCathodeLine[X / wSize]), cv::Scalar(0, 0, 255), 1);
            cv::line(drawCathodeLineImg, cv::Point(X, lstCathodeLine[X / wSize]), cv::Point(X + wSize, lstCathodeLine[X / wSize]), cv::Scalar(0, 0, 255), 1);
            X += wSize;
        }
#endif

        return lstCathodeLine;
    }
    else
    {
        return VecDouble();
    }
}

#ifdef DEBUG_DOMINANT_POLES
std::vector<std::string> CylinderBatteryLower::debugTabPoles(const cv::Mat& drawImg, VecFloat meanROIs, int center, VecInt xCoords, VecInt yCoords, std::string fileName) {
    int noPoleCheck = 5;
    std::vector<std::string> roiText;
    for (int i = 0; i < xCoords.size(); i++) {
        if (xCoords[i] - center >= 0) {
            int first = 0;
            int leftMid = i - 1;
            int rightMid = i;
            int last = xCoords.size() - 1;

            //Calculate sum intensity of nearby respective poles
            float sumFirst = std::accumulate(meanROIs.begin() + 1, meanROIs.begin() + noPoleCheck + 1, 0.0);
            float sumLeftMid = std::accumulate(meanROIs.begin() + (leftMid - noPoleCheck), meanROIs.begin() + leftMid, 0.0);
            float sumRightMid = std::accumulate(meanROIs.begin() + rightMid + 1, meanROIs.begin() + rightMid + noPoleCheck + 1, 0.0);
            float sumLast = std::accumulate(meanROIs.begin() + (last - noPoleCheck), meanROIs.begin() + last, 0.0);

            //Calculate average intensity of nearby respective poles
            float avgFirst = sumFirst / (float)noPoleCheck;
            float avgLeftMid = sumLeftMid / (float)noPoleCheck;
            float avgRightMid = sumRightMid / (float)noPoleCheck;
            float avgLast = sumLast / (float)noPoleCheck;

            //Calculate intensity of respective poles
            float roiFirst = meanROIs[first];
            float roiLeftMid = meanROIs[leftMid];
            float roiRightMid = meanROIs[rightMid];
            float roiLast = meanROIs[last];

            //starting point of respective poles
            cv::Point firstPoint(xCoords[first], yCoords[first]);
            cv::Point leftMidPoint(xCoords[leftMid], yCoords[leftMid]);
            cv::Point rightMidPoint(xCoords[rightMid], yCoords[rightMid]);
            cv::Point lastPoint(xCoords[last], yCoords[last]);

            cv::circle(drawImg, firstPoint, 1, cv::Scalar(0, 255, 0));
            cv::circle(drawImg, leftMidPoint, 1, cv::Scalar(0, 255, 0));
            cv::circle(drawImg, rightMidPoint, 1, cv::Scalar(0, 255, 0));
            cv::circle(drawImg, lastPoint, 1, cv::Scalar(0, 255, 0));

            cv::putText(drawImg, "1", cv::Point(firstPoint.x, firstPoint.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
            cv::putText(drawImg, "2", cv::Point(leftMidPoint.x, leftMidPoint.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
            cv::putText(drawImg, "3", cv::Point(rightMidPoint.x, rightMidPoint.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
            cv::putText(drawImg, "4", cv::Point(lastPoint.x, lastPoint.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

            cv::putText(drawImg, "1.First Pole: " + std::to_string(roiFirst) + "    NEAR POLES: " + std::to_string(avgFirst), cv::Point(50, 150), 0, 0.5, cv::Scalar(0, 255, 0));
            cv::putText(drawImg, "2.LeftMid Pole: " + std::to_string(roiLeftMid) + "    NEAR POLES: " + std::to_string(avgLeftMid), cv::Point(50, 175), 0, 0.5, cv::Scalar(0, 255, 0));
            cv::putText(drawImg, "3.RightMid Pole: " + std::to_string(roiRightMid) + "    NEAR POLES: " + std::to_string(avgRightMid), cv::Point(50, 200), 0, 0.5, cv::Scalar(0, 255, 0));
            cv::putText(drawImg, "4.Last Pole: " + std::to_string(roiLast) + "    NEAR POLES: " + std::to_string(avgLast), cv::Point(50, 225), 0, 0.5, cv::Scalar(0, 255, 0));

            std::string inputText = fileName + "  " + std::to_string(roiFirst) + "  " + std::to_string(avgFirst) + " , "
                + std::to_string(roiLeftMid) + "  " + std::to_string(avgLeftMid) + " , "
                + std::to_string(roiRightMid) + "  " + std::to_string(avgRightMid) + " , "
                + std::to_string(roiLast) + "  " + std::to_string(avgLast);
            roiText.push_back(inputText);
            return roiText;
            break;
        }
    }
    return roiText;
}
#endif //DEBUG_DOMINANT_POLES

VecInt CylinderBatteryLower::CheckDominantPoles(const VecInt& anodeVec, const VecInt& cathodeVec, const VecFloat& avgInt, const VecInt& vectorXPos, int center,
    float firstThresh, float leftMidThresh, float rightMidThresh, float lastThresh)
{
    //firstThresh = 0.8;
    //leftMidThresh = 0.85;
    //rightMidThresh = 0.83;
    //lastThresh = 0.78;
    VecInt resultDelete;
    if (vectorXPos.size() < 2) return resultDelete;

    int nPole = 5;
    int noPoleCheck = avgInt.size() < nPole * 2 ? avgInt.size() / 2 - 1 : nPole;

    auto avgVecbegin = avgInt.begin();
    int rightMidIndex = 0;
    for (int i = 0; i < vectorXPos.size(); i++) {
        if (vectorXPos[i] - center >= 0) {
            rightMidIndex = i;
            break;
        }
    }

    int first = 0;
    int leftMid = rightMidIndex - 1;
    int rightMid = rightMidIndex;
    int last = vectorXPos.size() - 1;

    if (leftMid <= 0 || leftMid < noPoleCheck || rightMid + noPoleCheck > last || rightMid >= vectorXPos.size() - 1)
        return resultDelete;

    //Calculate sum intensity of nearby respective poles
    float sumFirst = std::accumulate(avgVecbegin + 1, avgVecbegin + noPoleCheck + 1, 0.0);
    float sumLeftMid = std::accumulate(avgVecbegin + (leftMid - noPoleCheck), avgVecbegin + leftMid, 0.0);
    float sumRightMid = std::accumulate(avgVecbegin + rightMid + 1, avgVecbegin + rightMid + noPoleCheck + 1, 0.0);
    float sumLast = std::accumulate(avgVecbegin + (last - noPoleCheck), avgVecbegin + last, 0.0);

    //Calculate average intensity of nearby respective poles
    float avgFirst = sumFirst / (float)noPoleCheck;
    float avgLeftMid = sumLeftMid / (float)noPoleCheck;
    float avgRightMid = sumRightMid / (float)noPoleCheck;
    float avgLast = sumLast / (float)noPoleCheck;

    //Calculate intensity of respective poles
    float roiFirst = avgInt[first];
    float roiLeftMid = avgInt[leftMid];
    float roiRightMid = avgInt[rightMid];
    float roiLast = avgInt[last];

    if (roiFirst / avgFirst < firstThresh) {
        resultDelete.push_back(first);
    }
    if (roiLeftMid / avgLeftMid < leftMidThresh) {
        resultDelete.push_back(leftMid);
    }
    if (roiRightMid / avgRightMid < rightMidThresh) {
        resultDelete.push_back(rightMid);
    }
    if (roiLast / avgLast < lastThresh) {
        resultDelete.push_back(last);
    }

    return resultDelete;
}

void CylinderBatteryLower::CheckBoundaryTab(const VecFloat& poleHeight, VecInt& anodePos, int heightAvg, int numTabPoles)
{
    if (poleHeight.size() != anodePos.size() && anodePos.size() < 10)
        return;

    VecInt idxTabBoundaryLeft, idxTabBoundaryRight;
    for (int i = 0; i < poleHeight.size(); i++)
    {
        if (poleHeight[i] > heightAvg * 3 / 2)
        {
            if (i < numTabPoles)
            {
                idxTabBoundaryLeft.push_back(i);
            }
            else if (i > poleHeight.size() - numTabPoles - 1)
            {
                idxTabBoundaryRight.push_back(i);
            }
        }
    }

    VecInt idxPolesAvg;
    if (!idxTabBoundaryLeft.empty() && !idxTabBoundaryRight.empty()
        && (idxTabBoundaryLeft.size() != idxTabBoundaryRight.size()))
    {
        idxPolesAvg = idxTabBoundaryLeft.size() > idxTabBoundaryRight.size() ? idxTabBoundaryRight : idxTabBoundaryLeft;

        for (int i = 0; i < idxPolesAvg.size(); i++)
        {
            int replaceIndex = idxPolesAvg[i] > 0 ? idxPolesAvg[i] - 1 : 1;
            anodePos[idxPolesAvg[i]] = anodePos[replaceIndex];
        }
    }
    else if (idxTabBoundaryLeft.size() == idxTabBoundaryRight.size() && idxTabBoundaryLeft.size() == 1)
    {
        if (*idxTabBoundaryLeft.begin() == 0) anodePos[0] = anodePos[1];
        if (*idxTabBoundaryRight.begin() == anodePos.size() - 1) anodePos[*idxTabBoundaryRight.begin()] = anodePos[*idxTabBoundaryRight.begin() - 1];
    }
}

bool CylinderBatteryLower::CheckCathodeLine(const VecDouble& cathodeList, int cathodeYCordThresh, float OKPercentThresh)
{
    int size = cathodeList.size();
    int count = 0;
    for (int i = 0; i < size; i++) {
        if (cathodeList[i] < cathodeYCordThresh) {
            count++;
        }
    }
    float percentageError = size != 0 ? float(count) / float(size) : 1;
    if (percentageError > 1 - OKPercentThresh) {
        return false;
    }
    return true;
}
}
}