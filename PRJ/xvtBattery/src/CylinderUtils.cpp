#include "xvtBattery/CylinderUtils.h"
#include "xvtBattery/CylinderBatteryBase.h"
#include "xvtBattery/CylinderBatteryResult.h"
#include "xvtBattery/PoleInfo.h"
#include "xvtCV/Peak.h"
#include "xvtCV/Polyfit.h"
#include "xvtCV/Utils.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/xvtRange.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <numeric>
#include <algorithm>

namespace xvt
{
namespace battery
{

DECLARE_ENUM_MAP_DATA(ERR_CODE){{ERR_CODE::NA, "NA"},
                                {ERR_CODE::OK, "OK"},
                                {ERR_CODE::NG, "NG"},
                                {ERR_CODE::errImageEmpty, "errImageEmpty"},
                                {ERR_CODE::errImageFormat, "errImageFormat"},
                                {ERR_CODE::errBatterySize, "errBatterySize"},
                                {ERR_CODE::errWidthBattery, "errWidthBattery"},
                                {ERR_CODE::errBeadingArea, "errBeadingArea"},
                                {ERR_CODE::errBeadingInsp, "errBeadingInsp"},
                                {ERR_CODE::errBatteryROI, "errBatteryROI"},
                                {ERR_CODE::errJRFinding, "errJRFinding"},
                                {ERR_CODE::errReferenceCase, "errReferenceCase"},
                                {ERR_CODE::errCenterPin, "errCenterPin"},
                                {ERR_CODE::errCenterPinROI, "errCenterPinROI"},
                                {ERR_CODE::errCenterPinDepth, "errCenterPinDepth"},
                                {ERR_CODE::errPixelSize, "errPixelSize"},
                                {ERR_CODE::errPoleXDetection, "errPoleXDetection"},
                                {ERR_CODE::errPoleXDetection2, "errPoleXDetection2"},
                                {ERR_CODE::errPoleXDetection3, "errPoleXDetection3"},
                                {ERR_CODE::errPoleXDetection4, "errPoleXDetection4"},
                                {ERR_CODE::errPoleXDetection5, "errPoleXDetection5"},
                                {ERR_CODE::errPoleCathodeLine, "errPoleCathodeLine"},
                                {ERR_CODE::errPoleCathodeLine1, "errPoleCathodeLine1"},
                                {ERR_CODE::errPoleCathodeLine2, "errPoleCathodeLine2"},
                                {ERR_CODE::errPoleCathodeLine3, "errPoleCathodeLine3"},
                                {ERR_CODE::errPoleCathodeLine4, "errPoleCathodeLine4"},
                                {ERR_CODE::errPoleCathodeLine5, "errPoleCathodeLine5"},
                                {ERR_CODE::errPoleCathodeLine6, "errPoleCathodeLine6"},
                                {ERR_CODE::errPoleAnodeLine, "errPoleAnodeLine"},
                                {ERR_CODE::errPoleNotRefinable, "errPoleNotRefinable"},
                                {ERR_CODE::errPoleRefinement, "errPoleRefinement"},
                                {ERR_CODE::errROIRefinement, "errROIRefinement"}};
DECLARE_CONVERT_ENUM_FUNCS(ERR_CODE);

void show_window(const std::string &name, const cv::Mat &img, cv::WindowFlags flag)
{
    namedWindow(name, flag);
    imshow(name, img);
}

inline double gauss(double sigma, double x)
{
    double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
    double divider = sqrt(2 * CV_PI * pow(sigma, 2));
    return (1 / divider) * exp(expVal);
}

VecDouble gaussKernel(int samples, double sigma)
{
    VecDouble v;

    bool doubleCenter = false;
    if (samples % 2 == 0)
    {
        doubleCenter = true;
        samples--;
    }
    int steps = (samples - 1) / 2;
    double stepSize = (3 * sigma) / steps;

    for (int i = steps; i >= 1; i--)
    {
        v.push_back(gauss(sigma, i * stepSize * -1));
    }

    v.push_back(gauss(sigma, 0));
    if (doubleCenter)
    {
        v.push_back(gauss(sigma, 0));
    }

    for (int i = 1; i <= steps; i++)
    {
        v.push_back(gauss(sigma, i * stepSize));
    }
    assert(v.size() == samples);

    return v;
}

VecDouble weightedAvgSmoothen(VecDouble values)
{
    VecDouble output;
    int lstSize = values.size();
    if (lstSize > 2)
    {
        output.push_back(values.front());
        for (int idx = 1; idx < lstSize - 1; idx++)
        {
            output.push_back(0.2 * values[idx - 1] + 0.6 * values[idx] + 0.2 * values[idx + 1]);
        }
        output.push_back(values.back());
        return output;
    }
    else
        return values;
}

VecDouble gaussSmoothen(VecDouble values, double sigma, int samples)
{
    VecDouble out;
    if (values.size() < samples)
    {
        return out;
    }
    auto kernel = gaussKernel(samples, sigma);
    int sampleSide = samples / 2;
    double sumKernel = 0.0;
    for (int kIdx = 0; kIdx < kernel.size(); kIdx++)
    {
        sumKernel += kernel[kIdx];
    }
    double kernelScaleFactor = 1.0 / sumKernel;
    for (int kIdx = 0; kIdx < kernel.size(); kIdx++)
    {
        kernel[kIdx] *= kernelScaleFactor;
    }
    /*kernel[sampleSide] += (1.0 - sumKernel);*/

    int valueIdx = samples / 2 + 1;
    unsigned long ubound = values.size();
    VecDouble tmpValues(values);
    for (int i = 0; i < sampleSide; i++)
    {
        tmpValues.insert(tmpValues.begin(), values[ubound - i - 1]);
    }
    for (int i = 0; i < sampleSide; i++)
    {
        tmpValues.push_back(values[sampleSide * 2 - i]);
    }
    for (unsigned long i = sampleSide; i < ubound + sampleSide; i++)
    {
        double sample = 0;
        int sampleCtr = 0;
        for (long j = i - sampleSide; j <= i + sampleSide; j++)
        {
            if (j >= 0 && j < ubound + sampleSide * 2)
            {
                int sampleWeightIndex = sampleSide + (j - i);
                sample += kernel[sampleWeightIndex] * tmpValues[j];
                sampleCtr++;
            }
        }
        double smoothed = sample / (double)sampleCtr;
        //std::cout << " S: " << sample << " C: " << sampleCtr << " V: " << values[i] << " SM: " << smoothed << std::endl;
        out.push_back(sample);
    }
    return out;
}

VecPoint rotate180vtPoint(const VecPoint &vtP_In, cv::Size imgSize)
{
    VecPoint vtP_Rotated;
    for (auto pnt = vtP_In.rbegin(); pnt != vtP_In.rend(); ++pnt)
    {
        //Rotated a point around the center
        float pX = imgSize.width - 1 - pnt->x;
        float pY = imgSize.height - 1 - pnt->y;
        vtP_Rotated.push_back(cv::Point(pX, pY));
    }
    return vtP_Rotated;
}

cv::Point rotate180Point(const cv::Point &pntIn, cv::Size imgSize)
{
    cv::Point pntRotated;

    //Rotated a point around the center
    float pX = imgSize.width - 1 - pntIn.x;
    float pY = imgSize.height - 1 - pntIn.y;
    pntRotated = cv::Point(pX, pY);

    return pntRotated;
}

std::string GetFileName(std::string const &path)
{
    return path.substr(path.find_last_of("/\\") + 1);
}

std::size_t replace_all(std::string &inout, std::string what, std::string with)
{
    std::size_t count{};
    for (std::string::size_type pos{}; inout.npos != (pos = inout.find(what.data(), pos, what.length())); pos += with.length(), ++count)
    {
        inout.replace(pos, what.length(), with.data(), with.length());
    }
    return count;
}

int getMaxAreaContourId(const VecVecPoint &contours, int minSize)
{
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() > minSize)
        {
            double newArea = cv::contourArea(contours[i]);
            if (newArea > maxArea)
            {
                maxArea = newArea;
                maxAreaContourId = i;
            }
        }
    }
    return maxAreaContourId;
}
int getMaxRectangleId(const VecRect &listRect)
{
    int id = -1;
    auto idx = std::max_element(listRect.begin(), listRect.end(), [](const cv::Rect &lp, const cv::Rect &rp) { return lp.width < rp.width; });
    id = std::distance(listRect.begin(), idx);
    return id;
}

int getPointOnContourId(const VecPoint &contour, cv::Point pt)
{
    int index = -1;
    if (!contour.empty())
    {
        double minDistance = DBL_MAX;
        int contourSize = contour.size();
        for (int i = 0; i < contourSize; i++)
        {
            cv::Point tmp = contour[i] - pt;
            double distance = sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
            if (minDistance > distance)
            {
                minDistance = distance;
                index = i;
            }
        }
    }

    return index;
}

auto FindCornerPoints(VecPoint const &contour, cv::Point &tl, cv::Point &tr, cv::Point &br, cv::Point &bl) -> InspectionResult
{
    InspectionResult finalResult(EResult::OK);
    if (contour.empty())
    {
        finalResult(EResult::ER, "FindCornerPoints: Contour is empty!");
    }
    else
    {
        auto batRect = cv::boundingRect(contour);
        int idxTL = getPointOnContourId(contour, batRect.tl());
        int idxTR = getPointOnContourId(contour, cv::Point(batRect.br().x, batRect.y));
        int idxBR = getPointOnContourId(contour, batRect.br());
        int idxBL = getPointOnContourId(contour, cv::Point(batRect.x, batRect.br().y));

        tl = contour[idxTL];
        tr = contour[idxTR];
        br = contour[idxBR];
        bl = contour[idxBL];
    }
    return finalResult;
}

cv::Mat histDisplay(VecInt histogram, const char *name, int minTh, int maxTh)
{
    int hist[256];
    for (int i = 0; i < 256; i++)
    {
        hist[i] = histogram[i];
    }
    // draw the histograms
    int hist_w = 512;
    int hist_h = 400;
    int bin_w = cvRound((double)hist_w / 256);

    cv::Mat histImage(hist_h + 30, hist_w, CV_8UC3, cv::Scalar(255, 255, 255));
    int max_his = 255;
    // find the maximum intensity element from histogram
    int max = hist[0];
    for (int i = 1; i < max_his; i++)
    {
        if (max < hist[i])
        {
            max = hist[i];
        }
    }

    // normalize the histogram between 0 and histImage.rows
    for (int i = 0; i < max_his; i++)
    {
        hist[i] = ((double)hist[i] / max) * hist_h;
    }

    // draw the intensity line for histogram
    cv::Scalar colorLine = cv::Scalar(0, 0, 0);
    for (int i = 0; i < max_his; i++)
    {
        if (i == minTh || i == maxTh)
        {
            colorLine = COLOR_CV_RED;
            cv::putText(histImage, std::to_string(i), cv::Point(bin_w * (i)-7, hist_h + 10), 0.15, 0.25, colorLine);
        }
        else if (i % 50 == 0 && i != 0)
        {
            cv::putText(histImage, std::to_string(i), cv::Point(bin_w * (i)-7, hist_h + 10), 0.15, 0.25, colorLine);
        }
        cv::line(histImage, cv::Point(bin_w * (i), hist_h), cv::Point(bin_w * (i), hist_h - hist[i]), colorLine, 1, 8, 0);
        colorLine = cv::Scalar(0, 0, 0);
    }

    cv::putText(histImage, "Grey Levels", cv::Point(bin_w * (125) - 7, hist_h + 25), 0.25, 0.35, cv::Scalar(0, 0, 0));
    // display histogram
    //cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
    //cv::imshow(name, histImage);
    return histImage;
}

ERR_CODE PoleLengthRefinement(VecInt lstPolePosXAll,
                              VecInt &anodePos,
                              VecInt &cathodePos,
                              int widthROI,
                              std::string &strDesc)
{
    // For our simplicity, let�s call right poles as R(n), left poles as L(n).
    // The left pole starts from the left-most pole and the right pole from the right-most pole. n = 0, 1,�, N
    // To each pole, the coordinates of starting (bottom) and ending position
    // are written as Ls(n), and Le(n) for left poles. Rs(n) and Re(n) respectively
    VecPoint Ls, Le;
    VecPoint Rs, Re;
    VecInt xLeft, xRight;

    for (auto it = 0; it < lstPolePosXAll.size(); it++)
    {
        if (lstPolePosXAll[it] < widthROI / 2)
        {
            Ls.push_back(cv::Point(lstPolePosXAll[it], anodePos[it]));
            Le.push_back(cv::Point(lstPolePosXAll[it], cathodePos[it]));
            xLeft.push_back(lstPolePosXAll[it]);
        }
        else
        {
            Rs.push_back(cv::Point(widthROI - lstPolePosXAll[it], anodePos[it]));
            Re.push_back(cv::Point(widthROI - lstPolePosXAll[it], cathodePos[it]));
            xRight.push_back(widthROI - lstPolePosXAll[it]);
        }
    }
    std::sort(Rs.begin(), Rs.end(), [](auto &left, auto &right) { return left.x < right.x; });
    std::sort(Re.begin(), Re.end(), [](auto &left, auto &right) { return left.x < right.x; });
    std::sort(xRight.begin(), xRight.end(), [](auto &left, auto &right) { return left < right; });

    // Deciding the which points are more consistent (reliable)
    VecPoint PosXAll;


    int MaxDis = 14;
    //To_CHECK
    int MaxDiff = 10; //upper dang de 15. can check lai
    int matchIdx = 0;

    for (auto it = 0; it < xLeft.size(); it++)
    {
        matchIdx = findClosestPoint(xRight, xLeft[it], MaxDis);
        if (matchIdx < 0)
        {
            PosXAll.push_back(cv::Point(xLeft[it], Ls[it].y));
        }
        else
        {
            xRight.erase(xRight.begin() + matchIdx);
        }
    }
    for (int it = 0; it < xRight.size(); it++)
    {
        PosXAll.push_back(cv::Point(widthROI - xRight[it], 0));
    }

    VecDouble coeffRs;
    VecDouble coeffRe;
    VecDouble coeffLs;
    VecDouble coeffLe;
    double error = 0;
    int order = 5;
    float pInlier = 0.4;

    if (Rs.size() <= order || Re.size() <= order || Ls.size() <= order || Le.size() <= order)
    {
        strDesc = "Number of pole is less than 5 => cannot fiting model";
        return ERR_CODE::errPoleRefinement;
    }

    bool rtnRs = Polyfit::LSQFit(Rs, order, coeffRs, error);
    bool rtnRe = Polyfit::LSQFit(Re, order, coeffRe, error);

    float sumLeft = 0.0;

    int nL = Ls.size();
    std::for_each(Ls.begin(), Ls.end(), [&](cv::Point &p) { sumLeft += p.y; });
    int avgLs = sumLeft / nL;
    std::for_each(Ls.begin(), Ls.end(), [&](cv::Point &p) { p.y -= avgLs; });
    bool rtnLs = Polyfit::LSQFit(Ls, order, coeffLs, error);
    std::for_each(Ls.begin(), Ls.end(), [&](cv::Point &p) { p.y += avgLs; });

    nL = Le.size();
    sumLeft = 0.0;
    std::for_each(Le.begin(), Le.end(), [&](cv::Point &p) { sumLeft += p.y; });
    int avgLe = sumLeft / nL;
    std::for_each(Le.begin(), Le.end(), [&](cv::Point &p) { p.y -= avgLe; });
    bool rtnLe = Polyfit::LSQFit(Le, order, coeffLe, error);
    std::for_each(Le.begin(), Le.end(), [&](cv::Point &p) { p.y += avgLe; });

    for (int it = 0; it < PosXAll.size(); it++)
    {
        auto result1 = std::find_if(Ls.begin(), Ls.end(), [&](const cv::Point &p) { return p.x == PosXAll[it].x; });

        if (result1 != Ls.end())
        {
            int indexLs = std::distance(Ls.begin(), result1);


            if (rtnRs == true && rtnRe == true)
            {
                float xs = Ls[indexLs].x;
                float ys = Polyfit::f(coeffRs, xs);

                VecPoint::iterator lsIter;
                if (indexLs >= Rs.size())
                    lsIter = Rs.end();
                else
                    lsIter = Rs.begin() + indexLs;

                Rs.insert(lsIter, cv::Point(xs, ys));

                float xe = Le[indexLs].x;
                float ye = Polyfit::f(coeffRe, xe);

                VecPoint::iterator leIter;
                if (indexLs >= Re.size())
                    leIter = Re.end();
                else
                    leIter = Re.begin() + indexLs;

                Re.insert(leIter, cv::Point(xe, ye));
            }
            else
            {
                Ls.erase(result1);
                Le.erase(Le.begin() + indexLs);
            }
        }
        else
        {
            auto result2 =
                std::find_if(Rs.begin(), Rs.end(), [&widthROI, &PosXAll, &it](const cv::Point &p) { return p.x == (widthROI - PosXAll[it].x); });
            int indexRs = std::distance(Rs.begin(), result2);

            double error = 0;
            int order = 5;
            float pInlier = 0.4;

            //VecPoint output1(Ls.begin(), Ls.end());
            if (rtnLs == true && rtnLe == true)
            {
                float xs = Rs[indexRs].x;
                float ys = Polyfit::f(coeffLs, xs) + avgLs;

                VecPoint::iterator rsIter;
                if (indexRs >= Ls.size())
                    rsIter = Ls.end();
                else
                    rsIter = Ls.begin() + indexRs;

                Ls.insert(rsIter, cv::Point(xs, ys));

                float xe = Re[indexRs].x;
                float ye = Polyfit::f(coeffLe, xe) + avgLe;

                VecPoint::iterator reIter;
                if (indexRs >= Le.size())
                    reIter = Le.end();
                else
                    reIter = Le.begin() + indexRs;
                Le.insert(reIter, cv::Point(xe, ye));
            }
            else
            {
                Rs.erase(result2);
                Re.erase(Re.begin() + indexRs);
                //Re.erase(std::find_if(Re.begin(), Re.end(), [&PosXAll, it](const auto& p) { return p.x == PosXAll[it].x; }));
            }
        }
    }

    VecInt listCathErrorIndex, listAnodeErrorIndex;
    if (Ls.size() != Rs.size() || Le.size() != Re.size() || Ls.size() != Le.size())
        return ERR_CODE::errPoleNotRefinable;

    // Check which data is more reliable between the starting points or ending points
    // Plot Es(n) = Ls(0)-Rs(0), Ls(1)-Rs(1),� Ls(N)-Rs(N).
    VecInt Es;

    for (int i = 0; i < Ls.size(); i++)
    {
        Es.push_back(Ls[i].y - Rs[i].y);
    }

    // By the same way, check the Ee(n) = Le(0)-Re(0), Le(1)-Re(1),� Le(N)-Re(N).
    VecInt Ee;

    for (int i = 0; i < Le.size(); i++)
    {
        Ee.push_back(Le[i].y - Re[i].y);
    }
    // Symmetry Checking: Check the D_L(n) = Le(0) � Ls(0), � Le(N) � Ls(N). D_R(n) = Re(0) � Rs(0),�, Re(N) � Rs(N).
    VecInt D_L, D_R;

    for (int i = 0; i < Le.size(); i++)
    {
        D_L.push_back(Le[i].y - Ls[i].y);
        D_R.push_back(Re[i].y - Rs[i].y);
    }

    // And also Diff_LR(n) = D_L(n) � D_R(n).
    VecInt Diff_LR;

    for (int i = 0; i < D_L.size(); i++)
    {
        Diff_LR.push_back(D_L[i] - D_R[i]);
    }

    // Let call list error of Cathod is ErrorS, Anode is ErrorE;
    VecInt listEsIndex, listEeIndex;
    std::vector<bool> bErrorDiff;
    int NumDiff = Diff_LR.size();
    bool hasReliablePoles = false;
    for (int i = 0; i < NumDiff; i++)
    {
        if (std::abs(Diff_LR[i]) > MaxDiff)
        {
            bErrorDiff.push_back(false);
            if (std::abs(Es[i]) > MaxDiff)
            {
                listEsIndex.push_back(i);
            }
            if (std::abs(Ee[i]) > MaxDiff)
            {
                listEeIndex.push_back(i);
            }
        }
        else
        {
            bErrorDiff.push_back(true);
            hasReliablePoles = true;
        }
    }

    if (!hasReliablePoles)
        return ERR_CODE::errPoleRefinement;

    //VecInt listCathErrorIndex, listAnodeErrorIndex;
    if (listEsIndex.size() > 0)
    {
        for (int i = 0; i < listEsIndex.size(); i++)
        {
            int idex = listEsIndex[i];
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
                    if (dRIndex > dLIndex)
                    {
                        refEs = idex - dLIndex;
                    }
                    else
                        refEs = dRIndex + idex;
                }
                else
                    refEs = idex - dLIndex;
            }
            else if (dRIndex != -1)
            {
                refEs = dRIndex + idex;
            }
            else
            {
                return ERR_CODE::errPoleRefinement;
            }

            if (std::abs(D_L[idex] - D_L[refEs]) > std::abs(D_R[idex] - D_R[refEs]))
            {
                auto result1 = std::find(lstPolePosXAll.begin(), lstPolePosXAll.end(), Ls[idex].x);
                if (result1 != lstPolePosXAll.end())
                {
                    int realIndex = std::distance(lstPolePosXAll.begin(), result1);
                    anodePos[realIndex] = Rs[idex].y;
                    listAnodeErrorIndex.push_back(realIndex);
                }
            }
            else
            {
                int PosX = widthROI - Rs[idex].x;

                auto result1 = std::find(lstPolePosXAll.begin(), lstPolePosXAll.end(), PosX);
                if (result1 != lstPolePosXAll.end())
                {
                    int realIndex = std::distance(lstPolePosXAll.begin(), result1);
                    anodePos[realIndex] = Ls[idex].y;
                    listAnodeErrorIndex.push_back(realIndex);
                }
            }
        }
        std::sort(listAnodeErrorIndex.begin(), listAnodeErrorIndex.end(), [](int const &i, int const &j) { return i < j; });
    }

    return ERR_CODE::OK;
}

void AutoEnhance(cv::Mat &rImg)
{
    if (!rImg.empty() && 0 == rImg.depth())
    {
        if (1 != rImg.channels())
        {
            cv::cvtColor(rImg, rImg, cv::COLOR_BGR2GRAY);
        }

        // Calculate histograms of arrays of images
        int histSize = 256;
        bool uniform = true, accumulate = false;
        float range[] = {0, 256}; //the upper boundary is exclusive
        const float *histRange[] = {range};
        cv::Mat gray_hist, enhance_hist;
        cv::calcHist(&rImg, 1, 0, cv::Mat(), gray_hist, 1, &histSize, histRange, uniform, accumulate);

        int minTh = 0;
        int maxTh = 0;
        float sumCTF = 0;
        cv::Mat otsuImage;
        std::vector<float> LUT;
        float total = rImg.total();
        for (int i = 0; i < histSize; i++)
        {
            sumCTF = sumCTF + gray_hist.at<float>(i);

            if (sumCTF / total > 0.02)
            {
                minTh = i;
                break;
            }
        }

        maxTh = (int)cv::threshold(rImg, otsuImage, 0, 255, cv::THRESH_OTSU);

        const int minAutoThreshold = 60;
        const int maxAutoThreshold = 170;
        if (maxTh < minAutoThreshold)
            maxTh = minAutoThreshold;
        if (maxTh > maxAutoThreshold)
            maxTh = maxAutoThreshold;
        if (minTh >= maxTh)
            minTh = maxTh;
        LUT = GetLUT(minTh, maxTh);
        //LUT = Utils::GetLUT((minTh + maxTh) >> 1, maxTh);
        cv::LUT(rImg, LUT, rImg);
    }
}

cv::Mat getHistogramImg(const cv::Mat &src, int minTh, int maxTh)
{
    cv::Mat dst = cv::Mat();
    src.convertTo(dst, CV_32FC1);
    // Calculate histograms of arrays of images
    int histSize = 256;
    float range[] = {0, 256}; //the upper boundary is exclusive
    const float *histRange[] = {range};
    cv::Mat gray_hist;
    bool uniform = true, accumulate = false;
    cv::calcHist(&dst, 1, 0, cv::Mat(), gray_hist, 1, &histSize, histRange, uniform, accumulate);
    gray_hist.at<float>(0, 0) = 0;
    cv::Mat hisImg = histDisplay(gray_hist, "", minTh, maxTh);
    dst.release();
    gray_hist.release();
    return hisImg;
}

VecPoint Dijkstra_Algorithm(const cv::Mat &src)
{
    VecPoint vtLinePoint;
    if (!src.empty())
    {
        cv::Mat dst;
        src.convertTo(dst, CV_64F);
        int ksize = 1;
        int preIdxStart, preIdxEnd;
        int preIdx;
        for (int i = 1; i < dst.cols; i++)
        {
            preIdx = i - 1;
            for (int j = 0; j < dst.rows; j++)
            {
                preIdxStart = std::max<int>(j - ksize, 0);
                preIdxEnd = std::min<int>(j + ksize, dst.rows - 1);
                dst.at<double>(j, i) +=
                    std::max<double>({dst.at<double>(preIdxStart, preIdx), dst.at<double>(j, preIdx), dst.at<double>(preIdxEnd, preIdx)});
            }
        }

        // find starting point to tracing back
        int finalColIdx = dst.cols - 1;
        std::vector<float> vtRow = cv::Mat_<float>(dst.col(finalColIdx));
        auto maxElement = std::max_element(vtRow.begin(), vtRow.end(), [](const float &lsh, const float &rsh) { return lsh < rsh; });
        VecInt vtLine(dst.cols, 0);
        vtLine[finalColIdx] = std::distance(vtRow.begin(), maxElement);
        int start, end;
        //ksize = 2;
        double mean;
        for (int i = finalColIdx - 1; i >= 0; i--)
        {
            vtRow = cv::Mat_<float>(dst.col(i));

            start = std::max<int>(0, vtLine[i + 1] - ksize);
            end = std::min<int>(src.rows, vtLine[i + 1] + ksize + 1);
            maxElement = std::max_element(vtRow.begin() + start, vtRow.begin() + end, [](const float &lsh, const float &rsh) { return lsh < rsh; });

            mean = std::accumulate(vtRow.begin(), vtRow.end(), 0.0) / (double)vtRow.size();
            if (*maxElement != mean)
                vtLine[i] = std::distance(vtRow.begin(), maxElement);
            else
                vtLine[i] = vtLine[i + 1];
        }

        cv::blur(vtLine, vtLine, cv::Size(30, 1), cv::Point(-1, -1), cv::BORDER_REFLECT);

        if (vtLine.size() == src.cols)
        {
            for (int i = 0; i < vtLine.size(); i++)
                vtLinePoint.emplace_back(i, vtLine[i]);
        }
        // Display
        cv::Mat visImg;
        cv::cvtColor(src, visImg, cv::COLOR_GRAY2BGR);  // Chuyển ảnh sang BGR để vẽ màu

        for (const auto& pt : vtLinePoint)
        {
            cv::circle(visImg, pt, 1, cv::Scalar(0, 0, 255), -1);  // vẽ đường màu đỏ
        }

        cv::imshow("Original Image", src);
        cv::imshow("Dijkstra Path Visualization", visImg);
        cv::waitKey(0);
    }

    return vtLinePoint;
}

int imagecount = 0;
//#define DEBUG_REFINE_ANODE_POS
void refineAnodePos(std::vector<listPole> listAllPoles,
                    VecInt &vtAnodePos,
                    VecInt &vtCathodePos,
                    VecInt &lstPolePosXAll,
                    cv::Mat image,
                    cv::Mat drawoutut)
{
    cv::Mat colorimge, fakecolr, colorimgeFinal;
#ifdef DEBUG_REFINE_ANODE_POS
    cv::cvtColor(image, colorimge, cv::COLOR_GRAY2BGR);
    cv::cvtColor(image, fakecolr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(image, colorimgeFinal, cv::COLOR_GRAY2BGR);
#endif // DEBUG_REFINE_ANODE_POS

    cv::Point startingPoint;
    if (listAllPoles.size() > 0 && vtAnodePos.size() > 0 && listAllPoles.size() == vtAnodePos.size())
    {
        int averageEnding = 0;
        int averageStarting = 0;
        for (int i = 0; i < listAllPoles.size(); i++)
        {
            averageEnding += listAllPoles[i].ending.y;
            averageStarting += listAllPoles[i].starting.y;
        }
        averageEnding = averageEnding / listAllPoles.size();
        averageStarting = averageStarting / listAllPoles.size();

        //Finding list points from the starting to ending
        for (int current = 0; current < listAllPoles.size(); current++)
        {
            std::vector<pointValue> list;
            for (int i = 0; i < listAllPoles[current].listMiniInten.size(); i++)
            {
                if (listAllPoles[current].listMiniInten[i].position.y >= listAllPoles[current].ending.y)
                {
                    list.push_back(listAllPoles[current].listMiniInten[i]);
                }
            }

            if (list.empty())
                continue;
            //Finding the vertical line go through starting to ending
            std::vector<std::pair<int, pointValue>> findingmode;
            std::sort(list.begin(), list.end(), [](pointValue &a, pointValue &b) { return a.position.x < b.position.x; });
            int step = 1;
            for (int i = 0; i < list.size() - 1; i = i + step)
            {
                int count = 1;
                for (int j = i + 1; j < list.size(); j++)
                {
                    if (list[i].position.x == list[j].position.x)
                    {
                        count++;
                        if (j == list.size() - 1)
                        {
                            std::pair<int, pointValue> re = std::pair<int, pointValue>(count, list[i]);
                            step = count;
                            findingmode.push_back(re);
                            break;
                        }
                    }
                    else
                    {
                        std::pair<int, pointValue> re = std::pair<int, pointValue>(count, list[i]);
                        step = count;
                        findingmode.push_back(re);
                        break;
                    }
                }
            }
            std::sort(findingmode.begin(), findingmode.end(), [](std::pair<int, pointValue> &a, std::pair<int, pointValue> &b) {
                return a.first > b.first;
            });
            int averageInten = 0;
            int count = 0;
            for (int i = 0; i < listAllPoles[current].listMiniInten.size(); i++)
            {
                if (findingmode[0].second.position.x == listAllPoles[current].listMiniInten[i].position.x)
                {
                    averageInten += listAllPoles[current].listMiniInten[i].intensity;
                    count++;
                }
            }
            if (count > 0)
                averageInten = averageInten / count;

            int reference = 0;
            if (findingmode[0].first >= 20)
            {
                reference = findingmode[0].second.position.x;
            }
            else
            {
                reference = listAllPoles[current].ending.x;
            }
            if (std::abs(reference - listAllPoles[current].starting.x) > 5 && findingmode.size() > 0 && averageInten != 0 &&
                std::abs(reference - listAllPoles[current].ending.x) < 5)
            {
#ifdef DEBUG_REFINE_ANODE_POS
                cv::circle(colorimge, listAllPoles[current].starting, 2, cv::Scalar(0, 0, 255));
#endif
                int countStep = 0;
                for (int i = listAllPoles[current].starting.y - 5; i > averageEnding; i--)
                {
                    cv::Point point{listAllPoles[current].starting.x, i};
                    if (image.at<uchar>(point) <= averageInten + 15)
                    {
                        std::vector<pointValue> listMinimumIntensity;
                        cv::Point ending;
                        int stepHorizontal = 3;
                        int stepVertical = 5;
                        findAnodeAutoByEdge(image, point, point.y, stepHorizontal, stepVertical, fakecolr, listMinimumIntensity, ending);
                        if (ending != listAllPoles[current].ending && countStep < 30 && std::abs(ending.y - point.y) > 30)
                        {
                            startingPoint = point;
                            break;
                        }
                    }
                    else
                    {
                        startingPoint = listAllPoles[current].starting;
                    }
                    countStep++;
                }
            }
            else
            {
                startingPoint = listAllPoles[current].starting;
            }
            std::vector<pointValue> listMinimumIntensity;
            cv::Point ending;
            int anodeYPos;
            int stepHorizontal = 3;
            int stepVertical = 5;
            anodeYPos =
                findAnodeAutoByEdge(image, startingPoint, startingPoint.y, stepHorizontal, stepVertical, colorimge, listMinimumIntensity, ending);
            listAllPoles[current].listMiniInten = listMinimumIntensity;
            listAllPoles[current].starting = startingPoint;
            listAllPoles[current].ending = ending;
            anodeYPos = std::max(0, std::min(anodeYPos, startingPoint.y));
            // Update anode y position
            vtAnodePos[current] = anodeYPos;

            // Update current cathode x and y position
            //vtCathodePos[current] = listAllPoles[current].starting.y;
            //lstPolePosXAll[current] = listAllPoles[current].starting.x;
        }

        //Remove duplicate poles
        VecInt listDuplicate;
        for (int current = 0; current < listAllPoles.size() - 1; current++)
        {
            int countright = 0;
            //Checking pole left and pole right of current poles for ensuring they are not connected
            VecPoint listPointCurrent;
            VecPoint listPointRight;
            for (int j = 0; j < listAllPoles[current].listMiniInten.size(); j++)
            {
                if (listAllPoles[current].listMiniInten[j].position.y > listAllPoles[current].ending.y)
                {
                    listPointCurrent.push_back(listAllPoles[current].listMiniInten[j].position);
                }
            }
            if (current < listAllPoles.size() - 1)
            {
                for (int j = 0; j < listAllPoles[current + 1].listMiniInten.size(); j++)
                {
                    if (listAllPoles[current + 1].listMiniInten[j].position.y > listAllPoles[current + 1].ending.y)
                    {
                        listPointRight.push_back(listAllPoles[current + 1].listMiniInten[j].position);
                    }
                }
                for (int i = 0; i < listPointCurrent.size(); i++)
                {
                    for (int j = 0; j < listPointRight.size(); j++)
                    {
                        if (listPointCurrent[i].x == listPointRight[j].x && listPointCurrent[i].y == listPointRight[j].y)
                        {
                            countright++;
                        }
                    }
                }
            }
            if (countright > 2)
            {
                listAllPoles.erase(listAllPoles.begin() + current);
                vtAnodePos.erase(vtAnodePos.begin() + current);
                vtCathodePos.erase(vtCathodePos.begin() + current);
                lstPolePosXAll.erase(lstPolePosXAll.begin() + current);
                current--;
            }
        }
    }
    /*for (int i = 0; i < listAllPoles.size(); i++) {
            std::vector<pointValue> listMinimumIntensity;
            cv::Point ending;
            int anodeYPos;
            int stepHorizontal = 3;
            int stepVertical = 5;
            anodeYPos = findAnodeAutoByEdge(image, listAllPoles[i].starting, listAllPoles[i].starting.y, stepHorizontal, stepVertical, colorimgeFinal, listMinimumIntensity, ending);
        }*/

    //cv::imwrite("D:\\\AnodeRefined\\drawimage" + std::to_string(imagecount) + ".jpg",drawoutut);
    //cv::imwrite("D:\\\AnodeRefined\\finalresult" + std::to_string(imagecount) + ".jpg", colorimgeFinal);
    imagecount++;
}

VecPoint findCathodeLineByAStar(const cv::Mat &srcImg, int centerNeglectionWidth)
{
    VecPoint cathodeLine;
    const int minImageHeight = 15;
    if (!srcImg.empty() && srcImg.rows > minImageHeight)
    {
        cathodeLine.reserve(srcImg.cols);
        int region = 10;
        //local histogram equalization
        cv::Mat localEnhanceCathodeLine = LocalHistogramEqualization(srcImg, region); //LOCAL_HISTOGRAM_REDUCE

        int borderMode = cv::BORDER_REFLECT;                                          //border types for blur, soble
        int filterSize = 7;
        double sigma = 1.0;
        cv::Mat dstImg = DiagonalFilter(localEnhanceCathodeLine, filterSize, sigma, borderMode);

        // start - end column of neglection area
        int startNeglectCol = (dstImg.cols - centerNeglectionWidth) >> 1;
        int endNeglectCol = (dstImg.cols + centerNeglectionWidth) >> 1;

        cv::Rect leftSideROI = cv::Rect(0, 0, startNeglectCol, dstImg.rows);
        cv::Rect righttSideROI = cv::Rect(endNeglectCol, 0, dstImg.cols - endNeglectCol, dstImg.rows);
        VecPoint leftCathodeLine = Dijkstra_Algorithm(dstImg(leftSideROI));
        VecPoint rightCathodeLine = Dijkstra_Algorithm(dstImg(righttSideROI));

        cathodeLine.insert(cathodeLine.begin(), leftCathodeLine.begin(), leftCathodeLine.end());
        float alpha = 0.0;
        for (int i = startNeglectCol; i < endNeglectCol; i++)
        {
            alpha = (i - startNeglectCol) / (float)(endNeglectCol - startNeglectCol);
            cathodeLine.push_back(cv::Point(i, leftCathodeLine.back().y * (1 - alpha) + alpha * rightCathodeLine.front().y));
        }

        for (int i = 0; i < rightCathodeLine.size(); i++)
        {
            cathodeLine.push_back(cv::Point(rightCathodeLine[i].x + endNeglectCol, rightCathodeLine[i].y));
        }
#ifdef DEBUG_FIND_LINE_AUTO
        cv::Mat resultImg;
        cv::cvtColor(srcImg, resultImg, cv::COLOR_GRAY2BGR);
        Drawing drawTool1;
        drawTool1.color = COLOR_CV_RED;
        drawTool1.Plot(resultImg, cathodeLine, false);
        show_window("_AnodeCathodeResult", resultImg);
#endif // DEBUG_FIND_LINE_AUTO
    }

    return cathodeLine;
}

float findTiltAngle(const VecPoint &contour, int &start, int &end)
{
    float angle = -1;

    if (contour.empty() || start > contour.size() - 1 || end > contour.size() - 1 || start > end)
    {
        return angle;
    }

    cv::Point extRight = *std::max_element(contour.begin(), contour.end(), [](const cv::Point &lhs, const cv::Point &rhs) { return lhs.x < rhs.x; });

    cv::Point extBot = *std::max_element(contour.begin(), contour.end(), [](const cv::Point &lhs, const cv::Point &rhs) { return lhs.y < rhs.y; });

    int borderPadding = 20;
    cv::Mat drawingContour = cv::Mat::zeros(cv::Size(extRight.x + borderPadding, extBot.y + borderPadding), 0);

    for (int i = start; i < end; i++)
    {
        drawingContour.at<uchar>(contour[i].y, contour[i].x) = 255;
    }

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(drawingContour, lines, 1, CV_PI / 360, 50, 50, 10);
    if (lines.size() == 0)
    {
        return angle;
    }

    std::vector<std::pair<float, int>> listIndexLine;
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::Point startingPoint = cv::Point(l[0], l[1]);
        cv::Point endingPoint = cv::Point(l[2], l[3]);
        float length = cv::norm(startingPoint - endingPoint);
        listIndexLine.push_back(std::make_pair(length, i));
    }
    std::sort(listIndexLine.begin(), listIndexLine.end(), [](const std::pair<float, int> &a, const std::pair<float, int> &b) {
        return a.first > b.first;
    });

    cv::Point startingPointRealLine = cv::Point(lines[listIndexLine[0].second][0], lines[listIndexLine[0].second][1]);
    cv::Point endingPointRealLine = cv::Point(lines[listIndexLine[0].second][2], lines[listIndexLine[0].second][3]);
    cv::Point startingPointVerticalLine = cv::Point(lines[listIndexLine[0].second][0], lines[listIndexLine[0].second][1]);
    cv::Point endingPointVerticalLine = cv::Point(lines[listIndexLine[0].second][0], lines[listIndexLine[0].second][3]);

    //cv::line(grayColor, startingPointRealLine, endingPointRealLine, cv::Scalar(0, 0, 255), 2);
    float angle1 = atan2(startingPointRealLine.y - endingPointRealLine.y, startingPointRealLine.x - endingPointRealLine.x);
    float angle2 = atan2(startingPointVerticalLine.y - endingPointVerticalLine.y, startingPointVerticalLine.x - endingPointVerticalLine.x);
    angle = std::abs((angle2 - angle1) * 180 / 3.14);

    int realStartIndex = getPointOnContourId(contour, startingPointRealLine);
    if (realStartIndex != -1)
        start = realStartIndex;

    int realEndIndex = getPointOnContourId(contour, endingPointRealLine);
    if (realEndIndex != -1)
        end = realEndIndex;

    return angle;
}

int findAnodeByLineTracing(const cv::Mat &inputImg,
                           cv::Point startPoint,
                           int defaultVal,
                           int stepHorizontal,
                           int stepVertical,
                           float breakThreshold,
                           cv::Mat &drawOuput,
                           int borderCheck,
                           int borderDistance,
                           bool restrictMove,
                           int moveAllow)
{
    /*stepVertical = 7;
        stepHorizontal = 3;*/
    bool writeDebugPole = drawOuput.empty() ? false : true;

    if (writeDebugPole)
    {
        cv::circle(drawOuput, startPoint, 1, cv::Scalar(0, 255, 0));
    }

    int stepSkip = 1;
    int stepFrog = 1;
    int stopCheck = stepVertical + stepFrog + 1;
    int borderOffset = 1;

    cv::Rect moveRect;

    moveRect.width = 2 * stepHorizontal + 1;
    moveRect.height = stepVertical + 1;

    //REPOSITION OF STARTING POINT BASED ON THE LOWEST INTENSITY POINT NEARBY//
    /*cv::Rect rectStart(startPoint.x - stepHorizontal, startPoint.y - stepVertical, moveRect.width, stepVertical);
        RefineROI(rectStart, inputImg.size());
        double minValue, maxValue;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(inputImg(rectStart), &minValue, &maxValue, &minLoc, &maxLoc);
        startPoint = minLoc + rectStart.tl();*/

    int const &startPointX = startPoint.x;
    int const &startPointY = startPoint.y;

    if (borderCheck == -1)
    {
        if (2 * borderDistance < startPointX)
        {
            //borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (borderDistance < startPointX && borderDistance > 2)
        {
            borderOffset = 2;
        }
        else
        {
            borderOffset = 0;
        }
    }
    else if (borderCheck == 1)
    {
        if (startPointX < inputImg.cols - 2 * borderDistance)
        {
            // borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (startPointX < inputImg.cols - borderDistance && borderDistance > 2)
        {
            borderOffset = 2;
        }
        else
        {
            borderOffset = 0;
        }
    }
    //--------------END REPOSITION-------------------------------------------//
    ///////////////////////////////////////////////////////////////////////////
    //cv::circle(drawOuput, startPoint, 1, cv::Scalar(255, 255, 0));

    cv::Point currentPoint(startPointX, startPointY);
    int outputYAnode = defaultVal;
    int numberOfBlankLine = 5;


    //cv::circle(ROI4Draw, startPoint, 1, cv::Scalar(0, 255, 0));
    /*int CathodeOffsetX = batteryROI.x + BatInsp.borderPadding;
        int CathodeOffsetY = yA2 + jrPos.first;*/
    cv::Point prePoint = currentPoint;
    //cv::Point offset(batteryROI.x + BatInsp.borderPadding, yA2 + jrPos.first);
    int stepCheck = 0;
    int imgColsHalf = inputImg.cols / 2;
    cv::Mat reduceRes;
    //LINE TRACING FROM STARTING POINT TO END OF ROI //
    int it = 0;
    int leftBorderX = startPointX - borderOffset;  //when checking the left border pole
    int rightBorderX = startPointX + borderOffset; //when checking the right border pole

    int leftLimit = stepHorizontal;
    int rightLimit = inputImg.cols - stepHorizontal;

    assert(leftLimit > 0 && rightLimit > 0);

    for (it = currentPoint.y - stepSkip; it > stopCheck; it -= stepFrog)
    {
        int currentPointXCord = currentPoint.x;

        moveRect.x = currentPointXCord - stepHorizontal;
        moveRect.y = it - stepVertical;
        moveRect.width = 2 * stepHorizontal + 1;
        if (borderCheck == -1)
        {
            //if (borderDistance < startPointX)
            //Point is out of the left border
            if (leftBorderX >= moveRect.x)
            {
                moveRect.width = moveRect.br().x - leftBorderX;
                moveRect.x = leftBorderX;
            }
        }
        else if (borderCheck == 1)
        {
            if (moveRect.br().x >= rightBorderX)
            {
                moveRect.width = rightBorderX - moveRect.x + 1;
            }
        }

        float avgVal = 0.0;
        RefineROI(moveRect, inputImg.size());
        cv::Mat tmpMat = inputImg(moveRect);
        if (!tmpMat.empty())
        {
            //Vertical projection
            cv::reduce(tmpMat, reduceRes, 0, cv::REDUCE_AVG, CV_32FC1);
            int reduceCol = reduceRes.cols;

#if 0
                //CENTER WEIGHT
                for (int w = 0; w < reduceCol; w++) {
                    float distance = abs((float)(reduceCol / 2.0) - w);
                    float scaleWeight = 1;
                    reduceRes.at<float>(0, w) -= scaleWeight / distance;
                }

                //DIRECTIONAL WEIGHT
                float smallWeight = 0.5;
                if (currentPointXCord < imgColsHalf) {
                    reduceRes.at<float>(0, stepHorizontal + 1) -= smallWeight;
                }
                else
                {
                    reduceRes.at<float>(0, stepHorizontal - 1) -= smallWeight;
                }
#endif
            auto minPos = std::min_element((float *)reduceRes.ptr(0), (float *)reduceRes.ptr(0) + reduceRes.cols);
            avgVal = *minPos;
            currentPoint = cv::Point(moveRect.x + std::distance((float *)reduceRes.ptr(0), minPos), it);

            //Right border
            if (borderCheck == 1 && currentPoint.x >= rightBorderX)
            {
                currentPoint.x = rightBorderX;
            }

            //Left border
            if (borderCheck == -1 && leftBorderX >= currentPoint.x)
            {
                currentPoint.x = leftBorderX;
            }

            //restrict movement
            if (restrictMove)
            {
                if (currentPoint.x < imgColsHalf && (currentPoint.x - startPointX) < -moveAllow)
                {
                    currentPoint.x = startPointX;
                }
                if (currentPoint.x > imgColsHalf && (currentPoint.x - startPointX) > moveAllow)
                {
                    currentPoint.x = startPointX;
                }
            }

            if (avgVal > breakThreshold)
            {
                stepCheck++;
                if (stepCheck >= numberOfBlankLine)
                {
                    if (writeDebugPole)
                    {
                        cv::line(drawOuput, currentPoint, prePoint, cv::Scalar(255, 0, 0));
                        prePoint = currentPoint;
                    }
                    //return(currentPointYCord);
                    break;
                }
            }
            else
            {
                if (writeDebugPole)
                {
                    cv::line(drawOuput, currentPoint, prePoint, cv::Scalar(0, 0, 255));
                    prePoint = currentPoint;
                }
                stepCheck = 0;
            }
        }
        else // Move out of image
        {
            //Use the currentPoint as the output
            //assert(false);
            break;
        }
    }
    //---------END LINE TRACING --------------------------//
    ///////////////////////////////////////////////////////

    outputYAnode = currentPoint.y + stepCheck * stepFrog;
    reduceRes.release();
    return outputYAnode;
}

std::pair<VecInt, VecDouble> LowerRemoveCloseLine(std::vector<std::tuple<int, double, double>> poleList, int polesMinDistance)
{
    VecInt finalpolePosition;
    VecDouble finalpoleIntensity;

    if (poleList.size() > 0)
    {
        // Add first pole to final result
        auto pole = poleList[0];
        finalpolePosition.push_back(std::get<0>(pole));
        finalpoleIntensity.push_back(std::get<1>(pole));
        double lastPeakDiff = std::get<2>(pole);
        double lastPoleIntensity = std::get<1>(pole);

        for (int i = 1; i < poleList.size(); i++)
        {
            double poleIntensity;
            int polePositionIdx;
            int poleIntensityChange;
            std::tie(polePositionIdx, poleIntensity, poleIntensityChange) = poleList[i];

            // Remove poles in the ROI2 (center part)
            //if ((polePositionIdx < leftMiddleArea) || (polePositionIdx > rightMiddleArea)) {
            int distance = polePositionIdx - finalpolePosition[finalpolePosition.size() - 1];
            if (distance < polesMinDistance) // 2 poles is too close: compare to find the select the correct pole
            {
                // select the pole which have lower intensity
                if (lastPoleIntensity - poleIntensity >
                    2) // If intensity of this pole is 200 lower than current last poles -> set this pole as the last poles
                {
                    finalpolePosition[finalpolePosition.size() - 1] = polePositionIdx;
                    lastPoleIntensity = poleIntensity;
                    lastPeakDiff = poleIntensityChange;
                }
                else if (poleIntensityChange > lastPeakDiff) // select the pole which have higher intensity change
                {
                    finalpolePosition[finalpolePosition.size() - 1] = polePositionIdx;
                    lastPoleIntensity = poleIntensity;
                    lastPeakDiff = poleIntensityChange;
                }
            }
            else
            {
                // Add current pole to final list poles
                lastPoleIntensity = poleIntensity;
                lastPeakDiff = poleIntensityChange;
                finalpolePosition.push_back(polePositionIdx);
                finalpoleIntensity.push_back(poleIntensity);
            }
            //}
        }
    }
    else
    {
        //Error log;
    }
    return std::make_pair(finalpolePosition, finalpoleIntensity);
}

int findAnodeAutoByEdge(const cv::Mat &inputImg,
                        cv::Point startPoint,
                        int defaultVal,
                        int stepHorizontal,
                        int stepVertical,
                        cv::Mat &drawOuput,
                        std::vector<pointValue> &listMinimumIntensity,
                        cv::Point &ending)
{
    int outputYAnode = defaultVal;

    bool isValidStartPoint = (startPoint.x >= 0 && startPoint.x < inputImg.cols) && (startPoint.y > 0 && startPoint.y < inputImg.rows);
    if (!inputImg.empty() && isValidStartPoint && (stepHorizontal > 0 && stepVertical > 0))
    {
        bool writeDebugPole = drawOuput.empty() ? false : true;
        cv::Mat sumAverageImage = cv::Mat::zeros(inputImg.size(), 0);
        int numberblank = 1;
        float weightSumDiff = 0.4;
        float weightSumCount = 0.6;
        cv::blur(inputImg, sumAverageImage, cv::Size(1, stepVertical));
        listMinimumIntensity.push_back({sumAverageImage.at<uchar>(startPoint), startPoint});
        cv::Point startingForLoop = cv::Point(startPoint.x, startPoint.y - 1);
        //Find all minimum points from startpoint which window 1x7
        cv::Point localMinimumPoint = startingForLoop;

        for (int i = startingForLoop.y; i >= 0; i--)
        {
            int startIdx = std::max(localMinimumPoint.x - stepHorizontal, 0);
            int endIdx = std::min(localMinimumPoint.x + stepHorizontal, inputImg.cols - 1);
            //VecInt vtRow = cv::Mat_<uchar>(sumAverageImage.row(i));
            auto vtRow = (sumAverageImage.row(i).data);

            auto minIntensity = std::min_element(vtRow + startIdx, vtRow + endIdx, [](const int &a, const int &b) { return a < b; });
            localMinimumPoint = cv::Point(std::distance(vtRow, minIntensity), i);
            if (writeDebugPole)
            {
                cv::circle(drawOuput, localMinimumPoint, 1, cv::Scalar(0, 0, 255));
            }
            listMinimumIntensity.push_back({*minIntensity, localMinimumPoint});
        }


        //Find average intensity of 5 first
        int sum = 0;
        if (listMinimumIntensity.size() > numberblank * 3)
        {
            for (int i = 0; i < numberblank * 3; i++)
            {
                sum = sum + listMinimumIntensity[i].intensity;
            }
        }
        int averageSum = sum / (numberblank * 3);
        int thresholdInten = averageSum * 2;

        std::reverse(listMinimumIntensity.begin(), listMinimumIntensity.end());
        //Put Derivative values into listIntenDev list and put counting derivative value into listCountIntenDev
        int pos = -1;
        //VecInt listIntensityAcc;
        VecInt listIntenDev;
        int sumaccumulate;
        if (listMinimumIntensity.size() > 1)
        {
            listIntenDev.push_back(-1);
            for (int i = 1; i < listMinimumIntensity.size() - 1; i++)
            {
                int diff = listMinimumIntensity[i].intensity - listMinimumIntensity[i + 1].intensity;
                listIntenDev.push_back(diff);
            }
            for (int i = 1; i < listIntenDev.size() - 1; i++)
            {
                if (listIntenDev[i] <= 1 && listIntenDev[i - 1] > 1 && listIntenDev[i + 1] > 1)
                {
                    listIntenDev[i] = listIntenDev[i - 1];
                }
            }
            listIntenDev.push_back(-1);

            VecInt listCountIntenDev(listIntenDev.size(), 0);
            for (int i = listIntenDev.size() - 2; i > 1; i--)
            {
                if (listIntenDev[i] > 1 && listIntenDev[i - 1] > 1 && listIntenDev[i + 1] <= 1)
                {
                    listCountIntenDev[i] = 1;
                }
                else if ((listIntenDev[i] > 1 && listIntenDev[i - 1] > 1 && listIntenDev[i + 1] > 1) ||
                         (listIntenDev[i] > 1 && listIntenDev[i - 1] <= 1 && listIntenDev[i + 1] > 1))
                {
                    listCountIntenDev[i] = listCountIntenDev[i + 1] + 1;
                }
            }
            //Putting sumDiff, the center position, the first position and weighted value of each cluster into listWeightDiff
            std::vector<candiEdP> listWeightDiff;
            int posMax = -1;
            for (int i = 1; i < listCountIntenDev.size() - 1; i++)
            {
                if (0 == listCountIntenDev[i - 1] && listCountIntenDev[i] >= 1 && listCountIntenDev[i + 1] >= 1)
                {
                    int sumDiff = 0;
                    for (int j = listCountIntenDev[i]; j > 0; j--)
                    {
                        sumDiff = sumDiff + listIntenDev[i + listCountIntenDev[i] - j];
                    }
                    int mean = sumDiff / listCountIntenDev[i];
                    int var = 0;
                    for (int j = listCountIntenDev[i]; j > 0; j--)
                    {
                        var += (listIntenDev[i + listCountIntenDev[i] - j] - mean) * (listIntenDev[i + listCountIntenDev[i] - j] - mean);
                    }
                    var /= listCountIntenDev[i];
                    //int std = var * var;
                    int WeightValue = (sumDiff * weightSumDiff) - (listCountIntenDev[i] * weightSumCount);
                    candiEdP t = {sumDiff, (i + (i + listCountIntenDev[i])) / 2, i + 2, listCountIntenDev[i], WeightValue, var};
                    listWeightDiff.push_back(t);
                }
            }
            if (0 != listWeightDiff.size())
            {
                //Check condition for each element in listWeightDiff, if sum average intensity 5 concecutive above first position - sum average intensity 5 concecutive above first position >0 -> stop at this point
                std::sort(listWeightDiff.begin(), listWeightDiff.end(), [](auto &left, auto &right) { return left.weightedSum > right.weightedSum; });
                for (int i = 0; i < listWeightDiff.size(); i++)
                {

                    int sumAverageCurrent = 0;
                    for (int k = 0; k < listWeightDiff[i].countDerivative - 1; k++)
                    {
                        sumAverageCurrent = sumAverageCurrent + listMinimumIntensity[listWeightDiff[i].firstPos + k].intensity;
                    }
                    int lastElement = listWeightDiff[i].firstPos + listWeightDiff[i].countDerivative;
                    int sumNextLast = 0;
                    if (lastElement + (numberblank * 3) < listCountIntenDev.size())
                    {
                        for (int i = lastElement; i < lastElement + (numberblank * 3); i++)
                        {
                            sumNextLast = sumNextLast + listMinimumIntensity[i].intensity;
                        }
                    }
                    int averageNextLast = sumNextLast / (numberblank * 3);

                    sumAverageCurrent = sumAverageCurrent / (listWeightDiff[i].countDerivative - 1);
                    bool stop = false;
                    if (averageNextLast <= thresholdInten)
                    {
                        int sum = 0;
                        int count = 0;
                        int sumaverage = 0;
                        for (int j = listWeightDiff[i].firstPos / 2; j < listWeightDiff[i].firstPos - 1; j++)
                        {
                            sum = sum + listMinimumIntensity[j].intensity;
                            count++;
                        }
                        sumaverage = count != 0 ? sum / count : -1;
                        if (sumaverage - sumAverageCurrent > 0)
                        {
                            stop = true;
                        }
                    }
                    if (stop == true)
                    {
                        posMax = listWeightDiff[i].position;
                        break;
                    }
                }
                if (-1 != posMax)
                {
                    if (writeDebugPole)
                    {
                        cv::circle(drawOuput, listMinimumIntensity[posMax].position, 2, cv::Scalar(0, 255, 255));
                    }
                    int offset = 5;
                    if (listMinimumIntensity[posMax].position.y + offset < inputImg.rows)
                    {
                        outputYAnode = listMinimumIntensity[posMax].position.y + offset;
                        ending = listMinimumIntensity[posMax].position;
                    }
                }
            }
            else
            {
                return outputYAnode;
            }
        }
        else
        {
            return outputYAnode;
        }
    }
    else
    {
        return outputYAnode;
    }
    return outputYAnode;
}


bool isPntsIn2Side(cv::Point p1, cv::Point p2, VecPoint Ls, VecPoint Rs)
{
    bool res = false;
    /*VecPoint::iterator l1It = std::find_if(Ls.begin(), Ls.end(), [&](const cv::Point& p) {
            return p.x == p1.x;
        });
        VecPoint::iterator r1It = std::find_if(Rs.begin(), Rs.end(), [&](const cv::Point& p) {
            return p.x == p1.x;
        });
        VecPoint::iterator l2It = std::find_if(Ls.begin(), Ls.end(), [&](const cv::Point& p) {
            return p.x == p2.x;
        });
        VecPoint::iterator r2It = std::find_if(Rs.begin(), Rs.end(), [&](const cv::Point& p) {
            return p.x == p2.x;
        });*/

    VecPoint::iterator l1It = std::find(Ls.begin(), Ls.end(), p1);
    VecPoint::iterator r1It = std::find(Rs.begin(), Rs.end(), p1);
    VecPoint::iterator l2It = std::find(Ls.begin(), Ls.end(), p2);
    VecPoint::iterator r2It = std::find(Rs.begin(), Rs.end(), p2);

    if ((l1It != Ls.end() && r2It != Rs.end()) || (r1It != Rs.end() && l2It != Ls.end()))
    {
        return true;
    }
    return false;
}

std::pair<int, int> findUnstableRegion(const VecInt &lstPolePosXAlLBeforeRemove,
                                       VecInt &lstPolePosXAll,
                                       VecInt &anodePosAfterRemove,
                                       VecInt &cathodePosAfterRemove,
                                       VecInt &listRemovebyDominant,
                                       int widthROI,
                                       int numberPoleChecking,
                                       cv::Mat &debugImg)
{
    int starting = 0;
    int last = 0;
    std::pair<int, int> result(0, 0);
    if (anodePosAfterRemove.size() == 0 || lstPolePosXAll.size() == 0 || cathodePosAfterRemove.size() == 0 || numberPoleChecking < 0)
    {
        return result;
    }
    //Push Pole Remove by Dominant into List
    std::vector<PoleRemoveInfo> listRemovePoleLeft;
    std::vector<PoleRemoveInfo> listRemovePoleRight;
    double MaxDis = 14;  //Max distance using for finding the matchign poles
    double MaxDiff = 10; // Using for deciding the untrusted poles by compare the pole length
    int countDominantLeft = 0;
    int countDominantRight = 0;
    int countDominant = 0;
    int indexDominantLeft = -1;
    int indexDominantRight = -1;
    int average = 0;

    if (listRemovebyDominant.size() > 0)
    {
        for (int k = 0; k < listRemovebyDominant.size(); k++)
        {
            if (lstPolePosXAlLBeforeRemove[listRemovebyDominant[k]] < widthROI / 2 && (listRemovebyDominant[k] < numberPoleChecking))
            {
                countDominantLeft = countDominantLeft + 1;
                ;
            }
            else if (lstPolePosXAlLBeforeRemove[listRemovebyDominant[k]] >= widthROI / 2 &&
                     ((lstPolePosXAlLBeforeRemove.size() - listRemovebyDominant[k]) <= (numberPoleChecking)))
            {
                countDominantRight = countDominantRight + 1;
                ;
            }
        }
    }

    countDominant = (countDominantLeft > countDominantRight) ? countDominantLeft : countDominantRight;
    int numberCheckingPoleLeft = numberPoleChecking - countDominantLeft;
    int numberCheckingPoleRight = numberPoleChecking - countDominantRight;
    if (numberCheckingPoleLeft > 0 && numberCheckingPoleRight > 0)
    {
        std::vector<PoleCompare> poleCompareList;
        poleCompareList = evaluatePoleForRefinement(lstPolePosXAll, anodePosAfterRemove, cathodePosAfterRemove, widthROI, MaxDis, MaxDiff, false);
        int countleft = 0;
        int countright = 0;
        int countAdjustedPole = 0;
        PoleCompare *trustedPoleBefore = nullptr;
        PoleCompare *trustedPoleAfter = nullptr;
        std::vector<PoleCompare *> untrustPoleList;
        for (auto p = poleCompareList.begin(); p != poleCompareList.end(); p++)
        {
            if (p->mType == PoleCompareStatus::Trust)
            {
                if (!untrustPoleList.empty())
                {
                    trustedPoleAfter = p._Ptr;
                }
                else
                {
                    trustedPoleBefore = p._Ptr;
                }
            }
            else if (p->mType == PoleCompareStatus::Untrust)
            {
                untrustPoleList.push_back(p._Ptr);
            }
            if (trustedPoleAfter != nullptr || (p + 1) == poleCompareList.end())
            {
                //update all untrusted poles.
                if (trustedPoleAfter != nullptr && trustedPoleBefore != nullptr)
                {
                    for (auto up : untrustPoleList)
                    {
                        up->mLeftReferencePole = GetRefPole(up->mLeftPole.mAnode.x, trustedPoleBefore->mLeftPole, trustedPoleAfter->mLeftPole);
                        up->mRightReferencePole = GetRefPole(up->mRightPole.mAnode.x, trustedPoleBefore->mRightPole, trustedPoleAfter->mRightPole);
                        float leftLengthError = abs(up->mLeftReferencePole.length() - up->mLeftPole.length());
                        float rightLengthError = abs(up->mRightReferencePole.length() - up->mRightPole.length());
                        if (leftLengthError > rightLengthError && countleft < numberCheckingPoleLeft)
                        {
                            for (int k = 0; k < lstPolePosXAll.size(); k++)
                            {
                                if (up->mLeftPole.mCathode.x == lstPolePosXAll[k] &&
                                    up->mLeftPole.mCathode.x < lstPolePosXAll[numberCheckingPoleLeft])
                                {
                                    listRemovePoleLeft.push_back({lstPolePosXAll[k], k});
                                }
                            }
                            countleft++;
                        }
                        else if (leftLengthError < rightLengthError && countright < numberCheckingPoleLeft)
                        {
                            for (int k = 0; k < lstPolePosXAll.size(); k++)
                            {
                                if (widthROI - up->mRightPole.mCathode.x == lstPolePosXAll[k] &&
                                    widthROI - up->mRightPole.mCathode.x > lstPolePosXAll[lstPolePosXAll.size() - numberCheckingPoleLeft - 1])
                                {
                                    listRemovePoleRight.push_back({lstPolePosXAll[k], k});
                                }
                            }
                            countright++;
                        }
                        //Ajust the untrust pole by the length of the trust pole
                        if (up->AdjustUncertainType(MaxDiff))
                        {
                            countAdjustedPole++;
                        }
                    }
                }
                else if (trustedPoleBefore != nullptr)
                {
                    // If we can find only the before trusted pole.
                    for (auto up : untrustPoleList)
                    {
                        up->mLeftReferencePole = trustedPoleBefore->mLeftPole;
                        up->mRightReferencePole = trustedPoleBefore->mRightPole;
                        //Ajust the untrust pole by the length of the trust pole
                        if (up->AdjustUncertainType(MaxDiff))
                        {
                            countAdjustedPole++;
                        }

                        if (up->mLeftPole.mType < PoleType::NotFound && up->mRightPole.mType < PoleType::NotFound)
                        {
                            trustedPoleBefore = up;
                        }
                    }
                }
                else if (trustedPoleAfter != nullptr)
                {
                    // If we can find only the after trusted pole.
                    auto pStart = untrustPoleList.rbegin();
                    auto pEnd = untrustPoleList.rend();
                    auto tmpTrustPole = trustedPoleAfter;
                    for (auto up = pStart; up != pEnd; up++)
                    {
                        (*up)->mLeftReferencePole = tmpTrustPole->mLeftPole;
                        (*up)->mRightReferencePole = tmpTrustPole->mRightPole;

                        //Ajust the untrust pole by the length of the trust pole
                        if ((*up)->AdjustUncertainType(MaxDiff))
                        {
                            countAdjustedPole++;
                        }
                        if ((*up)->mLeftPole.mType < PoleType::NotFound && (*up)->mRightPole.mType < PoleType::NotFound)
                        {
                            tmpTrustPole = *up;
                        }
                    }
                }


                untrustPoleList.clear();
                trustedPoleBefore = trustedPoleAfter;
                trustedPoleAfter = nullptr;
            }
        }
    }

    if (lstPolePosXAll.size() > 20)
    {
        for (int i = 0; i < lstPolePosXAll.size(); i++)
        {
            std::vector<PoleRemoveInfo>::iterator itleft =
                std::find_if(listRemovePoleLeft.begin(), listRemovePoleLeft.end(), [&](const PoleRemoveInfo &a) {
                    return a.pointX == cathodePosAfterRemove[i];
                });
            std::vector<PoleRemoveInfo>::iterator itright =
                std::find_if(listRemovePoleRight.begin(), listRemovePoleRight.end(), [&](const PoleRemoveInfo &a) {
                    return a.pointX == cathodePosAfterRemove[i];
                });

            if (itleft == listRemovePoleLeft.end() && itright == listRemovePoleRight.end())
            {
                average = average + std::abs(cathodePosAfterRemove[i] - anodePosAfterRemove[i]);
            }
        }
    }
    average = average / lstPolePosXAll.size();

    int numberRemovedLeft = 0;
    int numberRemovedRight = 0;

    if (listRemovePoleLeft.size() > 0)
        numberRemovedLeft = listRemovePoleLeft[listRemovePoleLeft.size() - 1].index + 1;
    if (listRemovePoleRight.size() > 0 && lstPolePosXAll.size() - listRemovePoleRight[0].index >= 0)
        numberRemovedRight = lstPolePosXAll.size() - listRemovePoleRight[0].index;

    if (numberRemovedLeft < numberCheckingPoleLeft)
    {
        int iter = numberRemovedLeft;
        int end = numberCheckingPoleLeft - numberRemovedLeft;
        for (int i = iter; i < iter + end; i++)
        {
            int length = std::abs(cathodePosAfterRemove[i] - anodePosAfterRemove[i]);
            if (length < 0.5 * average || length > 1.5 * average)
            {
                listRemovePoleLeft.push_back({lstPolePosXAll[i], i});
            }
        }
    }
    if (numberRemovedRight < numberCheckingPoleRight && lstPolePosXAll[lstPolePosXAll.size() - 1] > widthROI / 2)
    {
        int iter = lstPolePosXAll.size() - 1;
        if (listRemovePoleRight.size() > 0)
        {
            iter = listRemovePoleRight[0].index - 1;
        }
        int end = numberCheckingPoleRight - numberRemovedRight;
        for (int i = iter; i > iter - end; i--)
        {
            int length = std::abs(cathodePosAfterRemove[i] - anodePosAfterRemove[i]);
            if (length < 0.5 * average || length > 1.5 * average)
            {
                listRemovePoleRight.push_back({lstPolePosXAll[i], i});
            }
        }
    }

    std::sort(listRemovePoleLeft.begin(), listRemovePoleLeft.end(), [](auto &a, auto &b) { return a.index < b.index; });
    std::sort(listRemovePoleRight.begin(), listRemovePoleRight.end(), [](auto &a, auto &b) { return a.index < b.index; });


    if (listRemovePoleRight.size() > 0 && lstPolePosXAll.size() - listRemovePoleRight[0].index >= 0)
    {
        last = listRemovePoleRight[0].pointX;
        numberRemovedRight = lstPolePosXAll.size() - listRemovePoleRight[0].index;
        anodePosAfterRemove.erase(anodePosAfterRemove.end() - numberRemovedRight, anodePosAfterRemove.end());
        cathodePosAfterRemove.erase(cathodePosAfterRemove.end() - numberRemovedRight, cathodePosAfterRemove.end());
        lstPolePosXAll.erase(lstPolePosXAll.end() - numberRemovedRight, lstPolePosXAll.end());
    }
    if (listRemovePoleLeft.size() > 0)
    {
        numberRemovedLeft = listRemovePoleLeft[listRemovePoleLeft.size() - 1].index + 1;
        starting = listRemovePoleLeft[listRemovePoleLeft.size() - 1].pointX;
        anodePosAfterRemove.erase(anodePosAfterRemove.begin(), anodePosAfterRemove.begin() + numberRemovedLeft);
        cathodePosAfterRemove.erase(cathodePosAfterRemove.begin(), cathodePosAfterRemove.begin() + numberRemovedLeft);
        lstPolePosXAll.erase(lstPolePosXAll.begin(), lstPolePosXAll.begin() + numberRemovedLeft);
    }
    if (last > starting)
    {
        result.first = starting;
        result.second = last;
    }
    ////Apply Refinement for finding  untrust pole
    //numberPoleChecking = numberPoleChecking - countDominant;
    //if (numberPoleChecking > 0) {
    //    std::vector<PoleCompare> poleCompareList;
    //    poleCompareList=evaluatePoleForRefinement(lstPolePosXAll, anodePosAfterRemove, cathodePosAfterRemove, widthROI, 14, 10,false);
    //    //Push unstrust pole into list
    //    auto loopIter = std::min((int)poleCompareList.size(), numberPoleChecking);
    //    for (int i = 0; i < loopIter; i++) {
    //        if (poleCompareList[i].mType == PoleCompareStatus::Untrust) {
    //            for (int k = 0; k < lstPolePosXAll.size(); k++)
    //            {
    //                if (poleCompareList[i].mLeftPole.mCathode.x == lstPolePosXAll[k]) {
    //                    listRemovePole.push_back({ lstPolePosXAll[k], k });
    //                }
    //                if (widthROI - poleCompareList[i].mRightPole.mCathode.x == lstPolePosXAll[k]) {
    //                    listRemovePole.push_back({ lstPolePosXAll[k], k });
    //                }
    //            }
    //        }
    //    }
    //    std::sort(listRemovePole.begin(), listRemovePole.end(), [](auto& a, auto& b)
    //        {
    //            return a.index < b.index;
    //        });
    //    //Find position for removing pole
    //    if ((listRemovePole.size() > 2 && listRemovePole.size() % 2 == 0)||(listRemovePole.size() == 2 && listRemovePole[listRemovePole.size() / 2 - 1].index == 0)) {
    //        starting = listRemovePole[listRemovePole.size() / 2 - 1].pointX;
    //        int startingRemoveFromLeft = listRemovePole[listRemovePole.size() / 2 - 1].index;
    //        int startingRemoveFromRight = listRemovePole[listRemovePole.size() / 2].index - startingRemoveFromLeft - 1;
    //        int startRight = listRemovePole[listRemovePole.size() / 2].pointX;
    //        if (starting > 0 && starting < widthROI / 2 && startRight > 0 && startRight > widthROI / 2 && startRight<widthROI
    //            && anodePosAfterRemove.size()>listRemovePole.size()) {
    //            anodePosAfterRemove.erase(anodePosAfterRemove.begin(), anodePosAfterRemove.begin() + startingRemoveFromLeft + 1);
    //            anodePosAfterRemove.erase(anodePosAfterRemove.begin() + startingRemoveFromRight, anodePosAfterRemove.end());
    //            cathodePosAfterRemove.erase(cathodePosAfterRemove.begin(), cathodePosAfterRemove.begin() + startingRemoveFromLeft + 1);
    //            cathodePosAfterRemove.erase(cathodePosAfterRemove.begin() + startingRemoveFromRight, cathodePosAfterRemove.end());
    //            lstPolePosXAll.erase(lstPolePosXAll.begin(), lstPolePosXAll.begin() + startingRemoveFromLeft + 1);
    //            lstPolePosXAll.erase(lstPolePosXAll.begin() + startingRemoveFromRight, lstPolePosXAll.end());
    //        }
    //    }

    //}
    //
    //    if (!debugImg.empty()&&anodePosAfterRemove.size() == cathodePosAfterRemove.size()&& anodePosAfterRemove.size()==lstPolePosXAll.size()) {
    //        for (int i = 0; i < anodePosAfterRemove.size(); i++) {
    //            cv::circle(debugImg, cv::Point(lstPolePosXAll[i], anodePosAfterRemove[i]), 2, cv::Scalar(255, 0, 0));
    //            cv::circle(debugImg, cv::Point(lstPolePosXAll[i], cathodePosAfterRemove[i]), 2, cv::Scalar(0, 255, 0));

    //        }
    //    }


    return result;
}

void drawPoleTextResult(cv::Mat &resImg,
                        cv::Point const &startPos,
                        std::string const &poleNumStr,
                        cv::Scalar anodeColor,
                        std::string const &strAnode2Boundery,
                        std::string const &strCath2Ano,
                        cv::Scalar anode2CathodeColor,
                        std::string const &anode2CathodeLimitStr,
                        std::string const &strAno2Case,
                        cv::Scalar anode2CaseColor,
                        std::string const &anode2CaseLimitStr,
                        cv::Scalar const &inforColor,
                        float fontScale,
                        float fontWeight)
{
    //Draw the pole number
    cv::Point nextPos = drawText(resImg, startPos, poleNumStr, anodeColor, fontScale, fontWeight);
    //Draw the pole position infor
    nextPos = drawText(resImg, nextPos, strAnode2Boundery, inforColor, fontScale, fontWeight);
    //Draw the Anode2Cathode length
    nextPos = drawText(resImg, nextPos, strCath2Ano, anode2CathodeColor, fontScale, fontWeight);
    //Draw the anode to cathode length range
    nextPos = drawText(resImg, nextPos, anode2CathodeLimitStr, inforColor, fontScale, fontWeight);
    //Draw the anode to case length
    nextPos = drawText(resImg, nextPos, strAno2Case, anode2CaseColor, fontScale, fontWeight);
    //Draw the anode to case length range
    nextPos = drawText(resImg, nextPos, anode2CaseLimitStr, inforColor, fontScale, fontWeight);
}

bool drawPoleTextResult(cv::Mat &resImg,
                        const VecDouble &listAnode2CathodeDistance,
                        const VecDouble &listAnode2CaseDistance,
                        VecDouble const &listPolePos,
                        const std::vector<bool> &listAno2CathodDecision,
                        const std::vector<bool> &listAno2CaseDecision,
                        double MinCathode2Anode,
                        double MaxCathode2Anode,
                        double MinAnode2Case,
                        double MaxAnode2Case,
                        double cathode2AnodeOffset,
                        double anode2CaseOffset,
                        bool isCheckPoleNo,
                        int OneSidePoleNumb,
                        int nLeftPole,
                        float fontScale,
                        int textLineSpace,
                        cv::Point position //Draw top-left start position
)
{
    bool result = true;
    const int detectedPoleNumb = listAnode2CathodeDistance.size();
    //constexpr float fontScale = 0.7;
    constexpr int fontWeight = 2;
    // constexpr int textLineSpace = 25;
    constexpr int floatingPrecesion = 2;
    constexpr int lineThickness = 3;
    const int txtSpace = 2;
    constexpr int txtFloatWidth = 5;
    constexpr int txtNumWidth = 5;
    constexpr int txtPosWidth = txtFloatWidth + txtSpace;
    constexpr int txtAnode2CathodeWidth = txtFloatWidth + txtSpace;
    constexpr int txtAnode2CathodeLimitWidth = 4 * 2 + 3 + txtSpace;
    constexpr int txtAnode2CaseWidth = txtFloatWidth + txtSpace + 1;
    constexpr int txtAnode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;

    std::ostringstream anode2CathodeLimitStr;
    anode2CathodeLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << MinCathode2Anode << "~" << MaxCathode2Anode << "]";

    std::ostringstream anode2CaseLimitStr;
    anode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << MinAnode2Case << "~" << MaxAnode2Case << "]";

    std::ostringstream anode2CaseOffsetStr;
    anode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << anode2CaseOffset;

    std::ostringstream cathode2anodeOffsetStr;
    cathode2anodeOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2AnodeOffset;

    cv::String poleNumStr;

    int paddingX = 40;
    //int putTxtOffsetY = (resImg.rows <= 1800) ? resImg.rows * 0.64 : resImg.rows * 0.68;

    //cv::Scalar inforColor(180, 10, 10);
    cv::Scalar inforColor(245, 164, 66);
    cv::Scalar OKColor(60, 193, 5);
    cv::Scalar NGColor(0, 0, 255);
    cv::Scalar anode2CathodeColor(inforColor);
    cv::Scalar anode2CaseColor(inforColor);
    cv::Scalar anodeColor(inforColor);
    cv::Scalar cathodeColor(180, 10, 10);
    cv::Point startPos;

    std::ostringstream tmpStream;
    std::vector<std::pair<std::string, cv::Scalar>> resultHeader(6, std::pair<std::string, cv::Scalar>("", inforColor));
    tmpStream << std::left << std::setw(txtNumWidth) << std::setfill(' ') << "| No";
    resultHeader[0].first = tmpStream.str();
    tmpStream.str("");
    tmpStream << std::left << std::setw(txtPosWidth) << std::setfill(' ') << " | Pos";
    resultHeader[1].first = tmpStream.str();
    tmpStream.str("");
    tmpStream << std::left << std::setw(txtAnode2CathodeWidth + txtAnode2CathodeLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
              << " | Length[Range]" << cathode2anodeOffsetStr.str();
    resultHeader[2].first = tmpStream.str();
    tmpStream.str("");
    tmpStream << std::left << std::setw(txtAnode2CaseWidth + txtAnode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
              << " | A2Case[Range]" << anode2CaseOffsetStr.str();
    resultHeader[4].first = tmpStream.str();
    resultHeader[5].first = " |";

    auto msg =
        resultHeader[0].first + resultHeader[1].first + resultHeader[2].first + resultHeader[3].first + resultHeader[4].first + resultHeader[5].first;
    cv::Size txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, fontScale, fontWeight, 0);

    std::vector<std::pair<std::string, cv::Scalar>> resultInfo(6, std::pair<std::string, cv::Scalar>("", inforColor));
    resultInfo[3].first = anode2CathodeLimitStr.str();
    resultInfo[5].first = anode2CaseLimitStr.str();

    assert(listAnode2CathodeDistance.size() == listAnode2CaseDistance.size());

    for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
    {
        bool isAnode2CathodeError = (!listAno2CathodDecision.empty()) ? !listAno2CathodDecision[poleIdx] : false;
        bool isAnode2CaseError = (!listAno2CaseDecision.empty()) ? !listAno2CaseDecision[poleIdx] : false;

        //result &= isAnode2CathodeError || isAnode2CaseError;

        anode2CathodeColor = isAnode2CathodeError ? NGColor : OKColor;
        anode2CaseColor = isAnode2CaseError ? NGColor : OKColor;
        anodeColor = isAnode2CathodeError || isAnode2CaseError ? NGColor : OKColor;

        //Assign the pole number
        std::ostringstream poleNumber;
        poleNumber << std::fixed << std::right << std::setw(txtNumWidth - 3) << std::setfill(' ') << "P" << std::setw(2) << std::setfill('0')
                   << (poleIdx + 1);
        resultInfo[0].first = poleNumber.str();
        resultInfo[0].second = anodeColor;

        //Assign the X pole position
        std::ostringstream strAnode2Boundery;
        strAnode2Boundery << std::right << std::setw(txtPosWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                          << listPolePos[poleIdx];
        resultInfo[1].first = strAnode2Boundery.str();

        std::ostringstream strCath2Ano;
        strCath2Ano << std::setw(txtAnode2CathodeWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                    << listAnode2CathodeDistance[poleIdx];
        //Assign the anode to cathode infor
        resultInfo[2].first = strCath2Ano.str();
        resultInfo[2].second = anode2CathodeColor;

        std::ostringstream strAno2Case;
        strAno2Case << std::setw(txtAnode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                    << listAnode2CaseDistance[poleIdx];
        //Assign the anode to case infor
        resultInfo[4].first = strAno2Case.str();
        resultInfo[4].second = anode2CaseColor;

        //Cathode 2 Anode Measurement
        if (poleIdx < nLeftPole)
        {
            startPos.x = position.x + paddingX;
            startPos.y = position.y + (poleIdx % nLeftPole) * textLineSpace;
        }
        else
        {
            startPos.x = resImg.cols - txtSize.width - paddingX;
            startPos.y = position.y + (poleIdx - nLeftPole) * textLineSpace;
        }

        bool isFirstLeft = poleIdx == 0;
        bool isFirstRight = (poleIdx - nLeftPole) == 0;
        if (isFirstLeft || isFirstRight)
        {
            drawText(resImg, cv::Point(startPos.x, startPos.y - textLineSpace), resultHeader, fontScale, fontWeight);
        }

        drawText(resImg, startPos, resultInfo, fontScale, fontWeight);
    }

    ///Plot missing pole to results
    if (isCheckPoleNo)
    {
        int nRightPoles = detectedPoleNumb - nLeftPole;
        int nLeftMiss = OneSidePoleNumb - nLeftPole;
        int nRightMiss = OneSidePoleNumb - nRightPoles;
        poleNumStr = " P##   Missing";

        for (int pIdx = 0; pIdx < nLeftMiss; pIdx++)
        {
            int missPoleIdx = nLeftPole + pIdx;
            startPos = cv::Point(position.x + paddingX, position.y + missPoleIdx * textLineSpace);
            drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
        }

        startPos.x = resImg.cols - txtSize.width - paddingX;
        for (int pIdx = 0; pIdx < nRightMiss; pIdx++)
        {
            int missPoleIdx = nRightPoles + pIdx;
            startPos.y = position.y + missPoleIdx * textLineSpace;
            drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
        }
    }

    return result;
}

XVT_LIB_EXPORTS bool drawPoleTextResult(CylinderBatteryBase *BInspect, BatteryInspectionResult &IResult)
{
    cv::Mat resImg = IResult.resImg;
    const VecDouble &listAnode2CathodeDistance = IResult.sCathode2Anode;
    const VecDouble &listAnode2CaseDistance = IResult.sAnode2Case;
    const VecDouble &listCathode2CaseDistance = IResult.sCathode2Case;
    const VecDouble &listPolePos = IResult.sXPos;
    const std::vector<bool> &listAno2CathodDecision = IResult.vtAno2CathodDecision;
    const std::vector<bool> &listAno2CaseDecision = IResult.vtAno2CaseDecision;
    const std::vector<bool> &listCathode2CaseDecision = IResult.vtCathode2CaseDecision;
    int nLeftPole = IResult.nLeftPole;
    double cathode2AnodeOffset = BInspect->mCathode2AnodeOffset;
    double anode2CaseOffset = BInspect->mAnode2CaseOffset;
    double cathode2CaseOffset = BInspect->mCathode2CaseOffset;
    bool isCheckPoleNo = BInspect->mIsCheckPoleNo;
    int OneSidePoleNumb = BInspect->mOneSidePoleNumber;
    float fontScale = BInspect->mTextFontScale;
    int textLineSpace = BInspect->mTextLineSpace;
    cv::Point position = BInspect->mTextPosition;

    bool result = true;
    const int detectedPoleNumb = listAnode2CathodeDistance.size();
    //constexpr float fontScale = 0.7;
    constexpr int fontWeight = 2;
    // constexpr int textLineSpace = 25;
    constexpr int floatingPrecesion = 2;
    constexpr int lineThickness = 3;
    const int txtSpace = 2;
    constexpr int txtFloatWidth = 5;
    constexpr int txtNumWidth = 5;
    constexpr int txtPosWidth = txtFloatWidth + txtSpace;
    constexpr int txtAnode2CathodeWidth = txtFloatWidth + txtSpace;
    constexpr int txtAnode2CathodeLimitWidth = 4 * 2 + 3 + txtSpace;
    constexpr int txtAnode2CaseWidth = txtFloatWidth + txtSpace + 1;
    constexpr int txtAnode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;
    constexpr int txtCathode2CaseWidth = txtFloatWidth + txtSpace + 1;
    constexpr int txtCathode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;

    std::ostringstream anode2CathodeLimitStr;
    anode2CathodeLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << BInspect->mValidCathode2AnodeRange.GetLower() << "~"
                          << BInspect->mValidCathode2AnodeRange.GetUpper() << "]";

    std::ostringstream anode2CaseLimitStr;
    anode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << BInspect->mValidAnode2CaseRange.GetLower() << "~"
                       << BInspect->mValidAnode2CaseRange.GetUpper() << "]";

    std::ostringstream anode2CaseOffsetStr;
    anode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << anode2CaseOffset;

    std::ostringstream cathode2anodeOffsetStr;
    cathode2anodeOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2AnodeOffset;

    std::ostringstream cathode2CaseLimitStr;
    cathode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << BInspect->mValidCathode2CaseRange.GetLower() << "~"
                         << BInspect->mValidCathode2CaseRange.GetUpper() << "]";

    std::ostringstream cathode2CaseOffsetStr;
    cathode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2CaseOffset;

    cv::String poleNumStr;

    int paddingX = 40;
    //int putTxtOffsetY = (resImg.rows <= 1800) ? resImg.rows * 0.64 : resImg.rows * 0.68;

    //cv::Scalar inforColor(180, 10, 10);
    cv::Scalar inforColor(245, 164, 66);
    cv::Scalar OKColor(60, 193, 5);
    cv::Scalar NGColor(0, 0, 255);
    cv::Scalar anode2CathodeColor(inforColor);
    cv::Scalar anode2CaseColor(inforColor);
    cv::Scalar cathode2CaseColor(inforColor);
    cv::Scalar poleColor(inforColor);
    cv::Scalar cathodeColor(180, 10, 10);
    cv::Point startPos;

    std::ostringstream tmpStream;
    std::vector<std::pair<std::string, cv::Scalar>> resultHeader(8, std::pair<std::string, cv::Scalar>("", inforColor));
    tmpStream << std::left << std::setw(txtNumWidth) << std::setfill(' ') << "| No";
    resultHeader[0].first = tmpStream.str();
    tmpStream.str("");
    tmpStream << std::left << std::setw(txtPosWidth) << std::setfill(' ') << " | Pos";
    resultHeader[1].first = tmpStream.str();
    tmpStream.str("");
    std::vector<std::pair<std::string, cv::Scalar>> resultInfo(8, std::pair<std::string, cv::Scalar>("", inforColor));
    if (BInspect->mInspectingItems.ANODE_TO_CATHODE_LENGTH)
    {
        tmpStream << std::left << std::setw(txtAnode2CathodeWidth + txtAnode2CathodeLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
                  << " | Length[Range]" << cathode2anodeOffsetStr.str();
        resultInfo[3].first = anode2CathodeLimitStr.str();
    }
    resultHeader[2].first = tmpStream.str();
    tmpStream.str("");

    if (BInspect->mInspectingItems.ANODE_TO_CASE_GAP)
    {
        tmpStream << std::left << std::setw(txtAnode2CaseWidth + txtAnode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
                  << " | A2Case[Range]" << anode2CaseOffsetStr.str();
        resultInfo[5].first = anode2CaseLimitStr.str();
    }
    resultHeader[4].first = tmpStream.str();
    tmpStream.str("");
    if (BInspect->mInspectingItems.CATHODE_TO_CASE_GAP)
    {
        tmpStream << std::left << std::setw(txtCathode2CaseWidth + txtCathode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
                  << " | Cat2Case[Range]" << cathode2CaseOffsetStr.str();
        resultInfo[7].first = cathode2CaseLimitStr.str();
    }
    resultHeader[6].first = tmpStream.str();

    tmpStream.str("");
    resultHeader[7].first = " |";

    auto msg = resultHeader[0].first + resultHeader[1].first + resultHeader[2].first + resultHeader[3].first + resultHeader[4].first +
               resultHeader[5].first + resultHeader[6].first + resultHeader[7].first;
    cv::Size txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, fontScale, fontWeight, 0);

    assert(listAnode2CathodeDistance.size() == listAnode2CaseDistance.size());

    for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
    {
        bool isAnode2CathodeError = (!listAno2CathodDecision.empty()) ? !listAno2CathodDecision[poleIdx] : false;
        bool isAnode2CaseError = (!listAno2CaseDecision.empty()) ? !listAno2CaseDecision[poleIdx] : false;
        bool isCathode2CaseError = (!listCathode2CaseDecision.empty()) ? !listCathode2CaseDecision[poleIdx] : false;
        //result &= isAnode2CathodeError && isAnode2CaseError && isCathode2CaseError;

        anode2CathodeColor = isAnode2CathodeError ? NGColor : OKColor;
        anode2CaseColor = isAnode2CaseError ? NGColor : OKColor;
        cathode2CaseColor = isCathode2CaseError ? NGColor : OKColor;
        poleColor = (isAnode2CathodeError || isAnode2CaseError || isCathode2CaseError) ? NGColor : OKColor;

        //Assign the pole number
        std::ostringstream poleNumber;
        poleNumber << std::fixed << std::right << std::setw(txtNumWidth - 3) << std::setfill(' ') << "P" << std::setw(2) << std::setfill('0')
                   << (poleIdx + 1);
        resultInfo[0].first = poleNumber.str();
        resultInfo[0].second = poleColor;

        //Assign the X pole position
        std::ostringstream strAnode2Boundery;
        strAnode2Boundery << std::right << std::setw(txtPosWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                          << listPolePos[poleIdx];
        resultInfo[1].first = strAnode2Boundery.str();

        if (BInspect->mInspectingItems.ANODE_TO_CATHODE_LENGTH)
        {
            std::ostringstream strCath2Ano;
            strCath2Ano << std::setw(txtAnode2CathodeWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                        << listAnode2CathodeDistance[poleIdx];
            //Assign the anode to cathode infor
            resultInfo[2].first = strCath2Ano.str();
            resultInfo[2].second = anode2CathodeColor;
        }

        if (BInspect->mInspectingItems.ANODE_TO_CASE_GAP)
        {
            std::ostringstream strAno2Case;
            strAno2Case << std::setw(txtAnode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                        << listAnode2CaseDistance[poleIdx];
            //Assign the anode to case infor
            resultInfo[4].first = strAno2Case.str();
            resultInfo[4].second = anode2CaseColor;
        }

        if (BInspect->mInspectingItems.CATHODE_TO_CASE_GAP)
        {
            std::ostringstream strCat2Case;
            strCat2Case << std::setw(txtCathode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                        << listCathode2CaseDistance[poleIdx];
            //Assign the anode to case infor
            resultInfo[6].first = strCat2Case.str();
            resultInfo[6].second = cathode2CaseColor;
        }

        //Cathode 2 Anode Measurement
        if (poleIdx < nLeftPole)
        {
            startPos.x = position.x + paddingX;
            startPos.y = position.y + (poleIdx % nLeftPole) * textLineSpace;
        }
        else
        {
            startPos.x = resImg.cols - txtSize.width - paddingX;
            startPos.y = position.y + (poleIdx - nLeftPole) * textLineSpace;
        }

        bool isFirstLeft = poleIdx == 0;
        bool isFirstRight = (poleIdx - nLeftPole) == 0;
        if (isFirstLeft || isFirstRight)
        {
            drawText(resImg, cv::Point(startPos.x, startPos.y - textLineSpace), resultHeader, fontScale, fontWeight);
        }

        drawText(resImg, startPos, resultInfo, fontScale, fontWeight);
    }

    ///Plot missing pole to results
    if (isCheckPoleNo)
    {
        int nRightPoles = detectedPoleNumb - nLeftPole;
        int nLeftMiss = OneSidePoleNumb - nLeftPole;
        int nRightMiss = OneSidePoleNumb - nRightPoles;
        poleNumStr = " P##   Missing";

        for (int pIdx = 0; pIdx < nLeftMiss; pIdx++)
        {
            int missPoleIdx = nLeftPole + pIdx;
            startPos = cv::Point(position.x + paddingX, position.y + missPoleIdx * textLineSpace);
            drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
        }

        startPos.x = resImg.cols - txtSize.width - paddingX;
        for (int pIdx = 0; pIdx < nRightMiss; pIdx++)
        {
            int missPoleIdx = nRightPoles + pIdx;
            startPos.y = position.y + missPoleIdx * textLineSpace;
            drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
        }
    }

    return result;
}

//#define DEBUG_SHADOW
#ifdef DEBUG_SHADOW
cv::Mat imgDebug;
cv::Mat imgDebug1;
cv::Mat srcImgDebug;
VecInt listPole1 = {8, 9, 10, 28, 29, 30, 31, 32};
#endif
int plotPoleWithRect(cv::Mat &resImg,
                     const cv::Rect &poleRegionROI,
                     int CenterNeglectionWidth,
                     int SkipPoleDistance,
                     int nLeftPole,
                     int lineType,
                     bool isShowGrid,
                     const VecPoint &lstAnodePnt,
                     const VecPoint &lstCathodePnt,
                     const std::vector<bool> &listAno2CathodDecision,
                     const std::vector<bool> &listAno2CaseDecision,
                     const std::vector<bool> &listCathode2CaseDecision,
                     const std::vector<std::pair<int, int>> &lstPolePositionErr)
{
#ifdef DEBUG_SHADOW
    imgDebug = resImg.clone();
    imgDebug1 = resImg.clone();
    cv::cvtColor(imgDebug, srcImgDebug, cv::COLOR_RGB2GRAY);
#endif
    const int centerLeft = (poleRegionROI.width - CenterNeglectionWidth) / 2;
    const int centerRight = (poleRegionROI.width + CenterNeglectionWidth) / 2;

    VecInt lstCenterRef = {(poleRegionROI.x + centerLeft), (poleRegionROI.x + centerRight)};

    const int detectedPoleNumb = lstAnodePnt.size();
    constexpr int lineThickness = 3;

    cv::Point topPoint =
        *std::min_element(lstAnodePnt.begin(), lstAnodePnt.end(), [](const cv::Point &lp, const cv::Point &rp) { return lp.y < rp.y; });
    cv::Point botPoint =
        *std::max_element(lstCathodePnt.begin(), lstCathodePnt.end(), [](const cv::Point &lp, const cv::Point &rp) { return lp.y < rp.y; });

    int poleIdx = 0;
    int poleDistance = 15;
    int rectPoleHeight = botPoint.y - topPoint.y;
    int LimitDrawRight = poleRegionROI.x + poleRegionROI.width - SkipPoleDistance;
    cv::Rect poleRect(poleRegionROI.x + SkipPoleDistance + 1 - poleDistance, topPoint.y - poleDistance, poleDistance, rectPoleHeight + poleDistance * 2);
    int startLineX = 0, endLineX = 0;
    bool enabledDrawCurvedLine = (lineType == 1);

    std::vector<cv::Point> vtDrawCathode, vtDrawAnode;
    if (detectedPoleNumb > 0 && enabledDrawCurvedLine)
    {
        int start = poleDistance / 2;
        for (int z = start; z > 0; z--) {
            vtDrawCathode.emplace_back(lstCathodePnt[0].x - z, lstCathodePnt[0].y);
            vtDrawAnode.emplace_back(lstAnodePnt[0].x - z, lstAnodePnt[0].y);
        }

        for(int i = 1; i < detectedPoleNumb; i++)
        {
            float distance = lstCathodePnt[i].x - lstCathodePnt[i - 1].x;
            if (distance > CenterNeglectionWidth )
            {
                int halfDistance = poleDistance / 2;
                for(int z = 0; z < halfDistance; z++)
                {
                    vtDrawCathode.emplace_back(lstCathodePnt[i-1].x + z, lstCathodePnt[i-1].y);
                    vtDrawAnode.emplace_back(lstAnodePnt[i-1].x + z, lstAnodePnt[i-1].y);
                }

                for (int z = halfDistance; z > 0; z--)
                {
                    vtDrawCathode.emplace_back(lstCathodePnt[i].x - z, lstCathodePnt[i].y);
                    vtDrawAnode.emplace_back(lstAnodePnt[i].x - z, lstAnodePnt[i].y);
                }
            }
            else
            {
                for (auto j = lstCathodePnt[i-1].x; j < lstCathodePnt[i].x; j++)
                {
                    float xNext = j;
                    float alpha = abs(j - lstCathodePnt[i - 1].x) / abs(distance);
                    float yNext = lstCathodePnt[i - 1].y * (1 - alpha) + lstCathodePnt[i].y * alpha;
                    vtDrawCathode.emplace_back(xNext, yNext);
                    float yNextAnode = lstAnodePnt[i - 1].y * (1 - alpha) + lstAnodePnt[i].y * alpha;
                    vtDrawAnode.emplace_back(xNext, yNextAnode);
                }
            }

            if (i == detectedPoleNumb - 1)
            {
                int halfDistance = poleDistance / 2;
                for(int z = 0; z < halfDistance; z++)
                {
                    int x = lstCathodePnt[i].x + z;
                    int y = lstCathodePnt[i].y;
                    vtDrawCathode.emplace_back(x, y);
                    vtDrawAnode.emplace_back(x, lstAnodePnt[i].y);
                }
            }
        }
    }

    /*if (enabledDraw)
        for(auto& p : vtDrawCathode)
            cv::circle(resImg, p, 1, cv::Scalar(255, 0, 0));*/

    cv::String poleNumStr;
    cv::String poleDistanceStr;

    //cv::Scalar inforColor(180, 10, 10);
    cv::Scalar inforColor(245, 164, 66);
    cv::Scalar OKColor(60, 193, 5);
    cv::Scalar cathodeOKColor(180, 10, 10);
    cv::Scalar NGColor(0, 0, 255);

    cv::Scalar anodeColor(inforColor);
    cv::Scalar cathodeColor(180, 10, 10);

    for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
    {
        bool isAnode2CathodeError = listAno2CathodDecision[poleIdx];
        bool isAnode2CaseError = listAno2CaseDecision[poleIdx];
        bool isCathode2CaseError = listCathode2CaseDecision[poleIdx];

        anodeColor = isAnode2CathodeError && isAnode2CaseError ? OKColor : NGColor;
        cathodeColor = isCathode2CaseError ? cathodeOKColor : NGColor;

        int MidPos = 0;
        //Cathode 2 Anode Measurement
        if (poleIdx < nLeftPole)
        {

            if (poleIdx < nLeftPole - 1)
            {
                MidPos = (lstAnodePnt[poleIdx + 1].x + lstAnodePnt[poleIdx].x) / 2;
            }
            else
            {
                MidPos = lstAnodePnt[poleIdx].x + poleDistance;
                MidPos = MidPos > lstCenterRef[0] ? lstCenterRef[0] : MidPos;
            }
            poleRect.x += poleRect.width - 1;
            poleRect.width = MidPos - poleRect.x;
        }
        else
        {
            if (poleIdx < detectedPoleNumb - 1)
            {
                MidPos = (lstAnodePnt[poleIdx + 1].x + lstAnodePnt[poleIdx].x) / 2;
            }
            else
            {
                MidPos = lstAnodePnt[poleIdx].x + poleDistance;
                if (MidPos > LimitDrawRight) {
                    MidPos = LimitDrawRight;
                }
            }

            if (poleIdx == nLeftPole)
            {
                poleRect.x = lstAnodePnt[poleIdx].x - poleDistance;
                poleRect.x = poleRect.x < lstCenterRef[1] ? lstCenterRef[1] : poleRect.x;
            }
            else
            {
                poleRect.x += poleRect.width - 1;
            }
            poleRect.width = MidPos - poleRect.x;
        }

        // showPoleBox
        bool distanceError = false;
        for (int k = 0; k < lstPolePositionErr.size(); k++)
        {
            if (lstPolePositionErr[k].first == lstCathodePnt[poleIdx].x)
            {
                distanceError = true;
                break;
            }
        }

        if (!(distanceError == true && poleIdx < detectedPoleNumb - 1) && isShowGrid)
        {
            cv::rectangle(resImg, poleRect, cv::Scalar(220, 220, 220));
        }

        startLineX = lstAnodePnt[poleIdx].x - 5;
        startLineX = startLineX <= poleRect.x ? poleRect.x + lineThickness : startLineX;
        endLineX = lstAnodePnt[poleIdx].x + 5;
        endLineX = endLineX >= MidPos ? MidPos - lineThickness : endLineX;

        if (enabledDrawCurvedLine)
        {
            while (!vtDrawAnode.empty())
            {
                auto drawCathodePoint = vtDrawCathode.begin();
                cv::circle(resImg, *drawCathodePoint, 1, cathodeColor);
                vtDrawCathode.erase(drawCathodePoint);

                auto drawAnodePoint = vtDrawAnode.begin();
                cv::circle(resImg, *drawAnodePoint, 1, anodeColor);
                vtDrawAnode.erase(drawAnodePoint);

                if (vtDrawCathode.empty() || vtDrawCathode.front().x > poleRect.x + poleRect.width
                    || vtDrawAnode.empty())
                    break;
            }
        }
        else
        {
            cv::line(resImg, cv::Point(startLineX, lstCathodePnt[poleIdx].y), cv::Point(endLineX, lstCathodePnt[poleIdx].y), cathodeColor, lineThickness);
            cv::line(resImg, cv::Point(startLineX, lstAnodePnt[poleIdx].y), cv::Point(endLineX, lstAnodePnt[poleIdx].y), anodeColor, lineThickness);
        }
#ifdef DEBUG_SHADOW
        int readIdx = poleIdx + 1;
        cv::Rect poleDebugRect =
            cv::Rect(startLineX, lstAnodePnt[poleIdx].y, endLineX - startLineX, abs(lstAnodePnt[poleIdx].y - lstCathodePnt[poleIdx].y));
        cv::rectangle(imgDebug, poleDebugRect, cv::Scalar(220, 220, 220), 1);
        cv::putText(imgDebug, std::to_string(readIdx), cv::Point(startLineX, lstAnodePnt[poleIdx].y - 5), 1.5, 0.5, COLOR_CV_RED, 1);
        cv::line(imgDebug,
                 cv::Point(startLineX, lstCathodePnt[poleIdx].y),
                 cv::Point(endLineX, lstCathodePnt[poleIdx].y),
                 cathodeColor,
                 lineThickness);
        cv::line(imgDebug, cv::Point(startLineX, lstAnodePnt[poleIdx].y), cv::Point(endLineX, lstAnodePnt[poleIdx].y), anodeColor, lineThickness);
        cv::Mat poleDebugImg = srcImgDebug(poleDebugRect);
        cv::Mat1f reduceRowImg;
        cv::reduce(poleDebugImg, reduceRowImg, 1, cv::REDUCE_AVG, CV_32F); //Reduce to one row.
        std::vector<float> columnsum = cv::Mat_<float>(reduceRowImg);
        cv::Mat Kernely = (cv::Mat_<float>(2, 1) << -1, 1);
        cv::Mat dy;
        cv::filter2D(reduceRowImg, dy, -1, Kernely, cv::Point(-1, -1), 0.0, cv::BORDER_REPLICATE);

        dy = cv::abs(dy);
        cv::threshold(dy, dy, 0, 255, cv::THRESH_TOZERO);
        // Remove padding and take the abs of the output
        //dy = cv::Mat(dy, cv::Rect(0, 1, dy.cols, dy.rows - 1));
        //cv::Sobel(columnsum, columnsumDiff, -1, 1, 0, 3);
        std::vector<float> columnsumDiff = cv::Mat_<float>(dy);
        cv::cvtColor(poleDebugImg, poleDebugImg, cv::COLOR_GRAY2BGR);
        std::vector<size_t> lIdx(columnsum.size());
        std::iota(lIdx.begin(), lIdx.end(), 0);
        Drawing drawTool;
        drawTool.yTopOrigin = true;
        drawTool.thickness = 1;
        drawTool.color = cv::Scalar(0, 255, 255);
        cv::Mat poleDebugImg1 = poleDebugImg.clone();
        drawTool.Plot(poleDebugImg1, columnsumDiff, lIdx);
        poleDebugImg1.copyTo(imgDebug1(poleDebugRect));

        if (std::find(listPole1.begin(), listPole1.end(), readIdx) != listPole1.end())
        {
            cv::imwrite("pole" + std::to_string(readIdx) + ".png", poleDebugImg);
            cv::imwrite("poleDebug" + std::to_string(readIdx) + ".png", poleDebugImg1);
        }
#endif
    }

    for (auto &X : lstPolePositionErr)
    {
        cv::Rect rect = cv::Rect(X.first, topPoint.y - 15, X.second - X.first, rectPoleHeight + 30);
        cv::rectangle(resImg, rect, cv::Scalar(0, 0, 255), 2);
    }

    //Print poles region area rectangle
    cv::rectangle(resImg, poleRegionROI, cv::Scalar(120, 120, 255), 2);

    //Print 2 sides neglection area line
    cv::line(resImg,
             cv::Point(lstCenterRef[0], poleRegionROI.y),
             cv::Point(lstCenterRef[0], poleRegionROI.y + poleRegionROI.height),
             cv::Scalar(255, 255, 0),
             2);
    cv::line(resImg,
             cv::Point(lstCenterRef[1], poleRegionROI.y),
             cv::Point(lstCenterRef[1], poleRegionROI.y + poleRegionROI.height),
             cv::Scalar(255, 255, 0),
             2);
#ifdef DEBUG_SHADOW
    cv::imwrite("test.jpg", imgDebug(poleRegionROI));
    cv::imwrite("test1.jpg", imgDebug1(poleRegionROI));
#endif
    return 0;
}

SegmentMeans calc_mean_region(const std::vector<float>& signal, int neglect_center) {
    SegmentMeans means = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    if (signal.size() - neglect_center < 7 || signal.empty()) {
        return means;
    }
    int IdxleftEnd = (int)((signal.size() - neglect_center) >> 1);

    int segmentSize = IdxleftEnd / 3;
    means.meanLeftOuter = std::accumulate(signal.begin(), signal.begin() + segmentSize,0.0f) / segmentSize;
    means.meanLeftMiddle = std::accumulate(signal.begin() + segmentSize, signal.begin() + 2 * segmentSize, 0.0f)/ segmentSize;
    means.meanLeftInner = std::accumulate(signal.begin() + 2 * segmentSize, signal.begin() + IdxleftEnd, 0.0f) / segmentSize;
    means.meanRightInner = std::accumulate(signal.end () - 3 * segmentSize, signal.end() - 2 * segmentSize, 0.0f) / segmentSize;
    means.meanRightMiddle = std::accumulate(signal.end() - 2 * segmentSize, signal.end() - segmentSize, 0.0f) / segmentSize;
    means.meanRightOuter = std::accumulate(signal.end() - segmentSize, signal.end(), 0.0f) / segmentSize;
    return means;
}

void MeanCalibration(const cv::Mat& src, cv::Mat& dst, std::vector<cv::Rect>& regions, std::vector<float>& listMean, bool enabled)
{
    if (!src.empty() && !regions.empty())
	{
		listMean.clear();
        float valueConst = 255;
		float defaultValue = 3;
		float sumMean = 0.0f;

		for (cv::Rect& region : regions)
		{
            region = CreateROI(region, src.size());
			float meanValue = 0.0f;
            if(!region.empty())
            {
                meanValue = cv::mean(src(region))[0];
                sumMean += meanValue;
            }

            listMean.push_back(meanValue);
		}

		float averMean = sumMean / (float)regions.size();

		if (abs(averMean - valueConst) > defaultValue && enabled) {
            dst = (valueConst / averMean) * src;
		}
        else
        {
            dst = src.clone();
        }
	}
	else
	{
        dst = src.clone();
	}
}

ERR_CODE PoleLengthRefinement(const VecInt &lstPolePosXAll,
                              VecInt &anodePos,
                              VecInt &cathodePos,
                              int widthROI,
                              std::string &strDesc,
                              cv::Mat debugImg)
{
    if (lstPolePosXAll.size() == 0 || anodePos.size() == 0 || cathodePos.size() == 0 || lstPolePosXAll.size() != anodePos.size() ||
        anodePos.size() != cathodePos.size())
    {
        return ERR_CODE::errPoleNotRefinable;
    }

    double MaxDis = 14;  //Max distance using for finding the matchign poles
    double MaxDiff = 10; // Using for deciding the untrusted poles by compare the pole length
    std::vector<PoleCompare> poleCompareList;
    poleCompareList = evaluatePoleForRefinement(lstPolePosXAll, anodePos, cathodePos, widthROI, MaxDis, MaxDiff, true);
#ifdef DEBUG_POLE_LENGTH_REFINE
    Drawing drawTool;
    VecPoint vtLeftPoly, vtRightPoly;
    cv::Scalar leftLineColor(108, 159, 188);
    cv::Scalar rightLineColor = COLOR_CV_BLUE;
    Polyfit poliFit;
    for (int i = 0; i < widthROI / 2; i++)
    {
        float xs = i;
        float ys = poliFit.f(coeffLs, order, xs) + avgLs;
        vtLeftPoly.push_back(cv::Point(xs, ys));
    }
    for (int i = 0; i < widthROI / 2; i++)
    {
        float xs = i;
        float ys = poliFit.f(coeffRs, order, xs) + avgRs;
        vtRightPoly.push_back(cv::Point(xs, ys));
    }
    cv::Mat polyImage = debugImg.empty() ? cv::Mat(200, widthROI / 2, CV_8UC3) : debugImg;
    drawTool.type = DrawType::Line;
    drawTool.color = leftLineColor;
    drawTool.plot(polyImage, std::string("+ Left"), cv::Point(20, 20));
    drawTool.plot(polyImage, vtLeftPoly, false);
    drawTool.color = rightLineColor;
    drawTool.plot(polyImage, std::string("O Right"), cv::Point(widthROI / 4, 20));
    drawTool.plot(polyImage, vtRightPoly, false);

    drawTool.type = DrawType::Plus;
    cv::Scalar leftColor;
    cv::Scalar rightColor;
    cv::Scalar compareColor;
    for (int i = 0; i < poleCompareList.size(); i++)
    {
        PoleCompare &p = poleCompareList[i];
        if (p.mType == PoleCompareStatus::Trust)
            compareColor = COLOR_CV_GREEN;
        else if (p.mType == PoleCompareStatus::Adjusted)
            compareColor = COLOR_YELLOW;
        else
            compareColor = COLOR_CV_RED;

        drawTool.type = DrawType::Plus;
        leftColor = ChoseColor(p.mLeftPole.mType, leftLineColor);
        drawTool.plot(polyImage, p.mLeftPole.mAnode, leftColor);
        drawTool.plot(polyImage, p.mLeftPole.mCathode, leftColor);

        drawTool.type = DrawType::Circle;
        rightColor = ChoseColor(p.mRightPole.mType, rightLineColor);
        drawTool.plot(polyImage, p.mRightPole.mAnode, rightColor);
        drawTool.plot(polyImage, p.mRightPole.mCathode, rightColor);

        cv::line(polyImage, p.mRightPole.mAnode, p.mLeftPole.mAnode, compareColor);
    }

    //        show_window("Debug PolyFit Add", polyImage);
    //        cv::waitKey(1);
#endif // DEBUG_POLE_LENGTH_REFINE

    // =========== Compare an change the value of pole base on the information of both side.===========//
    float leftRefPoleLength;
    float rightRefPoleLength;
    int countAdjustedPole = 0;
    PoleCompare *trustedPoleBefore = nullptr;
    PoleCompare *trustedPoleAfter = nullptr;
    std::vector<PoleCompare *> untrustPoleList;
    for (auto p = poleCompareList.begin(); p != poleCompareList.end(); p++)
    {
        if (p->mType == PoleCompareStatus::Trust)
        {
            if (!untrustPoleList.empty())
            {
                trustedPoleAfter = p._Ptr;
            }
            else
            {
                trustedPoleBefore = p._Ptr;
            }
        }
        else if (p->mType == PoleCompareStatus::Untrust)
        {
            untrustPoleList.push_back(p._Ptr);
        }

        if (trustedPoleAfter != nullptr || (p + 1) == poleCompareList.end())
        {
            //update all untrusted poles.
            if (trustedPoleAfter != nullptr && trustedPoleBefore != nullptr)
            {
                for (auto up : untrustPoleList)
                {
                    up->mLeftReferencePole = GetRefPole(up->mLeftPole.mAnode.x, trustedPoleBefore->mLeftPole, trustedPoleAfter->mLeftPole);
                    up->mRightReferencePole = GetRefPole(up->mRightPole.mAnode.x, trustedPoleBefore->mRightPole, trustedPoleAfter->mRightPole);

#if 0
                        if (up->mLeftPole.mType != PoleType::InsertedOutRange)
                        {
                            float& leftX = up->mLeftPole.mAnode.x;
                            cathodeFit = Polyfit::f(coeffLe, order, leftX) + avgLe;
                            anodeFit = Polyfit::f(coeffLs, order, leftX) + avgLs;
                            PoleInfo leftFitPole = PoleInfo(cv::Point(leftX, anodeFit), cv::Point(leftX, cathodeFit), PoleType::Inserted);
                            //If error = anodePos(pole) - anodePos(fitPole) is small then update trusted pole as fit pole
                            if (abs(leftFitPole.mAnode.y - up->mLeftPole.mAnode.y) < MaxDiff / 2.0)
                            {
                                up->mLeftReferencePole = leftFitPole;
                            }
                        }

                        if (up->mRightPole.mType != PoleType::InsertedOutRange)
                        {
                            float& rightX = up->mRightPole.mAnode.x;
                            cathodeFit = Polyfit::f(coeffRe, order, rightX) + avgRe;
                            anodeFit = Polyfit::f(coeffRs, order, rightX) + avgRs;
                            PoleInfo rightFitPole = PoleInfo(cv::Point(rightX, anodeFit), cv::Point(rightX, cathodeFit), PoleType::Inserted);
                            //If error = anodePos(pole) - anodePos(fitPole) is small then update trusted pole as fit pole
                            if (abs(rightFitPole.mAnode.y - up->mRightPole.mAnode.y) < MaxDiff / 2.0)
                            {
                                up->mRightReferencePole = rightFitPole;
                            }
                        }
#endif

                    //Ajust the untrust pole by the length of the trust pole
                    if (up->AdjustUncertainType(MaxDiff))
                    {
                        countAdjustedPole++;
                    }
                }
            }
            else if (trustedPoleBefore != nullptr)
            {
                // If we can find only the before trusted pole.
                for (auto up : untrustPoleList)
                {
                    up->mLeftReferencePole = trustedPoleBefore->mLeftPole;
                    up->mRightReferencePole = trustedPoleBefore->mRightPole;
                    //Ajust the untrust pole by the length of the trust pole
                    if (up->AdjustUncertainType(MaxDiff))
                    {
                        countAdjustedPole++;
                    }

                    if (up->mLeftPole.mType < PoleType::NotFound && up->mRightPole.mType < PoleType::NotFound)
                    {
                        trustedPoleBefore = up;
                    }
                }
            }
            else if (trustedPoleAfter != nullptr)
            {
                // If we can find only the after trusted pole.
                auto pStart = untrustPoleList.rbegin();
                auto pEnd = untrustPoleList.rend();
                auto tmpTrustPole = trustedPoleAfter;
                for (auto up = pStart; up != pEnd; up++)
                {
                    (*up)->mLeftReferencePole = tmpTrustPole->mLeftPole;
                    (*up)->mRightReferencePole = tmpTrustPole->mRightPole;

                    //Ajust the untrust pole by the length of the trust pole
                    if ((*up)->AdjustUncertainType(MaxDiff))
                    {
                        countAdjustedPole++;
                    }
                    if ((*up)->mLeftPole.mType < PoleType::NotFound && (*up)->mRightPole.mType < PoleType::NotFound)
                    {
                        tmpTrustPole = *up;
                    }
                }
            }

            untrustPoleList.clear();
            trustedPoleBefore = trustedPoleAfter;
            trustedPoleAfter = nullptr;
        }
    }

#ifdef DEBUG_POLE_LENGTH_REFINE
    for (int i = 0; i < poleCompareList.size(); i++)
    {
        PoleCompare &p = poleCompareList[i];
        if (p.mType == PoleCompareStatus::Adjusted)
        {
            compareColor = COLOR_YELLOW;

            drawTool.type = DrawType::Plus;
            leftColor = ChoseColor(p.mLeftPole.mType, leftLineColor);
            drawTool.plot(polyImage, p.mLeftPole.mAnode, leftColor);
            drawTool.plot(polyImage, p.mLeftPole.mCathode, leftColor);

            drawTool.type = DrawType::Circle;
            rightColor = ChoseColor(p.mRightPole.mType, rightLineColor);
            drawTool.plot(polyImage, p.mRightPole.mAnode, rightColor);
            drawTool.plot(polyImage, p.mRightPole.mCathode, rightColor);

            cv::line(polyImage, p.mRightPole.mAnode, p.mLeftPole.mAnode, compareColor);
        }
    }

    show_window("Debug PolyFit Add", polyImage);
    cv::waitKey(1);
#endif // DEBUG_POLE_LENGTH_REFINE

    anodePos.clear();
    cathodePos.clear();
    for (const PoleCompare &p : poleCompareList)
    {
        // Collect all the left pole that is not inserted
        if (p.mLeftPole.mType == PoleType::Real || p.mLeftPole.mType == PoleType::Adjusted)
        {
            anodePos.push_back(p.mLeftPole.mAnode.y);
            cathodePos.push_back(p.mLeftPole.mCathode.y);
        }
    }

    for (auto p = poleCompareList.rbegin(); p != poleCompareList.rend(); p++)
    {
        // Collect all the right pole that is not inserted
        if (p->mRightPole.mType == PoleType::Real || p->mRightPole.mType == PoleType::Adjusted)
        {
            anodePos.push_back(p->mRightPole.mAnode.y);
            cathodePos.push_back(p->mRightPole.mCathode.y);
        }
    }

    if (countAdjustedPole)
        strDesc = std::to_string(countAdjustedPole) + " Poles Adjusted";

    if ((lstPolePosXAll.size() != anodePos.size()) || (anodePos.size() != cathodePos.size()))
    {
        strDesc = "Polelength Refinement changed the detected pole number";
        return ERR_CODE::errPoleRefinement;
    }
    return ERR_CODE::OK;
}

int findClosestPoint(VecInt vtDst, int value, int maxRange)
{
    if (vtDst.size() <= 0)
        return -1;
    auto it = std::lower_bound(vtDst.begin(), vtDst.end(), value);
    int found = -1;
    int vIdx;
    if (it == vtDst.end())
    {
        found = vtDst.back();
        if (abs(found - value) <= maxRange)
        {
            return (vtDst.size() - 1);
        }
        else
            return -1;
    }
    found = *it;
    vIdx = std::distance(vtDst.begin(), it);
    if (it != vtDst.begin())
    {
        int found2 = *(--it);
        if (abs(value - found2) < abs(value - found))
        {
            found = found2;
            vIdx -= 1;
        }
        if (abs(found - value) > maxRange)
            vIdx = -1;
    }
    else
    {
        if (abs(found - value) > maxRange)
            vIdx = -1;
    }
    return vIdx;
}

std::pair<VecInt, VecDouble> RemoveCloseLine(std::vector<std::tuple<int, double, double>> poleList, int minDis)
{
    VecInt finalpolePosition;
    VecDouble finalpoleIntensity;

    if (poleList.size() > 0)
    {
        // Add first pole to final result
        auto pole = poleList[0];
        finalpolePosition.push_back(std::get<0>(pole));
        finalpoleIntensity.push_back(std::get<1>(pole));
        double lastPeakDiff = std::get<2>(pole);
        double lastPoleIntensity = std::get<1>(pole);

        for (int i = 1; i < poleList.size(); i++)
        {
            double poleIntensity;
            int polePositionIdx;
            int poleIntensityChange;
            std::tie(polePositionIdx, poleIntensity, poleIntensityChange) = poleList[i];

            // Remove poles in the ROI2 (center part)
            //if ((polePositionIdx < leftMiddleArea) || (polePositionIdx > rightMiddleArea)) {
            int distance = polePositionIdx - finalpolePosition[finalpolePosition.size() - 1];
            if (distance < minDis) // 2 poles is too close: compare to find the select the correct pole
            {
                // select the pole which have lower intensity
                if (lastPoleIntensity - poleIntensity >
                    2) // If intensity of this pole is 200 lower than current last poles -> set this pole as the last poles
                {
                    finalpolePosition[finalpolePosition.size() - 1] = polePositionIdx;
                    lastPoleIntensity = poleIntensity;
                    lastPeakDiff = poleIntensityChange;
                }
                else if (poleIntensityChange > lastPeakDiff) // select the pole which have higher intensity change
                {
                    finalpolePosition[finalpolePosition.size() - 1] = polePositionIdx;
                    lastPoleIntensity = poleIntensity;
                    lastPeakDiff = poleIntensityChange;
                }
            }
            else
            {
                // Add current pole to final list poles
                lastPoleIntensity = poleIntensity;
                lastPeakDiff = poleIntensityChange;
                finalpolePosition.push_back(polePositionIdx);
                finalpoleIntensity.push_back(poleIntensity);
            }
            //}
        }
    }
    else
    {
        //Error log;
    }

    // If 2 border pole is too dark (border case) -> remove it from pole list
    /* int sizeLst = finalpoleIntensity.size();
        if (finalpoleIntensity[sizeLst-1] <= 50)
        {
            finalpolePosition.pop_back();
            finalpoleIntensity.pop_back();
        }
        if (finalpoleIntensity[0] <= 50)
        {
            finalpolePosition.erase(finalpolePosition.begin());
            finalpoleIntensity.erase(finalpoleIntensity.begin());
        }*/
    return std::make_pair(finalpolePosition, finalpoleIntensity);
}


} // namespace battery
} // namespace xvt
