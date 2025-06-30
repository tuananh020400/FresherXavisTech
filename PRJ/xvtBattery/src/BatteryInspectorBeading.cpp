#include "xvtBattery/BatteryInspectorBeading.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/Contour.h"
#include "xvtCV/Peak.h"
#include "xvtCV/Utils.h"

namespace xvt
{
namespace battery
{

auto BatteryInspectorBeading::Inspect(cv::Mat const &inImg, const VecPoint &contour) const -> BatteryInspectorBeadingResult
{
    auto ispResult = BatteryInspectorBeadingResult();
    ispResult.mEnable = mEnable;

    if (!ispResult.mEnable)
    {
        ispResult(EResult::UC, "Disabled!");
        return ispResult;
    }

    cv::Mat src;
    if (!Convert8Bits(inImg, src, false))
    {
        ispResult.mPoints = contour;
        auto beadingResult = InspectBeading(src, ispResult);
    }
    else
    {
        ispResult(EResult::ER, "Image type is not supported!");
    }
    return ispResult;
}

auto BatteryInspectorBeading::InspectBeading(const cv::Mat &src, BatteryInspectorBeadingResult &BIresult) const -> InspectionResult
{
    InspectionResult finalResult;
    finalResult(EResult::ER, "FindBattery: Image Empty or Empty Object");
    const VecPoint &bounderContour = BIresult.mPoints;
    auto mCornerPoints = BIresult.GetCornerPoint();
    if (!src.empty() && bounderContour.size() > 0 && !mCornerPoints.empty())
    {
        /*
        *          .A0--------------------------------------------.A3
        *        /                                                   \
        *       |                                                     |
        *       .L1                                                   .L2
        *       .                                                     .
        *         .A1                                              .A4
        *           .                                            .
        *              .                                       .
        *                .G1                                .G2
        *             .                                         .
        *         .A2                                              .A5
        *       /                                                    \
        *       |                                                     |
        *       |                                                     |
        */
        //For Left Point References
        cv::Point A0 = cv::Point(-1, -1);
        cv::Point A1 = cv::Point(-1, -1);
        cv::Point A2 = cv::Point(-1, -1);
        cv::Point G1 = cv::Point(-1, -1);
        cv::Point G2 = cv::Point(-1, -1);
        cv::Point L1 = cv::Point(-1, -1);
        //For Right Point references
        cv::Point A3 = cv::Point(-1, -1);
        cv::Point A4 = cv::Point(-1, -1);
        cv::Point A5 = cv::Point(-1, -1);
        cv::Point P3 = cv::Point(-1, -1);
        cv::Point P4 = cv::Point(-1, -1);
        cv::Point L2 = cv::Point(-1, -1);

        int idxTopLeft;
        int idxTopRight;
        int idxBottomRight;
        int idxBottomLeft;

        bool foundTopLeft = FindClosestPointOnContour(bounderContour, mCornerPoints[0], idxTopLeft);
        bool foundTopRight = FindClosestPointOnContour(bounderContour, mCornerPoints[1], idxTopRight);
        bool foundBottomRight = FindClosestPointOnContour(bounderContour, mCornerPoints[2], idxBottomRight);
        bool foundBottomLeft = FindClosestPointOnContour(bounderContour, mCornerPoints[3], idxBottomLeft);

        if (idxTopLeft > idxTopRight)
        {
            idxTopRight = bounderContour.size() - 1;
        }

        if (foundTopLeft && foundTopRight && foundBottomRight && foundBottomLeft && (idxBottomLeft - idxTopLeft > 0) &&
            (idxTopRight - idxBottomRight > 0) && (idxBottomRight - idxBottomLeft > 0))
        {

            VecFloat difVecLeft, difVecRight;
            int idx = 0;
            double minProminence = 1.0;
            cv::Size kSize = cv::Size(3, 1);
            for (idx = idxTopLeft; idx < idxBottomLeft; idx++)
            {
                difVecLeft.push_back(bounderContour[idx].x);
            }
            for (idx = idxBottomRight; idx < idxTopRight; idx++)
            {
                difVecRight.push_back(bounderContour[idx].x);
            }

            cv::blur(difVecLeft, difVecLeft, kSize);
            cv::blur(difVecRight, difVecRight, kSize);

            FindPeaks findPeaks;
            findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
            findPeaks.Process(difVecLeft);
            VecPeak PeakLeft = findPeaks.GetPeakResult(minProminence, mBeadingHeightMin);

            findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
            findPeaks.Process(difVecRight);
            VecPeak PeakRight = findPeaks.GetPeakResult(minProminence, mBeadingHeightMin);

            if (!PeakRight.empty() && !PeakLeft.empty())
            {
                Peak maxPeakLeft = *std::max_element(PeakLeft.begin(), PeakLeft.end(), [](const Peak &lhs, const Peak &rhs) {
                    return abs(lhs.prominence) < abs(rhs.prominence);
                });

                Peak maxPeakRight = *std::max_element(PeakRight.begin(), PeakRight.end(), [](const Peak &lhs, const Peak &rhs) {
                    return abs(lhs.prominence) < abs(rhs.prominence);
                });

                int idxG1 = maxPeakLeft.index + idxTopLeft;
                int idxG2 = maxPeakRight.index + idxBottomRight;
                G1 = bounderContour[idxG1];
                G2 = bounderContour[maxPeakRight.index + idxBottomRight];
                auto objROI = cv::boundingRect(BIresult.mPoints);
                int xCenterObject = objROI.x + objROI.width / 2;
                bool checkG1 = (G1.x > bounderContour[idxTopLeft].x) && (G1.x > bounderContour[idxBottomLeft].x) && (G1.x < xCenterObject) &&
                               (idxG1 > idxTopLeft);
                bool checkG2 = (G2.x < bounderContour[idxTopRight].x) && (G1.x < xCenterObject) && (G2.x > xCenterObject) && (idxTopRight > idxG2);
                if (checkG1 && checkG2)
                {
                    //Find Point in sub-Roi 3.1
                    L1 = *std::min_element(bounderContour.begin() + idxTopLeft,
                                           bounderContour.begin() + idxG1,
                                           [](const cv::Point &lhs, const cv::Point &rhs) { return lhs.x < rhs.x; });

                    //Find Point in sub-Roi 3.2 in right

                    L2 = *std::max_element(bounderContour.begin() + idxG2,
                                           bounderContour.begin() + idxTopRight,
                                           [](const cv::Point &lhs, const cv::Point &rhs) { return lhs.x < rhs.x; });

                    //find maximumpoint on the left

                    if (mD1StartPosition >= 1)
                    {
                        A1 = G1;
                        A2 = G1;
                    }
                    else
                    {
                        int grooveWidth = abs(G1.x - L1.x) * (1 - mD1StartPosition);
                        for (idx = idxG1; idx > idxTopLeft; idx--)
                        {
                            if (G1.x - bounderContour[idx].x >= grooveWidth)
                            {
                                A1 = bounderContour[idx];
                                break;
                            }
                        }

                        for (idx = idxG1; idx < idxBottomLeft; idx++)
                        {
                            if (G1.x - bounderContour[idx].x >= grooveWidth)
                            {
                                A2 = bounderContour[idx];
                                break;
                            }
                        }
                    }

                    //for (int i = 0; i < A1.y; i++) {
                    //    if (grayImg.at<uchar>(i, A1.x) == 255) {
                    //        A0 = cv::Point(A1.x, i);
                    //        break;
                    //    }
                    //}

                    //find maximumpoint on the right
                    if (mD1StartPosition >= 1)
                    {
                        A4 = G2;
                        A5 = G2;
                    }
                    else
                    {
                        int grooveWidth = abs(G2.x - L2.x) * (1 - mD1StartPosition);
                        for (idx = idxG2; idx < idxTopRight; idx++)
                        {
                            if (bounderContour[idx].x - G2.x >= grooveWidth)
                            {
                                A4 = bounderContour[idx];
                                break;
                            }
                        }

                        for (idx = idxG2; idx > idxBottomRight; idx--)
                        {
                            if (bounderContour[idx].x - G2.x >= grooveWidth)
                            {
                                A5 = bounderContour[idx];
                                break;
                            }
                        }
                    }
                    finalResult(EResult::OK, "");
                    BIresult.mBottomLeftContour = A2;
                    BIresult.mBottomRightContour = A5;
                    BIresult.mGrooveLeftContour = G1;
                    BIresult.mGrooveRightContour = G2;
                }
                else
                {
                    finalResult(EResult::ER, "InspectBeading: Can't find G1 and G2");
                }
            }
            else
            {
                finalResult(EResult::ER, "InspectBeading: Can't find any peaks");
            }
        }
        else
        {
            finalResult(EResult::ER, "InspectBeading: Can't find any base points");
        }
    }

    BIresult &= finalResult;
    return finalResult;
}

void BatteryInspectorBeadingResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (!IsER())
    {
        pen.mColor = cv::Scalar(150, 150, 0);
        DrawText(img, "A2", mBottomLeftContour, TextAlign::BOTTOM_RIGHT, pen, cv::Point(-10, 0));
        cv::circle(img, mBottomLeftContour, 2, pen.mColor, 3);

        DrawText(img, "A5", mBottomRightContour, TextAlign::BOTTOM_LEFT, pen, cv::Point(10, 0));
        cv::circle(img, mBottomRightContour, 2, pen.mColor, 3);

        DrawText(img, "G1", mGrooveLeftContour, TextAlign::MIDDLE_CENTER, pen, cv::Point(0, -20));
        cv::circle(img, mGrooveLeftContour, 2, pen.mColor, 3);

        DrawText(img, "G2", mGrooveRightContour, TextAlign::MIDDLE_CENTER, pen, cv::Point(0, -20));
        cv::circle(img, mGrooveRightContour, 2, pen.mColor, 3);
    }
}

auto BatteryInspectorBeadingResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
}


} // namespace battery
} // namespace xvt
