#include "xvtCV/LineTrace.h"
#include "xvtCV/Utils.h"
#include "xvtCV/ColorDefine.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

namespace xvt {
void LineTrace::Draw(cv::Mat const& img, std::vector<std::pair<cv::Point, int>>const& traceList) const
{
    if (traceList.empty()) return;
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::circle(img, traceList[0].first, 1, color);
    for (int i = 1; i < traceList.size(); i++)
    {
        color = (traceList[i].second) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::line(img, traceList[i - 1].first, traceList[i].first, color);
        //cv::circle(img, traceList[i].first, 1, color);
    }
}

void LineTrace::SetAnchor(cv::Point p)&
{
    mAnchor.x = (std::min)(p.x, mMoveWindowSize.width - 1);
    mAnchor.y = (std::min)(p.y, mMoveWindowSize.height - 1);
}

cv::Point LineTrace::GetWindowPosOffset() const
{
    cv::Point p = cv::Point(0, 0);
    p.x = (mAnchor.x < 0) ? floor(mMoveWindowSize.width / 2) : mAnchor.x;
    p.y = (mAnchor.y < 0) ? floor(mMoveWindowSize.height / 2) : mAnchor.y;
    return p;
}

//#define _DEBUG_TRACE_LINE_IMG_
std::pair<int, float> minPosReduceImage(const cv::Mat& inImg) {
    cv::Mat reduceRes;
    cv::reduce(inImg, reduceRes, 0, cv::REDUCE_AVG, CV_32FC1);
    auto minPos = std::min_element((float*)reduceRes.ptr(0), (float*)reduceRes.ptr(0) + reduceRes.cols);
    float avgVal = *minPos;
    int d = std::distance((float*)reduceRes.ptr(0), minPos);
    reduceRes.release();
    return std::pair<int, float>(d, avgVal);
}

void TracePoint2(const cv::Mat& inImg, cv::Point2d currentPos, int side_length,
    int ahead_length, float breakThreshold, std::vector<cv::Point>& collectedPoints,
    int borderCheck, int borderDistance
)
{
    //int ahead_length = 5; // So point phia truoc
    //int side_length = 3; // So point o mot ben.
    int discontinuePoints = 0;
    int discontinuePointThd = 5;
    int borderOffset = 0;
    // Filter previous move vector and current vector
    float alpha = 0.6;

    //Angle threshold, compare to pre_Vector
    double alphaThresh = cos(95 * CV_PI / 180);

    cv::Point2d pre_Vector(0, -1);
    cv::Point2d shift_y_vector(pre_Vector);
    cv::Point2d tmpVector(pre_Vector);
    //cv::Point2d pre_move_vector(pre_Vector);
    cv::Point2d shift_x_vector(-shift_y_vector.y, shift_y_vector.x);//vector huong sang b�n phai vuong goc voi y

    shift_y_vector *= ahead_length;
    shift_x_vector *= side_length;

    cv::Point2d normVector;
    cv::Point2d predictMoveVector(pre_Vector);
    cv::Point2d actualMoveVector;
    cv::Point2d realMinPos;

    //Refinding the curent position point
    cv::Rect rectStart(currentPos.x - 2, currentPos.y - 2, 5, 5);
    double minValue, maxValue;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(inImg(rectStart), &minValue, &maxValue, &minLoc, &maxLoc);
    currentPos = minLoc + rectStart.tl();

    //Move to predict point
    cv::Point2d predictPoint = currentPos + predictMoveVector;

    //Caculate the warp points
    cv::Point2d tl = predictPoint + shift_y_vector - shift_x_vector;
    cv::Point2d tr = predictPoint + shift_y_vector + shift_x_vector;

    int h = ahead_length * 5;
    int w = (side_length * 2 + 1) * 5;
    //cv::Point centerPoint(side_length, h - 1);
    cv::Point2f verticesDst[4] = { cv::Point2f(0,0), cv::Point2f(w,0) ,cv::Point2f(w,h) ,cv::Point2f(0,h) };

    int startPointX = currentPos.x;

    if (borderCheck == -1) {
        if (2 * borderDistance < startPointX) {
            //borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (borderDistance < startPointX) {
            borderOffset = 2;
        }
        else {
            borderOffset = 0;
        }

    }
    else if (borderCheck == 1) {
        if (startPointX < inImg.cols - 2 * borderDistance) {
            //borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (startPointX < inImg.cols - borderDistance) {
            borderOffset = 2;
        }
        else {
            borderOffset = 0;
        }
    }

    int X_leftAnchor = startPointX - borderOffset;
    int X_rightAnchor = startPointX + borderOffset;
    int imgRows = inImg.rows;
    int imgCols = inImg.cols;
    while (tl.y > 0 && tl.x > 0 && tr.y > 0 && tr.x > 0 && tr.y < imgRows && tr.x < imgCols) {

        //Check if predictPoint is in the image.
        if (predictPoint.x < 0 || predictPoint.y < 0 || predictPoint.x >= imgCols || predictPoint.y >= imgRows) break;

        tl = predictPoint + shift_y_vector - shift_x_vector;
        cv::Point2d tr = predictPoint + shift_y_vector + shift_x_vector;
        cv::Point2d br = predictPoint + shift_x_vector;
        cv::Point2d bl = predictPoint - shift_x_vector;
        cv::Point2f vertices[4] = { tl, tr, br, bl };

        // Chi dung voi truong hop di thang, ko di theo huong bat ki
        if (shift_y_vector == cv::Point2d(0, -1)) {
            if (borderCheck == -1) {
                if (X_leftAnchor >= tl.x) {
                    tl.x = X_leftAnchor;
                    bl.x = X_leftAnchor;
                    w = tr.x - tl.x;
                }
            }
            else if (borderCheck == 1) {
                if (tr.x >= X_rightAnchor) {
                    tr.x = X_rightAnchor;
                    br.x = X_rightAnchor;
                    w = tr.x - tl.x;
                }
            }
        }

        cv::Mat warpImg = cv::Mat::zeros(h, w, CV_8UC1);
        cv::Mat perMat = cv::getPerspectiveTransform(verticesDst, vertices);
        cv::warpPerspective(inImg, warpImg, perMat, warpImg.size(), cv::WARP_INVERSE_MAP);

#ifdef _DEBUG_TRACE_LINE_IMG_
        cv::namedWindow("Trace ori", cv::WINDOW_NORMAL);
        cv::imshow("Trace ori", warpImg);
#endif //#ifdef _DEBUG_TRACE_LINE_IMG_

#ifdef _DEBUG_TRACE_LINE_IMG_
        std::cout << "perMat: " << perMat << std::endl;
#endif

        std::pair<int, float> minLoc = minPosReduceImage(warpImg);
        realMinPos = cv::Point(minLoc.first, h - 1);
        int direction = minLoc.first - w;
        //cv::Mat mean, stddev;
        //cv::meanStdDev(warpImg, mean, stddev);

#ifdef _DEBUG_TRACE_LINE_IMG_		
        std::cout << "Pos in warp: " << realMinPos << std::endl;
#endif
        if (realMinPos.x == w) {
            discontinuePoints = 0;
            realMinPos = predictPoint;
            actualMoveVector = predictMoveVector;
        }
        else {
            std::vector<cv::Point2d> warpPtList = { realMinPos };
            std::vector<cv::Point2d> imgPtList = { realMinPos };
            cv::perspectiveTransform(warpPtList, imgPtList, perMat);
            realMinPos = imgPtList[0];
            actualMoveVector = realMinPos - currentPos;

            if (minLoc.second < breakThreshold) {
#ifdef _DEBUG_TRACE_LINE_IMG_
                std::cout << "Pos in image: " << realMinPos << std::endl;
                std::cout << "Next pos vector " << actualMoveVector << std::endl;
#endif
                if (actualMoveVector != cv::Point2d()) {

                    normalizeVector(actualMoveVector, actualMoveVector);
#ifdef _DEBUG_TRACE_LINE_IMG_
                    std::cout << "Next pos vector2: " << actualMoveVector << std::endl;
#endif

                    double cosalpha = (actualMoveVector.x * pre_Vector.x + actualMoveVector.y * pre_Vector.y);

#ifdef _DEBUG_TRACE_LINE_IMG_
                    double alpha_angle = acos(cosalpha) * 180 / CV_PI;
                    std::cout << "Angle: " << alpha_angle << std::endl;
                    std::cout << "cosalpha: " << cosalpha << std::endl;
#endif

                    if (cosalpha > alphaThresh) {
                        discontinuePoints = 0;
                    }
                    else {
                        actualMoveVector = direction > 0 ? cv::Point2d(1, 0) : cv::Point2d(-1, 0);
                        /*actualMoveVector = direction > 0 ? cv::Point2d(0.7, -0.3) : cv::Point2d(-0.7, -0.3);
                        normalizeVector(actualMoveVector, actualMoveVector);*/
                        discontinuePoints++;
                    }
                }
                else {
                    actualMoveVector = predictMoveVector;
                    discontinuePoints = 0;
                }
            }
            else {
                actualMoveVector = predictMoveVector;
                discontinuePoints++;
            }
        }

#ifdef _DEBUG_TRACE_LINE_IMG_
        std::cout << "actualMoveVector: " << actualMoveVector << std::endl;
        std::cout << "predictMoveVector: " << predictMoveVector << std::endl;
#endif

        actualMoveVector = actualMoveVector * alpha + predictMoveVector * (1 - alpha);
        normalizeVector(actualMoveVector, actualMoveVector);

        if (shift_y_vector == cv::Point2d(0, -1)) {
            // Chi dung voi truong hop di thang ko di theo huong bat ki

            //Most left pole
            if (borderCheck == 1 && realMinPos.x >= X_rightAnchor) {
                realMinPos.x = X_rightAnchor;
                actualMoveVector = pre_Vector;
            }

            //Most right pole
            if (borderCheck == -1 && X_leftAnchor >= realMinPos.x) {
                realMinPos.x = X_leftAnchor;
                actualMoveVector = pre_Vector;
            }
        }

#ifdef _DEBUG_TRACE_LINE_IMG_
        double cosalpha = (actualMoveVector.x * pre_Vector.x + actualMoveVector.y * pre_Vector.y);

        double alpha_angle = acos(cosalpha) * 180 / CV_PI;
        std::cout << "Angle final: " << alpha_angle << std::endl;
        std::cout << "normalizeVector actualMoveVector: " << actualMoveVector << std::endl;
#endif
        //update the predict vector.
        predictMoveVector = actualMoveVector;

#ifdef _DEBUG_TRACE_LINE_IMG_
        std::cout << "discontinuePoints: " << discontinuePoints << std::endl;
        std::cout << "predictMoveVector vector final: " << predictMoveVector << std::endl;
#endif
        shift_y_vector = pre_Vector;
        shift_x_vector = cv::Point2d(-shift_y_vector.y, shift_y_vector.x);//vector huong sang ben phai vuong goc voi y
        shift_y_vector *= ahead_length;
        shift_x_vector *= side_length;

        //update current post to the found point
        currentPos = realMinPos;

        //double m = max(abs(predictMoveVector.y), 1.0);
        double m = cv::max(cv::max(abs(predictMoveVector.x), abs(predictMoveVector.y)), 1.0);
        cv::Point2d tmpMoveVector = predictMoveVector / m;
        //Predict the next point
        predictPoint = currentPos + tmpMoveVector;

        if (discontinuePoints > discontinuePointThd) {
            if (collectedPoints.size() <= discontinuePointThd) {
                collectedPoints.clear();
            }
            else {
                for (int i = 0; i < discontinuePointThd; i++) {
                    collectedPoints.pop_back();
                }
            }
            break;
        }
        else {
            //Store the current position
            collectedPoints.push_back(currentPos);
        }
#ifdef _DEBUG_TRACE_LINE_IMG_
        cv::Mat tmpMat;
        cv::cvtColor(inImg, tmpMat, cv::COLOR_GRAY2BGR);
        cv::line(tmpMat, predictPoint, predictPoint + tmpMoveVector, COLOR_CV_RED, 1);

        cv::line(tmpMat, cv::Point(vertices[0]), cv::Point(vertices[1]), COLOR_CV_GREEN, 1);
        cv::line(tmpMat, cv::Point(vertices[1]), cv::Point(vertices[2]), COLOR_CV_BLUE, 1);
        cv::line(tmpMat, cv::Point(vertices[2]), cv::Point(vertices[3]), COLOR_CV_RED, 1);
        cv::line(tmpMat, cv::Point(vertices[3]), cv::Point(vertices[0]), COLOR_CV_BLUE, 1);

        //cv::line(binImg, centerPoint, centerPoint + cv::Point(normVector * ahead_length / 2), COLOR_RED, 1);
        //cv::imshow("Trace bin", binImg);

        cv::namedWindow("Trace tmpMat", cv::WINDOW_NORMAL);
        cv::imshow("Trace tmpMat", tmpMat);
        cv::waitKey(1);

        std::cout << std::endl;

        tmpMat.release();
#endif // _DEBUG_TRACE_LINE_IMG_
    }
}

#define DEBUG_TRACE_ANODE
bool LineTrace::Trace(cv::Mat const& inputImg, cv::Point const& startPoint, std::vector<std::pair<cv::Point, int>>& traceList) const
{
    traceList.clear();
    bool rtn = !inputImg.empty() && inputImg.rows >= mYUpperLimit && inputImg.cols >= mXUpperLimit;
    bool writeDebugPole = true;

    if (rtn)
    {
        std::vector<float> reduceRes;
        int stepCheck = 0;
        bool isValidPoint = true;
        traceList.push_back(std::make_pair(startPoint, isValidPoint));
        cv::Point currentPoint = startPoint + static_cast<int>(mDirection) * mStep;;
        cv::Rect moveRect = cv::Rect(0, 0, mMoveWindowSize.width, mMoveWindowSize.height);
        cv::Point windowOffset = GetWindowPosOffset();
        while (IsContinue(currentPoint) && (stepCheck < mContinueThreshold))
        {
            float avgVal = 0.0;
            moveRect.x = currentPoint.x - windowOffset.x;
            moveRect.y = currentPoint.y - windowOffset.y;
            moveRect.width = mMoveWindowSize.width;
            moveRect.height = mMoveWindowSize.height;
            RefineROI(moveRect, inputImg.size());
            cv::Mat tmpMat = inputImg(moveRect);
            if (!tmpMat.empty())
            {
                //Vertical projection
                cv::reduce(tmpMat, reduceRes, 0, cv::REDUCE_AVG, CV_32FC1);
                auto minPos = std::min_element(reduceRes.begin(), reduceRes.end());
                currentPoint.x = moveRect.x + std::distance(reduceRes.begin(), minPos);

                isValidPoint = (*minPos <= mThreshold);
                stepCheck = isValidPoint ? 0 : stepCheck + 1;
                traceList.push_back(std::make_pair(currentPoint, isValidPoint));
            }
            else
            {
                break;
            }

            currentPoint += static_cast<int>(mDirection) * mStep;
        }

        // Remove the un-wanted points
        if (stepCheck > 0)
        {
            traceList.erase(traceList.end() - stepCheck, traceList.end());
        }
    }

    return rtn;
}
}