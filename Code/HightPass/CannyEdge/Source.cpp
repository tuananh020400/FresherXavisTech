
#ifndef _SOURCE_CPP_
#define _SOURCE_CPP_

#include "Header.h"

void CannyEdge::computeGradient(const Mat& gx, const Mat& gy, Mat& magnitude, Mat& angle)
{
    magnitude = Mat::zeros(gx.size(), CV_64F);
    angle = Mat::zeros(gx.size(), CV_64F);
    for (int y = 0; y < gx.rows; y++) {
        for (int x = 0; x < gx.cols; x++) {
            double Gx = gx.at<double>(y, x);
            double Gy = gy.at<double>(y, x);
            magnitude.at<double>(y, x) = sqrt(Gx * Gx + Gy * Gy);
            angle.at<double>(y, x) = atan2(Gy, Gx) * 180.0 / CV_PI; // Degree
            if (angle.at<double>(y, x) < 0)
                angle.at<double>(y, x) += 180; // Degree ranges 0-180
        }
    }
}

// Non-Maximum Suppression
Mat CannyEdge::nonMaxSuppression(const Mat& magnitude, const Mat& angle)
{
    Mat suppressed = Mat::zeros(magnitude.size(), CV_64F);
    for (int y = 1; y < magnitude.rows - 1; y++) {
        for (int x = 1; x < magnitude.cols - 1; x++) {
            double ang = angle.at<double>(y, x);
            double mag = magnitude.at<double>(y, x);
            double mag1, mag2;

            //Determine the two neighboring points along the gradient direction
            if ((ang >= 0 && ang < 22.5) || (ang >= 157.5 && ang <= 180)) {
                mag1 = magnitude.at<double>(y, x - 1);
                mag2 = magnitude.at<double>(y, x + 1);
            }
            else if (ang >= 22.5 && ang < 67.5) {
                mag1 = magnitude.at<double>(y - 1, x + 1);
                mag2 = magnitude.at<double>(y + 1, x - 1);
            }
            else if (ang >= 67.5 && ang < 112.5) {
                mag1 = magnitude.at<double>(y - 1, x);
                mag2 = magnitude.at<double>(y + 1, x);
            }
            else { // 112.5 - 157.5
                mag1 = magnitude.at<double>(y - 1, x - 1);
                mag2 = magnitude.at<double>(y + 1, x + 1);
            }

            // Non-Maximum Suppression
            if (mag >= mag1 && mag >= mag2) {
                suppressed.at<double>(y, x) = mag;
            }
            else {
                suppressed.at<double>(y, x) = 0;
            }
        }
    }
    return suppressed;
}

// Hysteresis Thresholding
Mat CannyEdge::hysteresis(const Mat& img, double lowThresh, double highThresh)
{
    Mat res = Mat::zeros(img.size(), CV_8U);

    int rows = img.rows;
    int cols = img.cols;

    // Strong edge: 255, weak edge: 100
    const uchar strong = 255;
    const uchar weak = 100;

    // Marked trong and weak edge
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            double val = img.at<double>(y, x);
            if (val >= highThresh)
                res.at<uchar>(y, x) = strong;
            else if (val >= lowThresh)
                res.at<uchar>(y, x) = weak;
        }
    }

    // Linking edge point
    bool changed;
    do {
        changed = false;
        for (int y = 1; y < rows - 1; y++) {
            for (int x = 1; x < cols - 1; x++) {
                if (res.at<uchar>(y, x) == weak) {
                    if (
                        res.at<uchar>(y - 1, x - 1) == strong || res.at<uchar>(y - 1, x) == strong ||
                        res.at<uchar>(y - 1, x + 1) == strong || res.at<uchar>(y, x - 1) == strong ||
                        res.at<uchar>(y, x + 1) == strong || res.at<uchar>(y + 1, x - 1) == strong ||
                        res.at<uchar>(y + 1, x) == strong || res.at<uchar>(y + 1, x + 1) == strong)
                    {
                        res.at<uchar>(y, x) = strong;
                        changed = true;
                    }
                    else
                    {
                        res.at<uchar>(y, x) = 0;
                    }
                }
            }
        }
    } while (changed);

    return res;
}

Mat CannyEdge::CannyEdgeFilter(const Mat& img, double lowThresh, double highThresh)
{
    //Gaussian Blur
    Mat imgBlur;
    GaussianBlur(img, imgBlur, Size(5, 5), 1.4, 1.4);

    //Sobel Culculate gradient in two directions
    Mat gx, gy, gx_abs, gy_abs;
    Sobel(imgBlur, gx, CV_64F, 1, 0, 3);
    Sobel(imgBlur, gy, CV_64F, 0, 1, 3);

    // Culculate gradient manitude and direction
    Mat magnitude, angle;
    CannyEdge::computeGradient(gx, gy, magnitude, angle);

    // Non-Maximum Suppression
    Mat nms = CannyEdge::nonMaxSuppression(magnitude, angle);

    // Hysteresis Thresholding
    Mat edges = hysteresis(nms, lowThresh, highThresh);

    return edges;
}

#endif // _SOURCE_CPP_

