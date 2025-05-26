
#ifndef _SOURCE_CPP_
#define _SOURCE_CPP_

#include "Header.h"

Mat CannyEdge::createGaussianKernel(int size, double sigma) {
    int half = size / 2;
    Mat dst(size, size, CV_64F);
    double sum = 0;
    const double pi = 3.14159265358979323846;

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            {
                int x = i - half;
                int y = j - half;
                double val = exp(-(x * x + y * y) / (2 * sigma * sigma)) / (2 * pi * sigma * sigma);
                dst.at<double>(i, j) = val;
                sum += val;
            }
        }
    }
    dst /= sum;
    return dst;
}

Mat CannyEdge::convolve(const Mat& img, const Mat& kernel)
{
    //Suppose that the kernel is square
    int ksize = kernel.rows;
    int offset = ksize / 2;
    Mat img_double;
    img.convertTo(img_double, CV_64F);
    Mat dst = Mat::zeros(img.size(), CV_64F);
    Mat padded;
    copyMakeBorder(img_double, padded, offset, offset, offset, offset, BORDER_REPLICATE);

    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            double sum = 0.0;
            for (int ky = -offset; ky <= offset; ky++) {
                for (int kx = -offset; kx <= offset; kx++) {
                    double val = padded.at<double>(y + ky + offset, x + kx + offset);
                    double kval = kernel.at<double>(ky + offset, kx + offset);
                    sum += val * kval;
                }
            }
            dst.at<double>(y, x) = sum;
        }
    }
    return dst;
}

void CannyEdge::computeSobel(const Mat& img, Mat& gx, Mat& gy)
{
    // Kernel Sobel 3x3
    Mat kx = (Mat_<double>(3, 3) <<
        -1, 0, 1,
        -2, 0, 2,
        -1, 0, 1);
    Mat ky = (Mat_<double>(3, 3) <<
        -1, -2, -1,
        0, 0, 0,
        1, 2, 1);

    gx = convolve(img, kx);
    gy = convolve(img, ky);
}

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
    imshow("Suppresed", suppressed);
    return suppressed;
}

// Hysteresis Thresholding
Mat CannyEdge::hysteresis(const Mat& img, double lowThresh, double highThresh)
{
    Mat res = Mat::zeros(img.size(), CV_8U);

    int rows = img.rows;
    int cols = img.cols;

    // Biên mạnh: 255, biên yếu: 100
    const uchar strong = 255;
    const uchar weak = 100;

    // Đánh dấu biên mạnh và yếu
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            double val = img.at<double>(y, x);
            if (val >= highThresh)
                res.at<uchar>(y, x) = strong;
            else if (val >= lowThresh)
                res.at<uchar>(y, x) = weak;
        }
    }

    // Nối biên yếu nếu liên kết với biên mạnh
    bool changed;
    do {
        changed = false;
        for (int y = 1; y < rows - 1; y++) {
            for (int x = 1; x < cols - 1; x++) {
                if (res.at<uchar>(y, x) == weak) {
                    // Kiểm tra 8 láng giềng có biên mạnh không
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
    // 1. Gaussian Blur
    Mat gaussKernel = CannyEdge::createGaussianKernel(5, 1.0);
    Mat imgBlur = convolve(img, gaussKernel);

    // 2. Tính Sobel gradient
    Mat gx, gy;
    CannyEdge::computeSobel(imgBlur, gx, gy);

    // 3. Độ lớn và hướng gradient
    Mat magnitude, angle;
    CannyEdge::computeGradient(gx, gy, magnitude, angle);

    // 4. Non-Maximum Suppression
    Mat nms = nonMaxSuppression(magnitude, angle);

    // 5. Hysteresis Thresholding
    Mat edges = hysteresis(nms, lowThresh, highThresh);

    return edges;
}

#endif // _SOURCE_CPP_

