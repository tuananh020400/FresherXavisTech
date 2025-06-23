#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int computeOtsuThreshold(const Mat& image)
{
    // Histogram
    vector<int> hist(256, 0);
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            int pixel = image.at<uchar>(y, x);
            hist[pixel]++;
        }
    }

    int total = image.rows * image.cols;

    // p(i)
    vector<float> prob(256, 0.0f);
    for (int i = 0; i < 256; i++) {
        prob[i] = (float)hist[i] / total;
    }

    // Culculate Otsu threshold
    float maxSigma = 0.0f;
    int bestThreshold = 0;

    float mg = 0.0f;
    for (int i = 0; i < 256; i++)
    {
        mg += i * prob[i];
    }

    for (int t = 0; t < 256; t++)
    {
        float p1 = 0.0f, p2 = 0.0f;
        float m1 = 0.0f, m2 = 0.0f;

        for (int i = 0; i <= t; i++) {
            p1 += prob[i];
            m1 += i * prob[i];
        }

        for (int i = t + 1; i < 256; i++) {
            p2 += prob[i];
            m2 += i * prob[i];
        }

        if (p1 == 0 || p2 == 0) continue;

        m1 /= p1;
        m2 /= p2;

        float sigma_b = p1 * (m1 - mg) * (m1 - mg) + p2 * (m2 - mg) * (m2 - mg);

        if (sigma_b > maxSigma)
        {
            maxSigma = sigma_b;
            bestThreshold = t;
        }
    }
    return bestThreshold;
}

Mat multiThreshold(Mat &image, int T1, int T2)
{
    Mat result = image.clone();
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            uchar pixel = image.at<uchar>(y, x);

            if (pixel < T1) {
                result.at<uchar>(y, x) = 0;       // Vùng tối
            }
            else if (pixel < T2) {
                result.at<uchar>(y, x) = 127;     // Vùng trung bình
            }
            else {
                result.at<uchar>(y, x) = 255;     // Vùng sáng
            }
        }
    }
    return result;
}
int main(void)
{
    Mat image = imread("D:/FresherXavisTech/Image/00000.tif", IMREAD_GRAYSCALE);
    Mat blured;
    Mat adaptiveThresholdedMean;
    Mat adaptiveThresholdedGauss;
    Mat Otsu;
    Mat Triangle;
    Mat OpenCVOtsu;
    int threshVal = computeOtsuThreshold(image);
    bilateralFilter(image, blured, 9, 3.0, 10.0, BORDER_CONSTANT);
    threshold(image, Triangle, 0, 355, THRESH_TRIANGLE);
    adaptiveThreshold(blured, adaptiveThresholdedMean, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 9, 5);
    adaptiveThreshold(blured, adaptiveThresholdedGauss, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 5);
    threshold(image, Otsu, threshVal, 127, THRESH_BINARY);
    threshold(image, OpenCVOtsu, 0, 255, THRESH_OTSU);
    return 0;
}

