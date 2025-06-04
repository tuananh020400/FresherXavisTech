#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int computeOtsuThreshold(const Mat& gray)
{
    // Histogram
    vector<int> hist(256, 0);
    for (int y = 0; y < gray.rows; y++) {
        for (int x = 0; x < gray.cols; x++) {
            int pixel = gray.at<uchar>(y, x);
            hist[pixel]++;
        }
    }

    int total = gray.rows * gray.cols;

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

