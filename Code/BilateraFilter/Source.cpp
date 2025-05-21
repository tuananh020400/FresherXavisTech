#include "Header.h"

Mat BilateralFilter::ManualBilateralFilter(const cv::Mat& image, int kernelSize, double sigmaIntensity, double sigmaSpace)
{
    int rows = image.rows;
    int cols = image.cols;
    int k = kernelSize / 2;
    Mat result = image.clone();

    Mat padded;
    copyMakeBorder(image, padded, k, k, k, k, cv::BORDER_DEFAULT);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            double weightSum = 0.0;
            double bilSum = 0.0;

            uchar center = padded.at<uchar>(y + k, x + k);

            for (int dy = -k; dy <= k; ++dy) {
                for (int dx = -k; dx <= k; ++dx) {
                    uchar neighbor = padded.at<uchar>(y + k + dy, x + k + dx);
                    double gaussSpace = exp(-(dx * dx + dy * dy) / (2 * sigmaSpace * sigmaSpace));
                    int diff = static_cast<int>(neighbor) - static_cast<int>(center);
                    double gaussIntensity = exp(-(diff * diff) / (2.0 * sigmaIntensity * sigmaIntensity));
                    double weight = gaussSpace * gaussIntensity;
                    bilSum += (neighbor * weight);
                    weightSum += weight;
                }
            }
            result.at<uchar>(y, x) = cv::saturate_cast<uchar>(bilSum / weightSum);
        }
    }
    return result;
}