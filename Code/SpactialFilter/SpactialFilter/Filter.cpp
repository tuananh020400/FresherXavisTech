#include "SpactialFilter.h"
Mat SpactialFiltering::ManualBoxFilter(const Mat& image, int KernelSize)
{
    int rows = image.rows;
    int cols = image.cols;
    int k = KernelSize / 2;
    Mat padded;
    copyMakeBorder(image, padded, k, k, k, k, BORDER_DEFAULT);

    Mat result = image.clone();
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int sum = 0;
            int count = 0;
            for (int dy = 0; dy < KernelSize; ++dy) {
                for (int dx = 0; dx < KernelSize; ++dx) {
                    sum += padded.at<uchar>(y + dy, x + dx);
                }
            }
            result.at<uchar>(y, x) = (sum / (KernelSize * KernelSize));
        }
    }
    return result;
}

Mat SpactialFiltering::CreateGaussKernel(int ksize, double sigma)
{
    Mat kernel(ksize, ksize, CV_64F);
    int k = ksize / 2;
    double sum = 0.0;
    for (int y = -k; y <= k; ++y) {
        for (int x = -k; x <= k; ++x) {
            double value = exp(-(x * x + y * y) / (2 * sigma * sigma));
            value /= (2 * CV_PI * sigma * sigma);
            sum += value;
            kernel.at<double>(y + k, x + k) = value;
        }
    }
    kernel /= sum;
    return kernel;
}

Mat SpactialFiltering::ManualGaussianFilter(const cv::Mat& image, int kernelSize, double sigma)
{
    int rows = image.rows;
    int cols = image.cols;
    int k = kernelSize / 2;
    Mat padded;
    copyMakeBorder(image, padded, k, k, k, k, cv::BORDER_DEFAULT);
    Mat result = image.clone();
    Mat kernel = SpactialFiltering::CreateGaussKernel(kernelSize, sigma);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            double sum = 0.0;
            for (int dy = 0; dy < kernelSize; ++dy) {
                for (int dx = 0; dx < kernelSize; ++dx) {
                    uchar pixel = padded.at<uchar>(y + dy, x + dx);
                    double weight = kernel.at<double>(dy, dx);
                    sum += pixel * weight;
                }
            }
            result.at<uchar>(y, x) = sum;
        }
    }
    return result;
}

Mat SpactialFiltering::ManualBilateralFilter(const cv::Mat& image, int kernelSize, double sigmaIntensity, double sigmaSpace)
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