#include "Header.h"

Mat SobelFilter::createGaussianKernel1D(int size, double sigma) {
    int r = size / 2;
    Mat dst(1, size, CV_64F);
    double sum = 0;
    for (int i = -r; i <= r; ++i)
    {
        double val = exp(-(i * i) / (2 * sigma * sigma));
        dst.at<double>(0, i + r) = val;
        sum += val;
    }
    dst /= sum;
    return dst;
}

Mat SobelFilter::createDerivativeKernel(int size, double sigma)
{
    int r = size / 2;
    Mat dst(1, size, CV_64F);
    double sum = 0;
    for (int i = -r; i <= r; ++i)
    {
        double val = -i * exp(-(i * i) / (2 * sigma * sigma)) / (sigma * sigma);
        dst.at<double>(0, i + r) = val;
        sum += abs(val);
    }
    dst /= sum;
    return dst;
}

Mat SobelFilter::createSobelKernel(int size, double sigma, bool derivativeX)
{
    Mat g = SobelFilter::createGaussianKernel1D(size, sigma);
    Mat g_deriv = SobelFilter::createDerivativeKernel(size, sigma);

    if (derivativeX) {
        return g_deriv.t() * g;
    }
    else {
        return g.t() * g_deriv;
    }
}

Mat SobelFilter::applyConvolution(const Mat& src, const Mat& kernel)
{
    CV_Assert(kernel.rows % 2 == 1 && kernel.cols % 2 == 1);

    // Bán kính kernel
    int kr = kernel.rows / 2;
    int kc = kernel.cols / 2;

    // Padding ảnh gốc
    Mat padded;
    copyMakeBorder(src, padded, kr, kr, kc, kc, BORDER_DEFAULT);

    // Tạo ảnh kết quả
    Mat dst = Mat::zeros(src.size(), CV_64F);

    // Chuyển kernel sang kiểu double
    Mat kernel_double;
    kernel.convertTo(kernel_double, CV_64F);

    // Chuyển src sang double để tránh lỗi khi nhân
    Mat src_double;
    padded.convertTo(src_double, CV_64F);

    // Tích chập
    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            double sum = 0.0;
            for (int i = 0; i < kernel.rows; ++i) {
                for (int j = 0; j < kernel.cols; ++j) {
                    double val = src_double.at<double>(y + i, x + j);
                    double k = kernel_double.at<double>(i, j);
                    sum += val * k;
                }
            }
            dst.at<double>(y, x) = sum;
        }
    }

    return dst;
}

Mat SobelFilter::ManualSobelFilter(const Mat& src, int ksize)
{
    Mat dst(src.size(), CV_8UC1);

    Mat kernelX = SobelFilter::createSobelKernel(ksize, 1.0, true);
    Mat kernelY = SobelFilter::createSobelKernel(ksize, 1.0, false);

    Mat gradX = applyConvolution(src, kernelX);  // giả sử trả về CV_64F hoặc CV_32F
    Mat gradY = applyConvolution(src, kernelY);

    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            double gx = gradX.at<double>(y, x);
            double gy = gradY.at<double>(y, x);
            double mag = sqrt(gx * gx + gy * gy);
            dst.at<uchar>(y, x) = saturate_cast<uchar>(mag);
        }
    }

    // Để hiển thị gradX, gradY đúng
    Mat absGradX, absGradY;
    convertScaleAbs(gradX, absGradX);
    convertScaleAbs(gradY, absGradY);

    imshow("Gradient X", absGradX);
    imshow("Gradient Y", absGradY);

    return dst;
}