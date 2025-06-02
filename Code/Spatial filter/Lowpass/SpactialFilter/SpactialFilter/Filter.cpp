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

// Trả về vector Gaussian 1D
Mat SpactialFiltering::createGaussianKernel1D(int ksize) {
    return getGaussianKernel(ksize, -1, CV_64F); // auto sigma
}

Mat SpactialFiltering::createDerivativeKernel(int ksize, int order, double sigma)
{
    if (ksize % 2 == 0 || ksize < 1)
        cout << "Kích thước kernel phải lẻ và > 0" << endl;

    if (sigma <= 0)
        sigma = 0.3 * ((ksize - 1) * 0.5 - 1) + 0.8;  // như trong OpenCV

    int half = ksize / 2;
    Mat kernel(ksize, 1, CV_64F);
    double sum = 0.0;

    for (int i = -half; i <= half; ++i)
    {
        double x = static_cast<double>(i);
        double g = exp(-(x * x) / (2 * sigma * sigma)) / (sqrt(2 * CV_PI) * sigma);
        double val = 0.0;

        if (order == 0) {
            val = g;
        }
        else if (order == 1) {
            val = -x / (sigma * sigma) * g;
        }
        else if (order == 2) {
            val = (x * x - sigma * sigma) / (sigma * sigma * sigma * sigma) * g;
        }
        else {
            CV_Error(Error::StsOutOfRange, "Chưa hỗ trợ đạo hàm cấp > 2");
        }
        kernel.at<double>(i + half, 0) = val;

        kernel.at<double>(i + half, 0) = val;
        sum += (order == 0) ? val : 0.0; // chỉ cần chuẩn hóa nếu order == 0
    }
    if (order == 0)
        kernel /= sum; // chuẩn hóa tổng về 1

    return kernel;
}

Mat SpactialFiltering::applyConvolution(const Mat& src, const Mat& kernel)
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

Mat SpactialFiltering::ManualSobelFilter(const Mat& src, int ksize)
{
    Mat dst = src.clone();
    Mat kd = SpactialFiltering::createDerivativeKernel(ksize, 1);// đạo hàm
    Mat kg = SpactialFiltering::createDerivativeKernel(ksize, 0);// làm mượt

    Mat kernelX = kd * kg.t();
    Mat kernelY = kg * kd.t();

    Mat gradX = applyConvolution(src, kernelX);
    Mat gradY = applyConvolution(src, kernelY);

    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            double gx = gradX.at<double>(y, x);
            double gy = gradY.at<double>(y, x);
            double mag = sqrt(gx * gx + gy * gy);
            dst.at<uchar>(y, x) = saturate_cast<uchar>(mag);
        }
    }
    return dst;
}

cv::Mat SpactialFiltering::ManualGaussianFilter(const Mat& image, int kernelSize, double sigma)
{
    CV_Assert(image.type() == CV_8UC1); // đảm bảo ảnh grayscale

    int k = kernelSize / 2;
    int rows = image.rows;
    int cols = image.cols;

    std::vector<float> kernel1D(kernelSize);
    float sum = 0.0f;
    for (int i = 0; i < kernelSize; ++i) {
        int x = i - k;
        kernel1D[i] = std::exp(-(x * x) / (2.0f * sigma * sigma));
        sum += kernel1D[i];
    }
    for (int i = 0; i < kernelSize; ++i)
        kernel1D[i] /= sum;

    cv::Mat temp(rows, cols, CV_32FC1);
    for (int y = 0; y < rows; ++y) {
        const uchar* srcRow = image.ptr<uchar>(y);
        float* dstRow = temp.ptr<float>(y);
        for (int x = 0; x < cols; ++x) {
            float val = 0.0f;
            for (int i = -k; i <= k; ++i) {
                int xi = std::max(0, std::min(cols - 1, x + i));
                val += kernel1D[i + k] * srcRow[xi];
            }
            dstRow[x] = val;
        }
    }

    cv::Mat result(rows, cols, CV_8UC1);
    for (int y = 0; y < rows; ++y) {
        uchar* dstRow = result.ptr<uchar>(y);
        for (int x = 0; x < cols; ++x) {
            float val = 0.0f;
            for (int i = -k; i <= k; ++i) {
                int yi = std::max(0, std::min(rows - 1, y + i));
                val += kernel1D[i + k] * temp.ptr<float>(yi)[x];
            }
            dstRow[x] = cv::saturate_cast<uchar>(val + 0.5f); // làm tròn & ép kiểu an toàn
        }
    }

    return result;
}


/// <summary>
/// Applies a manually implemented bilateral filter to the input image.
/// </summary>
/// <param name="image">The input grayscale image to be filtered.</param>
/// <param name="kernelSize">The size of the square kernel (must be an odd number).</param>
/// <param name="sigmaIntensity">Standard deviation for intensity (range) similarity.</param>
/// <param name="sigmaSpace">Standard deviation for spatial (geometric) closeness.</param>
/// <returns>A new image with the bilateral filter applied.</returns>
Mat SpactialFiltering::ManualBilateralFilter(const cv::Mat& image, int kernelSize, double sigmaIntensity, double sigmaSpace)
{
    int rows = image.rows;
    int cols = image.cols;
    int k = kernelSize / 2;
    Mat result = image.clone();

    Mat padded;
    copyMakeBorder(image, padded, k, k, k, k, BORDER_DEFAULT);

    // Calculate the Spatial Gaussian Kernel
    vector<vector<double>> spatialGauss(kernelSize, vector<double>(kernelSize, 0));
    for (int dy = -k; dy <= k; ++dy) {
        for (int dx = -k; dx <= k; ++dx) {
            spatialGauss[dy + k][dx + k] = exp(-(dx * dx + dy * dy) / (2 * sigmaSpace * sigmaSpace));
        }
    }

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            double weightSum = 0.0; // Normalization factor
            double bilSum = 0.0;

            double center = static_cast<double>(padded.at<uchar>(y + k, x + k));

            for (int dy = -k; dy <= k; ++dy) {
                for (int dx = -k; dx <= k; ++dx) {
                    double neighbor = static_cast<double>(padded.at<uchar>(y + k + dy, x + k + dx));
                    double diff = neighbor - center;
                    double gaussIntensity = exp(-(diff * diff) / (2.0 * sigmaIntensity * sigmaIntensity));
                    double weight = spatialGauss[dy + k][dx + k] * gaussIntensity;
                    bilSum += neighbor * weight;
                    weightSum += weight;
                }
            }
            result.at<uchar>(y, x) = cv::saturate_cast<uchar>(bilSum / weightSum);
        }
    }
    return result;
}

