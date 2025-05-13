#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

void CalculateHistAndCDF(Mat& image, vector<float>& hist, vector<float>& cdf);
Mat ManualHistogramEqualization(Mat& src);
Mat ManualMachingHistogram(Mat& src, Mat& ref);
Mat Display(Mat& image);
Mat LocalHistogramEqualization(Mat& image, int blockSize);

int main() {
    // Read the image in grayscale
    Mat image = imread("C:/Users/tuana/Downloads/Image1.png", IMREAD_GRAYSCALE);
    Mat ref = imread("C:/Users/tuana/Downloads/Screenshot 2025-05-09 154617.png", IMREAD_GRAYSCALE);
    if (image.empty() || ref.empty()) {
        cout << "Could not open or find the image!\n";
        return -1;
    }
    //Original Histogram
    Mat imageHist = Display(image);
    Mat refHist = Display(ref);

    imshow("Image Histogram", imageHist);
    imshow("Ref Histogram", refHist);

    ////Manual histogram equalization
    //Mat manualEqualized_img = image.clone();
    //manualEqualized_img = ManualHistogramEqualization(image);
    //Mat manualEqualizeHist = Display(manualEqualized_img);
    //imshow("Manual Equalize Image", manualEqualized_img);
    //imshow("Manual Equalize Histogram", manualEqualizeHist);

    //// Perform histogram equalization using OpenCV
    //Mat equalized_img = image.clone();
    //equalizeHist(image, equalized_img);
    //Mat EqualizeHist = Display(equalized_img);
    //imshow("Equalize Histogram", EqualizeHist);

    //// Manual histogram specification
    //Mat manualMatching_img = ManualMachingHistogram(image, ref);
    //imshow("Manual Matching Image", manualMatching_img);
    //Mat manualMatchingHist = Display(manualMatching_img);
    //imshow("Manual Matching Histogram", manualMatchingHist);

    // Manual local histogram equalization
    Mat localHistogram = LocalHistogramEqualization(image, 32);
    imshow("Local Histogram Equalization", localHistogram);

    waitKey(0);
    return 0;
}

void CalculateHistAndCDF(Mat& image, vector<float>& hist, vector<float>& cdf)
{
    int total = image.rows * image.cols;

    // Calculate Hist
    hist.assign(256, 0);
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            hist[image.at<uchar>(i, j)]++;
        }
    }
    for (int i = 0; i < 256; i++)
    {
        hist[i] /= total;
    }
    //Calculate CDF
    cdf.resize(256);
    cdf[0] = hist[0];
    for (int i = 1; i < 256; i++)
    {
        cdf[i] = cdf[i - 1] + hist[i];
    }
}

//Histogram Equalization
Mat ManualHistogramEqualization(Mat& src)
{
    vector<float> hist, cdf;
    CalculateHistAndCDF(src, hist, cdf);
    Mat output = src.clone();

    uchar LUT[256];
    for (int i = 0; i < 256; i++)
    {
        //LUT[i] = cvRound((cdf[i] - cdf[0]) / (1 - cdf[0]) * 255);
        LUT[i] = static_cast<uchar>((cdf[i] - cdf[0]) / (1.0f - cdf[0]) * 255.0f);
    }

    //Apply the CDF to the original image
    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
            int pixelValue = src.at<uchar>(i, j);
            output.at<uchar>(i, j) = LUT[pixelValue];
        }
    }

    return output;
}

Mat ManualMachingHistogram(Mat& src, Mat& ref)
{
    Mat result = src.clone();
    vector<float> hist_src, cdf_src;
    vector<float> hist_ref, cdf_ref;

    CalculateHistAndCDF(src, hist_src, cdf_src);
    CalculateHistAndCDF(ref, hist_ref, cdf_ref);

    uchar LUT[256];
    for (int i = 0; i < 256; ++i)
    {
        float val = cdf_src[i];
        uchar j = 0;
        // Tìm mức xám bên ảnh tham chiếu sao cho CDF gần nhất với val
        while (j < 255 && cdf_ref[j] < val) j++;
        LUT[i] = j;
    }
    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
            result.at<uchar>(i, j) = LUT[src.at<uchar>(i, j)];
        }
    }
    return result;
}

Mat Display(Mat& image)
{
    int histogram[256] = { 0 };

    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            unsigned char pixel_value = image.at<unsigned char>(y, x);
            histogram[pixel_value]++;
        }
    }

    // Tìm giá trị lớn nhất trong histogram để chuẩn hóa
    int histMax = *max_element(histogram, histogram + 256);

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double)hist_w / 256);

    Mat histImage(hist_h, hist_w, CV_8UC1, Scalar(255));

    // Vẽ các dòng của histogram
    for (int i = 1; i < 256; i++) {
        line(histImage,
            Point(bin_w * (i - 1), hist_h - (histogram[i - 1] * hist_h / histMax)),
            Point(bin_w * i, hist_h - (histogram[i] * hist_h / histMax)),
            Scalar(0), 2);
    }

    return histImage;
}
Mat LocalHistogramEqualization(Mat& image, int blockSize)
{
    Mat result = image.clone();

    for (int x = 0; x < image.cols; x += blockSize)
    {
        for (int y = 0; y < image.rows; y += blockSize)
        {
            int width = min(blockSize, image.cols - x);
            int height = min(blockSize, image.rows - y);
            Rect roi(x,y,width,height);
            Mat srcBlock = image(roi);
            Mat dstBlock = result(roi);
            equalizeHist(srcBlock, dstBlock);
            dstBlock.copyTo(result(roi));
        }
    }
    return result;
}