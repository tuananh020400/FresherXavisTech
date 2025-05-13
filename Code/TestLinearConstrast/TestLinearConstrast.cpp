#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

uchar piecewiseStretch(int r, int r1, int r2, int s1, int s2);

Mat BinarySlicing(const Mat& img, int lower, int upper);
Mat HighlightSlicing(const Mat& img, int lower, int upper);
Mat LinearStretched(const Mat& img);
Mat PiecewiseLinearStretched(const Mat& img, int r1, int r2, int s1, int s2);

int main() 
{
    Mat img = imread("D:/FresherXavisTech/Image/Screenshot 2025-05-08 083637.png", IMREAD_GRAYSCALE);
    if (img.empty())
    {
        cout << "Cannot open the image!";
        return -1;
    }

    Mat stretched;
    stretched = LinearStretched(img);
    imshow("Linear contrast stretched", stretched);

    Mat peacewise;
    peacewise = PiecewiseLinearStretched(img, 70, 180, 30, 220);
    imshow("Peacewise Linear contrast stretched", peacewise);

    Mat binary;
    binary = BinarySlicing(stretched, 100, 150);
    imshow("BinarySlicing", binary);

    Mat highlight;
    highlight = HighlightSlicing(stretched, 100, 150);
    imshow("HightlightSlicing", highlight);

    // Lưu và hiển thị kết quả
    imwrite("stretched.jpg", stretched);
    imshow("Original Image", img);
    
    waitKey(0);
    return 0;
}

Mat BinarySlicing(const Mat& img, int lower, int upper)
{
    Mat output = Mat::zeros(img.size(), CV_8UC1); // Ảnh nhị phân đầu ra

    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            uchar pixel = img.at<uchar>(i, j);
            if (pixel > lower && pixel < upper)
            {
                output.at<uchar>(i, j) = 255;
            }
            else
            {
                output.at<uchar>(i, j) = 0;
            }
        }
    }
    return output;
}

Mat LinearStretched(const Mat& img)
{
    int rows = img.rows;
    int cols = img.cols;

    uchar r_min = 255;
    uchar r_max = 0;

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            uchar pixel = img.at<uchar>(i, j);
            if (pixel < r_min) r_min = pixel;
            if (pixel > r_max) r_max = pixel;
        }
    }
    cout << "Min value: " << (int)r_min << endl;
    cout << "Max value: " << (int)r_max << endl;

    // Tạo ảnh kết quả
    Mat stretched = Mat::zeros(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            uchar pixel = img.at<uchar>(i, j);
            uchar new_val = (uchar)((float)(pixel - r_min) / (r_max - r_min) * 255.0f);
            stretched.at<uchar>(i, j) = new_val;
        }
    }

    return stretched;
}

Mat HighlightSlicing(const Mat& img, int lower, int upper)
{
    Mat output = img.clone();

    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            uchar pixel = img.at<uchar>(i, j);
            if (pixel > lower && pixel < upper)
            {
                output.at<uchar>(i, j) = 255;
            }
        }
    }
    return output;
}

// Hàm nội suy tuyến tính theo đoạn
uchar piecewiseStretch(int r, int r1, int r2, int s1, int s2) {
    if (r < r1)
        return (uchar)(s1 * r) / r1;
    else if (r <= r2)
        return (uchar)((s2 - s1) * (r - r1) / (r2 - r1) + s1);
    else
        return (uchar)((255 - s2) * (r - r2) / (255 - r2) + s2);
}

Mat PiecewiseLinearStretched(const Mat& img, int r1, int r2, int s1, int s2)
{
    Mat output = img.clone();
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            int r = img.at<uchar>(i, j);
            output.at<uchar>(i, j) = piecewiseStretch(r, r1, r2, s1, s2);
        }
    }
    return output;
}