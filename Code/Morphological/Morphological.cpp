#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat erosion(const Mat& img, const Mat& struct_elem) {
    CV_Assert(img.type() == CV_8UC1);
    int rows = img.rows;
    int cols = img.cols;
    int pad_y = struct_elem.rows / 2;
    int pad_x = struct_elem.cols / 2;
    Point anchor = Point(struct_elem.cols / 2, struct_elem.rows / 2);

    Mat eroded = Mat::zeros(rows, cols, CV_8UC1);
    Mat padded;
    copyMakeBorder(img, padded, pad_y, pad_y, pad_x, pad_x, BORDER_CONSTANT, 0);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            bool fits = true;
            for (int u = 0; u < struct_elem.rows; u++) {
                for (int v = 0; v < struct_elem.cols; v++) {
                    if (struct_elem.at<uchar>(u, v) == 0) continue;
                    int x = i + u;
                    int y = j + v;

                    if (padded.at<uchar>(x, y) == 0) {
                        fits = false;
                        break;
                    }
                }
                if (!fits) break;
            }
            eroded.at<uchar>(i, j) = fits ? 255 : 0;
        }
    }
    return eroded;
}

Mat dilation(const Mat& img, const Mat& struct_elem) {
    CV_Assert(img.type() == CV_8UC1);
    int rows = img.rows;
    int cols = img.cols;
    int pad_x = struct_elem.cols/2;
    int pad_y = struct_elem.rows/2;

    Mat padded;
    copyMakeBorder(img, padded, pad_y, pad_y, pad_x, pad_x, BORDER_CONSTANT, 0);

    Mat dilated = Mat::zeros(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            bool hit = false;
            for (int u = 0; u < struct_elem.rows; ++u) {
                for (int v = 0; v < struct_elem.cols; ++v) {
                    if (struct_elem.at<uchar>(u, v) == 0) continue;

                    int x = i + u;
                    int y = j + v;

                    if (padded.at<uchar>(x, y) != 0) {
                        hit = true;
                        break;
                    }
                }
                if (hit) break;
            }

            dilated.at<uchar>(i, j) = hit ? 255 : 0;
        }
    }
    return dilated;
}

Mat opening(const Mat& img, const Mat& struct_elem) {
    return dilation(erosion(img, struct_elem), struct_elem);
}

Mat closing(const Mat& img, const Mat& struct_elem) {
    return erosion(dilation(img, struct_elem), struct_elem);
}


int main()
{
    //Mat img = /*imread("D:/FresherXavisTech/Image/Gemini_Generated_Image_seb0pmseb0pmseb0.png", IMREAD_GRAYSCALE);*/
    //    (Mat_<uchar>(10, 10) <<
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 255, 255, 255, 0, 0, 255, 255, 255, 0,
    //    0, 255, 0, 255, 0, 0, 255, 0, 255, 0,
    //    0, 255, 255, 255, 0, 0, 255, 255, 255, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 255, 255, 255, 255, 255, 255, 255, 255, 0,
    //    0, 255, 0, 0, 0, 0, 0, 0, 255, 0,
    //    0, 255, 255, 255, 255, 255, 255, 255, 255, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    //    );

    // Vân tay
    //Mat img_origin = imread("D:/FresherXavisTech/Image/Screenshot 2025-05-29 112928.png", IMREAD_GRAYSCALE);
    //Mat img;
    //threshold(img_origin, img, 100, 255, THRESH_BINARY);
    
    // 5x5 SE
    Mat struct_elem = (Mat_<uchar>(5, 5) <<
    255, 255, 255, 255, 255,
    255, 0, 0, 0, 255,
    255, 0, 0, 0, 255,
    255, 0, 0, 0, 255,
    255, 255, 255, 255, 255
    );

    //// 3x3 SE
    //Mat struct_elem = (Mat_<uchar>(3, 3) <<
    //    255, 255, 255,
    //    255, 255, 255,
    //    255, 255, 255
    //    );

    Mat img = (Mat_<uchar>(16, 16) <<
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    );
     

    //Ảnh slide dilation
    //cv::Mat img = (cv::Mat_<uchar>(13, 14) <<
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 255, 255, 255, 0, 255, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 255, 255, 255, 0, 0, 255, 0, 0, 255, 0, 0,
    //    0, 0, 0, 255, 255, 255, 255, 255, 255, 0, 255, 255, 0, 0,
    //    0, 0, 0, 255, 255, 255, 255, 255, 255, 0, 255, 255, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    //    );
    ////Ảnh slide erotion
    //cv::Mat img = (cv::Mat_<uchar>(13, 19) <<
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0,
    //    0, 0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0,
    //    0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0,
    //    0, 0, 255, 255, 255, 255, 255, 255, 255, 0, 255, 0, 0, 0, 0, 0, 0, 255, 0,
    //    0, 0, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 255, 255, 255, 0,
    //    0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 0,
    //    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 0,
    //    0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,
    //    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,
    //    0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 0,
    //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0
    //    );

    // Chuyển phần tử cấu trúc sang 0 hoặc 1
    for (int i = 0; i < struct_elem.rows; i++) {
        for (int j = 0; j < struct_elem.cols; j++) {
            struct_elem.at<uchar>(i, j) = (struct_elem.at<uchar>(i, j) > 0) ? 1 : 0;
        }
    }

    Mat eroded = erosion(img, struct_elem);
    Mat opened = opening(img, struct_elem);
    Mat dilated = dilation(opened, struct_elem);
    Mat closed = closing(opened, struct_elem);

    imshow("Original", img);
    imshow("Erosion", eroded);
    imshow("Dilation", dilated);
    imshow("Opening", opened);
    imshow("Closing", closed);

    waitKey(0);
    return 0;
}
