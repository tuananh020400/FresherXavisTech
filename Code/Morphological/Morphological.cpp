#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class Morphological {
private:
    Mat B;       // structuring element (CV_8U, giá trị 0 hoặc 1)
    Point anchor; // điểm neo của phần tử cấu trúc

public:
    Morphological(const Mat& struct_elem) {
        B = struct_elem.clone();
        // Điểm neo mặc định ở giữa
        anchor = Point(B.cols / 2, B.rows / 2);
    }

    Mat erosion(const Mat& img) {
        CV_Assert(img.type() == CV_8UC1);
        int m = img.rows;
        int n = img.cols;

        Mat eroded = Mat::zeros(m, n, CV_8UC1);

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                bool fits = true;
                for (int u = 0; u < B.rows; u++) {
                    for (int v = 0; v < B.cols; v++) {
                        if (B.at<uchar>(u, v) == 0) continue; // phần tử cấu trúc chỉ quan tâm chỗ =1
                        int x = i + u - anchor.y;
                        int y = j + v - anchor.x;

                        if (x < 0 || x >= m || y < 0 || y >= n) {
                            fits = false;
                            break;
                        }
                        if (img.at<uchar>(x, y) == 0) { // ảnh nhị phân: 0 = background
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

    Mat dilation(const Mat& img) {
        CV_Assert(img.type() == CV_8UC1);
        int m = img.rows;
        int n = img.cols;

        Mat dilated = Mat::zeros(m, n, CV_8UC1);

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                if (img.at<uchar>(i, j) == 0) continue;

                for (int u = 0; u < B.rows; u++) {
                    for (int v = 0; v < B.cols; v++) {
                        if (B.at<uchar>(u, v) == 0) continue;
                        int x = i + u - anchor.y;
                        int y = j + v - anchor.x;

                        if (x >= 0 && x < m && y >= 0 && y < n) {
                            dilated.at<uchar>(x, y) = 255;
                        }
                    }
                }
            }
        }
        return dilated;
    }

    Mat opening(const Mat& img) {
        return dilation(erosion(img));
    }

    Mat closing(const Mat& img) {
        return erosion(dilation(img));
    }
};

int main()
{
    // Tạo ảnh nhị phân 10x10 với 0 và 255
    Mat img = (Mat_<uchar>(10, 10) <<
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 255, 255, 255, 0, 0, 255, 255, 255, 0,
        0, 255, 0, 255, 0, 0, 255, 0, 255, 0,
        0, 255, 255, 255, 0, 0, 255, 255, 255, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 255, 255, 255, 255, 255, 255, 255, 255, 0,
        0, 255, 0, 0, 0, 0, 0, 0, 255, 0,
        0, 255, 255, 255, 255, 255, 255, 255, 255, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        );

    // Phần tử cấu trúc hình chữ thập 3x3
    Mat struct_elem = (Mat_<uchar>(3, 3) <<
        0, 255, 0,
        255, 255, 255,
        0, 255, 0);

    // Chuyển phần tử cấu trúc sang 0 hoặc 1
    for (int i = 0; i < struct_elem.rows; i++) {
        for (int j = 0; j < struct_elem.cols; j++) {
            struct_elem.at<uchar>(i, j) = (struct_elem.at<uchar>(i, j) > 0) ? 1 : 0;
        }
    }

    Morphological morph(struct_elem);

    Mat eroded = morph.erosion(img);
    Mat dilated = morph.dilation(img);
    Mat opened = morph.opening(img);
    Mat closed = morph.closing(img);

    imshow("Original", img);
    imshow("Erosion", eroded);
    imshow("Dilation", dilated);
    imshow("Opening", opened);
    imshow("Closing", closed);

    waitKey(0);
    return 0;
}
