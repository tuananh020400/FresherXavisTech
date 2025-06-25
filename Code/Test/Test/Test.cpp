#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

const int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
const int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };

// Kiểm tra xem pixel có phải foreground
bool isForeground(const Mat& img, int y, int x) {
    return (x >= 0 && x < img.cols&& y >= 0 && y < img.rows&& img.at<uchar>(y, x) == 255);
}

// Border Following đơn giản
vector<Point> borderFollow(Mat& img, Mat& visited, int sy, int sx) {
    vector<Point> contour;
    int cx = sx, cy = sy, dir = 0;
    contour.push_back(Point(cx, cy));
    visited.at<uchar>(cy, cx) = 1;

    do {
        bool found = false;
        for (int i = 0; i < 8; i++) {
            int nd = (dir + i) % 8;
            int nx = cx + dx[nd];
            int ny = cy + dy[nd];

            if (isForeground(img, ny, nx) && visited.at<uchar>(ny, nx) == 0) {
                contour.push_back(Point(nx, ny));
                visited.at<uchar>(ny, nx) = 1;
                cx = nx; cy = ny;
                dir = (nd + 6) % 8; // quay ngược 2 hướng
                found = true;
                break;
            }
        }
        if (!found) break;
    } while (!(cx == sx && cy == sy));

    return contour;
}

int main() {
    // Tạo ảnh nhị phân nhỏ đơn giản
    Mat img = Mat::zeros(10, 10, CV_8UC1);
    img(Rect(3, 3, 4, 4)) = 255;

    // Vẽ thêm 1 lỗ nhỏ
    img.at<uchar>(4, 4) = 0;

    Mat visited = Mat::zeros(img.size(), CV_8UC1);
    vector<vector<Point>> contours;

    // Duyệt từng pixel
    for (int y = 1; y < img.rows - 1; y++) {
        for (int x = 1; x < img.cols - 1; x++) {
            if (img.at<uchar>(y, x) == 255 &&
                visited.at<uchar>(y, x) == 0 &&
                img.at<uchar>(y - 1, x) == 0) // biên ngoài
            {
                vector<Point> contour = borderFollow(img, visited, y, x);
                contours.push_back(contour);
            }
        }
    }

    // Hiển thị kết quả
    Mat colorImg;
    cvtColor(img, colorImg, COLOR_GRAY2BGR);
    for (const auto& contour : contours) {
        for (size_t i = 0; i < contour.size(); ++i) {
            circle(colorImg, contour[i], 1, Scalar(0, 0, 255), -1);
            if (i > 0)
                line(colorImg, contour[i - 1], contour[i], Scalar(0, 255, 0), 1);
        }
    }

    imshow("Contours by Border Following", colorImg);
    waitKey(0);
    return 0;
}
