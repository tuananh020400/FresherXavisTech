#include <opencv2/opencv.hpp>
#include <iostream>

// Hàm in ma trận đơn giản, không tô màu
void printMatSimple(const cv::Mat& mat, const std::string& name) {
    std::cout << name << " (" << mat.rows << "x" << mat.cols << "):\n";
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            std::cout << static_cast<int>(mat.at<uchar>(i, j)) << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// Hàm in ma trận có tô màu phần ma trận gốc (dùng ANSI escape code, áp dụng cho Linux/macOS console)
void printMatWithHighlight(const cv::Mat& mat, int top, int bottom, int left, int right, const cv::Mat& original) {
    int orig_rows = original.rows;
    int orig_cols = original.cols;

    std::cout << "Matrix with highlighted original region:\n";
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            int val = mat.at<uchar>(i, j);
            bool in_original = (i >= top && i < top + orig_rows) && (j >= left && j < left + orig_cols);
            if (in_original) {
                // Màu đỏ cho vùng ma trận gốc
                std::cout << "\033[31m" << val << "\t" << "\033[0m";
            }
            else {
                // Màu mặc định cho phần viền
                std::cout << val << "\t";
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

int main() {
    cv::Mat input = (cv::Mat_<uchar>(6, 6) <<
        1, 2, 3, 4, 5, 1,
        6, 7, 8, 9, 10, 1,
        11, 12, 13, 14, 15, 1,
        16, 17, 18, 19, 20, 1,
        21, 22, 23, 24, 25, 1,
        21, 22, 23, 24, 25, 1);

    // Thông số viền
    int top = 2, bottom = 2, left = 2, right = 2;

    // Tạo ma trận có viền
    cv::Mat bordered, bordered101;
    cv::copyMakeBorder(input, bordered, top, bottom, left, right, cv::BORDER_WRAP, cv::Scalar(0));
    cv::copyMakeBorder(input, bordered101, top, bottom, left, right, cv::BORDER_DEFAULT, cv::Scalar(0));

    // In ma trận gốc không màu
    printMatSimple(input, "Input");

    // In ma trận BORDER_REFLECT có tô màu vùng ma trận gốc
    std::cout << "BORDER_WRAP:\n";
    printMatWithHighlight(bordered, top, bottom, left, right, input);

    // In ma trận BORDER_REFLECT_101 có tô màu vùng ma trận gốc
    std::cout << "BORDER_REFLECT_101:\n";
    printMatWithHighlight(bordered101, top, bottom, left, right, input);

    return 0;
}
