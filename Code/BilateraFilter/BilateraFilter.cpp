#include "Header.h"

int main() {
    // Đọc ảnh màu
    Mat image = cv::imread("D:/FresherXavisTech/Image/Screenshot 2025-05-15 001732.png", IMREAD_GRAYSCALE); // thay bằng đường dẫn ảnh của bạn
    if (image.empty()) {
        std::cerr << "Không mở được ảnh\n";
        return -1;
    }
    imshow("Original", image);

    //Manual BilateralFilter
    Mat ManualBil = BilateralFilter::ManualBilateralFilter(image, 25, 75, 75);
    imshow("Manual Bilateral Filter", ManualBil);

    Mat Bil;
    bilateralFilter(image, Bil, 25, 75, 75, BORDER_DEFAULT);
    imshow("OpenCV Bilateral Filter", Bil);
    waitKey(0);
    return 0;
}