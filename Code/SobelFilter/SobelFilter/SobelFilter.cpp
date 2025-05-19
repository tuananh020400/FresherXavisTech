#include "Header.h"

int main() {
    // Đọc ảnh màu
    Mat image = cv::imread("D:/FresherXavisTech/Image/Screenshot 2025-05-15 001732.png", IMREAD_GRAYSCALE); // thay bằng đường dẫn ảnh của bạn
    if (image.empty()) {
        std::cerr << "Không mở được ảnh\n";
        return -1;
    }
    imshow("Original", image);

    /////////////////////////////////////////////////Manual Sobel Filter
    Mat ManualSobel = SobelFilter::ManualSobelFilter(image, 3);
    imshow("Manual Sobel Filter", ManualSobel);

    ////////////////////////////////////////////////OpenCV Sobel Filter
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y, grad;

    // Tính đạo hàm theo x và y
    Sobel(image, grad_x, CV_64F, 1, 0, 3);
    Sobel(image, grad_y, CV_64F , 0, 1, 3);
    // Chuyển sang CV_8U (vì CV_16S có thể chứa giá trị âm)
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);

    // Tổng hợp đạo hàm
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    // Hiển thị kết quả
    imshow("Sobel", grad);
    imshow("Gradian X", abs_grad_x);
    imshow("Gradian Y", abs_grad_y);

    waitKey(0);
    return 0;
}