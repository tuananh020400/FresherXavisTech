#include "SpactialFilter.h"

int main() {
    // Đọc ảnh màu
    Mat image = cv::imread("D:/FresherXavisTech/Image/Screenshot 2025-05-15 001732.png", IMREAD_GRAYSCALE); // thay bằng đường dẫn ảnh của bạn
    if (image.empty()) {
        std::cerr << "Không mở được ảnh\n";
        return -1;
    }
    imshow("Original", image);
    
    //// Manual Box filter
    //Mat manualBox = SpactialFiltering::ManualBoxFilter(image, 25);
    //imshow("Manual Box Filter", manualBox);

    //// OpenCV Box filter 
    //Mat Box = image.clone();
    //blur(image, Box, Size(25, 25));
    //imshow("Box Filter OpenCV", Box);

    //// Open Gauss Blur
    //Mat Gauss;
    //GaussianBlur(image, Gauss, Size(25,25), 3.0, 3.0, BORDER_DEFAULT);
    //imshow("OpenCV Gaussian Filter", Gauss);

    // Manual Gauss Blur
    Mat ManualGauss = SpactialFiltering::ManualGaussianFilter(image, 25, 3.0);
    imshow("Manual Gauss Filter", ManualGauss);

    //Manual BilateralFilter
    Mat ManualBil = SpactialFiltering::ManualBilateralFilter(image, 25, 75, 75);
    imshow("Manual Bilateral Filter", ManualBil);

    Mat Bil;
    bilateralFilter(image, Bil, 25, 75, 75, BORDER_DEFAULT);
    imshow("OpenCV Bilateral Filter", Bil);

    waitKey(0);
    return 0;
}