#include "SpactialFilter.h"
#include <chrono>
using namespace chrono;

int main() {
    // Đọc ảnh màu
    Mat image = cv::imread("D:/FresherXavisTech/Image/240129_A556_PANEL ID_43(14_38)_M 2(F 1_2).tif", IMREAD_GRAYSCALE); // thay bằng đường dẫn ảnh của bạn
    if (image.empty()) {
        std::cerr << "Không mở được ảnh\n";
        return -1;
    }
    imshow("Original", image);
    imwrite("D:/FresherXavisTech/Image/Result/Original.png", image);


    //// Manual Box filter
    //Mat manualBox = SpactialFiltering::ManualBoxFilter(image, 25);
    //imshow("Manual Box Filter", manualBox);

    //// OpenCV Box filter 
    //Mat Box = image.clone();
    //blur(image, Box, Size(25, 25));
    //imshow("Box Filter OpenCV", Box);

     //OpenCV Gauss Blur
    Mat Gauss;
    auto start = high_resolution_clock::now();
    GaussianBlur(image, Gauss, Size(9,9), 3.0, 3.0, BORDER_CONSTANT);
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start);
    cout << "Time of gauss: " << duration.count() << " ms" << endl;
    imshow("OpenCV Gaussian Filter", Gauss);
    imwrite("D:/FresherXavisTech/Image/Result/Gauss.png", Gauss);

    //// Manual Gauss Blur
    //Mat ManualGauss;
    //start = high_resolution_clock::now();
    //ManualGauss = SpactialFiltering::ManualGaussianFilter(image, 25, 5.0);
    //end = high_resolution_clock::now();
    //duration = duration_cast<milliseconds>(end - start);
    //cout << "Time of manual gauss: " << duration.count() << " ms" << endl;
    //imshow("Manual Gauss Filter", ManualGauss);

     
    // OpenCV Bilateral Filter
    Mat Bil;
    bilateralFilter(image, Bil, 9, 25, 3, BORDER_CONSTANT);
    imshow("OpenCV Bilateral Filter", Bil);
    imwrite("D:/FresherXavisTech/Image/Result/Bilateral.png", Bil);

    ////Manual BilateralFilter
    //Mat ManualBil = SpactialFiltering::ManualBilateralFilter(image, 25, 10, 200.0);
    //imshow("Manual Bilateral Filter", ManualBil);

    ////Manual Sobel Filter
    //Mat ManualSobel = SpactialFiltering::ManualSobelFilter(image, 3);
    //imshow("Manual Sobel Filter", ManualSobel);

    ////OpenCV Sobel Filter
    //Mat grad_x, grad_y;
    //Mat abs_grad_x, abs_grad_y, grad;

    //// Tính đạo hàm theo x và y
    //Sobel(image, grad_x, CV_16S, 1, 0, 3);
    //Sobel(image, grad_y, CV_16S, 0, 1, 3);
    //// Chuyển sang CV_8U (vì CV_16S có thể chứa giá trị âm)
    //convertScaleAbs(grad_x, abs_grad_x);
    //convertScaleAbs(grad_y, abs_grad_y);

    //// Tổng hợp đạo hàm
    //addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    //// Hiển thị kết quả
    //imshow("Sobel", grad);

    waitKey(0);
    return 0;
}