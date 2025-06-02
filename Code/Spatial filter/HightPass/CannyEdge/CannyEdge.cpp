#include "Header.h"
#include <chrono>

int main()
{
    Mat image = imread("D:/FresherXavisTech/Image/Picture1.png", IMREAD_GRAYSCALE); 
    if (image.empty()) {
        std::cerr << "Cannot open the image!\n";
        return -1;
    }
    imshow("Original image", image);
    
    /**********************
    *       Open CV       *
    ***********************/
    Mat blurred_image, OpenCVCanny;
    GaussianBlur(image, blurred_image, cv::Size(5, 5), 1.4, 1.4);
    auto start = std::chrono::high_resolution_clock::now();
    Canny(blurred_image, OpenCVCanny, 50, 100, 3, true);
    auto end = std::chrono::high_resolution_clock::now();
    chrono::duration<double, std::milli> duration = end - start;
    cout << "Time openCV canny edge: " << duration.count() << " ms\n";
    imshow("OpenCV Canny Edges", OpenCVCanny);
    imwrite("D:/FresherXavisTech/Image/Result/CannyOpenCV.jpg", OpenCVCanny);

    /**********************
    *       Manual        *
    ***********************/
    start = std::chrono::high_resolution_clock::now();
    Mat manualCanny = CannyEdge::CannyEdgeFilter(image, 50, 150);
    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    cout << "Time openCV canny edge: " << duration.count() << " ms\n";
    imshow("Manual Canny Edge", manualCanny);
    imwrite("D:/FresherXavisTech/Image/Result/CannyManual.jpg", manualCanny);
    waitKey(0);
    return 0;
}