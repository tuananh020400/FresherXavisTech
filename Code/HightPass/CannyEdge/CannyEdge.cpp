#include "Header.h"
#include <chrono>

int main()
{
    Mat img = imread("D:/FresherXavisTech/Image/Image2.jpg", IMREAD_GRAYSCALE); 
    if (img.empty()) {
        std::cerr << "Cannot open the image!\n";
        return -1;
    }
    imshow("Original image", img);
    
    /**********************
    *       Open CV       *
    ***********************/
    Mat blurred_image, OpenCVCanny;
    GaussianBlur(img, blurred_image, cv::Size(5, 5), 1.4, 1.4);
    auto start = std::chrono::high_resolution_clock::now();
    Canny(blurred_image, OpenCVCanny, 50, 150, 3, true);
    auto end = std::chrono::high_resolution_clock::now();
    chrono::duration<double, std::milli> duration = end - start;
    cout << "Time openCV canny edge: " << duration.count() << " ms\n";
    imshow("OpenCV Canny Edges", OpenCVCanny);

    /**********************
    *       Manual        *
    ***********************/
    start = std::chrono::high_resolution_clock::now();
    Mat manualCanny = CannyEdge::CannyEdgeFilter(img, 50, 150);
    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    cout << "Time openCV canny edge: " << duration.count() << " ms\n";
    imshow("Manual Canny Edge", manualCanny);

    waitKey(0);
    return 0;
}