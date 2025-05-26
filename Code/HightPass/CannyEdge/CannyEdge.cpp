#include "Header.h"

int main()
{
    Mat img = imread("D:/FresherXavisTech/Image/Screenshot 2025-05-26 163706.png", IMREAD_GRAYSCALE); 
    if (img.empty()) {
        std::cerr << "Cannot open the image!\n";
        return -1;
    }
    imshow("Ảnh gốc", img); Mat img_64;
    img.convertTo(img_64, CV_64F, 1.0 / 255.0);
    /////////////////OPENCV/////////////////
    Mat edges;
    Canny(img, edges, 50, 150);
    imshow("OpenCV Canny Filter", edges);
    ////////////////MANUAL//////////////////
    Mat ManualCanny = CannyEdge::CannyEdgeFilter(img, 50, 100);
    imshow("Manual Canny Filter", ManualCanny);

    waitKey(0);
    return 0;
}