#include "Header.h"

int main()
{
    Mat img = imread("D:/FresherXavisTech/Image/images.jpg", IMREAD_GRAYSCALE); 
    if (img.empty()) {
        std::cerr << "Cannot open the image!\n";
        return -1;
    }
    imshow("Original image", img);
    
    /**********************
    *       Open CV       *
    ***********************/
    Mat blurred_image;
    GaussianBlur(img, blurred_image, cv::Size(5, 5), 1.4, 1.4);
    imshow("Blurred Image (Gaussian)", blurred_image);

    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Mat grad_magnitude;
    Sobel(blurred_image, grad_x, CV_64F, 1, 0, 3);
    convertScaleAbs(grad_x, abs_grad_x);
    Sobel(blurred_image, grad_y, CV_64F, 0, 1, 3);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_magnitude);

    imshow("Gradient X", abs_grad_x);
    imshow("Gradient Y", abs_grad_y);
    imshow("Gradient Magnitude", grad_magnitude);
    
    Mat OpenCVCanny;
    Canny(blurred_image, OpenCVCanny, 50, 150);
    imshow("4. Canny Edges (Final result including NMS and Hysteresis)", OpenCVCanny);

    /**********************
    *       Manual        *
    ***********************/
    Mat manualCanny = CannyEdge::CannyEdgeFilter(img, 30, 100);
    imshow("Manual Canny Edge", manualCanny);
    waitKey(0);
    return 0;
}