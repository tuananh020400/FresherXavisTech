#include "Header.h"

int main()
{
    Mat img = imread("D:/FresherXavisTech/Image/Screenshot 2025-05-15 001732.png", IMREAD_GRAYSCALE); 
    if (img.empty()) {
        std::cerr << "Cannot open the image!\n";
        return -1;
    }
    imshow("Ảnh gốc", img);
    /////////////////OPENCV/////////////////
    Mat edges;;
    Canny(img, edges, 50, 150);
    imshow("Biên Canny", edges);
    ////////////////MANUAL//////////////////
    Mat ManualCanny = CannyEdge::CannyEdgeFilter(img, 50, 150);
    imshow("Manual Canny Filter", ManualCanny);

    waitKey(0);
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
