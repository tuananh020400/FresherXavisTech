#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    // Đọc ảnh màu
    Mat img = imread("D:/FresherXavisTech/Image/Screenshot 2025-05-15 001732.png", IMREAD_GRAYSCALE);
    if (img.empty()) {
        cout << "Cannot open the image!" << endl;
        return -1;
    }

    // Giảm kích thước ảnh: N/2 và N/4
    Mat img_half, img_quarter, imgUp;
    pyrDown(img, img_half);       // N/2
    pyrDown(img_half, img_quarter); // N/4
    pyrUp(img_half, imgUp);

    // Áp dụng Canny edge detection
    Mat edges_original, edges_half, edges_quarter, edges_imgUp;
    Canny(img, edges_original, 100, 200);
    Canny(img_half, edges_half, 100, 200);
    Canny(img_quarter, edges_quarter, 100, 200);
    Canny(imgUp, edges_imgUp, 100, 200);

    // Hiển thị kết quả
    imshow("Original Image", img);
    imshow("Edges - Original", edges_original);

    imshow("Image N/2", img_half);
    imshow("Edges - N/2", edges_half);

    //imshow("Image N/4", img_quarter);
    //imshow("Edges - N/4", edges_quarter);

    imshow("Img Up", imgUp);
    imshow("Edges ImgUp", edges_imgUp);

    waitKey(0);
    return 0; 
}
