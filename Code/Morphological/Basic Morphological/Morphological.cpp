#include "Header.h"
#include <vector>

void Basic()
{
    // Fingerprint
    Mat img_origin = imread("D:/FresherXavisTech/Image/Screenshot 2025-05-29 112928.png", IMREAD_GRAYSCALE);
    Mat img;
    threshold(img_origin, img, 100, 255, THRESH_BINARY);

    // 3x3 SE
    Mat struct_elem = (Mat_<uchar>(3, 3) <<
        255, 255, 255,
        255, 255, 255,
        255, 255, 255
        );

    // Normalize
    for (int i = 0; i < struct_elem.rows; i++) {
        for (int j = 0; j < struct_elem.cols; j++) {
            struct_elem.at<uchar>(i, j) = (struct_elem.at<uchar>(i, j) > 0) ? 1 : 0;
        }
    }

    Mat eroded = Morphological::erosion(img, struct_elem);
    Mat opened = Morphological::opening(img, struct_elem);
    Mat dilated = Morphological::dilation(opened, struct_elem);
    Mat closed = Morphological::closing(opened, struct_elem);

    imshow("Original", img);
    imshow("Erosion", eroded);
    imshow("Dilation", dilated);
    imshow("Opening", opened);
    imshow("Closing", closed);
}

void HMT1()
{
    Mat image = (Mat_<uchar>(7, 15) <<
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 0, 255, 255, 255, 0, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 255, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        );

    Mat struct_elem1 = (Mat_<int>(3, 3) <<
        1, 1, 1,
        1, -1, 1,
        1, 1, 1
        );
    Mat struct_elem2 = (Mat_<int>(3, 3) <<
        -1, -1, -1,
        1, 1, -1,
        1, 1, -1
        );
    Mat struct_elem3 = (Mat_<int>(3, 3) <<
        0, 0, -1,
        1, 1, -1,
        0, 0, -1
        );
    Mat result1 = Morphological::HMT(image, struct_elem1);
    Mat result2 = Morphological::HMT(image, struct_elem2);
    Mat result3 = Morphological::HMT(image, struct_elem3);
    waitKey(0);
}

void HMT2()
{
    Mat image = (Mat_<uchar>(8, 8) <<
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 255, 255, 255, 0, 0, 0, 255,
        0, 255, 255, 255, 0, 0, 0, 0,
        0, 255, 255, 255, 0, 255, 0, 0,
        0, 0, 255, 0, 0, 0, 0, 0, 
        0, 0, 255, 0, 0, 255, 255, 0,
        0, 255, 0, 255, 0, 0, 255, 0,
        0, 255, 255, 0, 0, 0, 0, 0
        );
    Mat struct_elem1 = (Mat_<int>(3, 3) <<
        0, 1, 0,
        1, -1, 1,
        0, 1, 0
        );
    Mat struct_elem2 = (Mat_<int>(3, 3) <<
        0, -1, -1,
        1, 1, -1,
        0, 1, 0
        );
    Mat struct_elem3 = (Mat_<int>(3, 3) <<
        -1, -1, 0,
        -1, 1, 0,
        -1, -1, 0
        );
    Mat result1 = Morphological::HMT(image, struct_elem1);
    Mat result2 = Morphological::HMT(image, struct_elem2);
    Mat result3 = Morphological::HMT(image, struct_elem3);
    waitKey(0);
}

void boundary()
{
    Mat img_origin = imread("D:/FresherXavisTech/Image/Screenshot 2025-06-02 142720.png", IMREAD_GRAYSCALE);
    Mat img;
    threshold(img_origin, img, 100, 255, THRESH_BINARY);

    // 3x3 SE
    Mat struct_elem = (Mat_<uchar>(3, 3) <<
        255, 255, 255,
        255, 255, 255,
        255, 255, 255
        );
    // 5x5 SE
    //Mat struct_elem = (Mat_<uchar>(5, 5) <<
    //    255, 255, 255, 255, 255,
    //    255, 255, 255, 255, 255,
    //    255, 255, 255, 255, 255,
    //    255, 255, 255, 255, 255,
    //    255, 255, 255, 255, 255
    //    );
    // Normalize
    for (int i = 0; i < struct_elem.rows; i++) {
        for (int j = 0; j < struct_elem.cols; j++) {
            struct_elem.at<uchar>(i, j) = (struct_elem.at<uchar>(i, j) > 0) ? 1 : 0;
        }
    }

    Mat eroded = Morphological::erosion(img, struct_elem);
    Mat dilated = Morphological::dilation(img, struct_elem);

    Mat result1;
    Mat result2;
    subtract(img, eroded, result1);
    subtract(dilated, eroded, result2);
    waitKey(0);
}

void thinning()
{
    Mat image = (Mat_<uchar>(5, 11) <<
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 0, 0, 255, 255, 255, 255, 0, 0
        );

    vector<Mat> struct_elem;
    struct_elem.push_back((Mat_<int>(3, 3) <<
        -1, -1, -1,
        0, 1, 0,
        1, 1, 1));  // 0°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        0, -1, -1,
        1, 1, -1,
        1, 1, 0));  // 45°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        1, 0, -1,
        1, 1, -1,
        1, 0, -1));  // 90°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        1, 1, 0,
        1, 1, -1,
        0, -1, -1));  // 135°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        1, 1, 1,
        0, 1, 0,
        -1, -1, -1));  // 180°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        0, 1, 1,
        -1, 1, 1,
        -1, -1, 0));  // 225°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        -1, 0, 1,
        -1, 1, 1,
        -1, 0, 1));  // 270°

    struct_elem.push_back((Mat_<int>(3, 3) <<
        -1, -1, 0,
        -1, 1, 1,
        0, 1, 1));  // 315°

    Mat prev = image.clone();
    Mat curr = image.clone();
    while (true) {
        Mat diff;
        for (int i = 0; i < 8; ++i) {
            Mat hmt = Morphological::HMT(curr, struct_elem[i]);
            subtract(curr, hmt, curr);
        }
        absdiff(curr, prev, diff);
        if (countNonZero(diff) == 0)
            break;
        curr.copyTo(prev);
    }

    waitKey(0);
}

void HMTcompare()
{
    Mat image = (Mat_<uchar>(5, 11) <<
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 0, 0, 255, 255, 255, 255, 0, 0
        );
    //Mat image(7, 13, CV_8UC1, Scalar(255));
    //line(image, Point(4, 5), Point(5, 5), Scalar(0));
    //rectangle(image, Point(0, 0), Point(12, 6), Scalar(0));
    //rectangle(image, Point(10, 2), Point(11, 5), Scalar(0));
    Mat struct_elem = (Mat_<int>(3, 3) <<
        -1, -1, -1,
        0, 1, 0,
        1, 1, 1
        );
    Mat imgNormal = image/255;
    Mat result1 = Morphological::HMT(image, struct_elem);
    Mat result2;
    morphologyEx(image, result2, MORPH_HITMISS, struct_elem, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    waitKey(0);
}
int main()
{
    thinning();
    return 0;
}
