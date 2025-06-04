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
    struct_elem = getStructuringElement(MORPH_RECT, Size(5, 5));

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
    Mat thinned = Morphological::thinning(closed);
    Mat skeleted = Morphological::Skeleton(closed, getStructuringElement(MORPH_RECT, Size(3, 3)));
    closed = Morphological::closing(skeleted, getStructuringElement(MORPH_RECT, Size(3, 3)));

    imshow("Original", img);
    imshow("Erosion", eroded);
    imshow("Dilation", dilated);
    imshow("Opening", opened);
    imshow("Closing", closed);
    imshow("Thinned", thinned);
    imshow("Skeleton", skeleted);
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

void Thin()
{
    Mat image = (Mat_<uchar>(5, 11) <<
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 0, 0, 255, 255, 255, 255, 0, 0
        );

    Mat thined = Morphological::thinning(image);
    waitKey(0);
}

void HMTcompare()
{
    //https://chatgpt.com/share/683e8104-3bc8-8004-8b35-dc5563e9a991
    //Mat image(7, 13, CV_8UC1, Scalar(255));
    //line(image, Point(4, 5), Point(5, 5), Scalar(0));
    //rectangle(image, Point(0, 0), Point(12, 6), Scalar(0));
    //rectangle(image, Point(10, 2), Point(11, 5), Scalar(0));
    Mat image = (Mat_<uchar>(5, 11) <<
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0,
        255, 255, 255, 0, 0, 255, 255, 255, 255, 0, 0
        );
    Mat struct_elem = (Mat_<int>(3, 3) <<
        -1, -1, -1,
        0, 1, 0,
        1, 1, 1
        );
    Mat result1 = Morphological::HMT(image, struct_elem);
    Mat result2;
    Mat result3;
    Mat result4;

    morphologyEx(image, result2, MORPH_HITMISS, struct_elem, Point(-1, -1), 1, BORDER_CONSTANT, Scalar(0));
    morphologyEx(image, result3, MORPH_HITMISS, struct_elem, Point(-1, -1), 1, BORDER_CONSTANT, Scalar(255));
    morphologyEx(image, result4, MORPH_HITMISS, struct_elem);
    waitKey(0);
}

void MorphologicalSkeleton()
{
    Mat img(10, 5, CV_8UC1, Scalar(255));
    line(img, Point(0, 1), Point(0, 4), Scalar(0));
    line(img, Point(4, 0), Point(4, 4), Scalar(0));
    line(img, Point(3, 0), Point(3, 2), Scalar(0));
    line(img, Point(1, 0), Point(2, 0), Scalar(0));

    Mat struct_elem = getStructuringElement(MORPH_RECT, Size(3, 3));

    Mat skeleton = Mat::zeros(img.size(), CV_8UC1);
    Mat eroded, opened, sk;

    morphologyEx(img, opened, MORPH_OPEN, struct_elem);
    subtract(img, opened, sk);
    bitwise_or(skeleton, sk, skeleton);
    Mat prev = img.clone();

    vector<Mat> restore;
    Mat restored = Mat::zeros(img.size(), CV_8UC1);
    bitwise_or(restored, sk, restored);

    // Skeleton
    while (true) {
        erode(prev, eroded, struct_elem, Point(-1, -1), 1, BORDER_CONSTANT, Scalar(0));
        morphologyEx(eroded, opened, MORPH_OPEN, struct_elem);

        subtract(eroded, opened, sk);

        if (countNonZero(eroded) == 0)
            break;

        restore.push_back(sk.clone());       // Lưu layer skeleton
        bitwise_or(skeleton, sk, skeleton);

        prev = eroded.clone();
    }

    // Restore from skeleton layers
    for (size_t i = 0; i < restore.size(); i++) {
        Mat tmp = restore[i].clone();
        for (size_t j = 0; j <= i; j++) {
            dilate(tmp, tmp, struct_elem);
        }
        bitwise_or(restored, tmp, restored);
    }

    imshow("Original", img);
    imshow("Skeleton", skeleton);
    imshow("Restored", restored);


    Mat result = Morphological::Skeleton(img, struct_elem);
    waitKey(0);
}

int main()
{
    cout << "morphologyDefaultBorderValue = " << morphologyDefaultBorderValue() << endl;
    //HMTcompare();
    //MorphologicalSkeleton();
    waitKey(0);
    return 0;
}
