#ifndef _MORPHOLOGICAL_C_
#define _MORPHOLOGICAL_C_

#include "Header.h"

Mat Morphological::erosion(const Mat& img, const Mat& struct_elem) {
    CV_Assert(img.type() == CV_8UC1);
    int rows = img.rows;
    int cols = img.cols;
    int pad_y = struct_elem.rows / 2;
    int pad_x = struct_elem.cols / 2;
    Point anchor = Point(struct_elem.cols / 2, struct_elem.rows / 2);

    Mat eroded = Mat::zeros(rows, cols, CV_8UC1);
    Mat padded;
    copyMakeBorder(img, padded, pad_y, pad_y, pad_x, pad_x, BORDER_CONSTANT, 0);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            bool fits = true;
            for (int u = 0; u < struct_elem.rows; u++) {
                for (int v = 0; v < struct_elem.cols; v++) {
                    if (struct_elem.at<uchar>(u, v) == 0) continue;
                    int x = i + u;
                    int y = j + v;

                    if (padded.at<uchar>(x, y) == 0) {
                        fits = false;
                        break;
                    }
                }
                if (!fits) break;
            }
            eroded.at<uchar>(i, j) = fits ? 255 : 0;
        }
    }
    return eroded;
}

Mat Morphological::dilation(const Mat& img, const Mat& struct_elem) {
    CV_Assert(img.type() == CV_8UC1);
    int rows = img.rows;
    int cols = img.cols;
    int pad_x = struct_elem.cols / 2;
    int pad_y = struct_elem.rows / 2;

    Mat padded;
    copyMakeBorder(img, padded, pad_y, pad_y, pad_x, pad_x, BORDER_CONSTANT, 0);

    Mat dilated = Mat::zeros(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            bool hit = false;
            for (int u = 0; u < struct_elem.rows; ++u) {
                for (int v = 0; v < struct_elem.cols; ++v) {
                    if (struct_elem.at<uchar>(u, v) == 0) continue;

                    int x = i + u;
                    int y = j + v;

                    if (padded.at<uchar>(x, y) != 0) {
                        hit = true;
                        break;
                    }
                }
                if (hit) break;
            }

            dilated.at<uchar>(i, j) = hit ? 255 : 0;
        }
    }
    return dilated;
}

Mat Morphological::opening(const Mat& img, const Mat& struct_elem) {
    return Morphological::dilation(Morphological::erosion(img, struct_elem), struct_elem);
}

Mat Morphological::closing(const Mat& img, const Mat& struct_elem) {
    return Morphological::erosion(Morphological::dilation(img, struct_elem), struct_elem);
}

Mat Morphological::HMT(const Mat& img, const Mat& struct_elem)
{
    CV_Assert(img.type() == CV_8UC1);
    CV_Assert(struct_elem.type() == CV_32SC1);

    // Foreground mask
    Mat fg_mask = (struct_elem == 1);

    // Background mask
    Mat bg_mask = (struct_elem == -1);

    Mat eroded_fg;
    morphologyEx(img, eroded_fg, MORPH_ERODE, fg_mask, Point(-1,-1), 1, BORDER_CONSTANT, Scalar(0));

    Mat src_inv;
    bitwise_not(img, src_inv); // 0 <-> 255

    Mat eroded_bg;
    morphologyEx(src_inv, eroded_bg, MORPH_ERODE, bg_mask, Point(-1, -1), 1, BORDER_CONSTANT, Scalar(255));

    Mat result;
    bitwise_and(eroded_fg, eroded_bg, result);

    return result;
}

Mat Morphological::thinning(const Mat& img)
{
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

    Mat prev = img.clone();
    Mat curr = img.clone();
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
    return curr;
}

#endif // !_MORPHOLOGICAL_C_
