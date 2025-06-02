#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Mat morphologyHMT(const Mat img, const Mat se)
{
    CV_Assert(img.type() == CV_8UC1);
    CV_Assert(se.type() == CV_32SC1);

    Mat result = Mat::zeros(img.size(), CV_8UC1);

    Mat padded;
    int padY = se.rows / 2;
    int padX = se.cols / 2;
    copyMakeBorder(img, padded, padY, padY, padX, padX, BORDER_DEFAULT);

    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j){
            bool match = true;
            for (int u = 0; u < se.rows && match; ++u) {
                for (int v = 0; v < se.cols; ++v) {
                    int seVal = se.at<int>(u, v);
                    if (seVal == 0) continue;
                    int imgVal = padded.at<uchar>(i + u, j + v) > 0 ? 1 : 0;
                    if ((seVal == 1 && imgVal != 1) || (seVal == -1 && imgVal != 0)) {
                        match = false;
                        break;
                    }
                }
            }
            if (match)
            {
                result.at<uchar>(i, j) = 255;
            }
        }
    }

    return result;
}

Mat morphologyHMTOpenCV(const Mat img, const Mat se)
{
    CV_Assert(img.type() == CV_8UC1);
    CV_Assert(se.type() == CV_32SC1);

    // Tạo mask foreground: nơi có giá trị 1
    Mat fg_mask = (se == 1);

    // Tạo mask background: nơi có giá trị -1
    Mat bg_mask = (se == -1);

    Mat eroded_fg;
    erode(img, eroded_fg, fg_mask);

    Mat src_inv;
    bitwise_not(img, src_inv); // 0 <-> 255

    Mat eroded_bg;
    erode(src_inv, eroded_bg, bg_mask);

    Mat result;
    bitwise_and(eroded_fg, eroded_bg, result);

    return result;
}

int main(void)
{
    Mat image = (Mat_<uchar>(7,15) << 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 0, 255, 255, 255, 0, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,
        0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, 255, 255, 255, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    );

    Mat se = (Mat_<int>(3, 3) <<
        0, 0, -1,
        1, 1, -1,
        0, 0, -1
        );
    Mat result;
    Mat result2 = morphologyHMT(image, se);
    Mat result3 = morphologyHMTOpenCV(image, se);
    morphologyEx(image, result, MORPH_HITMISS, se);
	waitKey(0);
	return 0;
}