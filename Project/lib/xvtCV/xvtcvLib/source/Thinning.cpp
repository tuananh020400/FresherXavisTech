#include "xvtCV/Thinning.h"
#include "opencv2/imgproc.hpp"

namespace xvt {

// Applies a thinning iteration to a binary image
static void ThinningIteration(cv::Mat& img, int iter, ThinningTypes thinningType){
    if (!img.empty())
    {
        cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

        if(thinningType == ThinningTypes::THINNING_ZHANGSUEN){
            for (int i = 1; i < img.rows-1; i++)
            {
                for (int j = 1; j < img.cols-1; j++)
                {
                    uchar p2 = img.at<uchar>(i-1, j);
                    uchar p3 = img.at<uchar>(i-1, j+1);
                    uchar p4 = img.at<uchar>(i, j+1);
                    uchar p5 = img.at<uchar>(i+1, j+1);
                    uchar p6 = img.at<uchar>(i+1, j);
                    uchar p7 = img.at<uchar>(i+1, j-1);
                    uchar p8 = img.at<uchar>(i, j-1);
                    uchar p9 = img.at<uchar>(i-1, j-1);

                    int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                             (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                             (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                             (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                    int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
                    int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

                    if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                        marker.at<uchar>(i,j) = 1;
                }
            }
        }
        if(thinningType == ThinningTypes::THINNING_GUOHALL){
            for (int i = 1; i < img.rows-1; i++)
            {
                for (int j = 1; j < img.cols-1; j++)
                {
                    uchar p2 = img.at<uchar>(i-1, j);
                    uchar p3 = img.at<uchar>(i-1, j+1);
                    uchar p4 = img.at<uchar>(i, j+1);
                    uchar p5 = img.at<uchar>(i+1, j+1);
                    uchar p6 = img.at<uchar>(i+1, j);
                    uchar p7 = img.at<uchar>(i+1, j-1);
                    uchar p8 = img.at<uchar>(i, j-1);
                    uchar p9 = img.at<uchar>(i-1, j-1);

                    int C  = ((!p2) & (p3 | p4)) + ((!p4) & (p5 | p6)) +
                             ((!p6) & (p7 | p8)) + ((!p8) & (p9 | p2));
                    int N1 = (p9 | p2) + (p3 | p4) + (p5 | p6) + (p7 | p8);
                    int N2 = (p2 | p3) + (p4 | p5) + (p6 | p7) + (p8 | p9);
                    int N  = N1 < N2 ? N1 : N2;
                    int m  = iter == 0 ? ((p6 | p7 | (!p9)) & p8) : ((p2 | p3 | (!p5)) & p4);

                    if ((C == 1) && ((N >= 2) && ((N <= 3)) & (m == 0)))
                        marker.at<uchar>(i,j) = 1;
                }
            }
        }

        img &= ~marker;
    }
}

// Apply the thinning procedure to a given image
void Thinning(const cv::Mat& src, cv::Mat& dst, ThinningTypes thinningType)
{
    if (!src.empty())
    {
        cv::Mat processed = cv::Mat::zeros(src.rows + 2, src.cols + 2, CV_8UC1);
        cv::Rect roi(1, 1, src.cols, src.rows);
        src.copyTo(processed(roi));
        CV_CheckTypeEQ(processed.type(), CV_8UC1, "");
        // Enforce the range of the input image to be in between 0 - 255
        processed /= 255;

        cv::Mat prev = cv::Mat::zeros(processed.size(), CV_8UC1);
        cv::Mat diff;
        int count = 0;
        do {
            ThinningIteration(processed, 0, thinningType);
            ThinningIteration(processed, 1, thinningType);
            absdiff(processed, prev, diff);
            processed.copyTo(prev);
            count++;
        }
        while (countNonZero(diff) > 0);

        processed *= 255;

        dst = processed(roi).clone();
    }
}

} //namespace xvt