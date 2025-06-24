#include "xvtCV/LocalThresholding.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace xvt {
namespace threshold {

enum class NiblackVersion
{
    NIBLACK = 0,
    SAUVOLA,
    WOLFJOLION,
    NICK,
};

#define uget(x,y)    at<unsigned char>(y,x)
#define uset(x,y,v)  at<unsigned char>(y,x)=v;
#define fget(x,y)    at<float>(y,x)
#define fset(x,y,v)  at<float>(y,x)=v;

double calcLocalStats(cv::Mat const& im, cv::Mat& map_m, cv::Mat& map_s, int winx, int winy)
{
    cv::Mat im_sum, im_sum_sq;
    cv::integral(im, im_sum, im_sum_sq, CV_64F);

    double m, s, max_s, sum, sum_sq;
    int wxh = (int)std::round(winx / 2.0f);
    int wyh = (int)std::round(winy / 2.0f);
    int x_firstth = wxh;
    int y_firstth = wyh;
    int y_lastth = im.rows - wyh - 1;
    double winarea = (double)winx * winy;

    max_s = 0;
    for (int j = y_firstth; j <= y_lastth; j++)
    {
        sum = sum_sq = 0;

        // for sum array iterator pointer
        double* sum_top_left = im_sum.ptr<double>(j - wyh);
        double* sum_top_right = sum_top_left + winx;
        double* sum_bottom_left = im_sum.ptr<double>(j - wyh + winy);
        double* sum_bottom_right = sum_bottom_left + winx;

        // for sum_sq array iterator pointer
        double* sum_eq_top_left = im_sum_sq.ptr<double>(j - wyh);
        double* sum_eq_top_right = sum_eq_top_left + winx;
        double* sum_eq_bottom_left = im_sum_sq.ptr<double>(j - wyh + winy);
        double* sum_eq_bottom_right = sum_eq_bottom_left + winx;

        sum = (*sum_bottom_right + *sum_top_left) - (*sum_top_right + *sum_bottom_left);
        sum_sq = (*sum_eq_bottom_right + *sum_eq_top_left) - (*sum_eq_top_right + *sum_eq_bottom_left);

        m = sum / winarea;
        s = sqrt((sum_sq - m * sum) / winarea);
        if (s > max_s) max_s = s;

        float* map_m_data = map_m.ptr<float>(j) + x_firstth;
        float* map_s_data = map_s.ptr<float>(j) + x_firstth;
        *map_m_data++ = static_cast<float>(m);
        *map_s_data++ = static_cast<float>(s);

        // Shift the window, add and remove	new/old values to the histogram
        for (int i = 1; i <= im.cols - winx; i++)
        {
            sum_top_left++, sum_top_right++, sum_bottom_left++, sum_bottom_right++;

            sum_eq_top_left++, sum_eq_top_right++, sum_eq_bottom_left++, sum_eq_bottom_right++;

            sum = (*sum_bottom_right + *sum_top_left) - (*sum_top_right + *sum_bottom_left);
            sum_sq = (*sum_eq_bottom_right + *sum_eq_top_left) - (*sum_eq_top_right + *sum_eq_bottom_left);

            m = sum / winarea;
            s = sqrt((sum_sq - m * sum) / winarea);
            if (s > max_s) max_s = s;

            *map_m_data++ = static_cast<float>(m);
            *map_s_data++ = static_cast<float>(s);
        }
    }

    return max_s;
}



/**********************************************************
 * The binarization routine
 **********************************************************/


void NiblackSauvolaWolfJolion(cv::Mat const& im, cv::Mat& output, NiblackVersion version,
                              int winx, int winy, double k, double dR)
{
    if (im.type() != CV_8UC1) {
        output.release(); 
        return;
    }

    if (!output.empty() && output.type() == CV_8UC1 && im.total() == output.total())
    {
        output = output.reshape(0, im.rows);
    }
    else
    {
        output.create(im.rows, im.cols, CV_8UC1);
    }

    if (output.empty() || winx >= im.cols || winy >= im.rows) return;

    double m, s, max_s;
    double th = 0;
    double min_I, max_I;
    int wxh = winx / 2;
    int wyh = winy / 2;
    int x_firstth = wxh;
    int x_lastth = im.cols - wxh - 1;
    int y_lastth = im.rows - wyh - 1;
    int y_firstth = wyh;
    // int mx, my;

    // Create local statistics and store them in a double matrices
    cv::Mat map_m = cv::Mat::zeros(im.rows, im.cols, CV_32F);
    cv::Mat map_s = cv::Mat::zeros(im.rows, im.cols, CV_32F);
    max_s = calcLocalStats(im, map_m, map_s, winx, winy);

    minMaxLoc(im, &min_I, &max_I);

    cv::Mat thsurf(im.rows, im.cols, CV_32F);

    // Create the threshold surface, including border processing
    // ----------------------------------------------------
    for (int j = y_firstth; j <= y_lastth; j++)
    {

        float* th_surf_data = thsurf.ptr<float>(j) + wxh;
        float* map_m_data = map_m.ptr<float>(j) + wxh;
        float* map_s_data = map_s.ptr<float>(j) + wxh;

        // NORMAL, NON-BORDER AREA IN THE MIDDLE OF THE WINDOW:
        for (int i = 0; i <= im.cols - winx; i++)
        {
            m = *map_m_data++;
            s = *map_s_data++;

            // Calculate the threshold
            switch (version)
            {

            case NiblackVersion::NIBLACK:
                th = m + k * s;
                break;

            case NiblackVersion::SAUVOLA:
                th = m * (1 + k * (s / dR - 1));
                break;

            case NiblackVersion::WOLFJOLION:
                th = m + k * (s / max_s - 1) * (m - min_I);
                break;
            case NiblackVersion::NICK:
                th = m + k * (sqrt(m * m + s * s));
                break;

            default:
                //"Unknown threshold type in ImageThresholder::surfaceNiblackImproved()\n";
                assert(false);
                exit(1);
            }

            // thsurf.fset(i+wxh,j,th);
            *th_surf_data++ = (float)th;


            if (i == 0)
            {
                // LEFT BORDER
                float* th_surf_ptr = thsurf.ptr<float>(j);
                for (int i = 0; i <= x_firstth; ++i)
                    *th_surf_data++ = (float)th;

                // LEFT-UPPER CORNER
                if (j == y_firstth)
                {
                    for (int u = 0; u < y_firstth; ++u)
                    {
                        float* th_surf_ptr = thsurf.ptr<float>(u);
                        for (int i = 0; i <= x_firstth; ++i)
                            *th_surf_data++ = (float)th;
                    }

                }

                // LEFT-LOWER CORNER
                if (j == y_lastth)
                {
                    for (int u = y_lastth + 1; u < im.rows; ++u)
                    {
                        float* th_surf_ptr = thsurf.ptr<float>(u);
                        for (int i = 0; i <= x_firstth; ++i)
                            *th_surf_data++ = (float)th;
                    }
                }
            }

            // UPPER BORDER
            if (j == y_firstth)
                for (int u = 0; u < y_firstth; ++u)
                    thsurf.fset(i + wxh, u, (float)th);

            // LOWER BORDER
            if (j == y_lastth)
                for (int u = y_lastth + 1; u < im.rows; ++u)
                    thsurf.fset(i + wxh, u, (float)th);
        }

        // RIGHT BORDER
        float* th_surf_ptr = thsurf.ptr<float>(j) + x_lastth;
        for (int i = x_lastth; i < im.cols; ++i)
            // thsurf.fset(i,j,th);
            *th_surf_ptr++ = (float)th;

        // RIGHT-UPPER CORNER
        if (j == y_firstth)
        {
            for (int u = 0; u < y_firstth; ++u)
            {
                float* th_surf_ptr = thsurf.ptr<float>(u) + x_lastth;
                for (int i = x_lastth; i < im.cols; ++i)
                    *th_surf_ptr++ = (float)th;
            }
        }

        // RIGHT-LOWER CORNER
        if (j == y_lastth)
        {
            for (int u = y_lastth + 1; u < im.rows; ++u)
            {
                float* th_surf_ptr = thsurf.ptr<float>(u) + x_lastth;
                for (int i = x_lastth; i < im.cols; ++i)
                    *th_surf_ptr++ = (float)th;
            }
        }
    }
    //std::cerr << "surface created" << std::endl;

    for (int y = 0; y < im.rows; ++y)
    {
        const unsigned char* im_data = im.ptr<unsigned char>(y);
        float* th_surf_data = thsurf.ptr<float>(y);
        unsigned char* output_data = output.ptr<unsigned char>(y);
        for (int x = 0; x < im.cols; ++x)
        {
            *output_data = *im_data >= *th_surf_data ? 255 : 0;
            im_data++;
            th_surf_data++;
            output_data++;
        }
    }
}

void ThresholdNiBlack(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K, int r)
{
    NiblackSauvolaWolfJolion(src, dst, NiblackVersion::NIBLACK, windowsSize.width, windowsSize.height, K, r);
}

void ThresholdSauvola(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K, int r)
{
    NiblackSauvolaWolfJolion(src, dst, NiblackVersion::SAUVOLA, windowsSize.width, windowsSize.height, K, r);
}

void ThresholdWolf(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K, int r)
{
    NiblackSauvolaWolfJolion(src, dst, NiblackVersion::WOLFJOLION, windowsSize.width, windowsSize.height, K, r);
}

void ThresholdNick(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K, int r)
{
    NiblackSauvolaWolfJolion(src, dst, NiblackVersion::NICK, windowsSize.width, windowsSize.height, K, r);
}

void TryAllLocalThreshold(cv::Mat const& src, cv::Size size)
{

    cv::Mat binaryImg(src.rows, src.cols, CV_8U);
    ThresholdNiBlack(src, binaryImg, size);
    int basicSize = 300;
    cv::namedWindow("NiBlack", cv::WINDOW_NORMAL);
    cv::moveWindow("NiBlack", 0, 0);
    cv::resizeWindow("NiBlack", basicSize, basicSize);
    cv::imshow("NiBlack", binaryImg);

    ThresholdSauvola(src, binaryImg, size);
    cv::namedWindow("Sauvola", cv::WINDOW_NORMAL);
    cv::moveWindow("Sauvola", basicSize, 0);
    cv::resizeWindow("Sauvola", basicSize, basicSize);
    cv::imshow("Sauvola", binaryImg);

    ThresholdWolf(src, binaryImg, size);
    cv::namedWindow("Wolf", cv::WINDOW_NORMAL);
    cv::moveWindow("Wolf", basicSize * 2, 0);
    cv::resizeWindow("Wolf", basicSize, basicSize);
    cv::imshow("Wolf", binaryImg);

    int blockSize = (size.width <= size.height) ? size.width : size.height;
    blockSize = (blockSize < 3) ? 3 : blockSize;
    cv::adaptiveThreshold(src, binaryImg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, 0);
    cv::namedWindow("ADAPTIVE_THRESH_MEAN", cv::WINDOW_NORMAL);
    cv::moveWindow("ADAPTIVE_THRESH_MEAN", basicSize * 3, 0);
    cv::resizeWindow("ADAPTIVE_THRESH_MEAN", basicSize, basicSize);
    cv::imshow("ADAPTIVE_THRESH_MEAN", binaryImg);

    cv::adaptiveThreshold(src, binaryImg, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, 0);
    cv::namedWindow("ADAPTIVE_THRESH_GAUSSIAN", cv::WINDOW_NORMAL);
    cv::moveWindow("ADAPTIVE_THRESH_GAUSSIAN", basicSize * 4, 0);
    cv::resizeWindow("ADAPTIVE_THRESH_GAUSSIAN", basicSize, basicSize);
    cv::imshow("ADAPTIVE_THRESH_GAUSSIAN", binaryImg);

    ThresholdNick(src, binaryImg, size);
    cv::namedWindow("NICK", cv::WINDOW_NORMAL);
    cv::moveWindow("NICK", basicSize * 5, 0);
    cv::resizeWindow("NICK", basicSize, basicSize);
    cv::imshow("NICK", binaryImg);
}

}//Threshold
}//xvt