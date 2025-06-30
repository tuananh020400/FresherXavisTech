#include "xvtCV/ContrastEnhancement.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace xvt {
namespace enhance {

void AGCIE(const cv::Mat& src, cv::Mat& dst)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1)
    {
        L = src.clone();
    }
    else
    {
        cv::cvtColor(src, HSV, cv::COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    cv::Mat L_norm;
    L.convertTo(L_norm, CV_64F, 1.0 / 255.0);

    cv::Mat mean, stddev;
    cv::meanStdDev(L_norm, mean, stddev);
    double mu = mean.at<double>(0, 0);
    double sigma = stddev.at<double>(0, 0);

    double tau = 3.0;

    double gamma;
    if (4 * sigma <= 1.0 / tau)
    { // low-contrast
        gamma = -std::log2(sigma);
    }
    else
    { // high-contrast
        gamma = std::exp((1.0 - mu - sigma) / 2.0);
    }

    std::vector<double> table_double(256, 0);
    for (int i = 1; i < 256; i++)
    {
        table_double[i] = i / 255.0;
    }

    if (mu >= 0.5)
    { // bright image
        for (int i = 1; i < 256; i++)
        {
            table_double[i] = std::pow(table_double[i], gamma);
        }
    }
    else
    { // dark image
        double mu_gamma = std::pow(mu, gamma);
        for (int i = 1; i < 256; i++)
        {
            double in_gamma = std::pow(table_double[i], gamma);;
            table_double[i] = in_gamma / (in_gamma + (1.0 - in_gamma) * mu_gamma);
        }
    }

    std::vector<uchar> table_uchar(256, 0);
    for (int i = 1; i < 256; i++)
    {
        table_uchar[i] = cv::saturate_cast<uchar>(255.0 * table_double[i]);
    }

    cv::LUT(L, table_uchar, L);

    if (channels == 1)
    {
        dst = L.clone();
    }
    else
    {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR_FULL);
    }

    return;
}

void IAGCWD(const cv::Mat& src, cv::Mat& dst, double alpha_dimmed, double alpha_bright, int T_t, double tau_t, double tau)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1)
    {
        L = src.clone();
    }
    else
    {
        cv::cvtColor(src, HSV, cv::COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    double mean_L = cv::mean(L).val[0];
    double t = (mean_L - T_t) / T_t;

    double alpha;
    bool truncated_cdf;
    if (t < -tau_t)
    {
        //process dimmed image
        alpha = alpha_dimmed;
        truncated_cdf = false;
    }
    else if (t > tau_t)
    {
        //process bright image
        alpha = alpha_bright;
        truncated_cdf = true;
        L = 255 - L;
    }
    else
    {
        //do nothing
        dst = src.clone();
        return;
    }

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    double total_pixels_inv = 1.0 / total_pixels;
    cv::Mat PDF = cv::Mat::zeros(256, 1, CV_64F);
    for (int i = 0; i < 256; i++)
    {
        PDF.at<double>(i) = hist.at<float>(i) * total_pixels_inv;
    }

    double pdf_min, pdf_max;
    cv::minMaxLoc(PDF, &pdf_min, &pdf_max);
    cv::Mat PDF_w = PDF.clone();
    for (int i = 0; i < 256; i++)
    {
        PDF_w.at<double>(i) = pdf_max * std::pow((PDF_w.at<double>(i) - pdf_min) / (pdf_max - pdf_min), alpha);
    }

    cv::Mat CDF_w = PDF_w.clone();
    double culsum = 0;
    for (int i = 0; i < 256; i++)
    {
        culsum += PDF_w.at<double>(i);
        CDF_w.at<double>(i) = culsum;
    }
    CDF_w /= culsum;

    cv::Mat inverse_CDF_w = 1.0 - CDF_w;
    if (truncated_cdf)
    {
        inverse_CDF_w = cv::max(tau, inverse_CDF_w);
    }

    std::vector<uchar> table(256, 0);
    for (int i = 1; i < 256; i++)
    {
        table[i] = cv::saturate_cast<uchar>(255.0 * std::pow(i / 255.0, inverse_CDF_w.at<double>(i)));
    }

    cv::LUT(L, table, L);

    if (t > tau_t)
    {
        L = 255 - L;
    }

    if (channels == 1)
    {
        dst = L.clone();
    }
    else
    {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR_FULL);
    }

    return;
}

void WTHE(const cv::Mat& src, cv::Mat& dst, float r, float v)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat YUV;
    std::vector<cv::Mat> YUV_channels;
    if (channels == 1)
    {
        L = src.clone();
    }
    else
    {
        cv::cvtColor(src, YUV, cv::COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        L = YUV_channels[0];
    }

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    float total_pixels_inv = 1.0f / total_pixels;
    cv::Mat P = hist.clone();
    for (int i = 0; i < 256; i++)
    {
        P.at<float>(i) = P.at<float>(i) * total_pixels_inv;
    }

    cv::Mat Pwt = P.clone();
    double minP, maxP;
    cv::minMaxLoc(P, &minP, &maxP);
    float Pu = (float)(v * maxP);
    float Pl = (float)minP;
    for (int i = 0; i < 256; i++)
    {
        float Pi = P.at<float>(i);
        if (Pi > Pu)
            Pwt.at<float>(i) = Pu;
        else if (Pi < Pl)
            Pwt.at<float>(i) = 0;
        else
            Pwt.at<float>(i) = std::pow((Pi - Pl) / (Pu - Pl), r) * Pu;
    }

    cv::Mat Cwt = Pwt.clone();
    float cdf = 0;
    for (int i = 0; i < 256; i++)
    {
        cdf += Pwt.at<float>(i);
        Cwt.at<float>(i) = cdf;
    }

    float Wout = 255.0f;
    float Madj = 0.0f;
    std::vector<uchar> table(256, 0);
    for (int i = 0; i < 256; i++)
    {
        table[i] = cv::saturate_cast<uchar>(Wout * Cwt.at<float>(i) + Madj);
    }

    cv::LUT(L, table, L);

    if (channels == 1)
    {
        dst = L.clone();
    }
    else
    {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_YUV2BGR);
    }

    return;
}

// This is a OpenCV-based implementation of Conv2 in Matlab.
cv::Mat Conv2(const cv::Mat& img, const cv::Mat& ikernel, ConvolutionType type)
{
    cv::Mat dest;
    cv::Mat kernel;
    cv::flip(ikernel, kernel, -1);
    cv::Mat source = img;
    if (ConvolutionType::CONVOLUTION_FULL == type)
    {
        source = cv::Mat();
        const int additionalRows = kernel.rows - 1, additionalCols = kernel.cols - 1;
        copyMakeBorder(img, source, (additionalRows + 1) / 2, additionalRows / 2, (additionalCols + 1) / 2, additionalCols / 2, cv::BORDER_CONSTANT, cv::Scalar(0));
    }
    cv::Point anchor(kernel.cols - kernel.cols / 2 - 1, kernel.rows - kernel.rows / 2 - 1);
    int borderMode = cv::BORDER_CONSTANT;
    filter2D(source, dest, img.depth(), kernel, anchor, 0, borderMode);

    if (ConvolutionType::CONVOLUTION_VALID == type)
    {
        dest = dest.colRange((kernel.cols - 1) / 2, dest.cols - kernel.cols / 2).rowRange((kernel.rows - 1) / 2, dest.rows - kernel.rows / 2);
    }
    return dest;
}

void LDR(const cv::Mat& src, cv::Mat& dst, double alpha)
{
    int R = src.rows;
    int C = src.cols;

    cv::Mat Y;
    std::vector<cv::Mat> YUV_channels;
    if (src.channels() == 1)
    {
        Y = src.clone();
    }
    else
    {
        cv::Mat YUV;
        cv::cvtColor(src, YUV, cv::COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        Y = YUV_channels[0];
    }

    cv::Mat U = cv::Mat::zeros(255, 255, CV_64F);
    {
        cv::Mat tmp_k(255, 1, CV_64F);
        for (int i = 0; i < 255; i++)
            tmp_k.at<double>(i) = (double)i + 1;

        for (int layer = 1; layer <= 255; layer++)
        {
            cv::Mat mi, ma;
            cv::min(tmp_k, (double)256 - layer, mi);
            cv::max(tmp_k - layer, 0, ma);
            cv::Mat m = mi - ma;
            m.copyTo(U.col(layer - 1));
        }
    }

    // unordered 2D histogram acquisition
    cv::Mat h2d = cv::Mat::zeros(256, 256, CV_64F);
    for (int j = 0; j < R; j++)
    {
        for (int i = 0; i < C; i++)
        {
            uchar ref = Y.at<uchar>(j, i);

            if (j != R - 1)
            {
                uchar trg = Y.at<uchar>(j + 1, i);
                h2d.at<double>((std::max)(ref, trg), (std::min)(ref, trg)) += 1;
            }
            if (i != C - 1)
            {
                uchar trg = Y.at<uchar>(j, i + 1);
                h2d.at<double>((std::max)(ref, trg), (std::min)(ref, trg)) += 1;
            }
        }
    }

    // Intra-Layer Optimization
    cv::Mat D = cv::Mat::zeros(255, 255, CV_64F);
    cv::Mat s = cv::Mat::zeros(255, 1, CV_64F);

    for (int layer = 1; layer <= 255; layer++)
    {
        cv::Mat h_l = cv::Mat::zeros(256 - layer, 1, CV_64F);

        int tmp_idx = 1;
        for (int j = 1 + layer; j <= 256; j++)
        {
            int i = j - layer;
            h_l.at<double>(tmp_idx - 1) = std::log(h2d.at<double>(j - 1, i - 1) + 1); // Equation (2)
            tmp_idx++;
        }

        s.at<double>(layer - 1) = cv::sum(h_l)[0];

        if (s.at<double>(layer - 1) == 0)
            continue;

        cv::Mat kernel = cv::Mat::ones(layer, 1, CV_64F);
        cv::Mat m_l = Conv2(h_l, kernel, ConvolutionType::CONVOLUTION_FULL); // Equation (30)

        double mi;
        cv::minMaxLoc(m_l, &mi, 0);
        cv::Mat d_l = m_l - mi;
        d_l = d_l.mul(1.0 / U.col(layer - 1)); // Equation (33)

        if (cv::sum(d_l)[0] == 0)
            continue;

        D.col(layer - 1) = d_l / cv::sum(d_l)[0];
    }

    // Inter - Layer Aggregation
    double max_s;
    cv::minMaxLoc(s, 0, &max_s);
    cv::Mat W;
    cv::pow(s / max_s, alpha, W); // Equation (23)
    cv::Mat d = D * W; // Equation (24)

    // reconstruct transformation function
    d /= cv::sum(d)[0];
    cv::Mat tmp = cv::Mat::zeros(256, 1, CV_64F);
    for (int k = 1; k <= 255; k++)
    {
        tmp.at<double>(k) = tmp.at<double>(k - 1) + d.at<double>(k - 1);
    }
    tmp.convertTo(tmp, CV_8U, 255.0);

    cv::LUT(Y, tmp, Y);

    if (src.channels() == 1)
    {
        dst = Y.clone();
    }
    else
    {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_YUV2BGR);
    }

    return;
}

void LDR2(const cv::Mat& src, cv::Mat& dst, double alpha)
{
    int R = src.rows;
    int C = src.cols;

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&src, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    double min = 0.0, max = 0.0;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(hist, &min, &max, &minLoc, &maxLoc);
    int minL = minLoc.y <= 1 ? 1 : (int)min;
    int maxL = maxLoc.y;


    cv::Mat Y;
    std::vector<cv::Mat> YUV_channels;
    if (src.channels() == 1)
    {
        Y = src.clone();
    }
    else
    {
        cv::Mat YUV;
        cv::cvtColor(src, YUV, cv::COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        Y = YUV_channels[0];
    }

    cv::Mat U = cv::Mat::zeros(255, 255, CV_32F);
    {
        cv::Mat tmp_k(255, 1, CV_32F);
        for (int i = 0; i < 255; i++)
            tmp_k.at<float>(i) = (float)i + 1;

        for (int layer = 1; layer <= 255; layer++)
        {
            cv::Mat mi, ma;
            cv::min(tmp_k, 256 - (double)layer, mi);
            cv::max(tmp_k - layer, 0, ma);
            cv::Mat m = mi - ma;
            m.copyTo(U.col(layer - 1));
        }
    }

    // unordered 2D histogram acquisition
    cv::Mat h2d = cv::Mat::zeros(256, 256, CV_32F);
    for (int j = 0; j < R; j++)
    {
        for (int i = 0; i < C; i++)
        {
            uchar ref = Y.at<uchar>(j, i);

            if (j != R - 1)
            {
                uchar trg = Y.at<uchar>(j + 1, i);
                h2d.at<float>((std::max)(ref, trg), (std::min)(ref, trg)) += 1;
            }
            if (i != C - 1)
            {
                uchar trg = Y.at<uchar>(j, i + 1);
                h2d.at<float>((std::max)(ref, trg), (std::min)(ref, trg)) += 1;
            }
        }
    }

    // Intra-Layer Optimization
    cv::Mat D = cv::Mat::zeros(255, 255, CV_32F);
    cv::Mat s = cv::Mat::zeros(255, 1, CV_32F);

    for (int layer = minL; layer <= maxL; layer++)
    {
        cv::Mat h_l = cv::Mat::zeros(256 - layer, 1, CV_32F);

        int tmp_idx = 1;
        for (int j = 1 + layer; j <= maxL + 1; j++)
        {
            int i = j - layer;
            h_l.at<float>(tmp_idx - 1) = std::log(h2d.at<float>(j - 1, i - 1) + 1); // Equation (2)
            tmp_idx++;
        }

        s.at<float>(layer - 1) = (float)cv::sum(h_l)[0];

        if (s.at<float>(layer - 1) == 0)
            continue;

        cv::Mat kernel = cv::Mat::ones(layer, 1, CV_64F);
        cv::Mat m_l = Conv2(h_l, kernel, ConvolutionType::CONVOLUTION_FULL); // Equation (30)

        double mi;
        cv::minMaxLoc(m_l, &mi, 0);
        cv::Mat d_l = m_l - mi;
        d_l = d_l.mul(1.0 / U.col(layer - 1)); // Equation (33)

        if (cv::sum(d_l)[0] == 0)
            continue;

        D.col(layer - 1) = d_l / cv::sum(d_l)[0];
    }

    // Inter - Layer Aggregation
    double max_s;
    cv::minMaxLoc(s, 0, &max_s);
    cv::Mat W;
    cv::pow(s / max_s, alpha, W); // Equation (23)
    cv::Mat d = D * W; // Equation (24)

    // reconstruct transformation function
    d /= cv::sum(d)[0];
    cv::Mat tmp = cv::Mat::zeros(256, 1, CV_64F);
    for (int k = 1; k <= 255; k++)
    {
        tmp.at<float>(k) = tmp.at<float>(k - 1) + d.at<float>(k - 1);
    }
    tmp.convertTo(tmp, CV_8U, 255.0);

    cv::LUT(Y, tmp, Y);

    if (src.channels() == 1)
    {
        dst = Y.clone();
    }
    else
    {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_YUV2BGR);
    }

    return;
}

void AGCWD(const cv::Mat& src, cv::Mat& dst, double alpha)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1)
    {
        L = src.clone();
    }
    else
    {
        cv::cvtColor(src, HSV, cv::COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    double total_pixels_inv = 1.0 / total_pixels;
    cv::Mat PDF = cv::Mat::zeros(256, 1, CV_64F);
    for (int i = 0; i < 256; i++)
    {
        PDF.at<double>(i) = hist.at<float>(i) * total_pixels_inv;
    }

    double pdf_min, pdf_max;
    cv::minMaxLoc(PDF, &pdf_min, &pdf_max);
    cv::Mat PDF_w = PDF.clone();
    for (int i = 0; i < 256; i++)
    {
        PDF_w.at<double>(i) = pdf_max * std::pow((PDF_w.at<double>(i) - pdf_min) / (pdf_max - pdf_min), alpha);
    }

    cv::Mat CDF_w = PDF_w.clone();
    double culsum = 0;
    for (int i = 0; i < 256; i++)
    {
        culsum += PDF_w.at<double>(i);
        CDF_w.at<double>(i) = culsum;
    }
    CDF_w /= culsum;

    std::vector<uchar> table(256, 0);
    for (int i = 1; i < 256; i++)
    {
        table[i] = cv::saturate_cast<uchar>(255.0 * std::pow(i / 255.0, 1 - CDF_w.at<double>(i)));
    }

    cv::LUT(L, table, L);

    if (channels == 1)
    {
        dst = L.clone();
    }
    else
    {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR_FULL);
    }

    return;
}

void JHE(const cv::Mat& src, cv::Mat& dst)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat YUV;
    std::vector<cv::Mat> YUV_channels;
    if (channels == 1)
    {
        L = src.clone();
    }
    else
    {
        cv::cvtColor(src, YUV, cv::COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        L = YUV_channels[0];
    }

    // Compute average image.
    cv::Mat avg_L;
    cv::boxFilter(L, avg_L, -1, cv::Size(3, 3), cv::Point(-1, -1), true, cv::BORDER_CONSTANT);

    // Computer joint histogram.
    cv::Mat jointHist = cv::Mat::zeros(256, 256, CV_32S);
    for (int r = 0; r < rows; r++)
    {
        uchar* L_it = L.ptr<uchar>(r);
        uchar* avg_L_it = avg_L.ptr<uchar>(r);
        for (int c = 0; c < cols; c++)
        {
            int i = L_it[c];
            int j = avg_L_it[c];
            jointHist.at<int>(i, j)++;
        }
    }

    // Compute CDF.
    cv::Mat CDF = cv::Mat::zeros(256, 256, CV_32S);
    int min_CDF = total_pixels + 1;
    int cumulative = 0;
    for (int i = 0; i < 256; i++)
    {
        int* jointHist_it = jointHist.ptr<int>(i);
        int* CDF_it = CDF.ptr<int>(i);
        for (int j = 0; j < 256; j++)
        {
            int count = jointHist_it[j];
            cumulative += count;
            if (cumulative > 0 && cumulative < min_CDF)
                min_CDF = cumulative;
            CDF_it[j] = cumulative;
        }
    }

    // Compute equalized joint histogram.
    cv::Mat h_eq = cv::Mat::zeros(256, 256, CV_8U);
    for (int i = 0; i < 256; i++)
    {
        uchar* h_eq_it = h_eq.ptr<uchar>(i);
        int* cdf_it = CDF.ptr<int>(i);
        for (int j = 0; j < 256; j++)
        {
            int cur_cdf = cdf_it[j];
            h_eq_it[j] = cv::saturate_cast<uchar>(255.0 * ((double)cur_cdf - min_CDF) / ((double)total_pixels - 1));
        }
    }

    // Map to get enhanced image.
    for (int r = 0; r < rows; r++)
    {
        uchar* L_it = L.ptr<uchar>(r);
        uchar* avg_L_it = avg_L.ptr<uchar>(r);
        for (int c = 0; c < cols; c++)
        {
            int i = L_it[c];
            int j = avg_L_it[c];
            L_it[c] = h_eq.at<uchar>(i, j);
        }
    }

    if (channels == 1)
    {
        dst = L.clone();
    }
    else
    {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, cv::COLOR_YUV2BGR);
    }

    return;
}

std::vector<cv::Mat> gaussian_pyramid(const cv::Mat& src, int nLevel)
{
    cv::Mat I = src.clone();
    std::vector<cv::Mat> pyr;
    pyr.push_back(I);
    for (int i = 2; i <= nLevel; i++)
    {
        cv::pyrDown(I, I);
        pyr.push_back(I);
    }
    return pyr;
}

std::vector<cv::Mat> laplacian_pyramid(const cv::Mat& src, int nLevel)
{
    cv::Mat I = src.clone();
    std::vector<cv::Mat> pyr;
    cv::Mat J = I.clone();
    for (int i = 1; i < nLevel; i++)
    {
        cv::pyrDown(J, I);
        cv::Mat J_up;
        cv::pyrUp(I, J_up, J.size());
        pyr.push_back(J - J_up);
        J = I;
    }
    pyr.push_back(J); // the coarest level contains the residual low pass image
    return pyr;
}

cv::Mat reconstruct_laplacian_pyramid(const std::vector<cv::Mat>& pyr)
{
    int nLevel = (int)pyr.size();
    cv::Mat R = pyr[(int64)nLevel - 1].clone();
    for (int i = nLevel - 2; i >= 0; i--)
    {
        cv::pyrUp(R, R, pyr[i].size());
        R = pyr[i] + R;
    }
    return R;
}

cv::Mat multiscale_blending(const std::vector<cv::Mat>& seq, const std::vector<cv::Mat>& W)
{
    int h = seq[0].rows;
    int w = seq[0].cols;
    int n = (int)seq.size();

    int nScRef = int(std::log((std::min)(h, w)) / log(2));

    int nScales = 1;
    int hp = h;
    int wp = w;
    while (nScales < nScRef)
    {
        nScales++;
        hp = (hp + 1) / 2;
        wp = (wp + 1) / 2;
    }
    //std::cout << "Number of scales: " << nScales << ", residual's size: " << hp << " x " << wp << std::endl;

    std::vector<cv::Mat> pyr;
    hp = h;
    wp = w;
    for (int scale = 1; scale <= nScales; scale++)
    {
        pyr.push_back(cv::Mat::zeros(hp, wp, CV_64F));
        hp = (hp + 1) / 2;
        wp = (wp + 1) / 2;
    }

    for (int i = 0; i < n; i++)
    {
        std::vector<cv::Mat> pyrW = gaussian_pyramid(W[i], nScales);
        std::vector<cv::Mat> pyrI = laplacian_pyramid(seq[i], nScales);

        for (int scale = 0; scale < nScales; scale++)
        {
            pyr[scale] += pyrW[scale].mul(pyrI[scale]);
        }
    }

    return reconstruct_laplacian_pyramid(pyr);
}

void robust_normalization(const cv::Mat& src, cv::Mat& dst, double wSat = 1.0, double bSat = 1.0)
{
    int H = src.rows;
    int W = src.cols;
    int D = src.channels();
    int N = H * W;

    double vmax;
    double vmin;
    if (D > 1)
    {
        std::vector<cv::Mat> src_channels;
        cv::split(src, src_channels);

        cv::Mat max_channel;
        cv::max(src_channels[0], src_channels[1], max_channel);
        cv::max(max_channel, src_channels[2], max_channel);
        cv::Mat max_channel_sort;
        cv::sort(max_channel.reshape(1, 1), max_channel_sort, cv::SORT_ASCENDING);
        vmax = max_channel_sort.at<double>(int(N - wSat * N / 100 + 1));

        cv::Mat min_channel;
        cv::min(src_channels[0], src_channels[1], min_channel);
        cv::min(min_channel, src_channels[2], min_channel);
        cv::Mat min_channel_sort;
        cv::sort(min_channel.reshape(1, 1), min_channel_sort, cv::SORT_ASCENDING);
        vmin = min_channel_sort.at<double>(int(bSat * N / 100));
    }
    else
    {
        cv::Mat src_sort;
        cv::sort(src.reshape(1, 1), src_sort, cv::SORT_ASCENDING);
        vmax = src_sort.at<double>(int(N - wSat * N / 100 + 1));
        vmin = src_sort.at<double>(int(bSat * N / 100));
    }

    if (vmax <= vmin)
    {
        if (D > 1)
            dst = cv::Mat(H, W, src.type(), cv::Scalar(vmax, vmax, vmax));
        else
            dst = cv::Mat(H, W, src.type(), cv::Scalar(vmax));
    }
    else
    {
        cv::Scalar Ones;
        if (D > 1)
        {
            cv::Mat vmin3 = cv::Mat(H, W, src.type(), cv::Scalar(vmin, vmin, vmin));
            cv::Mat vmax3 = cv::Mat(H, W, src.type(), cv::Scalar(vmax, vmax, vmax));
            dst = (src - vmin3).mul(1.0 / (vmax3 - vmin3));
            Ones = cv::Scalar(1.0, 1.0, 1.0);
        }
        else
        {
            dst = (src - vmin) / (vmax - vmin);
            Ones = cv::Scalar(1.0);
        }

        cv::Mat mask_over = dst > vmax;
        cv::Mat mask_below = dst < vmin;
        mask_over.convertTo(mask_over, CV_64F, 1.0 / 255.0);
        mask_below.convertTo(mask_below, CV_64F, 1.0 / 255.0);

        dst = dst.mul(Ones - mask_over) + mask_over;
        dst = dst.mul(Ones - mask_below);
    }

    return;
}

/***
@inproceedings{hessel2020extended,
  title={An extended exposure fusion and its application to single image contrast enhancement},
  author={Hessel, Charles and Morel, Jean-Michel},
  booktitle={The IEEE Winter Conference on Applications of Computer Vision},
  pages={137--146},
  year={2020}
}

This is a reimplementation from https://github.com/chlsl/simulated-exposure-fusion-ipol/
***/
void SEF(const cv::Mat& src, cv::Mat& dst, double alpha, double beta, double lambda)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1)
    {
        L = src.clone();
    }
    else
    {
        cv::cvtColor(src, HSV, cv::COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    cv::Mat L_norm;
    L.convertTo(L_norm, CV_64F, 1.0 / 255.0);

    cv::Mat src_norm;
    src.convertTo(src_norm, CV_64F, 1.0 / 255.0);

    cv::Mat C;
    if (channels == 1)
    {
        C = src_norm.mul(1.0 / (L_norm + std::pow(2, -16)));
    }
    else
    {
        cv::Mat temp = 1.0 / (L_norm + std::pow(2, -16));
        std::vector<cv::Mat> temp_arr = { temp.clone(),temp.clone(),temp.clone() };
        cv::Mat temp3;
        cv::merge(temp_arr, temp3);
        C = src_norm.mul(temp3);
    }

    // Compute median
    cv::Mat tmp = src.reshape(1, 1);
    cv::Mat sorted;
    cv::sort(tmp, sorted, cv::SORT_ASCENDING);
    double med = double(sorted.at<uchar>(rows * cols * channels / 2)) / 255.0;
    //std::cout << "med = " << med << std::endl;

    //Compute optimal number of images
    int Mp = 1;					// Mp = M - 1; M is the total number of images
    int Ns = int(Mp * med);		// number of images generated with fs
    int N = Mp - Ns;			// number of images generated with f
    int Nx = (std::max)(N, Ns);	// used to compute maximal factor
    double tmax1 = (1.0 + (Ns + 1.0) * (beta - 1.0) / Mp) / (std::pow(alpha, 1.0 / Nx));			// t_max k=+1
    double tmin1s = (-beta + (Ns - 1.0) * (beta - 1.0) / Mp) / (std::pow(alpha, 1.0 / Nx)) + 1.0;	// t_min k=-1
    double tmax0 = 1.0 + Ns * (beta - 1.0) / Mp;														// t_max k=0
    double tmin0 = 1.0 - beta + Ns * (beta - 1.0) / Mp;												// t_min k=0
    while (tmax1 < tmin0 || tmax0 < tmin1s)
    {
        Mp++;
        Ns = int(Mp * med);
        N = Mp - Ns;
        Nx = (std::max)(N, Ns);
        tmax1 = (1.0 + (Ns + 1.0) * (beta - 1.0) / Mp) / (std::pow(alpha, 1.0 / Nx));
        tmin1s = (-beta + (Ns - 1.0) * (beta - 1.0) / Mp) / (std::pow(alpha, 1.0 / Nx)) + 1.0;
        tmax0 = 1.0 + Ns * (beta - 1.0) / Mp;
        tmin0 = 1.0 - beta + Ns * (beta - 1.0) / Mp;

        //std::cerr << "The estimation of the number of image required in the sequence stopped, please check the parameters!" << std::endl;
        assert(Mp <= 49);
    }

    // std::cout << "M = " << Mp + 1 << ", with N = " << N << " and Ns = " << Ns << std::endl;

    // Remapping functions
    auto fun_f = [alpha, Nx](cv::Mat t, int k)
    {	// enhance dark parts
        return std::pow(alpha, k * 1.0 / Nx) * t;
    };
    auto fun_fs = [alpha, Nx](cv::Mat t, int k)
    {	// enhance bright parts
        return std::pow(alpha, -k * 1.0 / Nx) * (t - 1.0) + 1.0;
    };

    // Offset for the dynamic range reduction (function "g")
    auto fun_r = [beta, Ns, Mp](int k)
    {
        return (1.0 - beta / 2.0) - ((double)k + Ns) * (1.0 - beta) / Mp;
    };

    // Reduce dynamic (using offset function "r")
    double a = beta / 2 + lambda;
    double b = beta / 2 - lambda;
    auto fun_g = [fun_r, beta, a, b, lambda](cv::Mat t, int k)
    {
        auto rk = fun_r(k);
        cv::Mat diff = t - rk;
        cv::Mat abs_diff = cv::abs(diff);

        cv::Mat mask = abs_diff <= beta / 2;
        mask.convertTo(mask, CV_64F, 1.0 / 255.0);

        cv::Mat sign = diff.mul(1.0 / abs_diff);

        return mask.mul(t) + (1.0 - mask).mul(sign.mul(a - lambda * lambda / (abs_diff - b)) + rk);
    };

    // final remapping functions: h = g o f
    auto fun_h = [fun_f, fun_g](cv::Mat t, int k)
    {		// create brighter images (k>=0) (enhance dark parts)
        return fun_g(fun_f(t, k), k);
    };
    auto fun_hs = [fun_fs, fun_g](cv::Mat t, int k)
    {	// create darker images (k<0) (enhance bright parts)
        return fun_g(fun_fs(t, k), k);
    };

    // derivative of g with respect to t
    auto fun_dg = [fun_r, beta, b, lambda](cv::Mat t, int k)
    {
        auto rk = fun_r(k);
        cv::Mat diff = t - rk;
        cv::Mat abs_diff = cv::abs(diff);

        cv::Mat mask = abs_diff <= beta / 2;
        mask.convertTo(mask, CV_64F, 1.0 / 255.0);

        cv::Mat p;
        cv::pow(abs_diff - b, 2, p);

        return mask + (1.0 - mask).mul(lambda * lambda / p);
    };

    // derivative of the remapping functions: dh = f' x g' o f
    auto fun_dh = [alpha, Nx, fun_f, fun_dg](cv::Mat t, int k)
    {
        return std::pow(alpha, k * 1.0 / Nx) * fun_dg(fun_f(t, k), k);
    };
    auto fun_dhs = [alpha, Nx, fun_fs, fun_dg](cv::Mat t, int k)
    {
        return std::pow(alpha, -k * 1.0 / Nx) * fun_dg(fun_fs(t, k), k);
    };

    // Simulate a sequence from image L_norm and compute the contrast weights
    std::vector<cv::Mat> seq((int64)N + Ns + 1);
    std::vector<cv::Mat> wc((int64)N + Ns + 1);

    for (int k = -Ns; k <= N; k++)
    {
        cv::Mat seq_temp, wc_temp;
        if (k < 0)
        {
            seq_temp = fun_hs(L_norm, k);	// Apply remapping function
            wc_temp = fun_dhs(L_norm, k);	// Compute contrast measure
        }
        else
        {
            seq_temp = fun_h(L_norm, k);	// Apply remapping function
            wc_temp = fun_dh(L_norm, k);	// Compute contrast measure
        }

        // Detect values outside [0,1]
        cv::Mat mask_sup = seq_temp > 1.0;
        cv::Mat mask_inf = seq_temp < 0.0;
        mask_sup.convertTo(mask_sup, CV_64F, 1.0 / 255.0);
        mask_inf.convertTo(mask_inf, CV_64F, 1.0 / 255.0);
        // Clip them
        seq_temp = seq_temp.mul(1.0 - mask_sup) + mask_sup;
        seq_temp = seq_temp.mul(1.0 - mask_inf);
        // Set to 0 contrast of clipped values
        wc_temp = wc_temp.mul(1.0 - mask_sup);
        wc_temp = wc_temp.mul(1.0 - mask_inf);

        seq[(int64)k + Ns] = seq_temp.clone();
        wc[(int64)k + Ns] = wc_temp.clone();
    }

    // Compute well-exposedness weights and final normalized weights
    std::vector<cv::Mat> we((int64)N + Ns + 1);
    std::vector<cv::Mat> w((int64)N + Ns + 1);
    cv::Mat sum_w = cv::Mat::zeros(rows, cols, CV_64F);

    for (int i = 0; i < we.size(); i++)
    {
        cv::Mat p, we_temp, w_temp;
        cv::pow(seq[i] - 0.5, 2, p);
        cv::exp(-0.5 * p / (0.2 * 0.2), we_temp);

        w_temp = wc[i].mul(we_temp);

        we[i] = we_temp.clone();
        w[i] = w_temp.clone();

        sum_w = sum_w + w[i];
    }

    sum_w = 1.0 / sum_w;
    for (int i = 0; i < we.size(); i++)
    {
        w[i] = w[i].mul(sum_w);
    }

    // Multiscale blending
    cv::Mat lp = multiscale_blending(seq, w);

    if (channels == 1)
    {
        lp = lp.mul(C);
    }
    else
    {
        std::vector<cv::Mat> lp3 = { lp.clone(),lp.clone(),lp.clone() };
        cv::merge(lp3, lp);
        lp = lp.mul(C);
    }

    robust_normalization(lp, lp);

    lp.convertTo(dst, CV_8U, 255);

    return;
}

void PerformSUACE(cv::Mat& src, cv::Mat& dst, int distance, double sigma)
{

    CV_Assert(src.type() == CV_8UC1);
    // "distance and sigma must be greater 0"
    assert(distance > 0 && sigma > 0);

    dst = cv::Mat(src.size(), CV_8UC1);
    cv::Mat smoothed;
    int val;
    int a, b;
    int adjuster;
    int half_distance = distance / 2;
    double distance_d = distance;

    GaussianBlur(src, smoothed, cv::Size(0, 0), sigma);

    for (int x = 0; x < src.cols; x++)
        for (int y = 0; y < src.rows; y++)
        {
            val = src.at<uchar>(y, x);
            adjuster = smoothed.at<uchar>(y, x);
            if (((int64)val - adjuster) > distance_d)adjuster += (int)std::round(((double)val - adjuster) * 0.5);
            adjuster = adjuster < half_distance ? half_distance : adjuster;
            b = adjuster + half_distance;
            b = b > 255 ? 255 : b;
            a = b - distance;
            a = a < 0 ? 0 : a;

            if (val >= a && val <= b)
            {
                dst.at<uchar>(y, x) = (char)((((double)val - a) / distance_d) * 255.0);
            }
            else if (val < a)
            {
                dst.at<uchar>(y, x) = 0;
            }
            else if (val > b)
            {
                dst.at<uchar>(y, x) = 255;
            }
        }
}

void TryImageEnhancement(cv::Mat src)
{
    cv::Mat dst;

    AGCIE(src, dst);
    namedWindow("AGCIE", cv::WINDOW_NORMAL);
    cv::imshow("AGCIE", dst);

    IAGCWD(src, dst);
    namedWindow("IAGCWD", cv::WINDOW_NORMAL);
    cv::imshow("IAGCWD", dst);

    WTHE(src, dst);
    namedWindow("WTHE", cv::WINDOW_NORMAL);
    cv::imshow("WTHE", dst);

    LDR(src, dst, 5);
    namedWindow("LDR", cv::WINDOW_NORMAL);
    cv::imshow("LDR", dst);

    AGCWD(src, dst);
    namedWindow("AGCWD", cv::WINDOW_NORMAL);
    cv::imshow("AGCWD", dst);

    JHE(src, dst);
    namedWindow("JHE", cv::WINDOW_NORMAL);
    cv::imshow("JHE", dst);

    /*SEF(src, dst,0.25,0.125);
    namedWindow("SEF", cv::WINDOW_NORMAL);
    cv::imshow("SEF", dst);*/

    /*PerformSUACE(src, dst, 5, 3);
    namedWindow("SUACE", cv::WINDOW_NORMAL);
    cv::imshow("SUACE", dst);*/

    PerformSUACE(src, dst, 5, 3);
    namedWindow("SUACE", cv::WINDOW_NORMAL);
    cv::imshow("SUACE", dst);

}

}
}