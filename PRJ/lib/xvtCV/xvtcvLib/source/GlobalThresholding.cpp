#include "xvtCV/GlobalThresholding.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//#define __PERFORMANCE_ANALYSIS__
#include "xvtCV/ScopeTimer.h"

namespace xvt {
namespace threshold {

constexpr int NGRAY = 256;

std::vector<int> histogramCalculate(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper) {
    std::vector<int> data(256, 0);
    if (ignoreValueLower >= 0 && ignoreValueUpper >= 0 && ignoreValueLower <= 256 && ignoreValueUpper <= 256
            && ignoreValueLower<ignoreValueUpper) {
        //Calculate histogram
        int rows = src.rows;
        int cols = src.cols;
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                uchar value = src.at<uchar>(y, x);
                data.at(value)++;
            }
        }
        //Remove histogram data outside range
            std::fill(data.begin(), data.begin() + ignoreValueLower, 0);
            std::fill(data.begin() + ignoreValueUpper, data.end(), 0);
    }
    return data;
}
int Moments(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    //  W. Tsai, "Moment-preserving thresholding: a new approach," Computer Vision,
    // Graphics, and Image Processing, vol. 29, pp. 377-393, 1985.
    // Ported to ImageJ plugin by G.Landini from the the open source project FOURIER 0.8
    // by  M. Emre Celebi , Department of Computer Science,  Louisiana State University in Shreveport
    // Shreveport, LA 71115, USA
    //  http://sourceforge.net/projects/fourier-ipal
    //  http://www.lsus.edu/faculty/~ecelebi/fourier.htm
    double total = 0;
    double m0 = 1.0, m1 = 0.0, m2 = 0.0, m3 = 0.0, sum = 0.0, p0 = 0.0;
    double cd, c0, c1, z0, z1;	/* auxiliary variables */
    int threshold = -1;
    std::vector<int> data = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);
    double histo[256] = { 0 };

    for (int i :data)
        total += i;

    for (int i = 0; i < 256; i++)
        histo[i] = (double)(data[i]) / total; //normalised histogram

    /* Calculate the first, second, and third order moments */
    double tmp = 0.0;
    for (int i = 0; i < 256; i++)
    {
        tmp = i * histo[i];
        m1 += tmp;
        tmp *= i;
        m2 += tmp;
        tmp *= i;
        m3 += tmp;
    }
    /*
    First 4 moments of the gray-level image should match the first 4 moments
    of the target binary image. This leads to 4 equalities whose solutions
    are given in the Appendix of Ref. 1
    */
    cd = m0 * m2 - m1 * m1;
    c0 = (-m2 * m2 + m1 * m3) / cd;
    c1 = (m0 * -m3 + m2 * m1) / cd;
    z0 = 0.5 * (-c1 - std::sqrt(c1 * c1 - 4.0 * c0));
    z1 = 0.5 * (-c1 + std::sqrt(c1 * c1 - 4.0 * c0));
    p0 = (z1 - m1) / (z1 - z0);  /* Fraction of the object pixels in the target binary image */

    // The threshold is the gray-level closest  
    // to the p0-tile of the normalized histogram 
    sum = 0;
    for (int i = 0; i < 256; i++)
    {
        sum += histo[i];
        if (sum > p0)
        {
            threshold = i;
            break;
        }
    }
    return threshold;
}

void buildLookupTables(float P[][NGRAY], float S[][NGRAY], float H[][NGRAY], float h[])
{

    // initialize
    for (int j = 0; j < NGRAY; j++)
        for (int i = 0; i < NGRAY; ++i)
        {
            P[i][j] = 0.f;
            S[i][j] = 0.f;
            H[i][j] = 0.f;
        }
    // diagonal 
    for (int i = 1; i < NGRAY; ++i)
    {
        P[i][i] = h[i];
        S[i][i] = ((float)i) * h[i];
    }
    // calculate first row (row 0 is all zero)
    for (int i = 1; i < NGRAY - 1; ++i)
    {
        P[1][i + 1] = P[1][i] + h[i + 1];
        S[1][i + 1] = S[1][i] + ((float)(i + 1)) * h[i + 1];
    }
    // using row 1 to calculate others
    for (int i = 2; i < NGRAY; i++)
        for (int j = i + 1; j < NGRAY; j++)
        {
            P[i][j] = P[1][j] - P[1][i - 1];
            S[i][j] = S[1][j] - S[1][i - 1];
        }
    // now calculate H[i][j]
    for (int i = 1; i < NGRAY; ++i)
        for (int j = i + 1; j < NGRAY; j++)
        {
            if (P[i][j] != 0)
                H[i][j] = (S[i][j] * S[i][j]) / P[i][j];
            else
                H[i][j] = 0.f;
        }

}

float findMaxSigma(int mlevel, float H[][NGRAY], int t[])
{
    t[0] = 0;
    float maxSig = 0.f;
    switch (mlevel)
    {
    case 2:
        for (int i = 1; i < NGRAY - mlevel; i++) // t1
        {
            float Sq = H[1][i] + H[i + 1][255];
            if (maxSig < Sq)
            {
                t[1] = i;
                maxSig = Sq;
            }
        }
        break;
    case 3:
        for (int i = 1; i < NGRAY - mlevel; i++) // t1
            for (int j = i + 1; j < NGRAY - mlevel + 1; j++) // t2
            {
                float Sq = H[1][i] + H[i + 1][j] + H[j + 1][255];
                if (maxSig < Sq)
                {
                    t[1] = i;
                    t[2] = j;
                    maxSig = Sq;
                }
            }
        break;
    case 4:
        for (int i = 1; i < NGRAY - mlevel; i++) // t1
            for (int j = i + 1; j < NGRAY - mlevel + 1; j++) // t2
                for (int k = j + 1; k < NGRAY - mlevel + 2; k++) // t3
                {
                    float Sq = H[1][i] + H[i + 1][j] + H[j + 1][k] + H[k + 1][255];
                    if (maxSig < Sq)
                    {
                        t[1] = i;
                        t[2] = j;
                        t[3] = k;
                        maxSig = Sq;
                    }
                }
        break;
    case 5:
        for (int i = 1; i < NGRAY - mlevel; i++) // t1
            for (int j = i + 1; j < NGRAY - mlevel + 1; j++) // t2
                for (int k = j + 1; k < NGRAY - mlevel + 2; k++) // t3
                    for (int m = k + 1; m < NGRAY - mlevel + 3; m++) // t4
                    {
                        float Sq = H[1][i] + H[i + 1][j] + H[j + 1][k] + H[k + 1][m] + H[m + 1][255];
                        if (maxSig < Sq)
                        {
                            t[1] = i;
                            t[2] = j;
                            t[3] = k;
                            t[4] = m;
                            maxSig = Sq;
                        }
                    }
        break;
    }
    return maxSig;
}

bool bimodalTest(double y[])
{
    int len = 256;
    bool b = false;
    int modes = 0;

    for (int k = 1; k < len - 1; k++)
    {
        if (y[k - 1] < y[k] && y[k + 1] < y[k])
        {
            modes++;
            if (modes > 2)
                return false;
        }
    }
    if (modes == 2)
        b = true;
    return b;
}

double A(std::vector<int> y, int j)
{
    double x = 0;
    for (int i = 0; i <= j; i++)
        x += y[i];
    return x;
}

double B(std::vector<int> y, int j)
{
    double x = 0;
    for (int i = 0; i <= j; i++)
        x += i * (double)y[i];
    return x;
}

double C(std::vector<int> y, int j)
{
    double x = 0;
    for (int i = 0; i <= j; i++)
        x += i * (double)i * y[i];
    return x;
}

int MinErrorI(cv::Mat const& src,int ignoreValueLower, int ignoreValueUpper)
{
    // Kittler and J. Illingworth, "Minimum error thresholding," Pattern Recognition, vol. 19, pp. 41-47, 1986.
   // C. A. Glasbey, "An analysis of histogram-based thresholding algorithms," CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.
  // Ported to ImageJ plugin by G.Landini from Antti Niemisto's Matlab code (GPL)
  // Original Matlab code Copyright (C) 2004 Antti Niemisto
  // See http://www.cs.tut.fi/~ant/histthresh/ for an excellent slide presentation
  // and the original Matlab code.

    std::vector<int> data = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);

    int threshold = ThresholdMean(src); //Initial estimate for the threshold is found with the MEAN algorithm.
    int Tprev = -2;
    double mu, nu, p, q, sigma2, tau2, w0, w1, w2, sqterm, temp;
    //int counter=1;
    while (threshold != Tprev)
    {
        //Calculate some statistics.
        mu = B(data, threshold) / A(data, threshold);
        nu = (B(data, 255) - B(data, threshold)) / (A(data, 255) - A(data, threshold));
        p = A(data, threshold) / A(data, 255);
        q = (A(data, 255) - A(data, threshold)) / A(data, 255);
        sigma2 = C(data, threshold) / A(data, threshold) - (mu * mu);
        tau2 = (C(data, 255) - C(data, threshold)) / (A(data, 255) - A(data, threshold)) - (nu * nu);

        //The terms of the quadratic equation to be solved.
        w0 = 1.0 / sigma2 - 1.0 / tau2;
        w1 = mu / sigma2 - nu / tau2;
        w2 = (mu * mu) / sigma2 - (nu * nu) / tau2 + log10((sigma2 * (q * q)) / (tau2 * (p * p)));

        //If the next threshold would be imaginary, return with the current one.
        sqterm = (w1 * w1) - w0 * w2;
        if (sqterm < 0)
        {
            return threshold;
        }

        //The updated threshold is the integer part of the solution of the quadratic equation.
        Tprev = threshold;
        temp = (w1 + std::sqrt(sqterm)) / w0;

        if (isnan(temp))
        {
            threshold = Tprev;
        }
        else
            threshold = (int)std::floor(temp);
        //IJ.log("Iter: "+ counter+++"  t:"+threshold);
    }
    return threshold;
}

int ThresholdHuang(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // Implements Huang's fuzzy thresholding method 
    // Uses Shannon's entropy function (one can also use Yager's entropy function) 
    // Huang L.-K. and Wang M.-J.J. (1995) "Image Thresholding by Minimizing  
    // the Measures of Fuzziness" Pattern Recognition, 28(1): 41-51
    // M. Emre Celebi  06.15.2007
    // Ported to ImageJ plugin by G. Landini from E Celebi's fourier_0.8 routines
    int threshold = -1;
    int ih, it;
    int first_bin;
    int last_bin;
    double sum_pix;
    double num_pix;
    double term;
    double ent;  // entropy 
    double min_ent; // min entropy 
    double mu_x;

    std::vector<int> histogram =histogramCalculate(src, ignoreValueLower, ignoreValueUpper);

    /* Determine the first non-zero bin */
    first_bin = 0;
    for (ih = 0; ih < 256; ih++)
    {
        if (histogram[ih] != 0)
        {
            first_bin = ih;
            break;
        }
    }

    /* Determine the last non-zero bin */
    last_bin = 255;
    for (ih = 255; ih >= first_bin; ih--)
    {
        if (histogram[ih] != 0)
        {
            last_bin = ih;
            break;
        }
    }
    term = 1.0 / ((double)last_bin - first_bin);
    double mu_0[256];
    sum_pix = num_pix = 0;
    for (ih = first_bin; ih < 256; ih++)
    {
        sum_pix += (double)ih * histogram[ih];
        num_pix += histogram[ih];
        /* NUM_PIX cannot be zero ! */
        mu_0[ih] = sum_pix / num_pix;
    }

    double mu_1[256];
    sum_pix = num_pix = 0;
    for (ih = last_bin; ih > 0; ih--)
    {
        sum_pix += (double)ih * histogram[ih];
        num_pix += histogram[ih];
        /* NUM_PIX cannot be zero ! */
        mu_1[ih - 1] = sum_pix / (double)num_pix;
    }

    /* Determine the threshold that minimizes the fuzzy entropy */
    threshold = -1;
    min_ent = DBL_MAX;
    for (it = 0; it < 256; it++)
    {
        ent = 0.0;
        for (ih = 0; ih <= it; ih++)
        {
            /* Equation (4) in Ref. 1 */
            mu_x = 1.0 / (1.0 + term * abs(ih - mu_0[it]));
            if (!((mu_x < 1e-06) || (mu_x > 0.999999)))
            {
                /* Equation (6) & (8) in Ref. 1 */
                ent += histogram[ih] * (-mu_x * log(mu_x) - (1.0 - mu_x) * log(1.0 - mu_x));
            }
        }

        for (ih = it + 1; ih < 256; ih++)
        {
            /* Equation (4) in Ref. 1 */
            mu_x = 1.0 / (1.0 + term * abs(ih - mu_1[it]));
            if (!((mu_x < 1e-06) || (mu_x > 0.999999)))
            {
                /* Equation (6) & (8) in Ref. 1 */
                ent += histogram[ih] * (-mu_x * log(mu_x) - (1.0 - mu_x) * log(1.0 - mu_x));
            }
        }
        /* No need to divide by NUM_ROWS * NUM_COLS * LOG(2) ! */
        if (ent < min_ent)
        {
            min_ent = ent;
            threshold = it;
        }
    }
    return threshold;
}

int ThresholdIsoData(cv::Mat const& src,int ignoreValueLower,int ignoreValueUpper)
{
    // Also called intermeans
    // Iterative procedure based on the isodata algorithm [T.W. Ridler, S. Calvard, Picture 
    // thresholding using an iterative selection method, IEEE Trans. System, Man and 
    // Cybernetics, SMC-8 (1978) 630-632.] 
    // The procedure divides the image into objects and background by taking an initial threshold,
    // then the averages of the pixels at or below the threshold and pixels above are computed. 
    // The averages of those two values are computed, the threshold is incremented and the 
    // process is repeated until the threshold is larger than the composite average. That is,
    //  threshold = (average background + average objects)/2
    // The code in ImageJ that implements this function is the getAutoThreshold() method in the ImageProcessor class. 
    //
    // From: Tim Morris (dtm@ap.co.umist.ac.uk)
    // Subject: Re: Thresholding method?
    // posted to sci.image.processing on 1996/06/24
    // The algorithm implemented in NIH Image sets the threshold as that grey
    // value, G, for which the average of the averages of the grey values
    // below and above G is equal to G. It does this by initialising G to the
    // lowest sensible value and iterating:

    // L = the average grey value of pixels with intensities < G
    // H = the average grey value of pixels with intensities > G
    // is G = (L + H)/2?
    // yes => exit
    // no => increment G and repeat
    //
    // However the code commented below have the occasional discrepancy with IJ code.
    // so I am reproducing the original IJ implementation for compatibility.
    int level;
    double result, sum1, sum2, sum3, sum4;
    std::vector<int> data = histogramCalculate(src,ignoreValueLower,ignoreValueUpper);
    int maxValue = data.size() - 1;
    int dataLength = 256;
    int imin = 0;
    while ((data[imin] == 0) && (imin < maxValue))
        imin++;
    int imax = maxValue;
    while ((data[imax] == 0) && (imax > 0))
        imax--;
    if (imin >= imax)
    {
        level = dataLength / 2;
        return level;
    }

    int movingIndex = imin;
    int inc = (std::max)(imax / 40, 1);
    do
    {
        sum1 = sum2 = sum3 = sum4 = 0.0;
        for (int i = imin; i <= movingIndex; i++)
        {
            sum1 += (double)i * data[i];
            sum2 += data[i];
        }
        for (int i = (movingIndex + 1); i <= imax; i++)
        {
            sum3 += (double)i * data[i];
            sum4 += data[i];
        }
        result = (sum1 / sum2 + sum3 / sum4) / 2.0;
        movingIndex++;
    } while (((int64)movingIndex + 1) <= result && movingIndex < imax - 1);

    //.showProgress(1.0);
    level = (int)round(result);
    return level;
}

int ThresholdIntermodes(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
    // Annals of the New York Academy of Sciences, vol. 128, pp. 1035-1053, 1966.
    // ported to ImageJ plugin by G.Landini from Antti Niemisto's Matlab code (GPL)
    // Original Matlab code Copyright (C) 2004 Antti Niemisto
    // See http://www.cs.tut.fi/~ant/histthresh/ for an excellent slide presentation
    // and the original Matlab code.
    //
    // Assumes a bimodal histogram. The histogram needs is smoothed (using a
    // running average of size 3, iteratively) until there are only two local maxima.
    // j and k
    // Threshold t is (j+k)/2.
    // Images with histograms having extremely unequal peaks or a broad and
    // ﬂat valley are unsuitable for this method.
    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);
    int iter = 0;
    int threshold = -1;
    double iHisto[256];
    for (int i = 0; i < 256; i++)
        iHisto[i] = (double)histogram[i];
    double tHisto[256];

    while (!bimodalTest(iHisto))
    {
        //smooth with a 3 point running mean filter
        for (int i = 1; i < 255; i++)
            tHisto[i] = (iHisto[i - 1] + iHisto[i] + iHisto[i + 1]) / 3;
        tHisto[0] = (iHisto[0] + iHisto[1]) / 3; //0 outside
        tHisto[255] = (iHisto[254] + iHisto[255]) / 3; //0 outside
        std::copy(tHisto, tHisto + 256, iHisto);
        //iHisto = tHisto;
        iter++;
        if (iter > 10000)
        {
            threshold = -1;
            return threshold;
        }
    }

    // The threshold is the mean between the two peaks.
    int tt = 0;
    for (int i = 1; i < 255; i++)
    {
        if (iHisto[i - 1] < iHisto[i] && iHisto[i + 1] < iHisto[i])
        {
            tt += i;
            //IJ.log("mode:" +i);
        }
    }
    threshold = (int)floor(tt / 2.0);
    return threshold;
}

int ThresholdLi(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    int threshold=0;
    if (!src.empty()) {
        // Implements Li's Minimum Cross Entropy thresholding method
        // This implementation is based on the iterative version (Ref. 2) of the algorithm.
        // 1) Li C.H. and Lee C.K. (1993) "Minimum Cross Entropy Thresholding" 
        //    Pattern Recognition, 26(4): 617-625
        // 2) Li C.H. and Tam P.K.S. (1998) "An Iterative Algorithm for Minimum 
        //    Cross Entropy Thresholding"Pattern Recognition Letters, 18(8): 771-776
        // 3) Sezgin M. and Sankur B. (2004) "Survey over Image Thresholding 
        //    Techniques and Quantitative Performance Evaluation" Journal of 
        //    Electronic Imaging, 13(1): 146-165 
        //    http://citeseer.ist.psu.edu/sezgin04survey.html
        // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
        int ih;
        int num_pixels;
        int sum_back; /* sum of the background pixels at a given threshold */
        int sum_obj;  /* sum of the object pixels at a given threshold */
        int num_back; /* number of background pixels at a given threshold */
        int num_obj;  /* number of object pixels at a given threshold */
        double old_thresh;
        double new_thresh;
        double mean_back; /* mean of the background pixels at a given threshold */
        double mean_obj;  /* mean of the object pixels at a given threshold */
        double mean;  /* mean gray-level in the image */
        double tolerance; /* threshold tolerance */
        double temp;
        std::vector<int> data = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);
        tolerance = 0.5;
        num_pixels = 0;
        for (ih = 0; ih < 256; ih++)
            num_pixels += data[ih];

        /* Calculate the mean gray-level */
        mean = 0.0;
        for (ih = 0 + 1; ih < 256; ih++) //0 + 1?
            mean += (double)ih * data[ih];
        mean /= num_pixels;
        /* Initial estimate */
        new_thresh = mean;

        do
        {
            old_thresh = new_thresh;
            threshold = (int)(old_thresh + 0.5);	/* range */
            /* Calculate the means of background and object pixels */
            /* Background */
            sum_back = 0;
            num_back = 0;
            for (ih = 0; ih <= threshold; ih++)
            {
                sum_back += ih * data[ih];
                num_back += data[ih];
            }
            mean_back = (num_back == 0 ? 0.0 : (sum_back / (double)num_back));
            /* Object */
            sum_obj = 0;
            num_obj = 0;
            for (ih = threshold + 1; ih < 256; ih++)
            {
                sum_obj += ih * data[ih];
                num_obj += data[ih];
            }
            mean_obj = (num_obj == 0 ? 0.0 : (sum_obj / (double)num_obj));

            /* Calculate the new threshold: Equation (7) in Ref. 2 */
            //new_thresh = simple_round ( ( mean_back - mean_obj ) / ( Math.log ( mean_back ) - Math.log ( mean_obj ) ) );
            //simple_round ( double x ) {
            // return ( int ) ( IS_NEG ( x ) ? x - .5 : x + .5 );
            //}
            //
            //#define IS_NEG( x ) ( ( x ) < -DBL_EPSILON ) 
            //DBL_EPSILON = 2.220446049250313E-16
            temp = (mean_back - mean_obj) / (log(mean_back) - log(mean_obj));

            if (temp < -2.220446049250313E-16)
                new_thresh = (int)(temp - 0.5);
            else
                new_thresh = (int)(temp + 0.5);
            /*  Stop the iterations when the difference between the
            new and old threshold values is less than the tolerance */
        } while (abs(new_thresh - old_thresh) > tolerance);
    }
    return threshold;
}

int ThresholdMaxEntropy(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // Implements Kapur-Sahoo-Wong (Maximum Entropy) thresholding method
    // Kapur J.N., Sahoo P.K., and Wong A.K.C. (1985) "A New Method for
    // Gray-Level Picture Thresholding Using the Entropy of the Histogram"
    // Graphical Models and Image Processing, 29(3): 273-285
    // M. Emre Celebi
    // 06.15.2007
    // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
    int threshold = -1;
    int ih, it;
    int first_bin;
    int last_bin;
    double tot_ent;  /* total entropy */
    double max_ent;  /* max entropy */
    double ent_back; /* entropy of the background pixels at a given threshold */
    double ent_obj;  /* entropy of the object pixels at a given threshold */
    double norm_histo[256]; /* normalized histogram */
    double P1[256]; /* cumulative normalized histogram */
    double P2[256];

    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);


    int total = 0;
    for (ih = 0; ih < 256; ih++)
        total += histogram[ih];

    for (ih = 0; ih < 256; ih++)
        norm_histo[ih] = (double)histogram[ih] / total;

    P1[0] = norm_histo[0];
    P2[0] = 1.0 - P1[0];
    for (ih = 1; ih < 256; ih++)
    {
        P1[ih] = P1[ih - 1] + norm_histo[ih];
        P2[ih] = 1.0 - P1[ih];
    }

    /* Determine the first non-zero bin */
    first_bin = 0;
    for (ih = 0; ih < 256; ih++)
    {
        if (!(abs(P1[ih]) < 2.220446049250313E-16))
        {
            first_bin = ih;
            break;
        }
    }

    /* Determine the last non-zero bin */
    last_bin = 255;
    for (ih = 255; ih >= first_bin; ih--)
    {
        if (!(abs(P2[ih]) < 2.220446049250313E-16))
        {
            last_bin = ih;
            break;
        }
    }

    // Calculate the total entropy each gray-level
    // and find the threshold that maximizes it 
    max_ent = -DBL_MAX;

    for (it = first_bin; it <= last_bin; it++)
    {
        /* Entropy of the background pixels */
        ent_back = 0.0;
        for (ih = 0; ih <= it; ih++)
        {
            if (histogram[ih] != 0)
            {
                ent_back -= (norm_histo[ih] / P1[it]) * log(norm_histo[ih] / P1[it]);
            }
        }

        /* Entropy of the object pixels */
        ent_obj = 0.0;
        for (ih = it + 1; ih < 256; ih++)
        {
            if (histogram[ih] != 0)
            {
                ent_obj -= (norm_histo[ih] / P2[it]) * log(norm_histo[ih] / P2[it]);
            }
        }

        /* Total entropy */
        tot_ent = ent_back + ent_obj;

        // IJ.log(""+max_ent+"  "+tot_ent);
        if (max_ent < tot_ent)
        {
            max_ent = tot_ent;
            threshold = it;
        }
    }
    return threshold;
}

int ThresholdMean(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // C. A. Glasbey, "An analysis of histogram-based thresholding algorithms,"
    // CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.
    //
    // The threshold is the mean of the greyscale data

    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);

    int threshold = -1;
    double tot = 0, sum = 0;
    for (int i = 0; i < 256; i++)
    {
        tot += histogram[i];
        sum += ((double)i * histogram[i]);
    }
    threshold = (int)floor(sum / tot);
    return threshold;
}

int ThresholdMinimum(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
    // Annals of the New York Academy of Sciences, vol. 128, pp. 1035-1053, 1966.
    // ported to ImageJ plugin by G.Landini from Antti Niemisto's Matlab code (GPL)
    // Original Matlab code Copyright (C) 2004 Antti Niemisto
    // See http://www.cs.tut.fi/~ant/histthresh/ for an excellent slide presentation
    // and the original Matlab code.
    //
    // Assumes a bimodal histogram. The histogram needs is smoothed (using a
    // running average of size 3, iteratively) until there are only two local maxima.
    // Threshold t is such that yt-1 > yt <= yt+1.
    // Images with histograms having extremely unequal peaks or a broad and
    // flat valleys are unsuitable for this method.

    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);

    int iter = 0;
    int threshold = -1;
    double iHisto[256];
    for (int i = 0; i < 256; i++)
        iHisto[i] = (double)histogram[i];
    double tHisto[256];

    while (!bimodalTest(iHisto))
    {
        //smooth with a 3 point running mean filter
        for (int i = 1; i < 255; i++)
            tHisto[i] = (iHisto[i - 1] + iHisto[i] + iHisto[i + 1]) / 3;
        tHisto[0] = (iHisto[0] + iHisto[1]) / 3; //0 outside
        tHisto[255] = (iHisto[254] + iHisto[255]) / 3; //0 outside
        std::copy(std::begin(tHisto), std::end(tHisto), std::begin(iHisto));
        iter++;
        if (iter > 10000)
        {
            threshold = -1;
            return threshold;
        }
    }
    // The threshold is the minimum between the two peaks.
    for (int i = 1; i < 255; i++)
    {
        if (iHisto[i - 1] > iHisto[i] && iHisto[i + 1] >= iHisto[i])
        {
            threshold = i;
            break;
        }
    }
    return threshold;
}

int ThresholdPercentile(cv::Mat const& src, float percentile, int ignoreValueLower, int ignoreValueUpper)
{

    int histSize = 256;
    std::vector<int> hist = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);
    // Compute the CDF of histogram
    float cdfHist = 0.0;
    int cdfIdx = 1;
    cdfHist += hist.at(0);
    for (int i = 0; i < histSize; i++)
    {
        cdfHist += hist.at(i);
        if (cdfHist >= percentile)
        {
            cdfIdx = i;
            break;
        }
    }
    return cdfIdx;
}

int ThresholdRenyiEntropy(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // Kapur J.N., Sahoo P.K., and Wong A.K.C. (1985) "A New Method for
    // Gray-Level Picture Thresholding Using the Entropy of the Histogram"
    // Graphical Models and Image Processing, 29(3): 273-285
    // M. Emre Celebi
    // 06.15.2007
    // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines

    int threshold;
    int opt_threshold;

    int ih, it;
    int first_bin;
    int last_bin;
    int tmp_var;
    int t_star1, t_star2, t_star3;
    int beta1, beta2, beta3;
    double alpha;/* alpha parameter of the method */
    double term;
    double tot_ent;  /* total entropy */
    double max_ent;  /* max entropy */
    double ent_back; /* entropy of the background pixels at a given threshold */
    double ent_obj;  /* entropy of the object pixels at a given threshold */
    double omega;
    double norm_histo[256] = { 0 }; /* normalized histogram */
    double P1[256]; /* cumulative normalized histogram */
    double P2[256];

    std::vector<int> data = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);


    int total = 0;
    for (ih = 0; ih < 256; ih++)
        total += data[ih];

    for (ih = 0; ih < 256; ih++)
        norm_histo[ih] = (double)data[ih] / total;

    P1[0] = norm_histo[0];
    P2[0] = 1.0 - P1[0];
    for (ih = 1; ih < 256; ih++)
    {
        P1[ih] = P1[ih - 1] + norm_histo[ih];
        P2[ih] = 1.0 - P1[ih];
    }

    /* Determine the first non-zero bin */
    first_bin = 0;
    for (ih = 0; ih < 256; ih++)
    {
        if (!(std::abs(P1[ih]) < 2.220446049250313E-16))
        {
            first_bin = ih;
            break;
        }
    }

    /* Determine the last non-zero bin */
    last_bin = 255;
    for (ih = 255; ih >= first_bin; ih--)
    {
        if (!(std::abs(P2[ih]) < 2.220446049250313E-16))
        {
            last_bin = ih;
            break;
        }
    }

    /* Maximum Entropy Thresholding - BEGIN */
    /* ALPHA = 1.0 */
    /* Calculate the total entropy each gray-level
    and find the threshold that maximizes it
    */
    threshold = 0; // was MIN_INT in original code, but if an empty image is processed it gives an error later on.
    max_ent = 0.0;

    for (it = first_bin; it <= last_bin; it++)
    {
        /* Entropy of the background pixels */
        ent_back = 0.0;
        for (ih = 0; ih <= it; ih++)
        {
            if (data[ih] != 0)
            {
                ent_back -= (norm_histo[ih] / P1[it]) * std::log(norm_histo[ih] / P1[it]);
            }
        }

        /* Entropy of the object pixels */
        ent_obj = 0.0;
        for (ih = it + 1; ih < 256; ih++)
        {
            if (data[ih] != 0)
            {
                ent_obj -= (norm_histo[ih] / P2[it]) * std::log(norm_histo[ih] / P2[it]);
            }
        }

        /* Total entropy */
        tot_ent = ent_back + ent_obj;

        // IJ.log(""+max_ent+"  "+tot_ent);

        if (max_ent < tot_ent)
        {
            max_ent = tot_ent;
            threshold = it;
        }
    }
    t_star2 = threshold;

    /* Maximum Entropy Thresholding - END */
    threshold = 0; //was MIN_INT in original code, but if an empty image is processed it gives an error later on.
    max_ent = 0.0;
    alpha = 0.5;
    term = 1.0 / (1.0 - alpha);
    for (it = first_bin; it <= last_bin; it++)
    {
        /* Entropy of the background pixels */
        ent_back = 0.0;
        for (ih = 0; ih <= it; ih++)
            ent_back += std::sqrt(norm_histo[ih] / P1[it]);

        /* Entropy of the object pixels */
        ent_obj = 0.0;
        for (ih = it + 1; ih < 256; ih++)
            ent_obj += std::sqrt(norm_histo[ih] / P2[it]);

        /* Total entropy */
        tot_ent = term * ((ent_back * ent_obj) > 0.0 ? std::log(ent_back * ent_obj) : 0.0);

        if (tot_ent > max_ent)
        {
            max_ent = tot_ent;
            threshold = it;
        }
    }

    t_star1 = threshold;

    threshold = 0; //was MIN_INT in original code, but if an empty image is processed it gives an error later on.
    max_ent = 0.0;
    alpha = 2.0;
    term = 1.0 / (1.0 - alpha);
    for (it = first_bin; it <= last_bin; it++)
    {
        /* Entropy of the background pixels */
        ent_back = 0.0;
        for (ih = 0; ih <= it; ih++)
            ent_back += (norm_histo[ih] * norm_histo[ih]) / (P1[it] * P1[it]);

        /* Entropy of the object pixels */
        ent_obj = 0.0;
        for (ih = it + 1; ih < 256; ih++)
            ent_obj += (norm_histo[ih] * norm_histo[ih]) / (P2[it] * P2[it]);

        /* Total entropy */
        tot_ent = term * ((ent_back * ent_obj) > 0.0 ? std::log(ent_back * ent_obj) : 0.0);

        if (tot_ent > max_ent)
        {
            max_ent = tot_ent;
            threshold = it;
        }
    }

    t_star3 = threshold;

    /* Sort t_star values */
    if (t_star2 < t_star1)
    {
        tmp_var = t_star1;
        t_star1 = t_star2;
        t_star2 = tmp_var;
    }
    if (t_star3 < t_star2)
    {
        tmp_var = t_star2;
        t_star2 = t_star3;
        t_star3 = tmp_var;
    }
    if (t_star2 < t_star1)
    {
        tmp_var = t_star1;
        t_star1 = t_star2;
        t_star2 = tmp_var;
    }

    /* Adjust beta values */
    if (std::abs(t_star1 - t_star2) <= 5)
    {
        if (std::abs(t_star2 - t_star3) <= 5)
        {
            beta1 = 1;
            beta2 = 2;
            beta3 = 1;
        }
        else
        {
            beta1 = 0;
            beta2 = 1;
            beta3 = 3;
        }
    }
    else
    {
        if (std::abs(t_star2 - t_star3) <= 5)
        {
            beta1 = 3;
            beta2 = 1;
            beta3 = 0;
        }
        else
        {
            beta1 = 1;
            beta2 = 2;
            beta3 = 1;
        }
    }
    //IJ.log(""+t_star1+" "+t_star2+" "+t_star3);
    /* Determine the optimal threshold value */
    omega = P1[t_star3] - P1[t_star1];
    opt_threshold = (int)(t_star1 * (P1[t_star1] + 0.25 * omega * beta1) + 0.25 * t_star2 * omega * beta2 + t_star3 * (P2[t_star3] + 0.25 * omega * beta3));

    return opt_threshold;
}

int ThresholdShanbhag(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // Shanhbag A.G. (1994) "Utilization of Information Measure as a Means of
    //  Image Thresholding" Graphical Models and Image Processing, 56(5): 414-419
    // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
    int threshold;
    int ih, it;
    int first_bin;
    int last_bin;
    double term;
    double tot_ent;  /* total entropy */
    double min_ent;  /* max entropy */
    double ent_back; /* entropy of the background pixels at a given threshold */
    double ent_obj;  /* entropy of the object pixels at a given threshold */
    double norm_histo[256]; /* normalized histogram */
    double P1[256]; /* cumulative normalized histogram */
    double P2[256];

    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);


    int total = 0;
    for (ih = 0; ih < 256; ih++)
        total += histogram[ih];

    for (ih = 0; ih < 256; ih++)
        norm_histo[ih] = (double)histogram[ih] / total;

    P1[0] = norm_histo[0];
    P2[0] = 1.0 - P1[0];
    for (ih = 1; ih < 256; ih++)
    {
        P1[ih] = P1[ih - 1] + norm_histo[ih];
        P2[ih] = 1.0 - P1[ih];
    }

    /* Determine the first non-zero bin */
    first_bin = 0;
    for (ih = 0; ih < 256; ih++)
    {
        if (!(abs(P1[ih]) < 2.220446049250313E-16))
        {
            first_bin = ih;
            break;
        }
    }

    /* Determine the last non-zero bin */
    last_bin = 255;
    for (ih = 255; ih >= first_bin; ih--)
    {
        if (!(abs(P2[ih]) < 2.220446049250313E-16))
        {
            last_bin = ih;
            break;
        }
    }

    // Calculate the total entropy each gray-level
    // and find the threshold that maximizes it 
    threshold = -1;
    min_ent = DBL_MAX;

    for (it = first_bin; it <= last_bin; it++)
    {
        /* Entropy of the background pixels */
        ent_back = 0.0;
        term = 0.5 / P1[it];
        for (ih = 1; ih <= it; ih++)
        { //0+1?
            ent_back -= norm_histo[ih] * log(1.0 - term * P1[ih - 1]);
        }
        ent_back *= term;

        /* Entropy of the object pixels */
        ent_obj = 0.0;
        term = 0.5 / P2[it];
        for (ih = it + 1; ih < 256; ih++)
        {
            ent_obj -= norm_histo[ih] * log(1.0 - term * P2[ih]);
        }
        ent_obj *= term;

        /* Total entropy */
        tot_ent = abs(ent_back - ent_obj);

        if (tot_ent < min_ent)
        {
            min_ent = tot_ent;
            threshold = it;
        }
    }
    return threshold;
}

int ThresholdTriangle(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    //  Zack, G. W., Rogers, W. E. and Latt, S. A., 1977,
    //  Automatic Measurement of Sister Chromatid Exchange Frequency,
    // Journal of Histochemistry and Cytochemistry 25 (7), pp. 741-753
    //
    //  modified from Johannes Schindelin plugin
    // 
    // find min and max

    int dataLength = 256;
    std::vector<int> data = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);
    int min = 0, dmax = 0, max = 0, min2 = 0;
    for (int i = 0; i < dataLength; i++)
    {
        if (data[i] > 0)
        {
            min = i;
            break;
        }
    }
    if (min > 0) min--; // line to the (p==0) point, not to data[min]

    // The Triangle algorithm cannot tell whether the data is skewed to one side or another.
    // This causes a problem as there are 2 possible thresholds between the max and the 2 extremes
    // of the histogram.
    // Here I propose to find out to which side of the max point the data is furthest, and use that as
    //  the other extreme.
    for (int i = 255; i > 0; i--)
    {
        if (data[i] > 0)
        {
            min2 = i;
            break;
        }
    }
    if (min2 < 255) min2++; // line to the (p==0) point, not to data[min]

    for (int i = 0; i < 256; i++)
    {
        if (data[i] > dmax)
        {
            max = i;
            dmax = data[i];
        }
    }
    // find which is the furthest side
    //IJ.log(""+min+" "+max+" "+min2);
    bool inverted = false;
    if ((max - min) < (min2 - max))
    {
        // reverse the histogram
        //IJ.log("Reversing histogram.");
        inverted = true;
        int left = 0;          // index of leftmost element
        int right = 255; // index of rightmost element
        while (left < right)
        {
            // exchange the left and right elements
            int temp = data[left];
            data[left] = data[right];
            data[right] = temp;
            // move the bounds toward the center
            left++;
            right--;
        }
        min = 255 - min2;
        max = 255 - max;
    }

    if (min == max)
    {
        //IJ.log("Triangle:  min == max.");
        return min;
    }

    // describe line by nx * x + ny * y - d = 0
    double nx, ny, d;
    // nx is just the max frequency as the other point has freq=0
    nx = data[max];   //-min; // data[min]; //  lowest value bmin = (p=0)% in the image
    ny = (double)min - max;
    d = std::sqrt(nx * nx + ny * ny);
    nx /= d;
    ny /= d;
    d = nx * min + ny * data[min];

    // find split point
    int split = min;
    double splitDistance = 0;
    for (int i = min + 1; i <= max; i++)
    {
        double newDistance = nx * i + ny * data[i] - d;
        if (newDistance > splitDistance)
        {
            split = i;
            splitDistance = newDistance;
        }
    }
    split--;

    if (inverted)
    {
        // The histogram might be used for something else, so let's reverse it back
        int left = 0;
        int right = 255;
        while (left < right)
        {
            int temp = data[left];
            data[left] = data[right];
            data[right] = temp;
            left++;
            right--;
        }
        return (255 - split);
    }
    else
        return split;
}

int ThresholdYen(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // Implements Yen  thresholding method
    // 1) Yen J.C., Chang F.J., and Chang S. (1995) "A New Criterion 
    //    for Automatic Multilevel Thresholding" IEEE Trans. on Image 
    //    Processing, 4(3): 370-378
    // 2) Sezgin M. and Sankur B. (2004) "Survey over Image Thresholding 
    //    Techniques and Quantitative Performance Evaluation" Journal of 
    //    Electronic Imaging, 13(1): 146-165
    //    http://citeseer.ist.psu.edu/sezgin04survey.html
    //
    // M. Emre Celebi
    // 06.15.2007
    // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
    int threshold;
    int ih, it;
    double crit;
    double max_crit;
    double norm_histo[256]; /* normalized histogram */
    double P1[256]; /* cumulative normalized histogram */
    double P1_sq[256];
    double P2_sq[256];

    int dataLength = 256;
    std::vector<int> data = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);

    int total = 0;
    for (ih = 0; ih < 256; ih++)
        total += data[ih];

    for (ih = 0; ih < 256; ih++)
        norm_histo[ih] = (double)data[ih] / total;

    P1[0] = norm_histo[0];
    for (ih = 1; ih < 256; ih++)
        P1[ih] = P1[ih - 1] + norm_histo[ih];

    P1_sq[0] = norm_histo[0] * norm_histo[0];
    for (ih = 1; ih < 256; ih++)
        P1_sq[ih] = P1_sq[ih - 1] + norm_histo[ih] * norm_histo[ih];

    P2_sq[255] = 0.0;
    for (ih = 254; ih >= 0; ih--)
        P2_sq[ih] = P2_sq[ih + 1] + norm_histo[ih + 1] * norm_histo[ih + 1];

    /* Find the threshold that maximizes the criterion */
    threshold = -1;
    max_crit = -DBL_MAX;
    for (it = 0; it < 256; it++)
    {
        crit = -1.0 * ((P1_sq[it] * P2_sq[it]) > 0.0 ? std::log(P1_sq[it] * P2_sq[it]) : 0.0) + 2 * ((P1[it] * (1.0 - P1[it])) > 0.0 ? std::log(P1[it] * (1.0 - P1[it])) : 0.0);
        if (crit > max_crit)
        {
            max_crit = crit;
            threshold = it;
        }
    }
    return threshold;
}

std::vector<int> ThresholdOtsuMulti(cv::Mat const& src)
{
    int pixelsCount = src.cols * src.rows;
    std::vector<int> histogram = histogramCalculate(src, 0, 256);

    double c = 0;
    double Mt = 0;

    double p[256] = { 0 };
    for (int i = 0; i < 256; i++)
    {
        p[i] = (double)histogram[i] / (double)pixelsCount;
        Mt += i * p[i];
    }

    int optimalTreshold1 = 0;
    int optimalTreshold2 = 0;

    double maxBetweenVar = 0;

    double w0 = 0;
    double m0 = 0;
    double c0 = 0;
    double p0 = 0;

    double w1 = 0;
    double m1 = 0;
    double c1 = 0;
    double p1 = 0;

    for (int tr1 = 0; tr1 < 256; tr1++)
    {
        p0 += p[tr1];
        w0 += (tr1 * p[tr1]);
        if (p0 != 0)
        {
            m0 = w0 / p0;
        }

        c0 = p0 * (m0 - Mt) * (m0 - Mt);

        c1 = 0;
        w1 = 0;
        m1 = 0;
        p1 = 0;
        for (int tr2 = tr1 + 1; tr2 < 256; tr2++)
        {

            p1 += p[tr2];
            w1 += (tr2 * p[tr2]);
            if (p1 != 0)
            {
                m1 = w1 / p1;
            }

            c1 = p1 * (m1 - Mt) * (m1 - Mt);

            double p2 = 1 - (p0 + p1);
            double w2 = Mt - (w0 + w1);
            double m2 = w2 / p2;
            double c2 = p2 * (m2 - Mt) * (m2 - Mt);

            double c = c0 + c1 + c2;

            if (maxBetweenVar < c)
            {
                maxBetweenVar = c;
                optimalTreshold1 = tr1;
                optimalTreshold2 = tr2;
            }
        }
    }
    std::vector<int> thresholds;
    thresholds.push_back(optimalTreshold1);
    thresholds.push_back(optimalTreshold2);
    return thresholds;
}

std::vector<int> ThresholdOtsuMulti(cv::Mat const& src, int level)
{
    //const ushort histSize = 256;
    const int MLEVEL = level;
    int histogram[256] = { 0 };
    int pixelsCount = src.cols * src.rows;

    cv::Mat hist;
    int histSize = 256;
    float range[] = { 0, 255 }; //the upper boundary is exclusive
    const float* histRange = { range };
    calcHist(&src, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, true);
    hist = hist / ((double)src.rows * src.cols);

    /*for (int y = 0; y < src.rows; y++)
    {
        for (int x = 0; x < src.cols; x++)
        {
            uchar value = src.at<uchar>(y, x);
            histogram[value]++;
        }
    }
    double c = 0;
    double Mt = 0;*/

    float p[256] = { 0 };
    for (int i = 0; i < histSize; i++)
    {
        p[i] = hist.at<float>(i);
        //p[i] = (double)histogram[i] / (double)pixelsCount;
        //Mt += i * p[i];
    }

    float P[NGRAY][NGRAY];
    float S[NGRAY][NGRAY];
    float H[NGRAY][NGRAY];
    buildLookupTables(P, S, H, p);

    int* threshold = new int[level];

    ////////////////////////////////////////////////////////
    // now M level loop   MLEVEL dependent term
    ////////////////////////////////////////////////////////
    float maxSig = findMaxSigma(MLEVEL, H, threshold);

    std::vector<int> oThresholds;
    for (int tIdx = 0; tIdx < MLEVEL; tIdx++)
    {
        oThresholds.push_back((int)threshold[tIdx]);
    }
    delete[] threshold;
    return oThresholds;
}

std::vector<int> ThresholdOtsuThreeLevel(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    int pixelsCount = src.cols * src.rows;
    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);

    double c = 0;
    double Mt = 0;

    double p[256] = { 0 };
    for (int i = 0; i < 256; i++)
    {
        p[i] = (double)histogram[i] / (double)pixelsCount;
        Mt += i * p[i];
    }

    int optimalTreshold1 = 0;
    int optimalTreshold2 = 0;
    int optimalTreshold3 = 0;

    double maxBetweenVar = 0;

    double w0 = 0;
    double m0 = 0;
    double c0 = 0;
    double p0 = 0;

    double w1 = 0;
    double m1 = 0;
    double c1 = 0;
    double p1 = 0;

    double w2 = 0;
    double m2 = 0;
    double c2 = 0;
    double p2 = 0;

    for (int tr1 = 0; tr1 < 256; tr1++)
    {
        p0 += p[tr1];
        w0 += (tr1 * p[tr1]);
        if (p0 != 0)
        {
            m0 = w0 / p0;
        }

        c0 = p0 * (m0 - Mt) * (m0 - Mt);

        c1 = 0;
        w1 = 0;
        m1 = 0;
        p1 = 0;
        for (int tr2 = tr1 + 1; tr2 < 256; tr2++)
        {

            p1 += p[tr2];
            w1 += (tr2 * p[tr2]);
            if (p1 != 0)
            {
                m1 = w1 / p1;
            }

            c1 = p1 * (m1 - Mt) * (m1 - Mt);

            c2 = 0;
            w2 = 0;
            m2 = 0;
            p2 = 0;
            for (int tr3 = tr2 + 1; tr3 < 256; tr3++)
            {

                p2 += p[tr3];
                w2 += (tr3 * p[tr3]);
                if (p2 != 0)
                {
                    m2 = w2 / p2;
                }

                c2 = p2 * (m2 - Mt) * (m2 - Mt);

                double p3 = 1 - (p0 + p1 + p2);
                double w3 = Mt - (w0 + w1 + w2);
                double m3 = w3 / p3;
                double c3 = p3 * (m3 - Mt) * (m3 - Mt);

                double c = c0 + c1 + c2 + c3;

                if (maxBetweenVar < c)
                {
                    maxBetweenVar = c;
                    optimalTreshold1 = tr1;
                    optimalTreshold2 = tr2;
                    optimalTreshold3 = tr3;
                }
            }
        }
    }
    std::vector<int> lstThresholds;
    lstThresholds.push_back(optimalTreshold1);
    lstThresholds.push_back(optimalTreshold2);
    lstThresholds.push_back(optimalTreshold3);
    return lstThresholds;
}

void ThresholdIntegral(cv::Mat const& inputMat, cv::Mat& outputMat, float T, int part)
{
    int nRows, nCols, S, s2, i, j;
    int x1, y1, x2, y2, count, sum, ip, it;
    cv::Mat sumMat;
    int* p_y1, * p_y2;
    const uchar* p_inputMat;
    uchar* p_outputMat;

    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    nRows = inputMat.rows;
    // cols -> width -> x
    nCols = inputMat.cols;

    // create the integral image
    cv::integral(inputMat, sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    S = MAX(nRows, nCols);
    S = (S > (part + part)) ? (S / part) : 2;

    // perform thresholding
    s2 = S / 2;

    for (i = 0; i < nRows; ++i)
    {
        y1 = (i < s2) ? 0 : (i - s2);
        y2 = ((i + s2) < nRows) ? (i + s2) : (nRows - 1);

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for (j = 0; j < nCols; ++j)
        {
            x1 = (j < s2) ? 0 : (j - s2);
            x2 = ((j + s2) < nCols) ? (j + s2) : (nCols - 1);

            count = (x2 - x1) * (y2 - y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            ip = (int)(p_inputMat[j] * count);
            it = (int)(sum * (1.0f - T));
            p_outputMat[j] = (ip < it) ? 0 : 255;
        }
    }
}

int ThresholdHuang2(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    // Implements Huang's fuzzy thresholding method 
    // Uses Shannon's entropy function (one can also use Yager's entropy function) 
    // Huang L.-K. and Wang M.-J.J. (1995) "Image Thresholding by Minimizing  
    // the Measures of Fuzziness" Pattern Recognition, 28(1): 41-51
    // Reimplemented (to handle 16-bit efficiently) by Johannes Schindelin Jan 31, 2011

    std::vector<int> histogram = histogramCalculate(src, ignoreValueLower, ignoreValueUpper);
    
    int cols = src.cols;
    int rows = src.rows;
    int pixelsCount = cols * rows;

   
    // find first and last non-empty bin
    int first, last;
    for (first = 0; first < 256 && histogram[first] == 0; first++)
        ; // do nothing
    for (last = 256 - 1; last > first && histogram[last] == 0; last--)
        ; // do nothing
    if (first == last)
        return 0;

    // calculate the cumulative density and the weighted cumulative density
    double* S = new double[last + 1]();
    double* W = new double[last + 1]();
    S[0] = histogram[0];
    for (int i = 1; i <= last; i++) {
        S[i] = S[i - 1] + histogram[i];
        W[i] = W[i - 1] + i * histogram[i];
    }
    // precalculate the summands of the entropy given the absolute difference x - mu (integral)
    double C = last - first;
    double* Smu = new double[last + 1 - first]();
    int SmuSize = last + 1 - first;
    for (int i = 1; i < SmuSize; i++) {
        double mu = 1 / (1 + i / C);
        Smu[i] = -mu * std::log(mu) - (1 - mu) * std::log(1 - mu);
    }

    // calculate the threshold
    int bestThreshold = 0;
    double bestEntropy = DBL_MAX;
    for (int threshold = first; threshold <= last; threshold++) {
        double entropy = 0;
        int mu = (int)std::round(W[threshold] / S[threshold]);
        for (int i = first; i <= threshold; i++)
            entropy += Smu[std::abs(i - mu)] * histogram[i];
        mu = (int)std::round((W[last] - W[threshold]) / (S[last] - S[threshold]));
        for (int i = threshold + 1; i <= last; i++)
            entropy += Smu[std::abs(i - mu)] * histogram[i];

        if (bestEntropy > entropy) {
            bestEntropy = entropy;
            bestThreshold = threshold;
        }
    }
    delete[] S;
    delete[] W;
    delete[] Smu;
    return bestThreshold;
}

void TryAllGolbalThreshold(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper)
{
    cv::Mat binaryImg;
    int thresh = ThresholdHuang(src,ignoreValueLower,ignoreValueUpper);
    int basicSize = 300;
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdHuang", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdHuang", 0, 0);
    cv::resizeWindow("ThresholdHuang", basicSize, basicSize);
    cv::imshow("ThresholdHuang", binaryImg);

    thresh = ThresholdIntermodes(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdIntermodes", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdIntermodes", basicSize, 0);
    cv::resizeWindow("ThresholdIntermodes", basicSize, basicSize);
    cv::imshow("ThresholdIntermodes", binaryImg);

    thresh = ThresholdIsoData(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdIsoData", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdIsoData", basicSize * 2, 0);
    cv::resizeWindow("ThresholdIsoData", basicSize, basicSize);
    cv::imshow("ThresholdIsoData", binaryImg);

    thresh = ThresholdLi(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdLi", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdLi", basicSize * 3, 0);
    cv::resizeWindow("ThresholdLi", basicSize, basicSize);
    cv::imshow("ThresholdLi", binaryImg);

    thresh = ThresholdMaxEntropy(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdMaxEntropy", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdMaxEntropy", basicSize * 4, 0);
    cv::resizeWindow("ThresholdMaxEntropy", basicSize, basicSize);
    cv::imshow("ThresholdMaxEntropy", binaryImg);

    thresh = ThresholdMean(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("Mean", cv::WINDOW_NORMAL);
    cv::moveWindow("Mean", basicSize * 0, basicSize);
    cv::resizeWindow("Mean", basicSize, basicSize);
    cv::imshow("Mean", binaryImg);

    thresh = MinErrorI(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("MinErrorI", cv::WINDOW_NORMAL);
    cv::moveWindow("MinErrorI", basicSize * 1, basicSize);
    cv::resizeWindow("MinErrorI", basicSize, basicSize);
    cv::imshow("MinErrorI", binaryImg);

    thresh = ThresholdMinimum(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdMinimum", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdMinimum", basicSize * 2, basicSize);
    cv::resizeWindow("ThresholdMinimum", basicSize, basicSize);
    cv::imshow("ThresholdMinimum", binaryImg);

    thresh = Moments(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("Moments", cv::WINDOW_NORMAL);
    cv::moveWindow("Moments", basicSize * 3, basicSize);
    cv::resizeWindow("Moments", basicSize, basicSize);
    cv::imshow("Moments", binaryImg);

    cv::threshold(src, binaryImg, 0, 255, cv::THRESH_OTSU);
    cv::namedWindow("Otsu", cv::WINDOW_NORMAL);
    cv::moveWindow("Otsu", basicSize * 4, basicSize);
    cv::resizeWindow("Otsu", basicSize, basicSize);
    cv::imshow("Otsu", binaryImg);

    thresh = ThresholdPercentile(src);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("PercentileThreshold", cv::WINDOW_NORMAL);
    cv::moveWindow("PercentileThreshold", basicSize * 0, basicSize * 2);
    cv::resizeWindow("PercentileThreshold", basicSize, basicSize);
    cv::imshow("PercentileThreshold", binaryImg);

    thresh = ThresholdRenyiEntropy(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdRenyiEntropy", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdRenyiEntropy", basicSize * 1, basicSize * 2);
    cv::resizeWindow("ThresholdRenyiEntropy", basicSize, basicSize);
    cv::imshow("ThresholdRenyiEntropy", binaryImg);

    thresh = ThresholdShanbhag(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdShanbhag", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdShanbhag", basicSize * 2, basicSize * 2);
    cv::resizeWindow("ThresholdShanbhag", basicSize, basicSize);
    cv::imshow("ThresholdShanbhag", binaryImg);

    thresh = ThresholdTriangle(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdTriangle", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdTriangle", basicSize * 3, basicSize * 2);
    cv::resizeWindow("ThresholdTriangle", basicSize, basicSize);
    cv::imshow("ThresholdTriangle", binaryImg);

    thresh = ThresholdYen(src, ignoreValueLower, ignoreValueUpper);
    cv::threshold(src, binaryImg, thresh, 255, cv::THRESH_BINARY);
    cv::namedWindow("ThresholdYen", cv::WINDOW_NORMAL);
    cv::moveWindow("ThresholdYen", basicSize * 4, basicSize * 2);
    cv::resizeWindow("ThresholdYen", basicSize, basicSize);
    cv::imshow("ThresholdYen", binaryImg);
}

double Otsu8uWithMask(const cv::Mat1b src, const cv::Mat1b& mask)
{
    const int N = 256;
    int M = 0;
    int i, j, h[N] = { 0 };
    int cols = src.cols;
    int rows = src.rows;

    for (i = 0; i < rows; i++)
    {
        const uchar* psrc = src.ptr(i);
        const uchar* pmask = mask.ptr(i);
        for (j = 0; j < cols; j++)
        {
            if (pmask[j])
            {
                h[psrc[j]]++;
                ++M;
            }
        }
    }

    double mu = 0, scale = 1. / (M);
    for (i = 0; i < N; i++)
        mu += i * (double)h[i];

    mu *= scale;
    double mu1 = 0, q1 = 0;
    double max_sigma = 0, max_val = 0;

    for (i = 0; i < N; i++)
    {
        double p_i, q2, mu2, sigma;

        p_i = h[i] * scale;
        mu1 *= q1;
        q1 += p_i;
        q2 = 1. - q1;

        if (MIN(q1, q2) < FLT_EPSILON || MAX(q1, q2) > 1. - FLT_EPSILON)
            continue;

        mu1 = (mu1 + i * p_i) / q1;
        mu2 = (mu - q1 * mu1) / q2;
        sigma = q1 * q2 * (mu1 - mu2) * (mu1 - mu2);
        if (sigma > max_sigma)
        {
            max_sigma = sigma;
            max_val = i;
        }
    }

    return max_val;
}

double ThresholdWithMask(cv::Mat const& src, cv::Mat& dst, double thresh, double maxval, int type, const cv::Mat& mask)
{
    if (src.type() != CV_8U || mask.type() != CV_8U) throw("Not support type");

    if (mask.empty() || (mask.rows == src.rows && mask.cols == src.cols && countNonZero(mask) == src.rows * src.cols))
    {
        // If empty mask, or all-white mask, use cv::threshold
        thresh = cv::threshold(src, dst, thresh, maxval, type);
    }
    else
    {
        // Use mask
        bool use_otsu = (type & cv::THRESH_OTSU) != 0;
        if (use_otsu)
        {
            // If OTSU, get thresh value on mask only
            thresh = Otsu8uWithMask(src, mask);
            // Remove THRESH_OTSU from type
            type &= cv::THRESH_MASK;
        }

        // Apply cv::threshold on all image
        thresh = cv::threshold(src, dst, thresh, maxval, type);

        // Copy original image on inverted mask
        src.copyTo(dst, ~mask);
    }
    return thresh;
}

double Threshold(cv::Mat const& src, cv::Mat& dst, int lower, int upper, int type)
{
    double thd=0;
    if (!src.empty() && src.type() == CV_8U)
    {
        lower = (std::max)(lower, 0);
        upper = (std::max)(upper, 0);
        if (lower > upper) std::swap(lower, upper);

        bool isAuto = (type & cv::THRESH_OTSU) == cv::THRESH_OTSU || (type & cv::THRESH_TRIANGLE) == cv::THRESH_TRIANGLE;

        if ((lower == 0 || upper == 0) && !isAuto)
        {
            type |= cv::THRESH_OTSU;
            isAuto = true;
        }

        if (!isAuto)
        {
            isAuto = upper == 255;
        }

        if (!isAuto)
        {
            thd = cv::threshold(src, dst, upper, 255, cv::ThresholdTypes::THRESH_TOZERO_INV);
            thd = cv::threshold(dst, dst, lower, 255, type);
        }
        else 
        {
            thd = cv::threshold(src, dst, lower, 255, type);
        }
    }

    return thd;
}

}//Threshold
}//xvt