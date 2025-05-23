#ifndef _SOBEL_FILTER_H_
#define _SOBEL_FILTER_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

class SobelFilter
{
public:
	static Mat ManualSobelFilter(const Mat& src, int ksize = 3);
private:
	static Mat createGaussianKernel1D(int size, double sigma);
	static Mat createDerivativeKernel1D(int size, double sigma);
	static Mat createSobelKernel(int size, double sigma, bool derivativeX);
	static Mat applyConvolution(const Mat& src, const Mat& kernel);
};

#endif // _SOBEL_FILTER_H_
