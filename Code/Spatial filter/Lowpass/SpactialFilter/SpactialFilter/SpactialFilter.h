#ifndef  _SPACTIAL_FILTERING_H_
#define _SPACTIAL_FILTERING_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

class SpactialFiltering
{
public:
	static Mat ManualBoxFilter(const Mat& image, int KernelSize);
	static Mat ManualGaussianFilter(const cv::Mat& image, int kernelSize, double sigma);
	static Mat ManualBilateralFilter(const cv::Mat& image, int kernelSize, double sigmaIntensity, double sigmaSpace);
	static Mat ManualSobelFilter(const Mat& src, int ksize = 3);
private:
	//ManualGaussianFilter
	static Mat CreateGaussKernel(int ksize, double sigma);
	static Mat createGaussianKernel1D(int ksize);
	static Mat createDerivativeKernel(int ksize, int order, double sigma = -1.0);
	static Mat applyConvolution(const Mat& src, const Mat& kernel);
};

#endif //_SPACTIAL_FILTERING_H_