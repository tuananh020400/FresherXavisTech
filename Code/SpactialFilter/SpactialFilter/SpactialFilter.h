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
	static Mat ManualBilateralFilter(const cv::Mat& image, int kernelSize, double sigmaColor, double sigmaSpace);
private:
	static Mat CreateGaussKernel(int ksize, double sigma);
};

#endif //_SPACTIAL_FILTERING_H_