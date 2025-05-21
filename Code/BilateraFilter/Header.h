#ifndef  _SPACTIAL_FILTERING_H_
#define _SPACTIAL_FILTERING_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

class BilateralFilter
{
public:
	static Mat ManualBilateralFilter(const cv::Mat& image, int kernelSize, double sigmaColor, double sigmaSpace);
private:
};

#endif //_SPACTIAL_FILTERING_H_