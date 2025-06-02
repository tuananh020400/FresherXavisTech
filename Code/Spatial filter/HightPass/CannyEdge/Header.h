#ifndef _HEADER_H_
#define _HEADER_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

class CannyEdge
{
public:
	static Mat CannyEdgeFilter(const Mat& img, double lowThresh, double highThresh);

private:
	static void computeGradient(const Mat& gx, const Mat& gy, Mat& magnitude, Mat& angle);
	static Mat nonMaxSuppression(const Mat& magnitude, const Mat& angle);
	static Mat hysteresis(const Mat& img, double lowThresh, double highThresh);
};


#endif // _HEADER_H_



