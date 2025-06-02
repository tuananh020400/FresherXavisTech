#ifndef _MORPHOLOGICAL_H_
#define _MORPHOLOGICAL_H_

#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class Morphological
{
public:
	static Mat erosion(const Mat& img, const Mat& struct_elem);
	static Mat dilation(const Mat& img, const Mat& struct_elem);
	static Mat opening(const Mat& img, const Mat& struct_elem);
	static Mat closing(const Mat& img, const Mat& struct_elem);
	static Mat HMT(const Mat& img, const Mat& struct_elem);
private:
};
#endif // !_MORPHOLOGICAL_H_
