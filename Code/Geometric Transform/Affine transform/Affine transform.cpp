#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

unsigned char bilinearInterpolate(const Mat& src, double u, double v) {
	int u0 = static_cast<int>(u);
	int v0 = static_cast<int>(v);
	double du = u - u0;
	double dv = v - v0;

	if (u0 < 0 || u0 >= src.cols - 1 || v0 < 0 || v0 >= src.rows - 1) {
		return 0;
	}

	unsigned char p00 = src.at<unsigned char>(v0, u0);
	unsigned char p10 = src.at<unsigned char>(v0, u0 + 1);
	unsigned char p01 = src.at<unsigned char>(v0 + 1, u0);
	unsigned char p11 = src.at<unsigned char>(v0 + 1, u0 + 1);

	// Nội suy tuyến tính
	double interpolated = (1 - du) * (1 - dv) * p00 +
		du * (1 - dv) * p10 +
		(1 - du) * dv * p01 +
		du * dv * p11;

	return static_cast<unsigned char>(interpolated);
}

Mat zoom(const Mat& src, double sx, double sy)
{
	Mat dst(sy * src.rows, sx * src.cols, src.type());
	for (int y = 0; y < dst.rows; ++y) {
		for (int x = 0; x < dst.cols; ++x) {
			double u = x / sx; // u = x / sx
			double v = y / sy; // v = y / sy
			dst.at<unsigned char>(y, x) = bilinearInterpolate(src, u, v);
		}
	}
	return dst;
}

Mat rotate(const Mat& src, double angle)
{
	Mat dst(src.rows, src.cols, src.type());
	const double pi = 3.14159265359;
	double rad = angle * pi / 180.0;
	double cos_a = cos(rad);
	double sin_a = sin(rad);
	double cx = src.cols / 2.0;
	double cy = src.rows / 2.0;

	for (int y = 0; y < dst.rows; ++y) {
		for (int x = 0; x < dst.cols; ++x) {
			double x0 = x - cx;
			double y0 = y - cy;
			double u = -x0 * sin_a + y0 * cos_a + cy;  // u = x*cosθ + y*sinθ
			double v = x0 * cos_a + y0 * sin_a + cx; // v = -x*sinθ + y*cosθ

			dst.at<unsigned char>(y, x) = bilinearInterpolate(src, v, u);
		}
	}

	return dst;
}

Mat translate(const Mat& src, double tx, double ty) {
	Mat dst(src.rows, src.cols, src.type());
	for (int y = 0; y < dst.rows; ++y) {
		for (int x = 0; x < dst.cols; ++x) {
			double u = x - tx; // u = x - tx
			double v = y - ty; // v = y - ty
			dst.at<unsigned char>(y, x) = bilinearInterpolate(src, u, v);
		}
	}
	return dst;
}

int main() {
	Mat src = imread("D:/FresherXavisTech/Image/images.jpg", IMREAD_GRAYSCALE);
	if (src.empty()) {
		cout << "Cannot open the image!" << endl;
		return -1;
	}
	imshow("Original Image", src);

	Mat zoomed = zoom(src, 2, 0.5);
	imshow("Zoomed Image", zoomed);

	Mat rotated = rotate(src, 45.0);
	imshow("Rotated Image", rotated);
	
	Mat translated = translate(src, 50.0, 30.0);
	imshow("Translated Image", translated);

	waitKey(0);
	return 0;
}