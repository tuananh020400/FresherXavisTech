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

Mat skewImage(const Mat& src, float shear, const std::string& direction) {
	int width = src.cols;
	int height = src.rows;

	if (src.channels() != 1) {
		cerr << "Chỉ hỗ trợ ảnh grayscale (1 kênh).\n";
		return src.clone();
	}

	if (direction == "x") {
		int new_width = static_cast<int>(width + std::abs(shear) * height);
		Mat dst = Mat::zeros(height, new_width, src.type());

		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int new_x = static_cast<int>(x + shear * y);
				if (new_x >= 0 && new_x < dst.cols) {
					dst.at<uchar>(y, new_x) = src.at<uchar>(y, x);
				}
			}
		}

		return dst;
	}
	else if (direction == "y") {
		int new_height = static_cast<int>(height + std::abs(shear) * width);
		Mat dst = Mat::zeros(new_height, width, src.type());

		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int new_y = static_cast<int>(y + shear * x);
				if (new_y >= 0 && new_y < dst.rows) {
					dst.at<uchar>(new_y, x) = src.at<uchar>(y, x);
				}
			}
		}

		return dst;
	}
	else {
		cerr << "Chỉ hỗ trợ skew theo 'x' hoặc 'y'\n";
		return src.clone();
	}
}

int main() {
	Mat src = imread("D:/FresherXavisTech/Image/images.jpg", IMREAD_GRAYSCALE);
	if (src.empty()) {
		cout << "Cannot open the image!" << endl;
		return -1;
	}
	imshow("Original Image", src);

	Mat blured;
	GaussianBlur(src, blured, Size(7, 7), 20.0, 20.0);

	Mat zoomed = zoom(src, 2, 0.5);
	imshow("Zoomed Image", zoomed);

	Mat rotated = rotate(src, 45.0);
	imshow("Rotated Image", rotated);
	
	Mat translated = translate(src, 50.0, 30.0);
	imshow("Translated Image", translated);

	Mat skewedX = skewImage(src, 0.3f, "x");
	Mat skewedY = skewImage(src, 0.5f, "y");

	waitKey(0);
	return 0;
}