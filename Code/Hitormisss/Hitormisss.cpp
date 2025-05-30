#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Hàm erosion với SE có don't care (-1)
Mat erosion_with_dont_care(const Mat& image, const Mat& SE) {
	int img_h = image.rows;
	int img_w = image.cols;
	int se_h = SE.rows;
	int se_w = SE.cols;
	int pad_h = se_h / 2;
	int pad_w = se_w / 2;

	// Padding ảnh nhị phân bằng 0
	Mat padded_img;
	copyMakeBorder(image, padded_img, pad_h, pad_h, pad_w, pad_w, BORDER_CONSTANT, Scalar(0));

	Mat result = Mat::zeros(img_h, img_w, CV_8UC1);

	for (int i = 0; i < img_h; ++i) {
		for (int j = 0; j < img_w; ++j) {
			bool match = true;
			for (int x = 0; x < se_h; ++x) {
				for (int y = 0; y < se_w; ++y) {
					int se_val = SE.at<int>(x, y);
					if (se_val == -1)
						continue;  // don't care
					int img_val = padded_img.at<uchar>(i + x, j + y) > 0 ? 1 : 0;
					if (se_val != img_val) {
						match = false;
						break;
					}
				}
				if (!match) break;
			}
			result.at<uchar>(i, j) = match ? 255 : 0;  // ảnh nhị phân 0/255
		}
	}

	return result;
}

int main() {
	// Tạo ảnh nhị phân mẫu
	uchar data[] = {
		0,0,255,255,0,
		0,255,255,255,0,
		255,255,255,0,0,
		0,255,0,0,0,
		0,0,0,0,0
	};
	Mat image(5, 5, CV_8UC1, data);

	// Tạo SE với -1 là don't care (kiểu int)
	int se_data[] = {
		1, 0, -1,
		0, 1,  0,
		-1,0,  1
	};
	Mat SE(3, 3, CV_32SC1, se_data);

	Mat eroded = erosion_with_dont_care(image, SE);

	cout << "Input image:\n" << image << endl;
	cout << "Eroded image:\n" << eroded << endl;

	return 0;
}
