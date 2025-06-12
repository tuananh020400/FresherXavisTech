#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <cstdint>

using namespace std;
using namespace cv;

vector<Vec3f> houghCircle(const Mat& image, double dp, double minDist, double param1, double param2, int min_radius, int max_radius) {
	Mat edges;
	Canny(image, edges, param1 / 2, param1);

	Mat dx, dy;
	Sobel(image, dx, CV_32F, 1, 0, 3);
	Sobel(image, dy, CV_32F, 0, 1, 3);

	int acc_width = static_cast<int>(image.cols / dp);
	int acc_height = static_cast<int>(image.rows / dp);
	if (acc_width < 1 || acc_height < 1) {
		return vector<Vec3f>();
	}

	vector<vector<vector<int>>> accumulator(
		acc_height, vector<vector<int>>(
			acc_width, vector<int>(
				max_radius - min_radius + 1, 0)));

	const double pi = 3.141592653589793;

	for (int y = 0; y < image.rows; ++y) {
		for (int x = 0; x < image.cols; ++x) {
			if (edges.at<uchar>(y, x) == 255) {
				float gx = dx.at<float>(y, x);
				float gy = dy.at<float>(y, x);
				float theta = atan2(gy, gx);

				for (int r = min_radius; r <= max_radius; ++r) {
					for (int sign : {-1, 1}) {
						int a = static_cast<int>((x + sign * r * cos(theta)) / dp);
						int b = static_cast<int>((y + sign * r * sin(theta)) / dp);

						if (a >= 0 && a < acc_width && b >= 0 && b < acc_height) {
							accumulator[b][a][r - min_radius]++;
						}
					}
				}
			}
		}
	}

	vector<Vec3f> circles;
	for (int y = 0; y < acc_height; ++y) {
		for (int x = 0; x < acc_width; ++x) {
			for (int r = min_radius; r <= max_radius; ++r) {
				if (accumulator[y][x][r - min_radius] >= param2) {
					float img_x = x * dp;
					float img_y = y * dp;
					circles.emplace_back(img_x, img_y, static_cast<float>(r));
				}
			}
		}
	}

	vector<Vec3f> filtered_circles;
	for (const auto& circle : circles) {
		bool keep = true;
		for (const auto& kept_circle : filtered_circles) {
			float dx = circle[0] - kept_circle[0];
			float dy = circle[1] - kept_circle[1];
			float distance = sqrt(dx * dx + dy * dy);
			if (distance < minDist) {
				keep = false;
				break;
			}
		}
		if (keep) {
			filtered_circles.push_back(circle);
		}
	}

	return filtered_circles;
}

int main() {
	Mat image = imread("D:/FresherXavisTech/Image/test.tif", IMREAD_GRAYSCALE);
	if (image.empty()) {
		cerr << "Cannot open the image!" << endl;
		return -1;
	}

	Mat blurred;
	GaussianBlur(image, blurred, Size(5, 5), 1.5);

	Ptr<CLAHE> clahe = createCLAHE();
	clahe->setClipLimit(2.0);
	clahe->setTilesGridSize(Size(9, 9));
	Mat claheResult;
	clahe->apply(blurred, claheResult);

	vector<Vec3f> circles = houghCircle(claheResult, 1.0, 10.0, 100.0, 10.0, 1, 200);
	//HoughCircles(claheResult, circles, HOUGH_GRADIENT, 1.0, 20.0, 100.0, 30, 20, 100);

	Mat color_image = imread("D:/FresherXavisTech/Image/test.tif");
	for (const auto& circleE : circles) {
		float x = circleE[0];
		float y = circleE[1];
		float r = circleE[2];
		circle(color_image, Point(static_cast<int>(x), static_cast<int>(y)),
			static_cast<int>(r), Scalar(0, 255, 0), 2);
	}

	imwrite("output.jpg", color_image);
	imshow("Detected Circles", color_image);
	waitKey(0);

	return 0;
}