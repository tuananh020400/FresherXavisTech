#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

const int NUM_OCTAVES = 1;     // đơn giản, chỉ xử lý 1 octave
const int NUM_SCALES = 5;
const double SIGMA = 1.6;
const int DESCRIPTOR_SIZE = 128;

struct MyKeyPoint {
    Point2f pt;
    float angle;
    float size;
    vector<float> descriptor; // 128 chiều
};

// Gaussian Pyramid
vector<Mat> buildGaussianPyramid(const Mat& img, int numScales, double sigma) {
    vector<Mat> pyramid;
    for (int i = 0; i < numScales; ++i) {
        double curr_sigma = sigma * pow(2.0, i / 2.0);
        Mat blurred;
        GaussianBlur(img, blurred, Size(0, 0), curr_sigma);
        pyramid.push_back(blurred);
    }
    return pyramid;
}

// DoG Pyramid
vector<Mat> buildDoGPyramid(const vector<Mat>& gaussians) {
    vector<Mat> dog;
    for (size_t i = 1; i < gaussians.size(); ++i) {
        Mat diff = gaussians[i] - gaussians[i - 1];
        dog.push_back(diff);
    }
    return dog;
}

// So sánh 26 điểm lân cận
bool isExtrema(const vector<Mat>& dog, int s, int y, int x) {
    float val = dog[s].at<float>(y, x);
    for (int ds = -1; ds <= 1; ++ds) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (ds == 0 && dy == 0 && dx == 0) continue;
                float neighbor = dog[s + ds].at<float>(y + dy, x + dx);
                if ((val >= neighbor && val < 0) || (val <= neighbor && val > 0))
                    return false;
            }
        }
    }
    return true;
}

// Gradient magnitude + angle
void computeGradient(const Mat& img, Mat& mag, Mat& angle) {
    mag = Mat(img.size(), CV_32F);
    angle = Mat(img.size(), CV_32F);
    for (int y = 1; y < img.rows - 1; ++y) {
        for (int x = 1; x < img.cols - 1; ++x) {
            float dx = img.at<float>(y, x + 1) - img.at<float>(y, x - 1);
            float dy = img.at<float>(y - 1, x) - img.at<float>(y + 1, x);
            mag.at<float>(y, x) = sqrt(dx * dx + dy * dy);
            angle.at<float>(y, x) = atan2(dy, dx);
        }
    }
}

// Histogram 8 hướng quanh keypoint (4x4 vùng)
vector<float> computeDescriptor(const Mat& mag, const Mat& angle, Point2f pt) {
    vector<float> descriptor(DESCRIPTOR_SIZE, 0.0f);
    int size = 16;
    int binPerCell = 8;
    int cellSize = 4;

    for (int i = -8; i < 8; ++i) {
        for (int j = -8; j < 8; ++j) {
            int y = pt.y + i;
            int x = pt.x + j;
            if (x < 1 || x >= mag.cols - 1 || y < 1 || y >= mag.rows - 1)
                continue;

            float m = mag.at<float>(y, x);
            float theta = angle.at<float>(y, x); // rad

            if (theta < 0) theta += 2 * CV_PI;
            int bin = static_cast<int>(round((theta * binPerCell) / (2 * CV_PI))) % binPerCell;

            int row = (i + 8) / cellSize;
            int col = (j + 8) / cellSize;
            int index = (row * 4 + col) * binPerCell + bin;

            if (index >= 0 && index < DESCRIPTOR_SIZE)
                descriptor[index] += m;
        }
    }

    // Chuẩn hóa
    float norm = 0;
    for (float val : descriptor) norm += val * val;
    norm = sqrt(norm);
    if (norm > 0)
        for (float& val : descriptor) val /= norm;

    return descriptor;
}

// Main
int main() {
    Mat img = imread("D:/FresherXavisTech/Image/1.jpg", IMREAD_GRAYSCALE);
    if (img.empty()) return -1;

    img.convertTo(img, CV_32F, 1.0 / 255.0);

    auto gaussianPyramid = buildGaussianPyramid(img, NUM_SCALES, SIGMA);
    auto dogPyramid = buildDoGPyramid(gaussianPyramid);

    vector<MyKeyPoint> myKeypoints;

    for (int s = 1; s < dogPyramid.size() - 1; ++s) {
        for (int y = 1; y < img.rows - 1; ++y) {
            for (int x = 1; x < img.cols - 1; ++x) {
                if (isExtrema(dogPyramid, s, y, x)) {
                    MyKeyPoint kp;
                    kp.pt = Point2f(x, y);
                    kp.size = SIGMA * pow(2.0, s / 2.0);
                    myKeypoints.push_back(kp);
                }
            }
        }
    }

    // Gradient + descriptor
    Mat gradMag, gradAngle;
    computeGradient(img, gradMag, gradAngle);
    for (auto& kp : myKeypoints) {
        kp.descriptor = computeDescriptor(gradMag, gradAngle, kp.pt);
    }

    // Vẽ điểm đặc trưng
    Mat vis;
    img.convertTo(vis, CV_8U, 255.0);
    cvtColor(vis, vis, COLOR_GRAY2BGR);
    for (auto& kp : myKeypoints) {
        circle(vis, kp.pt, 2, Scalar(0, 0, 255), -1);
    }
    imshow("Keypoints", vis);
    waitKey(0);
    return 0;
}