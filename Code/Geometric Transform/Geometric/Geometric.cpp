#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

const int NUM_OCTAVES = 1;
const int NUM_SCALES = 5;
const double SIGMA = 1.6;
const int DESCRIPTOR_SIZE = 128;

/**
 * @brief Structure representing a keypoint with SIFT-like descriptor.
 */
struct MyKeyPoint {
    Point2f pt;                ///< Keypoint location (x, y).
    float angle;               ///< Dominant orientation (gradient direction).
    float size;                ///< Keypoint scale.
    vector<float> descriptor;  ///< 128-dimensional SIFT descriptor.
};

/**
 * @brief Build a Gaussian pyramid for an image.
 * @param img Input image (CV_32F).
 * @param numScales Number of scales in the pyramid.
 * @param sigma Initial Gaussian sigma.
 * @return Vector of blurred images at different scales.
 */
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

/**
 * @brief Build a Difference of Gaussian (DoG) pyramid from a Gaussian pyramid.
 * @param gaussians Vector of Gaussian-blurred images.
 * @return Vector of DoG images.
 */
vector<Mat> buildDoGPyramid(const vector<Mat>& gaussians) {
    vector<Mat> dog;
    for (size_t i = 1; i < gaussians.size(); ++i) {
        Mat diff = gaussians[i] - gaussians[i - 1];
        dog.push_back(diff);
    }
    return dog;
}

/**
 * @brief Check if a pixel is a local extremum in its 3x3x3 neighborhood in the DoG pyramid.
 * @param dog DoG pyramid.
 * @param s Scale index.
 * @param y Row index.
 * @param x Column index.
 * @return True if the pixel is a local extremum, false otherwise.
 */
bool isExtrema(const vector<Mat>& dog, int s, int y, int x) {
    float val = dog[s].at<float>(y, x);
    for (int ds = -1; ds <= 1; ++ds) {
        int ss = s + ds;
        if (ss < 0 || ss >= dog.size()) continue;
        for (int dy = -1; dy <= 1; ++dy) {
            int yy = y + dy;
            if (yy < 1 || yy >= dog[ss].rows - 1) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                int xx = x + dx;
                if (xx < 1 || xx >= dog[ss].cols - 1) continue;
                if (ds == 0 && dy == 0 && dx == 0) continue;
                float neighbor = dog[ss].at<float>(yy, xx);
                if ((val >= neighbor && val < 0) || (val <= neighbor && val > 0))
                    return false;
            }
        }
    }
    return true;
}

/**
 * @brief Compute gradient magnitude and orientation for an image.
 * @param img Input image (CV_32F).
 * @param mag Output gradient magnitude (CV_32F).
 * @param angle Output gradient orientation (CV_32F, radians).
 */
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

/**
 * @brief Compute a 128-dimensional SIFT-like descriptor for a keypoint.
 * @param mag Gradient magnitude image.
 * @param angle Gradient orientation image.
 * @param pt Keypoint location.
 * @return 128-dimensional descriptor vector.
 */
vector<float> computeDescriptor(const Mat& mag, const Mat& angle, Point2f pt) {
    vector<float> descriptor(DESCRIPTOR_SIZE, 0.0f);
    int binPerCell = 8;
    int cellSize = 4;

    for (int i = -8; i < 8; ++i) {
        for (int j = -8; j < 8; ++j) {
            int y = pt.y + i;
            int x = pt.x + j;
            if (x < 1 || x >= mag.cols - 1 || y < 1 || y >= mag.rows - 1)
                continue;

            float m = mag.at<float>(y, x);
            float theta = angle.at<float>(y, x);
            if (theta < 0) theta += 2 * CV_PI;
            int bin = static_cast<int>(round((theta * binPerCell) / (2 * CV_PI))) % binPerCell;

            int row = (i + 8) / cellSize;
            int col = (j + 8) / cellSize;
            int index = (row * 4 + col) * binPerCell + bin;

            if (index >= 0 && index < DESCRIPTOR_SIZE)
                descriptor[index] += m;
        }
    }

    // Normalize descriptor
    float norm = 0;
    for (float val : descriptor) norm += val * val;
    norm = sqrt(norm);
    if (norm > 0)
        for (float& val : descriptor) val /= norm;

    return descriptor;
}

/**
 * @brief Detect keypoints in a DoG pyramid and compute their descriptors.
 * @param dog DoG pyramid.
 * @param mag Gradient magnitude image.
 * @param angle Gradient orientation image.
 * @param scaleImg Image at the current scale (not used in this implementation).
 * @return Vector of detected keypoints with descriptors.
 */
vector<MyKeyPoint> findKeypoints(const vector<Mat>& dog, const Mat& mag, const Mat& angle, const Mat& scaleImg) {
    vector<MyKeyPoint> keypoints;
    for (int s = 1; s < dog.size() - 1; ++s) {
        for (int y = 1; y < dog[s].rows - 1; ++y) {
            for (int x = 1; x < dog[s].cols - 1; ++x) {
                if (isExtrema(dog, s, y, x)) {
                    Point2f pt(x, y);
                    float theta = angle.at<float>(y, x);
                    MyKeyPoint kp = { pt, theta, SIGMA * pow(2, s / 2.0) };
                    kp.descriptor = computeDescriptor(mag, angle, pt);
                    keypoints.push_back(kp);
                }
            }
        }
    }
    return keypoints;
}

/**
 * @brief Match keypoints between two images using the ratio test.
 * @param kp1 Keypoints from image 1.
 * @param kp2 Keypoints from image 2.
 * @return Vector of matched keypoint pairs (point in image 1, point in image 2).
 */
vector<pair<Point2f, Point2f>> matchKeypoints(const vector<MyKeyPoint>& kp1, const vector<MyKeyPoint>& kp2) {
    vector<pair<Point2f, Point2f>> matches;
    for (const auto& k1 : kp1) {
        float bestDist = 1e9, secondBest = 1e9;
        Point2f bestPt;
        for (const auto& k2 : kp2) {
            float dist = 0;
            for (int i = 0; i < DESCRIPTOR_SIZE; ++i) {
                float d = k1.descriptor[i] - k2.descriptor[i];
                dist += d * d;
            }
            dist = sqrt(dist);
            if (dist < bestDist) {
                secondBest = bestDist;
                bestDist = dist;
                bestPt = k2.pt;
            }
            else if (dist < secondBest) {
                secondBest = dist;
            }
        }
        if (bestDist < 0.75f * secondBest)
            matches.push_back({ k1.pt, bestPt });
    }
    return matches;
}

/**
 * @brief Estimate homography matrix using RANSAC from matched keypoints.
 * @param matches Vector of matched keypoint pairs.
 * @return 3x3 homography matrix (CV_64F) or empty Mat if estimation fails.
 */
Mat computeHomographyRANSAC(const vector<pair<Point2f, Point2f>>& matches) {
    vector<Point2f> src, dst;
    for (auto& m : matches) {
        src.push_back(m.first);
        dst.push_back(m.second);
    }
    return findHomography(src, dst, RANSAC);
}

/**
 * @brief Stitch two images together using a given homography.
 * @param img1 First input image (CV_32F).
 * @param img2 Second input image (CV_32F).
 * @param H Homography matrix from img1 to img2.
 * @return Stitched image (CV_32F).
 */
Mat stitchImages(const Mat& img1, const Mat& img2, const Mat& H) {
    vector<Point2f> corners_img1 = {
        Point2f(0, 0), Point2f(img1.cols, 0),
        Point2f(img1.cols, img1.rows), Point2f(0, img1.rows)
    };
    vector<Point2f> warped_corners;
    perspectiveTransform(corners_img1, warped_corners, H);

    float min_x = 0, min_y = 0, max_x = img2.cols, max_y = img2.rows;
    for (const auto& pt : warped_corners) {
        min_x = min(min_x, pt.x);
        min_y = min(min_y, pt.y);
        max_x = max(max_x, pt.x);
        max_y = max(max_y, pt.y);
    }

    int offset_x = cvRound(-min_x);
    int offset_y = cvRound(-min_y);
    Mat offset = (Mat_<double>(3, 3) << 1, 0, offset_x, 0, 1, offset_y, 0, 0, 1);
    Mat H_offset = offset * H;

    Size canvas_size(cvRound(max_x - min_x), cvRound(max_y - min_y));
    Mat result;
    warpPerspective(img1, result, H_offset, canvas_size);

    // Copy img2 into the canvas
    for (int y = 0; y < img2.rows; ++y) {
        for (int x = 0; x < img2.cols; ++x) {
            float val = img2.at<float>(y, x);
            if (val > 0)
                result.at<float>(y + offset_y, x + offset_x) = val;
        }
    }

    return result;
}

/**
 * @brief Main function: loads two images, detects and matches keypoints, estimates homography, and stitches images.
 */
int main() {
    Mat img1 = imread("D:/FresherXavisTech/Image/2.png", IMREAD_GRAYSCALE);
    Mat img2 = imread("D:/FresherXavisTech/Image/1.png", IMREAD_GRAYSCALE);
    if (img1.empty() || img2.empty()) {
        cout << "Không thể mở ảnh!" << endl;
        return -1;
    }

    img1.convertTo(img1, CV_32F, 1.0 / 255);
    img2.convertTo(img2, CV_32F, 1.0 / 255);

    auto gp1 = buildGaussianPyramid(img1, NUM_SCALES, SIGMA);
    auto dog1 = buildDoGPyramid(gp1);
    Mat mag1, angle1;
    computeGradient(gp1[2], mag1, angle1);  // middle scale
    auto kp1 = findKeypoints(dog1, mag1, angle1, gp1[2]);

    auto gp2 = buildGaussianPyramid(img2, NUM_SCALES, SIGMA);
    auto dog2 = buildDoGPyramid(gp2);
    Mat mag2, angle2;
    computeGradient(gp2[2], mag2, angle2);
    auto kp2 = findKeypoints(dog2, mag2, angle2, gp2[2]);

    auto matches = matchKeypoints(kp1, kp2);
    cout << "Số cặp match: " << matches.size() << endl;

    Mat H = computeHomographyRANSAC(matches);
    if (H.empty()) {
        cout << "Không tìm được homography!" << endl;
        return -1;
    }

    Mat stitched = stitchImages(img1, img2, H);
    imshow("Kết quả ghép ảnh", stitched);
    waitKey(0);
    return 0;
}
