#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace cv;
using namespace std;

// Init accumulator for Hough Transform
vector<vector<int>> initAccumulator(int width, int height, int theta_steps, int& max_rho) {
    max_rho = static_cast<int>(sqrt(width * width + height * height)) + 1;
    return vector<vector<int>>(2 * max_rho, vector<int>(theta_steps, 0));
}

void houghTransform(const Mat& edge_img, vector<vector<int>>& accumulator, int max_rho, int theta_steps)
{
    const double PI = 3.14159265358979323846;
    for (int y = 0; y < edge_img.rows; ++y) 
    {
        for (int x = 0; x < edge_img.cols; ++x) 
        {
            if (edge_img.at<uchar>(y, x) > 0) 
            {
                for (int t = 0; t < theta_steps; ++t) 
                {
                    double theta = t * PI / theta_steps;
                    int rho = static_cast<int>(x * std::cos(theta) + y * std::sin(theta));
                    //rho may negative so + max_rho to convert range to [0, 2*maxrho]
                    accumulator[rho + max_rho][t]++;
                }
            }
        }
    }
}

vector<pair<int, int>> findLines(vector<vector<int>>& accumulator, int max_rho, int theta_steps, int threshold)
{
    vector<pair<int, int>> lines;
    for (int rho = 0; rho < 2 * max_rho; ++rho)
    {
        for (int theta = 0; theta < theta_steps; ++theta)
        {
            if (accumulator[rho][theta] > threshold)
            {
                lines.emplace_back(rho - max_rho, theta);
            }
        }
    }
    return lines;
}

//accumulator image
void saveAccumulator(vector<vector<int>>& accumulator, int max_rho, int theta_steps, const string& filename)
{
    Mat accum_img(2 * max_rho, theta_steps, CV_8UC1);
    int max_val = 0;
    for (const auto& row : accumulator) {
        for (int val : row) {
            if (val > max_val) max_val = val;
        }
    }
    for (int rho = 0; rho < 2 * max_rho; ++rho) {
        for (int theta = 0; theta < theta_steps; ++theta) {
            accum_img.at<uchar>(rho, theta) = static_cast<uchar>((accumulator[rho][theta] * 255.0) / max_val);
        }
    }
    //resize(accum_img, accum_img, cv::Size(10 * theta_steps, 2 * max_rho), 0, 0, cv::INTER_LINEAR);
    imwrite(filename, accum_img);
}

void drawLines(Mat& img, const std::vector<std::pair<int, int>>& lines, int theta_steps)
{
    const double PI = 3.14159265358979323846;
    for (const auto& linetmp : lines)
    {
        int rho = linetmp.first;
        int theta_idx = linetmp.second;
        double theta = theta_idx * PI / theta_steps;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        Point pt1, pt2;
        double x0 = cos_theta * rho, y0 = sin_theta * rho;

        pt1.x = cvRound(x0 + 1000 * (-sin_theta));
        pt1.y = cvRound(y0 + 1000 * (cos_theta));
        pt2.x = cvRound(x0 - 1000 * (-sin_theta));
        pt2.y = cvRound(y0 - 1000 * (cos_theta));

        line(img, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
    }
}

int main() {
    Mat img = imread("D:/FresherXavisTech/Image/test.tif", IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Cannot read the image!" << std::endl;
        return -1;
    }
    Mat blur1;
    bilateralFilter(img, blur1, -1, 4.0, 30);

    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->setTilesGridSize(Size(9, 9));
    Mat claheResult;
    clahe->apply(blur1, claheResult);

    //Mat blur2;
    //bilateralFilter(claheResult, blur2, -1, 3.0, 30);

    //Mat blured;
    //GaussianBlur(claheResult, blured, Size(5, 5), 4, -1);
    //Mat thresholded;
    //adaptiveThreshold(img, thresholded, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 1.8);

    Mat edges;
    Canny(claheResult, edges, 50, 100, 3);

    int theta_steps = 1800;
    int max_rho;
    auto accumulator = initAccumulator(edges.cols, edges.rows, theta_steps, max_rho);

    houghTransform(edges, accumulator, max_rho, theta_steps);
    //HoughLines(edges, accumulator, max_rho, theta_steps, 100);

    saveAccumulator(accumulator, max_rho, theta_steps, "D:/FresherXavisTech/Image/accumulator.png");

    int threshold = 180; // Điều chỉnh ngưỡng theo nhu cầu
    auto lines = findLines(accumulator, max_rho, theta_steps, threshold);

    Mat color_img = imread("D:/FresherXavisTech/Image/test.tif");
    Mat color_edge;
    cvtColor(edges, color_edge, COLOR_GRAY2BGR);

    drawLines(color_img, lines, theta_steps);
    drawLines(color_edge, lines, theta_steps);

    imwrite("output_lines.jpg", color_img);
    imshow("Detected Lines", color_img);
    imshow("Detected Lines Edge", color_edge);
    imshow("Edges", edges);
    waitKey(0);

    return 0;
}