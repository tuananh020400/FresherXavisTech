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
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
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
    Mat img = imread("D:/FresherXavisTech/Image/Resultbinary_image_output.png", IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Không thể đọc ảnh!" << std::endl;
        return -1;
    }

    Mat edges;
    Canny(img, edges, 50, 150, 3);

    int theta_steps = 180;
    int max_rho;
    auto accumulator = initAccumulator(edges.cols, edges.rows, theta_steps, max_rho);

    houghTransform(edges, accumulator, max_rho, theta_steps);

    saveAccumulator(accumulator, max_rho, theta_steps, "D:/FresherXavisTech/Image/accumulator.png");

    int threshold = 100; // Điều chỉnh ngưỡng theo nhu cầu
    auto lines = findLines(accumulator, max_rho, theta_steps, threshold);

    Mat color_img;
    cvtColor(edges, color_img, COLOR_GRAY2BGR);

    drawLines(color_img, lines, theta_steps);

    imwrite("output_lines.jpg", color_img);
    imshow("Detected Lines", color_img);
    imshow("Edges", edges);
    waitKey(0);

    return 0;
}