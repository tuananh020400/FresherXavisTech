#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;
using namespace cv;

void logTransformation(Mat& input, Mat& output)
{
	output = input.clone();
	double c = 255.0 / log(1 + 255.0);

	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			output.at<uchar>(i, j) = static_cast<uchar>(c * log(1 + input.at<uchar>(i, j)));
		}
	}
}

void negativeTransformation(Mat& input, Mat& output)
{
	output = input.clone();

	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			output.at<uchar>(i, j) = 255 - input.at<uchar>(i, j);
		}
	}
}

void gammaTransformation(Mat& input, Mat& output, float gamma)
{
	//double c = 255 / pow(255, gamma);
	double c = 1;
	output = input.clone();

	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			uchar pixel = input.at<uchar>(i, j);
			output.at<uchar>(i, j) = static_cast<uchar>(c * std::pow(pixel, gamma));
		}
	}
}

int main(void)
{
	Mat image = imread("C:/Users/tuana/Downloads/Screenshot 2025-05-07 100901.png", IMREAD_GRAYSCALE);
	if (image.empty())
	{
		cout << "Could not open or find the image!" << endl;
		return -1;
	}

	imshow("Original Image", image);

	Mat imgLog;
	logTransformation(image, imgLog);
	imshow("Log Tranformation Image", imgLog);

	//Mat imgNeg;
	//negativeTransformation(image, imgNeg);
	//imshow("Negative Transformation", imgNeg);

	//Mat imgGam;
	//gammaTransformation(image, imgGam, 0.7);
	//imshow("Gamma Transformation", imgGam);

	//imwrite("Log Tranformation Image", imgLog);

	waitKey(0);
}