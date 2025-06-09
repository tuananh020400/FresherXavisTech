#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

float calculateMean(const Mat& mat, int startX, int startY, int width, int height)
{
	float sum = 0.0f;
	int count = 0;
	for (int y = startY; y < startY + height; ++y) {
		for (int x = startX; x < startX + width; ++x) {
			sum += mat.at<uchar>(y, x);
			count++;
		}
	}
	return sum / count;
}

float calculateNCC(const Mat& image, const Mat& templ, int x, int y)
{
	// Mean intensity of template
	float meanTempl = calculateMean(templ, 0, 0, templ.cols, templ.rows);
	// Mean intensity of subregion in original image
	float meanImage = calculateMean(image, x, y, templ.cols, templ.rows);

	// Calculate numerator and denominators for NCC
	double numerator = 0.0;
	double sumTemplSquared = 0.0;
	double sumImageSquared = 0.0;

	for (int ty = 0; ty < templ.rows; ++ty) {
		for (int tx = 0; tx < templ.cols; ++tx) {
			float templVal = templ.at<uchar>(ty, tx) - meanTempl;
			float imageVal = image.at<uchar>(y + ty, x + tx) - meanImage;
			numerator += templVal * imageVal;
			sumTemplSquared += templVal * templVal;
			sumImageSquared += imageVal * imageVal;
		}
	}

	double denominator = sqrt(sumTemplSquared * sumImageSquared);

	return static_cast<float>(numerator / denominator);
}

void manualTemplateMatch(const Mat& image, const Mat& templ, Mat& result, int method)
{
	if (image.empty() || templ.empty()) {
		cout << "Source image or template is empty!" << endl;
		return;
	}
	if (templ.rows > image.rows || templ.cols > image.cols) {
		cout << "Template is larger than source image!" << endl;
		return;
	}
	if (image.type() != CV_8U || templ.type() != CV_8U) {
		cout << "Source image and template must be grayscale (CV_8U)!" << endl;
		return;
	}

	// Initialize result matrix
	int result_cols = image.cols - templ.cols + 1;
	int result_rows = image.rows - templ.rows + 1;
	result.create(result_rows, result_cols, CV_32F);

	// Compute NCC for each sliding position
	for (int y = 0; y < result_rows; ++y) {
		for (int x = 0; x < result_cols; ++x) {
			result.at<float>(y, x) = calculateNCC(image, templ, x, y); // Assign NCC value
		}
	}
}

//Find all matching locations above a threshold
vector<Point> findMatches(const Mat& result, float threshold)
{
	vector<Point> matches;
	for (int y = 0; y < result.rows; ++y) {
		for (int x = 0; x < result.cols; ++x) {
			if (result.at<float>(y, x) >= threshold) {
				matches.push_back(Point(x, y));
			}
		}
	}
	return matches;
}

vector<Point> nonMaximumSuppression(const vector<Point>& matches, const Mat& result, int distanceThreshold)
{
	vector<Point> filteredMatch;

	vector<pair<Point, float>> matchScores;
	for (const auto& match : matches)
	{
		float score = result.at<float>(match.y, match.x);
		matchScores.push_back({ match, score });
	}

	sort(matchScores.begin(), matchScores.end(),
		[](const pair<Point, float>& a, const pair<Point, float>& b)
		{
			return a.second > b.second;
		});

	while (!matchScores.empty())
	{
		Point bestMatch = matchScores[0].first;
		filteredMatch.push_back(bestMatch);

		//Remove matches that are too close to the best match
		vector<pair<Point, float>> remainingMatches;
		for (const auto& match : matchScores)
		{
			double distance = norm(match.first - bestMatch);
			if (distance >= distanceThreshold) {
				remainingMatches.push_back(match); // Keep if far enough
			}
		}
		matchScores = remainingMatches;
	}

	return filteredMatch;
}

int main() {
	// Load source image and template
	Mat image = imread("D:/FresherXavisTech/Image/Imagebinary_image_LE_upscale_balanced_x4.jpg", IMREAD_GRAYSCALE);
	Mat templ = imread("D:/FresherXavisTech/Image/Imagebinary_image_LE_upscale_balanced_x4 - Copy.jpg", IMREAD_GRAYSCALE);

	// Check if images were loaded successfully
	if (image.empty() || templ.empty()) {
		cout << "Failed to load source image or template!" << endl;
		return -1;
	}

	// Create result matrix and perform template matching
	Mat result;
	matchTemplate(image, templ, result, TM_CCOEFF_NORMED);

	float threshold = 0.65f;
	vector<Point> matches = findMatches(result, threshold);

	int distanceThreshold = max(templ.rows, templ.cols);
	vector<Point> filteredMatches = nonMaximumSuppression(matches, result, distanceThreshold);

	// Draw rectangles around all filtered matched regions
	Mat display_image = imread("D:/FresherXavisTech/Image/Imagebinary_image_LE_upscale_balanced_x4.jpg");
	for (const auto& matchLoc : filteredMatches)
	{
		rectangle(display_image, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
			Scalar(0, 255, 0), 2); // Green color, thickness 2
	}

	// Display the number of matches found
	cout << "Found " << filteredMatches.size() << " matches with correlation >= " << threshold << endl;

	// Display the result
	imshow("Kết quả Template Matching", display_image);
	waitKey(0);
	destroyAllWindows();

	return 0;
}