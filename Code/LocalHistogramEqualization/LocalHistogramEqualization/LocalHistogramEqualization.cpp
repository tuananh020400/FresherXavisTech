#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Basic Local Histogram Equalization
Mat localHistEqualBlock(const Mat& src, int blockSize) {
    Mat dst = src.clone();
    Mat padded;
    int border = blockSize / 2;
    copyMakeBorder(src, padded, border, border, border, border, BORDER_REFLECT);

    for (int y = 0; y < src.rows; y += blockSize)
    {
        for (int x = 0; x < src.cols; x += blockSize)
        {
            Rect roi(x, y,
                min(blockSize, src.cols - x),
                min(blockSize, src.rows - y));
            Mat block = src(roi);
            Mat equalizedBlock;
            equalizeHist(block, equalizedBlock);
            equalizedBlock.copyTo(dst(roi));
        }
    }
    return dst;
}

// Local Histogram Equalization using sliding window
Mat localHistEqualSliding(const Mat& src, int WindowSize) {
    Mat dst = src.clone();
    Mat padded;
    int padding = WindowSize / 2;

    copyMakeBorder(src, padded, padding, padding, padding, padding, BORDER_REFLECT);

    for (int y = 0; y < src.rows; ++y)
    {
        for (int x = 0; x < src.cols; ++x)
        {
            // Cửa sổ xung quanh điểm (x,y)
            Rect roi(x, y, WindowSize, WindowSize);
            Mat window = padded(roi);

            // Tính histogram equalization cho vùng cửa sổ
            Mat eqWindow;
            equalizeHist(window, eqWindow);

            // Cập nhật lại pixel tại tâm
            dst.at<uchar>(y, x) = eqWindow.at<uchar>(padding,padding);
        }
    }
    return dst;
}

// CLAHE
Mat applyCLAHE(const Mat& src, double clipLimit, Size tileGridSize) {
    Ptr<CLAHE> clahe = createCLAHE(clipLimit, tileGridSize);
    Mat dst;
    clahe->apply(src, dst);
    return dst;
}

// Histogram Equalization a tile
void calcHistogram(const Mat& tile, vector<int>& hist) {
    for (int i = 0; i < tile.rows; ++i) {
        for (int j = 0; j < tile.cols; ++j) {
            int pixelValue = tile.at<uchar>(i, j);
            hist[pixelValue]++;
        }
    }
}

// Clip Histogram
void clipHistogram(vector<int>& hist, int clipLimit)
{
    int totalPixels = 0;
    int excess = 0;

    // Tính totalPixels
    for (int i = 0; i < hist.size(); ++i) {
        totalPixels += hist[i];
    }

    int maxAllowed = static_cast<int>(clipLimit * totalPixels / 256);

    // Clipping các giá trị quá mức
    for (int i = 0; i < hist.size(); ++i)
    {
        if (hist[i] > maxAllowed) 
        {
            excess += hist[i] - maxAllowed;
            hist[i] = maxAllowed;
        }
    }
    // Redistribute excess
    int bonus = excess / 256;
    int remain = excess % 256;
    for (int k = 0; k < 256; ++k)
        hist[k] += bonus;
    for (int k = 0; k < remain; ++k)
        hist[k]++;
}

// Equalization Histogram a tile
void equalizeHistogram(vector<int>& hist, Mat& tile)
{
    int totalPixels = tile.rows * tile.cols;

    vector<int> cdf(hist.size(), 0);
    cdf[0] = hist[0];
    for (int i = 1; i < hist.size(); ++i) {
        cdf[i] = cdf[i - 1] + hist[i];
    }

    // Normalize the CDF
    int minCDF = *min_element(cdf.begin(), cdf.end());
    for (int i = 0; i < hist.size(); ++i) {
        cdf[i] = (cdf[i] - minCDF) * 255 / (totalPixels - minCDF);
    }

    // Áp dụng CDF cho từng pixel trong tile
    for (int i = 0; i < tile.rows; ++i) {
        for (int j = 0; j < tile.cols; ++j) {
            int pixelValue = tile.at<uchar>(i, j);
            tile.at<uchar>(i, j) = cdf[pixelValue];
        }
    }
}

// Manual CLAHE not using Bilinear
Mat ManualCLAHE(const Mat& img, int tileSize, int clipLimit)
{
    int rows = img.rows;
    int cols = img.cols;
    Mat result = img.clone();
    // Tránh chỉnh sửa img qua img(roi)
    Mat temp = img.clone();
    for (int i = 0; i < rows; i += tileSize)
    {
        for (int j = 0; j < cols; j += tileSize)
        {
            Rect roi(j, i, min(tileSize, cols - j), min(tileSize, rows - i));
            Mat tile = temp(roi);

            // Tính toán histogram cho tile
            vector<int> hist(256, 0);
            calcHistogram(tile, hist);

            //ClipHistogram
            clipHistogram(hist, clipLimit);

            // Histogram equalization cho tile
            equalizeHistogram(hist, tile);
            
            tile.copyTo(result(roi));
        }
    }

    return result;
}

// Manual CLAHE using Bilinear
Mat ManualCLAHE_Bilinear(const Mat& src, int tileSize, double clipLimit)
{
    Mat dst = src.clone();
    int rows = src.rows;
    int cols = src.cols;
    int nTilesY = (rows + tileSize - 1) / tileSize;
    int nTilesX = (cols + tileSize - 1) / tileSize;

    // Precompute LUTs cho mỗi tile
    vector<vector<vector<uchar>>> LUTs(nTilesY, vector<vector<uchar>>(nTilesX, vector<uchar>(256, 0)));

    for (int i = 0; i < nTilesY; ++i)
    {
        for (int j = 0; j < nTilesX; ++j)
        {
            int y0 = i * tileSize;
            int x0 = j * tileSize;
            int h = min(tileSize, rows - y0);
            int w = min(tileSize, cols - x0);
            int totalPixels = w * h;
            Rect roi(x0, y0, w, h);
            Mat tile = src(roi);

            // Histogram
            vector<int> hist(256, 0);
            calcHistogram(tile, hist);

            // Clip Histogram
            clipHistogram(hist, clipLimit);

            // CDF
            vector<int> cdf(256, 0);
            cdf[0] = hist[0];
            for (int k = 1; k < 256; ++k)
                cdf[k] = cdf[k - 1] + hist[k];

            int minCDF = 0;
            for (int k = 0; k < 256; ++k)
            {
                if (cdf[k] != 0)
                {
                    minCDF = cdf[k];
                    break;
                }
            }

            for (int k = 0; k < 256; ++k)
            {
                LUTs[i][j][k] = saturate_cast<uchar>((cdf[k] - minCDF) * 255 / max(1, totalPixels - minCDF));
            }
        }
    }

    // Apply bilinear interpolation
    for (int y = 0; y < rows; ++y)
    {
        float gy = (float)y / tileSize;
        int y1 = static_cast<int>(floor(gy));
        int y2 = min(y1 + 1, nTilesY - 1);
        float dy = gy - y1;
        y1 = min(y1, nTilesY - 1);

        for (int x = 0; x < cols; ++x)
        {
            float gx = (float)x / tileSize;
            int x1 = static_cast<int>(floor(gx));
            int x2 = min(x1 + 1, nTilesX - 1);
            float dx = gx - x1;
            x1 = min(x1, nTilesX - 1);

            int val = src.at<uchar>(y, x);

            float tl = LUTs[y1][x1][val];
            float tr = LUTs[y1][x2][val];
            float bl = LUTs[y2][x1][val];
            float br = LUTs[y2][x2][val];

            float top = tl * (1.0f - dx) + tr * dx;
            float bottom = bl * (1.0f - dx) + br * dx;
            float interpolated = top * (1.0f - dy) + bottom * dy;

            dst.at<uchar>(y, x) = static_cast<uchar>(round(interpolated));
        }
    }

    return dst;
}


int main() {

    // Original Image
    Mat img = imread("D:/FresherXavisTech/Image/Image2.jpg", IMREAD_GRAYSCALE);
    if (img.empty()) {
        cout << "Can not open the image!" << endl;
        return -1;
    }
    imshow("Original Image", img);

    // Histogram Equalization using OpenCV library
    Mat eqlHist = img.clone();
    equalizeHist(img, eqlHist);
    imshow("HE OpenCV", eqlHist);

    // Basic Local Histogram Equalization
    Mat blockEqualized = localHistEqualBlock(img, 60);
    imshow("Block Equalization", blockEqualized);

    // Local Histogram Equalization using sliding window
    Mat slidingEqualized = localHistEqualSliding(img, 15);
    imshow("Sliding Window Equalization", slidingEqualized);

    //CLAHE
    Mat claheImg = applyCLAHE(img, 2.0, Size(8, 8));
    imshow("CLAHE", claheImg);

    Mat CLAHEimg = ManualCLAHE(img, 32, 4.0);
    imshow("Manual CLAHE", CLAHEimg);

    Mat CLAHEImgBil = ManualCLAHE_Bilinear(img, 32, 3.0f);
    imshow("Manual CLAHE Bilinear", CLAHEImgBil);

    waitKey(0);
    return 0;
}
