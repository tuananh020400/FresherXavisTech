#include "xvtCV/Utils.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <unordered_map>

namespace xvt {

auto ReadImage(std::wstring const& filePath, int flags) -> cv::Mat
{
    std::ifstream f(filePath, std::iostream::binary | std::iostream::ate);
    cv::Mat frame = cv::Mat();

    if (f.is_open())
    {
        auto fileSize = f.tellg();// get current file pointer
        f.seekg(0, f.beg);// seek back to beginning of file
        std::vector<char> buffer(fileSize);
        if (f.read(buffer.data(), fileSize))
        {
            frame = cv::imdecode(buffer, flags);
        }
    }
    f.close();

    return frame;
}

auto ReadImageRaw(std::wstring const& filePath, int w, int h, int type) -> cv::Mat
{
    std::ifstream f(filePath, std::iostream::binary | std::iostream::ate);
    cv::Mat img;
    if (f.is_open())
    {
        auto fileSize = f.tellg();// get current file pointer
        f.seekg(0, f.beg);// seek back to beginning of file

        bool isRead = fileSize > 0;
        if (isRead)
        {
            img = cv::Mat(h, w, type);
            auto imgSize = img.total() * img.elemSize();

            char* buffer = (char*)img.data;
            f.read(buffer, imgSize);

            isRead = f.rdstate() == std::ios_base::goodbit;
        }
    }
    f.close();
    return img;
}

auto ReadImageRaw(std::wstring const& filePath, int n, int w, int h, int type) ->std::vector<cv::Mat>
{
    std::ifstream f(filePath, std::iostream::binary | std::iostream::ate);
    std::vector<cv::Mat> vec;
    if (f.is_open())
    {
        vec = std::vector<cv::Mat>(n);
        auto fileSize = f.tellg();// get current file pointer
        f.seekg(0, f.beg);// seek back to beginning of file

        bool isRead = fileSize > 0;
        int count = 0;
        for (auto & img : vec)
        {
            img = cv::Mat(h, w, type);
            auto imgSize = img.total() * img.elemSize();

            char* buffer = (char*)img.data;
            f.read(buffer, imgSize);

            isRead = f.rdstate() == std::ios_base::goodbit;
            count++;
            if (!isRead)
            {
                if (count < n)
                {
                    vec.erase(vec.begin() + count, vec.end());
                }
                break;
            }
        }
    }
    f.close();
    return vec;
}

void WriteImage(std::wstring const& filePath, cv::Mat const& image)
{
    if (image.empty()) return;
    auto const p = filePath.find_last_of('.');
    std::string ext = p > 0 && p != std::wstring::npos ? std::string(filePath.begin() + p, filePath.end()) : ".jpg";
    std::ofstream f(filePath, std::iostream::binary | std::iostream::out);
    if (f.is_open())
    {
        // Put it in a vector
        size_t len = image.total() * image.elemSize();
        std::vector<uchar> buffer;
        buffer.reserve(len);

        // Decode the vector
        cv::imencode(ext, image, buffer);
        f.write((char*)&buffer[0], buffer.size());
        buffer.clear();
    }
    f.close();
}

int Convert8Bits(const cv::Mat& src, cv::Mat& dst, bool isClone)
{
    int rtn = 0;

    if (!src.empty())
    {
        int type = src.type();
        switch (type)
        {
        case CV_8UC1:
            dst = isClone ? src.clone() : src;
            break;
        case CV_8UC3:
            cv::cvtColor(src, dst, cv::ColorConversionCodes::COLOR_RGB2GRAY);
            break;
        case CV_16UC1:
            src.convertTo(dst, CV_8UC1, 1.0 / 257.0);
            //cv::normalize(src, dst, 0, 255, cv::NORM_L2, CV_8UC1);
            break;
        case CV_32FC1:
            cv::normalize(src, dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            break;
        default:
            rtn = 2;
            break;
        }

    }
    else
    {
        rtn = 1;
    }

    return rtn;
}

//Convert to RGB images
int ConvertRGB(const cv::Mat& src, cv::Mat& dst, bool isClone)
{
    int rtn = 0;
    if (!src.empty()) {
        int type = src.type();
        switch (type)
        {
        case CV_8UC1:
            cv::cvtColor(src, dst, cv::COLOR_GRAY2RGB);
            break;
        case CV_16UC1:
            src.convertTo(dst, CV_8UC1, 1.0 / 257.0);
            cv::cvtColor(dst, dst, cv::COLOR_GRAY2RGB);
            break;
        case CV_32FC1:
            cv::normalize(src, dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::cvtColor(dst, dst, cv::COLOR_GRAY2RGB);
            break;
        case CV_8UC3:
            dst = isClone ? src.clone() : src;
            break;
        case CV_8UC4:
            cv::cvtColor(src, dst, cv::COLOR_RGBA2RGB);
        default:
            rtn = 2;
            break;
        }
    }
    else {
        rtn = 1;
    }
    return rtn;
}

bool LexicoCompare(const cv::Point& p1, const cv::Point& p2) {
    if (p1.x < p2.x) { return true; }
    if (p1.x > p2.x) { return false; }
    return (p1.y < p2.y);
}

auto Sobel(const cv::Mat& inputImage, float wx, float wy) -> cv::Mat
{
    cv::Mat edgeImage;
    if (!inputImage.empty())
    {
        double w = (double)wx + wy;
        wx /= w;
        wy /= w;
        // Apply the Sobel edge detection
        cv::Mat gradientX, gradientY;
        cv::Sobel(inputImage, gradientX, CV_16S, 1, 0);
        cv::Sobel(inputImage, gradientY, CV_16S, 0, 1);

        // Convert the gradients to absolute values and combine them
        cv::convertScaleAbs(gradientX, gradientX);
        cv::convertScaleAbs(gradientY, gradientY);
        cv::addWeighted(gradientX, wx, gradientY, wy, 0, edgeImage);
    }
    return edgeImage;
}

auto Sobel(const cv::Mat& inputImage, float wx, float wy, int kSize, int order) -> cv::Mat
{
    cv::Mat edgeImage;
    double w = (double)wx + wy;
    wx /= w;
    wy /= w;
    if (!inputImage.empty())
    {
        // Apply the Sobel edge detection
        cv::Mat gradientX, gradientY;
        cv::Sobel(inputImage, gradientX, CV_64F, order, 0, kSize);
        cv::Sobel(inputImage, gradientY, CV_64F, 0, order, kSize);

        // Convert the gradients to absolute values and combine them
        gradientX = cv::abs(gradientX);
        gradientY = cv::abs(gradientY);
        cv::addWeighted(gradientX, wx, gradientY, wy, 0, edgeImage);

    }
    return edgeImage;
}

void RemoveDuplicatePoint(std::vector<cv::Point>& points) {
    // Note: std::unique leaves a 'queue' of duplicated elements
    // at the end of the vector, and returns an iterator that indicates
    // where to stop (and where to 'erase' the queue)
    std::sort(points.begin(), points.end(), LexicoCompare);
    points.erase(std::unique(points.begin(), points.end(), [](const cv::Point& p1, const cv::Point& p2) {
        return p1 == p2;
        }), points.end());
}

struct PointHash {
    std::size_t operator()(const cv::Point& point) const {
        std::size_t h1 = std::hash<int>{}(point.x);
        std::size_t h2 = std::hash<int>{}(point.y);
        return h1 ^ (h2 << 1);
    }
};

struct PointEqual {
    bool operator()(const cv::Point& p1, const cv::Point& p2) const {
        return p1.x == p2.x && p1.y == p2.y;
    }
};

std::vector<cv::Point> FindPointsWithOneNeighbor(const std::vector<cv::Point>& points, const std::vector<cv::Point>& directions) {
    std::vector<cv::Point> result;
    std::unordered_map<cv::Point, int, PointHash, PointEqual> pointCount;
    // Count the neighbors for each point
    for (const auto& point : points)
    {
        for (auto const& d: directions)
        {
            auto neighbor = point + d;
            pointCount[neighbor]++;
        }
    }

    // Add points with only one neighbor to the result
    for (const auto& point : points)
    {
        if (pointCount[point] == 1)
        {
            result.push_back(point);
        }
    }

    return result;
}
}