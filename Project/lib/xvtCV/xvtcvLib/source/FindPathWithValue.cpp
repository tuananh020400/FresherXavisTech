#include "xvtCV/FindPathWithValue.h"
#include "opencv2/imgproc.hpp"
#include <queue>
namespace xvt
{

struct Node {
    cv::Point point; //Current Point
    double distance; //Distance from start point to current point
    int prevIndex; // Index of the previous node in the path

    Node(cv::Point point, double distance, int prevIndex = -1)
        : point(point), distance(distance), prevIndex(prevIndex) {}

    bool operator>(const Node& other) const {
        return distance > other.distance;
    }
};

double euclideanDistance(const cv::Point& p1, const cv::Point& p2) {
    return cv::norm(p1 - p2);
}

std::vector<cv::Point> FindPathWithPixelValue(const cv::Mat& image, const cv::Point& start, const cv::Point& end, 
                                                    std::vector<cv::Point> const& directions , int pixelThreshold)
{
    if (!image.empty())
    {
        cv::Size imageSize = image.size();
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        std::vector<Node> nodes(imageSize.height * imageSize.width, { cv::Point(), std::numeric_limits<double>::infinity(), -1 });

        int startIndex = start.y * imageSize.width + start.x;
        int endIndex = end.y * imageSize.width + end.x;

        pq.push({ start, 0, -1 });
        nodes[startIndex].distance = 0;
        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();

            int currentIndex = current.point.y * imageSize.width + current.point.x;
            if (currentIndex == endIndex) {
                // Reconstruct path
                std::vector<cv::Point> path;
                while (currentIndex != startIndex) {
                    path.push_back(nodes[currentIndex].point);
                    currentIndex = nodes[currentIndex].prevIndex;
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const cv::Point& direction : directions) {
                cv::Point neighborPoint = current.point + direction;
                if (neighborPoint.x >= 0 && neighborPoint.y >= 0 && neighborPoint.x < imageSize.width && neighborPoint.y < imageSize.height) {
                    int neighborIndex = neighborPoint.y * imageSize.width + neighborPoint.x;
                    if (image.at<uchar>(neighborPoint) >= pixelThreshold) {
                        double distance = current.distance + euclideanDistance(current.point, neighborPoint);

                        if (distance < nodes[neighborIndex].distance) {
                            nodes[neighborIndex] = { neighborPoint, distance, currentIndex };
                            pq.push({ neighborPoint, distance, currentIndex });
                        }
                    }
                }
            }
        }
    }
    // No path found
    return {};
}
}