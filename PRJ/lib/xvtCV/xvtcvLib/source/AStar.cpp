#include "xvtCV/AStar.h"
#include "opencv2/imgproc.hpp"
#include <numeric>
#include "xvtCV/Utils.h"

auto xvt::FindPathByAStar(const cv::Mat& src, int blurSize) -> VecPoint
{
    VecPoint vtLinePoint;
    if (!src.empty())
    {
        cv::Mat dst;
        src.convertTo(dst, CV_64F);
        int ksize = 1;
        int preIdxStart, preIdxEnd;
        int preIdx;
        const int rows = dst.rows;
        const int cols = dst.cols;
        for (int i = 1; i < cols; i++)
        {
            preIdx = i - 1;
            for (int j = 0; j < rows; j++)
            {
                preIdxStart = (std::max<int>)(j - ksize, 0);
                preIdxEnd = (std::min<int>)(j + ksize, rows - 1);
                dst.at<double>(j, i) += (std::max<double>)({ dst.at<double>(preIdxStart, preIdx), dst.at<double>(j, preIdx), dst.at<double>(preIdxEnd, preIdx) });
            }
        }

        // find starting point to tracing back
        const int finalColIdx = cols - 1;
        std::vector<float> vtRow = cv::Mat_<float>(dst.col(finalColIdx));
        auto maxElements = xvt::FindAllMax(vtRow.begin(), vtRow.end(), [](const float& lsh, const float& rsh) { return lsh < rsh; });
        std::vector<int> vtLine(cols, 0);
        int64 maxIdx = 0;
        for (auto p : maxElements)
        {
            maxIdx += std::distance(vtRow.begin(), p);
        }
        vtLine[finalColIdx] = maxIdx / maxElements.size();

        int start, end;
        //ksize = 2;
        double mean;
        for (int i = finalColIdx - 1; i >= 0; i--)
        {
            vtRow = cv::Mat_<float>(dst.col(i));
            auto yIdx = vtLine[i + 1];
            start = (std::max<int>)(0, yIdx - ksize);
            end = (std::min<int>)(rows, yIdx + ksize + 1);

            auto centerValue = vtRow[yIdx];
            auto maxElement = std::max_element(vtRow.begin() + start, vtRow.begin() + end, [](const float& lsh, const float& rsh) { return lsh < rsh; });

            if (*maxElement != centerValue)
                vtLine[i] = std::distance(vtRow.begin(), maxElement);
            else
                vtLine[i] = yIdx;
        }

        if(blurSize>0)
            cv::blur(vtLine, vtLine, cv::Size(blurSize, 1), cv::Point(-1, -1), cv::BORDER_REFLECT);

        if (vtLine.size() == cols)
        {
            for (int i = 0; i < cols; i++)
                vtLinePoint.emplace_back(i, vtLine[i]);
        }
    }

    return vtLinePoint;
}
