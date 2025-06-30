#include "xvtCV/xvtPen.h"
#include "xvtCV/Utils.h"
#include <opencv2/imgproc.hpp>

namespace xvt {

CVPen::CVPen(cv::Scalar color
             , int thickness
             , double fontScale
             , int space
             , int lineType
             , int fontFace
)
    : mThickness{ thickness }
    , mColor{ color }
    , mFontFace{ fontFace }
    , mFontScale{ fontScale }
    , mLineType{ lineType }
    , mSpace{space}
{
   
}

auto DrawText(cv::Mat& img, std::string const& text, cv::Point p, CVPen const& pen) -> cv::Point
{
    assert(!img.empty());
    std::vector<std::string> lineList;
    std::stringstream ss(text);
    std::string line;
    int maxWidth = 0;

    while (std::getline(ss, line, '\n'))
    {
        lineList.push_back(line);
    }

    for (int i = 0, n = lineList.size(); i < n; i++)
    {
        auto& text = lineList[i];
        auto textSize = GetTextSize(text, pen);
        if (maxWidth < textSize.width) maxWidth = textSize.width;
        p.y += textSize.height;
        if (i > 0) p.y += pen.mSpace;
        cv::putText(img, text, p, pen.mFontFace, pen.mFontScale, pen.mColor, pen.mThickness, pen.mLineType);
    }

    return p + cv::Point(maxWidth, 0);
}

void DrawText(cv::Mat& src, const std::string& text, const cv::Point& point, TextAlign pos, CVPen const& pen, cv::Point offset) {
    if (ConvertRGB(src, src, false) != 0) return;

    // Get the size of the text
    cv::Size textSize = GetTextSize(text, pen);

    // Calculate the starting position for the text to be centered at the given point
    cv::Point textOrg(point.x, point.y);

    //  Position condition
    if (IsAlign<TextAlign::TOP>(pos)) {
        textOrg.y = textOrg.y + textSize.height;
    }
    else if (IsAlign<TextAlign::MIDDLE>(pos)) {
        textOrg.y = textOrg.y + textSize.height / 2;
    }
    
    if (IsAlign<TextAlign::CENTER>(pos)) {
        textOrg.x = textOrg.x - textSize.width / 2;
    }
    else if (IsAlign<TextAlign::RIGHT>(pos)) {
        textOrg.x = textOrg.x - textSize.width;
    }

    //Plus offset
    textOrg += offset;

    // Write the text on the image
    cv::putText(src, text, textOrg, pen.mFontFace, pen.mFontScale, pen.mColor, pen.mThickness, pen.mLineType);
}

void DrawTextList(cv::Mat& src, const std::vector<std::string>& textList, const std::vector<cv::Point>& points, TextAlign pos, CVPen const& pen, cv::Point offset) {
    if (points.empty()) return;

    int sizeTextList = textList.size();
    int sizePoints = points.size();
    std::string text;
    cv::Point point;

    size_t size = (sizeTextList >= sizePoints) ? sizePoints : sizeTextList;
    for (int i = 0; i < size; i++) {
        text = textList[i];
        point = points[i];
        DrawText(src, text, point, pos, pen, offset);
    }
}
}
