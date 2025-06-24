#include "xvtCV/Drawing.h"
#include "xvtCV/Utils.h"
#include "xvtCV/ColorDefine.h"

#pragma warning(push, 0)
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#pragma warning(pop)

#include <iomanip>
#include <numeric>

namespace xvt {

Drawing::Drawing()
{
    ResetDefault();
}

void Drawing::Plot(cv::Mat& img, const std::vector<cv::Point2f>& points, const bool isUpdateScale)
{
    std::vector<float> xdata;
    std::vector<float> ydata;
    for (cv::Point2f p : points)
    {
        xdata.push_back(p.x);
        ydata.push_back(p.y);
    }
    Plot(img, xdata, ydata, isUpdateScale);
}

void Drawing::Plot(cv::Mat& img, const std::vector<cv::Point>& points, const bool isUpdateScale)
{
    std::vector<int> xdata;
    std::vector<int> ydata;
    for (cv::Point p : points)
    {
        xdata.push_back(p.x);
        ydata.push_back(p.y);
    }
    Plot(img, xdata, ydata, isUpdateScale);
}

void Drawing::Plot(cv::Mat& img, const cv::Point2f& p)
{
    if (img.empty()) return;
    if (img.rows <= 2 * padding_X || img.cols <= 2 * padding_X) throw std::invalid_argument("ERROR: img.rows <= 2 * padding_X || img.cols <= 2 * padding_X");
    if (img.channels() == 1)
        cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
    else if (img.channels() != 3) throw std::invalid_argument("Only support cv::Mat channle=1 || channel=3");

    auto p1 = GetDrawPoint(p.x, p.y);

    int symbolSize = thickness * 2;
    switch (type)
    {
        case DrawType::Circle:
            cv::circle(img, p1, symbolSize, color, thickness);
            break;
        case DrawType::Star:
            //break;
        case DrawType::Caster:
            //break;
        case DrawType::Line:
            //break;
        case DrawType::Plus:
            cv::line(img, cv::Point(p1.x - symbolSize, p1.y), cv::Point(p1.x + symbolSize, p1.y), color, thickness, mLineType);
            cv::line(img, cv::Point(p1.x, p1.y - symbolSize), cv::Point(p1.x, p1.y + symbolSize), color, thickness, mLineType);
            break;
        case DrawType::Pixel:
        default:
            // Default draw pixel by pixel
            if (p1.x >= 0 && p1.x < img.cols && p1.y >= 0 && p1.y < img.rows)
                img.at<cv::Vec3b>(p1) = cv::Vec3b(color[0], color[1], color[2]);
            break;
    }
}

void Drawing::Plot(cv::Mat& img, const cv::Point2f& p, const cv::Scalar& c)
{
    cv::Scalar oldColor = color;
    color = c;
    Plot(img, p);
    color = oldColor;
}

void Drawing::Plot(cv::Mat& img, std::string const& text, cv::Point const& p, float fontScale, const bool isDrawPoint)
{
    int font = cv::FONT_HERSHEY_PLAIN;
    //int font = cv::FONT_HERSHEY_COMPLEX;

    auto p1 = isDrawPoint ? p : GetDrawPoint(p.x, p.y);
    auto textSize = cv::getTextSize(text, font, fontScale, thickness, 0);

    cv::Point textPos;
    textPos.x = p1.x + textSize.width < img.cols ? p1.x : p1.x - textSize.width;
    textPos.y = p1.y - textSize.height > 0 ? p1.y : p1.y + textSize.height;

    cv::putText(img, text, textPos, font, fontScale, color, thickness);
}

void Drawing::Plot(cv::Mat& img, cv::Rect const& rect)
{
    bool rtn = cvt2BGR(img);
    if (rtn)
    {
        cv::rectangle(img, rect, color, thickness);
    }
}

void Drawing::Plot(cv::Mat& img, cv::Rect const& rect, cv::Scalar const& c)
{
    if (cvt2BGR(img))
    {
        cv::rectangle(img, rect, c, thickness);
    }
}

void Drawing::Plot(cv::Mat& img, std::vector<cv::Rect> const& rectList, cv::Scalar const& c)
{
    if (cvt2BGR(img))
    {
        for (auto& rect : rectList)
        {
            cv::rectangle(img, rect, c, thickness);
        }
    }
}

inline void Drawing::ResetDefault()
{
    type = DrawType::Line;
    thickness = 1;
    color = COLOR_CV_YELLOW;
    yTopOrigin = true;
    scale_X = 1;
    scale_Y = 1;
    offSet_X = 0;
    offSet_Y = 0;
    padding_X = 0;
    padding_Y = 0;
    //offset = cv::Point2f(0.0, 0.0);
}

float GetRullerResolution(float length)
{
    int n = log(length) / log(10);
    constexpr float resolution = 10.0f;
    length = (int)(length / 100) * 100;
    float oneReValue = length / resolution;
    return oneReValue;
}

// Start point is draw point in the image
void Drawing::DrawRuler(cv::Mat& img, cv::Point start, cv::Point end)
{
    //img = cv::Mat (30, 256, CV_8UC3, COLOR_CV_GRAY(255));
    cv::Size imgSize = img.size();
    cv::Point2d direction = end - start;
    auto stepWorldLength1 = GetLength(cv::Point2d(direction.x / scale_X, direction.y / scale_Y));
    const float length = GetLength(direction);
    direction /= length;

    cv::Point startWorld = GetWorldPoint(start);
    cv::Point endWorld = GetWorldPoint(end);

    cv::line(img, start, end, COLOR_CV_BLUE, 1);
    auto reso = GetRullerResolution(length);
    float a = round(length / reso);

    constexpr float fontScale = 0.28;
    constexpr int   fontWeight = 1;
    constexpr int   makerSize = 5;
    cv::Point2d p;
    cv::Point2d initPoint = start;
    auto step = reso * direction;
    auto stepWorldLength = GetLength(cv::Point2d(step.x / scale_X, step.y / scale_Y));
    float startRuler = (direction.x * direction.y == 0) ? startWorld.x * abs(direction.x) + startWorld.y * abs(direction.y) : 0;
    cv::Point2d markDir = yTopOrigin ? cv::Point2d(-direction.y, -direction.x) : cv::Point2d(direction.y, direction.x);

    int precision = 2;
    if (stepWorldLength1 > 100) precision = 0;
    else if (stepWorldLength1 > 10)precision = 1;
    else precision = 2;

    for (int i = 0; i <= a; i++)
    {
        p = initPoint + (step * i);
        DrawMaker(img, p, COLOR_CV_BLUE, markDir, makerSize);
        std::ostringstream str;
        auto value = startRuler + stepWorldLength * i;

        str << std::fixed << std::setprecision(precision) << value;
        cv::Size txtSize = cv::getTextSize(str.str(), cv::FONT_HERSHEY_SIMPLEX, fontScale, fontWeight, 0);
        cv::putText(img, str.str()
                    , p + markDir * (makerSize + 2 + txtSize.height * abs(markDir.y) + txtSize.width * abs(markDir.x))
                    , cv::FONT_HERSHEY_SIMPLEX, fontScale, COLOR_CV_BLUE, fontWeight);
    }
}

void Drawing::DrawRuler(cv::Mat img, double lower_x, double upper_x, double lower_y, double upper_y, const bool isUpdateScale)
{
    if (isUpdateScale)
    {
        CaculateScale(lower_x, upper_x, lower_y, upper_y, img.size());
    }

    {
        // draw the histograms
        constexpr int rulerXSize = 50;
        constexpr int rulerYSize = 40;
        padding_X = (std::max)(padding_X, rulerXSize);
        padding_Y = (std::max)(padding_Y, rulerYSize);
    }

    if (img.empty())
    {

        int hist_w = 256 * 2 + padding_X * 2;
        int hist_h = 256 + padding_Y * 2;

        img = cv::Mat(hist_h, hist_w, CV_8UC3, COLOR_CV_GRAY(200));
    }
    else
    {
        {
            cv::Mat tmp = img;
            img = cv::Mat(tmp.rows + padding_Y * 2, tmp.cols + padding_X * 2, CV_8UC3, COLOR_CV_GRAY(200));
            tmp.copyTo(img(cv::Rect(padding_X, padding_Y, tmp.cols, tmp.rows)));
        }

        if (img.channels() == 1)
            cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
        else if (img.channels() != 3) throw std::invalid_argument("Only support cv::Mat channle=1 || channel=3");
    }

    DrawRuler(img, GetDrawPoint(lower_x, lower_y), GetDrawPoint(upper_x, lower_y));
    DrawRuler(img, GetDrawPoint(lower_x, lower_y), GetDrawPoint(lower_x, upper_y));
}

void Drawing::DrawDashedLine(cv::Mat& img
                             , cv::Point pt1
                             , cv::Point pt2
                             , cv::Scalar color
                             , int thickness
                             , std::string style
                             , int gap
)
{
    float dx = (float)pt1.x - pt2.x;
    float dy = (float)pt1.y - pt2.y;
    float dist = std::hypot(dx, dy);

    std::vector<cv::Point> pts;
    for (int i = 0; i < dist; i += gap)
    {
        float r = static_cast<float>(i / dist);
        int x = static_cast<int>((pt1.x * (1.0 - r) + (double)pt2.x * r) + .5);
        int y = static_cast<int>((pt1.y * (1.0 - r) + (double)pt2.y * r) + .5);
        pts.emplace_back(x, y);
    }

    int pts_size = (int)pts.size();
    if (pts_size > 0)
    {
        if (style == "dotted")
        {
            for (int i = 0; i < pts_size; ++i)
            {
                cv::circle(img, pts[i], thickness, color, -1);
            }
        }
        else
        {
            cv::Point s = pts[0];
            cv::Point e = pts[0];

            for (int i = 0; i < pts_size; ++i)
            {
                s = e;
                e = pts[i];
                if (i % 2 == 1)
                {
                    cv::line(img, s, e, color, thickness);
                }
            }
        }
    }
}

void Drawing::ShowImg(const std::string& imgTitle, const cv::Mat& imgInput)
{
    if (imgInput.empty()) return;
    cv::namedWindow(imgTitle, cv::WINDOW_NORMAL);
    //cv::setWindowProperty(imgTitle, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow(imgTitle, imgInput);
    //cv::waitKey();
}

void Drawing::DrawDimention(cv::Mat& img, cv::Point p1, cv::Point p2, cv::Scalar color, int thickness, std::string str, int padding, int length)
{
    if (p1 == p2) return;

    cv::Point2f unit = p1 - p2;
    unit /= cv::norm(p1 - p2);
    unit = cv::Point2f(-unit.y, unit.x);

    cv::Point lengVec = cv::Point(unit * length / 2.0);
    cv::Point padVec = cv::Point(unit * padding);

    cv::line(img, p1, p2, color, thickness);

    if (length != 0)
    {
        cv::line(img, p1 - lengVec, p1 + lengVec, color, thickness);
        cv::line(img, p2 - lengVec, p2 + lengVec, color, thickness);
    }

    if (!str.empty())
    {
        int font = cv::FONT_HERSHEY_PLAIN;
        double fontScale = 2.5;
        auto textSize = cv::getTextSize(str, font, fontScale, thickness, 0);
        double cos45 = cos(CV_PI / 4.0);
        cv::Point textPos = (p1 + p2) / 2 + padVec;
        textPos = abs(unit.x) > cos45 ? (unit.x > 0 ? textPos + cv::Point(0, textSize.height / 2) : textPos + cv::Point(-textSize.width, textSize.height / 2))
            : textPos - cv::Point(textSize.width / 2, 0);
        cv::putText(img, str, textPos, cv::FONT_HERSHEY_PLAIN, fontScale, color, thickness);
    }
}

void Drawing::DrawDimention(cv::Mat& img, cv::Point p1, cv::Point p2, float value, cv::Scalar color, int thickness, int padding, int length)
{
    std::ostringstream strItem;
    strItem << std::fixed << std::setprecision(3) << value;
    DrawDimention(img, p1, p2, color, thickness, strItem.str(), padding, length);
}

cv::Mat DrawHistogram(std::vector<int> histogramValue, int minRange, int maxRange, bool isNormalize)
{
    size_t histSize = histogramValue.size();
    if (histSize < 1)return cv::Mat();
    std::vector<double> normalizedHistogram;
    if (isNormalize)
    {
        cv::normalize(histogramValue, normalizedHistogram, 1.0, cv::NORM_L1);
    }
    size_t histSize1 = (maxRange - minRange + 1);
    const double step = histSize1 == histSize ? 1.0 : (double)histSize1 / histSize;

    std::vector<int> xData(histSize);
    xData[0] = minRange;
    for (auto i = xData.begin() + 1, end = xData.end(); i < end; i++)
    {
        *i = *(i - 1) + step;
    }

    double minY = 0, maxY = 0;
    if (isNormalize)
    {
        auto minY1 = std::minmax_element(normalizedHistogram.begin(), normalizedHistogram.end());
        minY = *minY1.first;
        maxY = *minY1.second;
    }
    else
    {
        auto minY1 = std::minmax_element(histogramValue.begin(), histogramValue.end());
        minY = *minY1.first;
        maxY = *minY1.second;
    }

    // draw the histograms

    constexpr int rulerXSize = 50;
    constexpr int rulerYSize = 40;
    int hist_w = histSize * 2 + rulerXSize * 2;
    constexpr int hist_h = 256 + rulerYSize * 2;

    cv::Mat histImage(hist_h, hist_w, CV_8UC3, COLOR_CV_GRAY(200));
    xvt::Drawing drawTool = xvt::Drawing();
    drawTool.padding_X = rulerXSize;
    drawTool.padding_Y = rulerYSize;
    drawTool.yTopOrigin = false;
    drawTool.color = COLOR_CV_BLACK;
    drawTool.thickness = 2;

    if (isNormalize)
    {
        drawTool.Plot(histImage, xData, normalizedHistogram);
    }
    else
    {
        drawTool.Plot(histImage, xData, histogramValue);
    }

    drawTool.DrawRuler(histImage, minRange, maxRange, minY, maxY);

    return histImage;
}

cv::Mat GetHistoImage(const cv::Mat& src, int minTh, int maxTh)
{
    cv::Mat dst = cv::Mat();
    if (src.empty()) return dst;

    double minVal = 0, maxVal = 255;
    cv::minMaxLoc(src, &minVal, &maxVal, nullptr, nullptr);
    const int imgDeepth = src.elemSize1();

    double minRange = 0;
    double maxRange = 256;
    int histSize = maxRange - minRange;
    if (imgDeepth > 1)
    {
        int bin = 1 << ((imgDeepth - 1) * 8);
        minRange = minVal;
        maxRange = maxVal + 1;
        histSize = maxRange - minRange;
        int adjust = histSize % bin;
        if (adjust)
        {
            int b = (bin - adjust) / 2;
            minRange -= b;
            maxRange += bin - b;
            histSize = (maxRange - minRange) / bin;
        }
    }

    float range[] = { minRange, maxRange + 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    cv::Mat gray_hist;
    bool uniform = true, accumulate = false;
    cv::calcHist(&src, 1, 0, cv::Mat(), gray_hist, 1, &histSize, histRange, uniform, accumulate);


    cv::Mat hisImg = DrawHistogram(gray_hist, minRange, maxRange);


    //if (minIdx >=0)
    {
        cv::putText(hisImg
                    , "Min: " + std::to_string((int)minVal)
                    , cv::Point(50, 20)
                    , cv::FONT_HERSHEY_SIMPLEX, 0.3, COLOR_CV_BLUE, 1);
    }
    //if (maxIdx >= 0)
    {
        cv::putText(hisImg
                    , "Max: " + std::to_string((int)maxVal)
                    , cv::Point(150, 20)
                    , cv::FONT_HERSHEY_SIMPLEX, 0.3, COLOR_CV_BLUE, 1);
    }

    dst.release();
    gray_hist.release();
    return hisImg;
}

void Draw(cv::Mat& img, cv::RotatedRect const& rect, CVPen pen, cv::Point2f offset)
{
    if (!xvt::ConvertRGB(img, img, false) && !rect.size.empty())
    {
        cv::Point2f p[4];
        rect.points(p);
        cv::line(img, p[0] + offset, p[1] + offset, pen.mColor, pen.mThickness, pen.mLineType);
        cv::line(img, p[2] + offset, p[1] + offset, pen.mColor, pen.mThickness, pen.mLineType);
        cv::line(img, p[2] + offset, p[3] + offset, pen.mColor, pen.mThickness, pen.mLineType);
        cv::line(img, p[0] + offset, p[3] + offset, pen.mColor, pen.mThickness, pen.mLineType);
    }
}

}//endnamespace xvt
