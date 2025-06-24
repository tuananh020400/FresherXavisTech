#include "xvtCV/GammaCorrector.h"
#include <opencv2/imgproc.hpp>

namespace xvt {
namespace enhance {

GammaCorrector::GammaCorrector(float g)
{
    mLUT = std::vector<uchar>(256, 1);
    mGammaFunc = GammaCorrectFuncXavis;
    SetGamma(g);
}

void GammaCorrector::SetGamma(float g)&
{
    if (g != mGamma && g >= 0 && g < 26)
    {
        mGamma = g;

        CalLUT();
    }
}

void GammaCorrector::Apply(cv::Mat const& src, cv::Mat& dst) const&
{
    if (src.empty() || mLUT.empty()) return;
    if (src.elemSize1() != 1)return;

    if (mGamma == 1)
    {
        dst = src.clone();
    }
    else
    {
        cv::LUT(src, mLUT, dst);
    }
}

uchar GammaCorrector::GammaCorrectFuncOpencv(int i, float g)
{
    return g == 1 ? i : cv::saturate_cast<uchar>(pow(i / 255.0, g) * 255.0);
}

uchar GammaCorrector::GammaCorrectFuncXavis(int i, float g)
{
    return g == 1 ? i : cv::saturate_cast<uchar>(255.0 - pow(1.0 - i / 255.0, g) * 255.0);
}

uchar GammaCorrector::SigmoidCorrectFunc(int i, float g)
{
    return g == 0 ? i : cv::saturate_cast<uchar>(255.0 / (1 + exp((0.5 - i / 255.0) * g)));
}

void GammaCorrector::CalLUT() &
{
    for (size_t i = 0, size = mLUT.size(); i < size; i++)
    {
        mLUT[i] = mGammaFunc(i, mGamma);
    }
}
}
}