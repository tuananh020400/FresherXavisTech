#include "xvtBattery/CylinderInspectorUpper.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Utils.h"

namespace xvt {
namespace battery {
auto CylinderInspectorUpper::Inspect(cv::Mat const &inImg) const -> CylinderBatteryUpperResult
{
    ScopeTimer t("Cylinder Upper");

    cv::Mat src;
    CylinderBatteryUpperResult result;
    if (!Convert8Bits(inImg, src, false))
    {
        t.GetElapsedTime();
        auto baseResult = mIspBase.Inspect(src);
        baseResult.mProTime = t.GetElapsedTime().count();
        result.SetBaseResult(std::move(baseResult));

        // find beading
        auto beadingResult = mIspBeading.Inspect(src, result.mBaseResult.mPoints);
        beadingResult.mProTime = t.GetElapsedTime().count();
        result.SetBeadingResult(std::move(beadingResult));

        // find jrROi
        cv::Rect jrROI = CreateROI(cv::Point(result.mBaseResult.GetROI().x, result.mBeadingResult.mBottomLeftContour.y),
                          cv::Size(result.mBaseResult.GetROI().width, result.mBaseResult.GetROI().br().y - result.mBeadingResult.mBottomLeftContour.y),
                          src.size());

        auto jrResult = mIspJR.Inspect(src, jrROI);
        jrResult.mProTime = t.GetElapsedTime().count();
        result.SetJRResult(std::move(jrResult));

        auto cathodeResult = mIspCathode.Inspect(src, result.mJRResult.mPoleRoi, result.mPoleResult.mPoles);
        cathodeResult.mProTime = t.GetElapsedTime().count();
        result.SetCathodeResult(std::move(cathodeResult));

        auto anodeResult = mIspAnode.Inspect(src, result.mJRResult.mPoleRoi, result.mPoleResult.mPoles);
        anodeResult.mProTime = t.GetElapsedTime().count();
        result.SetAnodeResult(std::move(anodeResult));
    }
    else
    {
        result(EResult::ER, "Image type is not supported!");
    }

    auto tmp = t.GetTotalElapsedTime();
    result.mProTime = tmp.count();
    return result;
}

void CylinderBatteryUpperResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    pen.mThickness = 1;
    mBaseResult.DrawResult(img, cv::Point(), pen);
    mBeadingResult.DrawResult(img, cv::Point(), pen);
    mJRResult.DrawResult(img, cv::Point(), pen);
    mCathodeResult.DrawResult(img, cv::Point(), pen);
    mAnodeResult.DrawResult(img, cv::Point(), pen);
    mPoleResult.DrawResult(img, cv::Point(), pen);

#if 1
    auto procesText = "Total: " + std::to_string((int)mProTime) + ", C: " + std::to_string((int)mBaseResult.mProTime) +
                      ", B: " + std::to_string((int)mBeadingResult.mProTime) + ", JR: " + std::to_string((int)mJRResult.mProTime) +
                      ", P: " + std::to_string((int)mCathodeResult.mProTime) + ", N: " + std::to_string((int)mAnodeResult.mProTime) + "(ms)";
    pen.mColor = COLOR_CV_GRAY(10);
    DrawText(img, procesText, cv::Point(50, img.rows - 50), pen);
#endif // _DEBUG
}

auto CylinderBatteryUpperResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
}
} // namespace battery
} // namespace xvt