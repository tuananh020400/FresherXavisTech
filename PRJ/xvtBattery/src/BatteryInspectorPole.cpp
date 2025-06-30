#include "xvtBattery/BatteryInspectorPole.h"
#include <xvtBattery/BatteryUtils.h>
#include <xvtCV/ColorDefine.h>
#include <xvtCV/Utils.h>
#include <iomanip>

namespace xvt
{
namespace battery
{

auto BatteryInspectorPole::Inspect(std::vector<PoleInfo> &poles, cv::Point caseRef) const -> PoleResult
{
    auto ispResult = PoleResult();
    ispResult.mEnable = mEnable;

    if (!ispResult.mEnable)
    {
        ispResult(EResult::UC, "Disabled!");
        return ispResult;
    }

    return ispResult;
}

void PoleResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    cv::Mat drawImg = img(GetROI());
    if (!mPoles.empty() && !drawImg.empty())
    {
        pen.mColor = COLOR_CV_CYAN;
        for (const auto &p : mPoles)
        {
            cv::circle(drawImg, p.mCathode, 2, cv::Scalar(180, 10, 10), 3);
            cv::circle(drawImg, p.mAnode, 2, cv::Scalar(245, 164, 66), 3);

            if (!p.mPoints.empty())
                xvt::DrawPoints(drawImg, p.mPoints, cv::Vec3b(245, 164, 66));
        }
    }
}

auto PoleResult::DrawResultStr(cv::Mat &image, std::string const &name, CVPen const &pen, cv::Point const &offset, bool isDrawOKResult) const
    -> cv::Point
{
    size_t detectedPoleNumb = Size();
    if (mXPoles.empty() || detectedPoleNumb <= 0)
    {
        return cv::Point();
    }

    cv::Mat resImg = image;
    const VecDouble &listAnode2CathodeDistance = GetLenghts();
    const VecDouble &listAnode2CaseDistance = {};
    const VecDouble &listCathode2CaseDistance = {};
    //const VecDouble &listPolePos = mXPoles;
    const std::vector<bool> listAno2CathodDecision (detectedPoleNumb, true);
    const std::vector<bool> listAno2CaseDecision (detectedPoleNumb, true);
    const std::vector<bool> listCathode2CaseDecision (detectedPoleNumb, true);
    int nLeftPole = mLeftNoPole;
    double cathode2AnodeOffset = 0;
    double anode2CaseOffset = 0;
    double cathode2CaseOffset = 0;
    bool isCheckPoleNo = 0;
    int OneSidePoleNumb = 24;
    float fontScale = pen.mFontScale * 0.75;
    int textLineSpace = pen.mSpace;
    cv::Point position = offset;

    bool result = true;
    //constexpr float fontScale = 0.7;
    constexpr int fontWeight = 2;
    // constexpr int textLineSpace = 25;
    constexpr int floatingPrecesion = 2;
    int lineThickness = pen.mThickness;
    const int txtSpace = 2;
    constexpr int txtFloatWidth = 5;
    constexpr int txtNumWidth = 5;
    constexpr int txtPosWidth = txtFloatWidth + txtSpace;
    constexpr int txtAnode2CathodeWidth = txtFloatWidth + txtSpace;
    constexpr int txtAnode2CathodeLimitWidth = 4 * 2 + 3 + txtSpace;
    constexpr int txtAnode2CaseWidth = txtFloatWidth + txtSpace + 1;
    constexpr int txtAnode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;
    constexpr int txtCathode2CaseWidth = txtFloatWidth + txtSpace + 1;
    constexpr int txtCathode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;

    std::ostringstream anode2CathodeLimitStr;
    anode2CathodeLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << mValidCathode2AnodeRange.GetLower() << "~"
                          << mValidCathode2AnodeRange.GetUpper() << "]";

    std::ostringstream anode2CaseLimitStr;
    anode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << mValidAnode2CaseRange.GetLower() << "~"
                       << mValidAnode2CaseRange.GetUpper() << "]";

    std::ostringstream anode2CaseOffsetStr;
    anode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << anode2CaseOffset;

    std::ostringstream cathode2anodeOffsetStr;
    cathode2anodeOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2AnodeOffset;

    std::ostringstream cathode2CaseLimitStr;
    cathode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << mValidCathode2CaseRange.GetLower() << "~"
                         << mValidCathode2CaseRange.GetUpper() << "]";

    std::ostringstream cathode2CaseOffsetStr;
    cathode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2CaseOffset;

    cv::String poleNumStr;

    int paddingX = 40;

    //cv::Scalar inforColor(180, 10, 10);
    cv::Scalar inforColor(245, 164, 66);
    cv::Scalar OKColor(60, 193, 5);
    cv::Scalar NGColor(0, 0, 255);
    cv::Scalar NCColor(0, 165, 255);
    cv::Scalar anode2CathodeColor(inforColor);
    cv::Scalar anode2CaseColor(inforColor);
    cv::Scalar cathode2CaseColor(inforColor);
    cv::Scalar poleColor(inforColor);
    cv::Scalar cathodeColor(180, 10, 10);
    cv::Point startPos;

    std::ostringstream tmpStream;
    std::vector<std::pair<std::string, cv::Scalar>> resultHeader(8, std::pair<std::string, cv::Scalar>("", inforColor));
    tmpStream << std::left << std::setw(txtNumWidth) << std::setfill(' ') << "| No";
    resultHeader[0].first = tmpStream.str();
    tmpStream.str("");
    tmpStream << std::left << std::setw(txtPosWidth) << std::setfill(' ') << " | Pos";
    resultHeader[1].first = tmpStream.str();
    tmpStream.str("");
    std::vector<std::pair<std::string, cv::Scalar>> resultInfo(8, std::pair<std::string, cv::Scalar>("", inforColor));
    if (1)
    {
        tmpStream << std::left << std::setw(txtAnode2CathodeWidth + txtAnode2CathodeLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
                  << " | Length[Range]" << cathode2anodeOffsetStr.str();
        resultInfo[3].first = anode2CathodeLimitStr.str();
    }
    resultHeader[2].first = tmpStream.str();
    tmpStream.str("");

    if (0)
    {
        tmpStream << std::left << std::setw(txtAnode2CaseWidth + txtAnode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
                  << " | A2Case[Range]" << anode2CaseOffsetStr.str();
        resultInfo[5].first = anode2CaseLimitStr.str();
    }
    resultHeader[4].first = tmpStream.str();
    tmpStream.str("");
    if (0)
    {
        tmpStream << std::left << std::setw(txtCathode2CaseWidth + txtCathode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
                  << " | Cat2Case[Range]" << cathode2CaseOffsetStr.str();
        resultInfo[7].first = cathode2CaseLimitStr.str();
    }
    resultHeader[6].first = tmpStream.str();

    tmpStream.str("");
    resultHeader[7].first = " |";

    auto msg = resultHeader[0].first + resultHeader[1].first + resultHeader[2].first + resultHeader[3].first + resultHeader[4].first +
               resultHeader[5].first + resultHeader[6].first + resultHeader[7].first;
    cv::Size txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, fontScale, fontWeight, 0);

    //assert(listAnode2CathodeDistance.size() == listAnode2CaseDistance.size());

    for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
    {
        bool isAnode2CathodeError = !mPoles[poleIdx].IsOK();
        bool isAnode2CaseError = !listAno2CaseDecision[poleIdx];
        bool isCathode2CaseError = !listCathode2CaseDecision[poleIdx];
        //result &= isAnode2CathodeError && isAnode2CaseError && isCathode2CaseError;
        if (mPoles[poleIdx].mType == PoleType::Incorrect) 
        {
            anode2CathodeColor = NCColor;
            anode2CaseColor = NCColor;
            cathode2CaseColor = NCColor;
            poleColor = NCColor;
        }
        else 
        {
            anode2CathodeColor = isAnode2CathodeError ? NGColor : OKColor;
            anode2CaseColor = isAnode2CaseError ? NGColor : OKColor;
            cathode2CaseColor = isCathode2CaseError ? NGColor : OKColor;
            poleColor = (isAnode2CathodeError || isAnode2CaseError || isCathode2CaseError) ? NGColor : OKColor;
        }
        
        //Assign the pole number
        std::ostringstream poleNumber;
        poleNumber << std::fixed << std::right << std::setw(txtNumWidth - 3) << std::setfill(' ') << "P" << std::setw(2) << std::setfill('0')
                   << (poleIdx + 1);
        resultInfo[0].first = poleNumber.str();
        resultInfo[0].second = poleColor;

        //Assign the X pole position
        std::ostringstream strAnode2Boundery;
        strAnode2Boundery << std::right << std::setw(txtPosWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                          << mXPoles[poleIdx];
        resultInfo[1].first = strAnode2Boundery.str();

        if (1)
        {
            std::ostringstream strCath2Ano;
            strCath2Ano << std::setw(txtAnode2CathodeWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                        << listAnode2CathodeDistance[poleIdx];
            //Assign the anode to cathode infor
            resultInfo[2].first = strCath2Ano.str();
            resultInfo[2].second = anode2CathodeColor;
        }

        if (0)
        {
            std::ostringstream strAno2Case;
            strAno2Case << std::setw(txtAnode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                        << listAnode2CaseDistance[poleIdx];
            //Assign the anode to case infor
            resultInfo[4].first = strAno2Case.str();
            resultInfo[4].second = anode2CaseColor;
        }

        if (0)
        {
            std::ostringstream strCat2Case;
            strCat2Case << std::setw(txtCathode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
                        << listCathode2CaseDistance[poleIdx];
            //Assign the anode to case infor
            resultInfo[6].first = strCat2Case.str();
            resultInfo[6].second = cathode2CaseColor;
        }

        //Cathode 2 Anode Measurement
        if (poleIdx < nLeftPole)
        {
            startPos.x = position.x + paddingX;
            startPos.y = position.y + (poleIdx % nLeftPole) * textLineSpace;
        }
        else
        {
            startPos.x = resImg.cols - txtSize.width - paddingX;
            startPos.y = position.y + (poleIdx - nLeftPole) * textLineSpace;
        }

        bool isFirstLeft = poleIdx == 0;
        bool isFirstRight = (poleIdx - nLeftPole) == 0;
        if (isFirstLeft || isFirstRight)
        {
            drawText(resImg, cv::Point(startPos.x, startPos.y - textLineSpace), resultHeader, fontScale, fontWeight);
        }

        drawText(resImg, startPos, resultInfo, fontScale, fontWeight);
    }

    ///Plot missing pole to results
    if (isCheckPoleNo)
    {
        int nRightPoles = detectedPoleNumb - nLeftPole;
        int nLeftMiss = OneSidePoleNumb - nLeftPole;
        int nRightMiss = OneSidePoleNumb - nRightPoles;
        poleNumStr = " P##   Missing";

        for (int pIdx = 0; pIdx < nLeftMiss; pIdx++)
        {
            int missPoleIdx = nLeftPole + pIdx;
            startPos = cv::Point(position.x + paddingX, position.y + missPoleIdx * textLineSpace);
            drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
        }

        startPos.x = resImg.cols - txtSize.width - paddingX;
        for (int pIdx = 0; pIdx < nRightMiss; pIdx++)
        {
            int missPoleIdx = nRightPoles + pIdx;
            startPos.y = position.y + missPoleIdx * textLineSpace;
            drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
        }
    }
    return cv::Point();
}

auto PoleResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
}

} // namespace battery
} // namespace xvt