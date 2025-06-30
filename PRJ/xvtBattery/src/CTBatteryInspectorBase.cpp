#include "xvtCV/AStar.h"
#include "xvtCV/Contour.h"
#include "xvtCV/xvtTable.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtBattery/VersionInfo.h"
#include "xvtBattery/CTBatteryInspectorBase.h"


#define _DEBUG_TOP_REF_LINE             (_DEBUG && 0)
#define _DEBUG_CATHODE                  (_DEBUG && 0)
#define _DEBUG_CATHODE_X                (_DEBUG && 0)
#define _DEBUG_CATHODE_X_AUTO           (_DEBUG && 0)
#define _DEBUG_REFINE_CATHODE_LINE      (_DEBUG && 0)
#define _DEBUG_GET_CATHODE_LR           (_DEBUG && 0)
#define _DEBUG_REMOVE_CATHODE           (_DEBUG && 0)
#define _DEBUG_ANODE                    (_DEBUG && 0)

namespace xvt{
namespace battery
{

auto FindAnodeStartingPosition(const cv::Mat& src, VecPoint vPoints) -> VecPoint
{
    VecPoint vtResult;
    auto dst = src.clone();
    auto size = src.size();
    cv::Rect tmpRect1(-1, 0, 3, 1);

    for (auto p : vPoints)
    {
        cv::Rect rect = CreateROI(tmpRect1 + p, size);
        dst(rect).setTo(255);
    }

    auto listCathode = xvt::FindPathByAStar(dst, 9);

    auto mask = cv::Mat(src.size(), src.type(), cv::Scalar(0));
    cv::Rect tmpRect2(0, -40, 1, 40);

    for (auto p : listCathode)
    {
        cv::Rect rect = CreateROI(tmpRect2 + p, size);
        mask(rect).setTo(255);
    }
    cv::Mat dst2;
    cv::bitwise_and(src, mask, dst2);

    VecFloat reduceVec = ReduceBlackBackgroundMat(dst2, 0, 3);
    FindPeaks findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
    findPeaks.Process(reduceVec);
    int min_dis = 3;
    int max_dis = 30;
    auto peakResult = findPeaks.GetPeakResult(1, 3);
    for (auto p : peakResult)
    {
        vtResult.push_back(listCathode[p.index]);
    }

    for (auto p : vtResult)
    {
        dst2.at<uchar>(p) = (255);
    }

    return vtResult;
}

CTBatteryInspector::CTBatteryInspector()
{
    mIspBase.mIsInvert = false;
}

bool IsBaseLineType(xvt::battery::BaseLineType t1, xvt::battery::BaseLineType const& t2)
{
    return (static_cast<int>(t1) & static_cast<int>(t2));
}

auto CTBatteryInspector::Inspect(cv::Mat const& img) const -> std::unique_ptr<xvt::IInspectionResult>
{
    auto ispResult = std::make_unique<CTBatteryInspectorResult>();
    Inspect(img, *(ispResult.get()));
    return ispResult;
}

auto CTBatteryInspector::Inspect2(cv::Mat const& img) const -> std::unique_ptr<xvt::IInspectionResult>
{
    auto ispResult = std::make_unique<CTBatteryInspectorResult>();
    ispResult->mEnable = mEnable;

    if (!ispResult->mEnable)
    {
        ispResult->operator()(EResult::ER, "Disabled!");
        return ispResult;
    }

    cv::Mat src;
    if (!Convert8Bits(img, src, false))
    {
        cv::Rect batteryBoundary;

        auto imgSize = img.size();
        auto settingROI = mIspBase.GetInspectROI(imgSize);

        double subROIY = 0;//YA2
        auto& beadingRes = ispResult->mBeadingResult;

        if (mIspJR.mBaseLineType == BaseLineType::NONE || mIspBeading.mEnable)
        {
            auto baseResult = mIspBase.Inspect(src);
            ispResult->mCornerPoint = baseResult.GetCornerPoint();
            if (!baseResult.IsER() && !baseResult.IsUC() && !ispResult->mCornerPoint.empty())
            {
                batteryBoundary = cv::boundingRect(baseResult.mPoints);
            }
            ispResult->CombineResult(&baseResult);
            ispResult->mOuterRoi = CreateROI(batteryBoundary, img.size());
            subROIY = ispResult->mOuterRoi.y;

            if (!baseResult.mPoints.empty())
                ispResult->mPoints = baseResult.mPoints;

            if (mIspBeading.mEnable)
            {
                // find beading
                cv::Mat drawCountourImg = cv::Mat::zeros(imgSize, CV_8UC1);
                cv::drawContours(drawCountourImg, VecVecPoint({ baseResult.mPoints }), -1, cv::Scalar(255));
                beadingRes = mIspBeading.Inspect(drawCountourImg, baseResult.mPoints);
                if (!beadingRes.IsER())
                {
                    /*beadingRes.mGrooveLeftContour  += settingROI.tl();
                    beadingRes.mGrooveRightContour += settingROI.tl();
                    beadingRes.mBottomLeftContour  += settingROI.tl();
                    beadingRes.mBottomRightContour += settingROI.tl();*/
                }

                if (beadingRes.IsOK())
                {
                    subROIY = (beadingRes.mBottomLeftContour.y + beadingRes.mBottomRightContour.y) / 2.0;
                }
                else
                {
                    subROIY = ispResult->mOuterRoi.y;
                }
            }
        }
        else
        {
            auto findBatteryResult = FindBatteryByTurnPoint(src(settingROI), mIspBase.mThreshold, ispResult->mCornerPoint);
            if (findBatteryResult.IsOK())
            {
                for (auto& c : ispResult->mCornerPoint) c += settingROI.tl();

                batteryBoundary = cv::boundingRect(ispResult->mCornerPoint);
            }
            ispResult->CombineResult(&findBatteryResult);

            if (!findBatteryResult.mPoints.empty())
                ispResult->mPoints = xvt::Transform(findBatteryResult.mPoints, settingROI.tl());

            cv::Point tl = ispResult->mCornerPoint[0];
            cv::Point tr = ispResult->mCornerPoint[1];
            int yPos = round((tl.y + tr.y) / 2.0f);
            ispResult->mOuterRoi = CreateROI(batteryBoundary.x, yPos, batteryBoundary.width, batteryBoundary.height + batteryBoundary.y - yPos, img.size());
            subROIY = ispResult->mOuterRoi.y;
        }

        if (ispResult->IsOK())
        {
            auto oX = mIspJR.mJROffsetX;
            auto oY = mIspJR.mJROffsetY;
            cv::Rect refJRROI = CreateROI(ispResult->mOuterRoi.x, settingROI.y, ispResult->mOuterRoi.width, settingROI.height, settingROI);
            auto JRROI = xvt::CreateROI(ispResult->mOuterRoi.x + oX,
                                        subROIY + oY,
                                        ispResult->mOuterRoi.width - 2 * oX,
                                        ispResult->mOuterRoi.height - oY,
                                        oX > 0 ? refJRROI : settingROI
            );

            auto findTopROIResult = FindTopReferenceLine(src, JRROI, ispResult->mCenterDetected);
            ispResult->mCenterWidth = mIspJR.mCenterNeglectionWidth != 0 ? mIspJR.mCenterNeglectionWidth : ispResult->mCenterDetected.width;
            if (findTopROIResult.IsOK() && !ispResult->mCenterDetected.empty())
            {
                ispResult->mCenter.x = ispResult->mCenterDetected.x + ispResult->mCenterDetected.width / 2;
            }
            else
            {
                ispResult->mCenter.x = mIspBase.mRoi.x + mIspBase.mRoi.width / 2;
                ispResult->AddMsg(findTopROIResult.GetMsg());
            }

            if (beadingRes.IsOK())
            {
                ispResult->mCenter.y = (beadingRes.mGrooveLeftContour.y + beadingRes.mGrooveRightContour.y) / 2;
            }
            else
            {
                if (ispResult->mCenterDetected.y > 0 && mIspJR.mJROffsetY < 0)
                {
                    subROIY = ispResult->mCenterDetected.y;
                }
                else
                {
                    subROIY = ispResult->mOuterRoi.y;
                }
                ispResult->mCenter.y = subROIY;
            }

            int difCenter = ispResult->mOuterRoi.x + ispResult->mOuterRoi.width / 2 - ispResult->mCenter.x;
            refJRROI = CreateROI(JRROI.x, settingROI.y, JRROI.width, settingROI.height, settingROI);
            cv::Rect poleROI = xvt::CreateROI(JRROI.x,
                                              subROIY + oY,
                                              JRROI.width,
                                              mIspJR.mHeight - oY,
                                              refJRROI
            );

            if (mIspJR.mEnableCheckLeaning)
            {
                int leftIdx = -1, rightIdx = -1;
                cv::Rect leaningROI = xvt::CreateROI(poleROI.x, ispResult->mCenterDetected.y+5, poleROI.width, poleROI.height / 2, poleROI);
                if (!leaningROI.empty())
                {
                    cv::Mat leaningImg = ~src(leaningROI);
                    cv::Mat reduceLeaningImg;
                    //cv::reduce(leaningImg, reduceLeaningImg, 0, cv::REDUCE_AVG);
                    auto lutLeaning = GetLUT(mIspJR.mLeaningThreshold, 255);
                    cv::Mat lutImg;
                    cv::LUT(leaningImg, lutLeaning, lutImg);
                    //cv::log(lutLeaning, lutLeaning);
                    xvt::Convert8Bits(lutImg, lutImg);
                    //cv::threshold(reduceLeaningImg, reduceLeaningImg, mIspJR.mLeaningThreshold, 255, cv::THRESH_TOZERO_INV);
                    CheckPoleLeaningAuto(lutImg, 11, leftIdx, rightIdx);
                    int offsetJR = 5;
                    if (rightIdx > 0 && rightIdx < leaningROI.width)
                    {
                        poleROI.width = rightIdx + offsetJR;
                    }

                    if (leftIdx > 0)
                    {
                        poleROI.x += leftIdx - offsetJR;
                        poleROI.width -= leftIdx - offsetJR;
                    }

                    RefineROI(poleROI, refJRROI);
                }
            }

            if (!poleROI.empty() && ispResult->mCenterWidth != 0)
            {
                auto cathodeIns = InspectCathode(src, poleROI, *ispResult);
                auto anodeIns = InspectAnode(src, poleROI, *ispResult);
                ispResult->CombineResult(&cathodeIns);
                ispResult->CombineResult(&anodeIns);
            }
            else
            {
                ispResult->CombineResult(&findTopROIResult);
            }
        }
        else
        {
        }

        if (mIspBase.mDirection == 1)
        {
            ispResult->mPoints = rotate180vtPoint(ispResult->mPoints, imgSize);
        }
    }
    else
    {
        ispResult->operator()(EResult::ER, "Image type is not supported!");
    }
    ispResult->mDisplayMode = mDisplayMode;
    ispResult->SetROI(mIspBase.mRoi);
    ispResult->mPixelSize = mPixelSize;
    ispResult->mDecimalNo = mDecimalNo;
    ispResult->mDirection = mIspBase.mDirection;
    return ispResult;
}

auto CTBatteryInspector::Inspect(const cv::Mat& inImg, CTBatteryInspectorResult& IspResult) const -> ERR_CODE
{
    cv::Mat src;
    cv::Mat rotatedImg;
    ScopeTimer t("Cylinder Battery CT Lower");

    IspResult.SetROI(mIspBase.mRoi);
    IspResult.mGamma = mGamma;
    auto res = Convert8Bits(inImg, src, true);
    cv::Size imgSize = inImg.size();

    if (res)
    {
        IspResult.Description = "Image empty or not support!";
        return res == 1 ? ERR_CODE::errImageEmpty : ERR_CODE::errImageFormat;
    }

    //-----end of pre-processing ------------------------------------------------------------------------
    std::string FinalDecision;

    ERR_CODE verifyERR = VerifyParameters(IspResult);
    if (verifyERR != ERR_CODE::OK)
    {
        return verifyERR;
    }

    cv::Rect settingROI = mIspBase.mRoi;
    if (false == RefineROI(settingROI, cv::Size(src.cols, src.rows)))
    {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errROIRefinement;
    }

    if (mIspBase.mDirection == 1)
    {
        cv::rotate(src, rotatedImg, cv::ROTATE_180);
    }
    else
    {
        rotatedImg = src.clone();
    }

    if (mPixelSize == 0)
    {
        IspResult.Description = "Pixel size cannot set to 0";
        return ERR_CODE::errPixelSize;
    }

    auto tmp = Inspect2(rotatedImg);
    auto tmp2 = xvt::DynamicCastUniquePtr<CTBatteryInspectorResult, xvt::IInspectionResult>(std::move(tmp));
    IspResult = *tmp2;
    xvt::enhance::GammaCorrector gamma;
    gamma.SetGamma(mGamma);
    gamma.Apply(src, IspResult.resImg);

    if (IspResult.IsER())
    {
        IspResult.Description = IspResult.GetMsg();
        return ERR_CODE::errInspection;
    }

    cv::Rect batteryROI = IspResult.mOuterRoi;

    if (batteryROI.width > 0 && batteryROI.height > 0)
    {
        //Check the width of battery is in the range of masterJigCellSize
        auto batteryWidth_mm = batteryROI.width * mPixelSize;

        if (!mIspBase.mValidBatteryWidthRange.IsInRange(round_f(batteryWidth_mm, mDecimalNo)))
        {
            IspResult.Description = "Error: Width of battery is out of range 2 : " + std::to_string(batteryWidth_mm);;
            return ERR_CODE::errWidthBattery;
        }
    }
    else
    {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }

    //======================================Finding Pole Positions====================================//
    // 1: From battery?s boundary found in section 1- select batteryROI_4
    //int yA2 = batteryROI.y;
    int roiBtmYPosition = settingROI.br().y;
    if (mIspBase.mDirection == 1)
    {
        roiBtmYPosition = src.rows - settingROI.y;
    }

    if (batteryROI.y >= roiBtmYPosition)
    {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }

    std::string descript = "";
    float maxA = -1.0, maxB = -1.0;
    float minA = -1.0, minB = -1.0;
    float averageA = 0.0f, averageB = 0.0f;
    int nPoleRight = 0;
    int nPoleLeft = 0;
    int fontWeight = 2;
    
    //Position in the original image
    VecPoint listAnodeOri;
    //Position in the original image
    VecPoint listCathodeOri;
    //mm distance
    std::vector<double> listAnode2CathodeDistance;
    std::vector<bool> listAno2CathodDecision;
    std::vector<std::pair<double, double>> listAnode2CathodeLRDistance;
    std::vector<std::pair<bool, bool>> listAno2CathodLRDecision;
    //mm distance
    std::vector<double> listAnode2CaseDistance;
    std::vector<bool> listAno2CaseDecision;
    //mm distance
    std::vector<double> listCathode2CaseDistance;
    std::vector<bool> listCathode2CaseDecision;
    std::vector<std::pair<double, double>> listCathode2CaseLRDistance;
    std::vector<std::pair<bool, bool>> listCathode2CaseLRDecision;
    //pixel distance to the battery ROI
    std::vector<double> listPolePos;
    VecPeak anodePeaks;
    //Check disappear poles
    std::vector<std::pair<int, int>> lstPolePositionErr;
    ERR_CODE errBlackRegion = ERR_CODE::OK;

    bool isAnode2CaseOK = true;
    bool isAnode2CathodeOK = true;
    bool isCathode2CaseOK = true;
    bool isCathodeLeft2CaseOK = true;
    bool isCathodeRight2CaseOK = true;

    //===============================================making Decisions==================================================//


    //int poleIdx = 0;
    //int missPoleIdx = 0;
    RemovePoles(rotatedImg, IspResult); //Remove dominant poles
    RefineAnodePoles(IspResult);

    //RemoveCathodeByAnode(IspResult, 1.0f);//Remove the cathode that has same position with anode
    RemoveCathodeByIntensity(rotatedImg, IspResult, 1.0f);

    GetCathodeLeftRight(rotatedImg, IspResult);

    RemoveCathodeCenterPoles(rotatedImg, IspResult);//remove redundant cathode in center region
    RemoveNoiseAnodeEndingPoles(rotatedImg, IspResult);//remove redundant anode at start/end region

    AddMissingAnodeEndingPoles(rotatedImg, IspResult);//if not end by anode add one more anode.

    // find all poles position
    std::vector<int> lstPolePosXAll, anodePos, cathodePos;
    for (const auto& p : IspResult.mAnodePoles.mPoles)
    {
        lstPolePosXAll.push_back(p.mCathode.x);
        cathodePos.push_back(p.mCathode.y);
        anodePos.push_back(p.mAnode.y);
        anodePeaks.push_back(p.mPeakValue);
    }

    if (lstPolePosXAll.empty()
        || cathodePos.size() != lstPolePosXAll.size()
        || anodePos.size() != lstPolePosXAll.size()
        || IspResult.mCathodePos.empty()
        )
    {
        IspResult.Description = "Cannot find any poles, please check again outer ROI Setting or input image";
        return ERR_CODE::errPoleXDetection5;
    }

    nPoleLeft = IspResult.mAnodePoles.mLeftNoPole;
    nPoleRight = IspResult.mAnodePoles.Size() - IspResult.mAnodePoles.mLeftNoPole;
    
    int lstPoleSize = lstPolePosXAll.size();

    // For preventing anode position out of range
    std::for_each(anodePos.begin(), anodePos.end(), [](int& ap) {ap = ap < 0 ? 0 : ap; });

    std::vector<double> lstAnodeDouble;
    auto firstAnode = anodePos.front();
    auto lastAnode = anodePos.back();
    std::for_each(anodePos.begin(), anodePos.end(), [&lstAnodeDouble](int& ap) {lstAnodeDouble.push_back(ap); });
    cv::GaussianBlur(lstAnodeDouble, lstAnodeDouble, cv::Size(3, 1), 0);
    //lstAnodeDouble = weightedAvgSmoothen(lstAnodeDouble);
    anodePos.clear();
    anodePos.push_back(firstAnode);
    std::for_each(lstAnodeDouble.begin()+1, lstAnodeDouble.end()-1, [&anodePos](double& ap) {anodePos.push_back(ap); });
    anodePos.push_back(lastAnode);

    assert(cathodePos.size() == lstPoleSize && anodePos.size() == lstPoleSize);

    auto cSize = IspResult.mCathodePos.size();

    //=============================================Making Decisions===================================
    //For calculate the anode to case distance
    int caseLine = IspResult.mOuterRoi.y;
    int noAnodeRef = 4;
    if (mIspJR.mBaseLineType == BaseLineType::ANODE && IspResult.mAnodePoles.mPoles.size() > noAnodeRef * 2)
    {
        float avgYAnode = 0.0;
        int count = 0;
        for (int i = 0; i < lstPoleSize; i++)
        {
            if (i < noAnodeRef || i > lstPoleSize - noAnodeRef - 1)
            {
                count++;
                avgYAnode += IspResult.mAnodePoles.mPoles[i].mAnode.y;
            }
        }
        if (count > 0) avgYAnode = (int)round(avgYAnode / count);

        if (avgYAnode > settingROI.y) caseLine = avgYAnode;
    }
    else if (mIspJR.mBaseLineType == BaseLineType::CENTER)
    {
        caseLine = IspResult.mCenter.y;
    }
    else if (mIspJR.mBaseLineType == BaseLineType::CORNER)
    {
        cv::Point tl = IspResult.mCornerPoint[0];
        cv::Point tr = IspResult.mCornerPoint[1];
        caseLine = round((tl.y + tr.y) / 2.0f);
    }
    else
    {
        caseLine = IspResult.mOuterRoi.y;
    }
    caseLine += mIspJR.mBaseLineOffset;

    const auto C2AValid = mIspPole.mValidCathode2AnodeRange;
    const auto A2BValid = mIspPole.mValidAnode2CaseRange;
    const auto C2BValid = mIspPole.mValidCathode2CaseRange;
    //int xLeftRef = mIspBase.mDirection == 0 ? batteryROI.x : batteryROI.br().x;
    int xLeftRef = IspResult.mCenter.x;
    int signX = mIspBase.mDirection == 0 ? 1 : -1;
    auto const& ofCa2An = mIspPole.mCathode2AnodeOffset;
    for (int poleIdx = 0; poleIdx < lstPoleSize; poleIdx++)
    {
        //Update the original position on the Image
        auto mAnode = cv::Point(lstPolePosXAll[poleIdx], anodePos[poleIdx]);
        auto mCathode = cv::Point(lstPolePosXAll[poleIdx], cathodePos[poleIdx]);

        //Get the distance from the pole to the left battery roi
        listPolePos.push_back(signX*(mAnode.x - xLeftRef) * mPixelSize);

        //Get the anode to case result
        double mAnode2CasePixel = abs(mAnode.y - caseLine) + mIspPole.mAnode2CaseOffset;
        double mAnode2Case_mm = round_f(mAnode2CasePixel * mPixelSize, mDecimalNo);
        auto mAnode2CaseResult = A2BValid.IsInRange(mAnode2Case_mm);

        double mAnode2Cathode_mm1 = mIspPole.ToMilimet(abs(mAnode.y - mCathode.y) + ofCa2An);
        //Get the anode to cathode result
        double mAnode2CathodePixel = abs(mAnode.y - mCathode.y) + mIspPole.mCathode2AnodeOffset;
        double mAnode2Cathode_mm = round_f(mAnode2CathodePixel * mPixelSize, mDecimalNo);
        auto mAnode2CathodeResult = A2BValid.IsInRange(mAnode2Cathode_mm);

        auto const& cCathodeLR = IspResult.vtCathodesLR[poleIdx];
        double mAnode2CathodeLPixel = cCathodeLR.first != cv::Point(0, 0) ? abs(mAnode.y - cCathodeLR.first.y) + ofCa2An : -1;
        double mAnode2CathodeL_mm = round_f(mAnode2CathodeLPixel * mPixelSize, mDecimalNo);
        auto mAnode2CathodeLResult = mAnode2CathodeL_mm < 0 || C2AValid.IsInRange(mAnode2CathodeL_mm);

        double mAnode2CathodeRPixel = cCathodeLR.second != cv::Point(0, 0) ? abs(mAnode.y - cCathodeLR.second.y) + ofCa2An : -1;
        double mAnode2CathodeR_mm = round_f(mAnode2CathodeRPixel * mPixelSize, mDecimalNo);
        auto mAnode2CathodeRResult = mAnode2CathodeRPixel < 0 || C2AValid.IsInRange(mAnode2CathodeR_mm);
        listAno2CathodLRDecision.push_back(std::make_pair(mAnode2CathodeLResult, mAnode2CathodeRResult));
        listAnode2CathodeLRDistance.push_back(std::make_pair(mAnode2CathodeL_mm, mAnode2CathodeR_mm));

        isAnode2CaseOK &= mAnode2CaseResult;
        isAnode2CathodeOK &= (mAnode2CathodeLResult && mAnode2CathodeRResult);

        listAno2CaseDecision.push_back(mAnode2CaseResult);
        listAno2CathodDecision.push_back(mAnode2CathodeResult);

        listAnode2CaseDistance.push_back(mAnode2Case_mm);
        listAnode2CathodeDistance.push_back(mAnode2Cathode_mm);

        listAnodeOri.push_back(mAnode);
        listCathodeOri.push_back(mCathode);
    }

    for (int i = 0; i < cSize; i++)
    {
        //Get the anode to cathode result
        double mCathode2CasePixel = abs(IspResult.mCathodePos[i].y - caseLine) + mIspPole.mCathode2CaseOffset;
        double mCathode2Case_mm = round_f(mCathode2CasePixel * mPixelSize, mDecimalNo);
        auto mCathode2CaseResult = C2BValid.IsInRange(mCathode2Case_mm);

        isCathode2CaseOK &= mCathode2CaseResult;
        listCathode2CaseDistance.push_back(mCathode2Case_mm);
        listCathode2CaseDecision.push_back(mCathode2CaseResult);
        double mXCathodePos_mm = round_f(signX * (IspResult.mCathodePos[i].x - xLeftRef) * mPixelSize, mDecimalNo);
        IspResult.mXCathodePos.push_back(mXCathodePos_mm);
        IspResult.mAnodePoles.mCathode2Case.emplace_back(mCathode2Case_mm, mCathode2CaseResult);
    }

    std::vector<std::string> errMsg;
    if (mIspBase.mDirection == 1)
    {
        auto imgSize = src.size();
        //cv::rotate(res, res, cv::ROTATE_180);
        //cv::Point ImgCenter = cv::Point((src.cols - 1) / 2, (src.rows - 1) / 2);
        listAnodeOri = rotate180vtPoint(listAnodeOri, imgSize);
        listCathodeOri = rotate180vtPoint(listCathodeOri, imgSize);
        IspResult.mCathodePos = rotate180vtPoint(IspResult.mCathodePos, imgSize);
        for (auto& p : IspResult.mCornerPoint)
            p = rotate180Point(p, imgSize);
        //std::reverse(IspResult.mCathodePos.begin(), IspResult.mCathodePos.end());

        std::reverse(anodePeaks.begin(), anodePeaks.end());
        std::reverse(listPolePos.begin(), listPolePos.end());
        std::reverse(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
        std::reverse(listAno2CaseDecision.begin(), listAno2CaseDecision.end());
        std::reverse(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
        std::reverse(listAno2CathodDecision.begin(), listAno2CathodDecision.end());
        std::reverse(listAno2CathodLRDecision.begin(), listAno2CathodLRDecision.end());
        std::reverse(listAnode2CathodeLRDistance.begin(), listAnode2CathodeLRDistance.end());
        std::reverse(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
        std::reverse(listCathode2CaseDecision.begin(), listCathode2CaseDecision.end());
        std::reverse(IspResult.mXCathodePos.begin(), IspResult.mXCathodePos.end());
        std::reverse(IspResult.mAnodePoles.mCathode2Case.begin(), IspResult.mAnodePoles.mCathode2Case.end());

        // Update the rectangle coordinates after rotation
        batteryROI.x = imgSize.width - batteryROI.x - batteryROI.width;
        batteryROI.y = imgSize.height - batteryROI.y - batteryROI.height;
        caseLine = imgSize.height - caseLine;

        IspResult.mCenterDetected.x = imgSize.width - IspResult.mCenterDetected.x - IspResult.mCenterDetected.width;
        IspResult.mCenterDetected.y = imgSize.height - IspResult.mCenterDetected.y - IspResult.mCenterDetected.height;

        IspResult.mPoleRegionRoi.x = imgSize.width - IspResult.mPoleRegionRoi.x - IspResult.mPoleRegionRoi.width;
        IspResult.mPoleRegionRoi.y = imgSize.height - IspResult.mPoleRegionRoi.y - IspResult.mPoleRegionRoi.height;

        IspResult.mCenter.x = imgSize.width - IspResult.mCenter.x;
        IspResult.mCenter.y = imgSize.height - IspResult.mCenter.y;
        IspResult.nLeftPole = nPoleRight;
        IspResult.nNoCathodeLeft = IspResult.mCathodePos.size() - IspResult.nNoCathodeLeft;
        IspResult.mAnodePoles.mLeftNoPole = nPoleRight;
    }
    else
    {
        IspResult.nLeftPole = nPoleLeft;
    }

    IspResult.mOuterRoi = batteryROI;
    IspResult.mAnodePoles.ClearAll();
    for (int i = 0; i < listAnodeOri.size(); i++)
    {
        PoleInfo tmpPole;
        tmpPole.mAnode = listAnodeOri[i];
        tmpPole.mCathode = listCathodeOri[i];
        tmpPole.mPeakValue = anodePeaks[i];
        IspResult.mAnodePoles.mPoles.emplace_back(tmpPole);
        IspResult.mAnodePoles.mCathode2Anode.emplace_back(listAno2CathodDecision[i], listAnode2CathodeDistance[i]);
        auto lResult = BResult(listAno2CathodLRDecision[i].first, listAnode2CathodeLRDistance[i].first);
        auto rResult = BResult(listAno2CathodLRDecision[i].second, listAnode2CathodeLRDistance[i].second);
        IspResult.mAnodePoles.mCathodeLR2Anode.emplace_back(lResult, rResult);
        IspResult.mAnodePoles.mAnode2Case.emplace_back(listAno2CaseDecision[i], listAnode2CaseDistance[i]);
    }
    IspResult.mAnodePoles.mXPoles = listPolePos;
    IspResult.mAnodePoles.mCaseRef = cv::Point(batteryROI.x, caseLine);
    IspResult.vtAno2CathodLRDecision = listAno2CathodLRDecision;
    IspResult.sCathode2AnodeLR = listAnode2CathodeLRDistance;
    //===============================================PLoting results in the 180 rotated image=================================================//

    std::string str = " ,maxB=" + std::to_string(maxB) + " ,minB=" + std::to_string(minB) + " ,averageB=" + std::to_string(averageB);
    IspResult.sData += str.c_str();
    if (!listAnode2CaseDistance.empty())
    {
        IspResult.maxAnode2Case = *max_element(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
        IspResult.minAnode2Case = *min_element(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
        IspResult.avgAnode2Case = std::accumulate(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end(), 0.0) / listAnode2CaseDistance.size();
    }

    if (listAnode2CathodeLRDistance.size() != 0)
    {
        IspResult.minCathode2Anode = INT_MAX;
        IspResult.maxCathode2Anode = INT_MIN;
        IspResult.avgCathode2Anode = 0;
        int count = 0;
        for (int i = 0; i < listAnode2CathodeLRDistance.size(); i++)
        {
            auto dis = listAnode2CathodeLRDistance[i];
            double minDis = std::min(dis.first, dis.second);
            double maxDis = std::max(dis.first, dis.second);

            if (minDis >= 0)
            {
                if (IspResult.minCathode2Anode > minDis)
                {
                    IspResult.minCathode2Anode = minDis;
                    IspResult.minLenghtIdx = i;
                }
                IspResult.avgCathode2Anode += minDis;
                count++;
            }

            if (maxDis >= 0)
            {
                if (IspResult.maxCathode2Anode < maxDis)
                {
                    IspResult.maxCathode2Anode = maxDis;
                    IspResult.maxLenghtIdx = i;
                }
                IspResult.avgCathode2Anode += maxDis;
                count++;
            }
        }

        if(count > 0)
            IspResult.avgCathode2Anode /= count;
        //IspResult.minCathode2Anode = *min_element(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
        //IspResult.maxCathode2Anode = *max_element(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
        //IspResult.avgCathode2Anode = std::accumulate(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end(), 0.0) / listAnode2CathodeDistance.size();
    }

    if (listCathode2CaseDistance.size() != 0)
    {
        IspResult.minCathode2Case = *min_element(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
        IspResult.maxCathode2Case = *max_element(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
        IspResult.avgCathode2Case = std::accumulate(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end(), 0.0) / listCathode2CaseDistance.size();
    }

    IspResult.mOuterRoi = batteryROI;
    IspResult.yA2 = (mIspBase.mDirection == 1) ? batteryROI.br().y : batteryROI.y;
    IspResult.sCathode2Anode = std::move(listAnode2CathodeDistance);
    IspResult.sAnode2Case = std::move(listAnode2CaseDistance);
    IspResult.sCathode2Case = std::move(listCathode2CaseDistance);
    IspResult.vtAnodes = std::move(listAnodeOri);
    IspResult.vtCathodes = std::move(listCathodeOri);
    IspResult.sXPos = std::move(listPolePos);
    IspResult.vtAno2CathodDecision = std::move(listAno2CathodDecision);
    IspResult.vtAno2CaseDecision = std::move(listAno2CaseDecision);
    IspResult.vtCathode2CaseDecision = std::move(listCathode2CaseDecision);
    IspResult.Anodes2CaseDecision = isAnode2CaseOK ? "OK" : "NG";
    IspResult.Anode2CathodeDecision = isAnode2CathodeOK ? "OK" : "NG";
    IspResult.Cathode2CaseDecision = isCathode2CaseOK ? "OK" : "NG";
    FinalDecision = (isAnode2CaseOK && isAnode2CathodeOK && isCathode2CaseOK) ? "OK" : "NG";
    
    bool anodeNoPoleNG = (A2BValid.mIsEnable || C2AValid.mIsEnable)
                            && (IspResult.mAnodePoles.mLeftNoPole < mIspJR.mOneSidePoleNumber
                            || (IspResult.mAnodePoles.mPoles.size() - IspResult.mAnodePoles.mLeftNoPole) < mIspJR.mOneSidePoleNumber);

    bool cathodeNoPoleNG = (A2BValid.mIsEnable || C2BValid.mIsEnable)
                            && (IspResult.nNoCathodeLeft < mIspJR.mOneSidePoleNumber
                            || (IspResult.mCathodePos.size() - IspResult.nNoCathodeLeft) < mIspJR.mOneSidePoleNumber);

    if (lstPolePositionErr.size() > 0
        || (mIspJR.mOneSidePoleNumber > 0 && (anodeNoPoleNG || cathodeNoPoleNG)))
    {
        FinalDecision = "NG";
        errMsg.push_back("Number of detected poles is not sufficient");
    }

    if (IspResult.Anode2CathodeDecision == "NG" && C2AValid.mIsEnable)
    {
        FinalDecision = "NG";
        errMsg.push_back("Cathode->Anode Distance NG");
    }

    if (IspResult.Anodes2CaseDecision == "NG"
        && A2BValid.mIsEnable)
    {
        FinalDecision = "NG";
        errMsg.push_back("Base->Anode Distance NG");
    }

    if (IspResult.Cathode2CaseDecision == "NG"
        && C2BValid.mIsEnable)
    {
        FinalDecision = "NG";
        errMsg.push_back("Base->Cathode Distance NG");
    }

    const bool checkVarA2B = !(mIspPole.mVariationAnode2Case < 1e-9);
    if ((A2BValid.mIsEnable || C2AValid.mIsEnable) && checkVarA2B)
    {
        bool isNGA2Case = abs(IspResult.maxAnode2Case - IspResult.minAnode2Case) > mIspPole.mVariationAnode2Case;
        IspResult.Anode2CaseVariationDecision = isNGA2Case ? "NG" : "OK";
        if (isNGA2Case)
        {
            FinalDecision = "NG";
        }
    }

    if (mInspectingItems.CHECK_CENTER_PIN && !IspResult.isPinExist)
    {
        FinalDecision = "NG";
        errMsg.push_back("The Pin not exist");
    }

    IspResult.mOneSidePoleNo = mIspJR.mOneSidePoleNumber;
    IspResult.finalDecision = FinalDecision;
    IspResult.mAnodePoles.mValidAnode2CaseRange = mIspPole.mValidAnode2CaseRange;
    IspResult.mAnodePoles.mValidCathode2AnodeRange = mIspPole.mValidCathode2AnodeRange;
    IspResult.mAnodePoles.mValidCathode2CaseRange = mIspPole.mValidCathode2CaseRange;
    IspResult.mAnodePoles.mVariationAnode2Case = mIspPole.mVariationAnode2Case;
    IspResult.cTime = GetStrCurrentTime();
    IspResult.mErrMsg = errMsg;
    auto tmp1 = t.Stop();
    IspResult.mProTime = tmp1.count();

    return ERR_CODE::OK;
}

auto CTBatteryInspector::Inspect(const std::vector<cv::Mat>& inImg, CTBatteryInspectorResult& BIresult) const -> ERR_CODE
{
    if (inImg.empty())
    {
        BIresult.Description = "Image empty or not support!";
        return ERR_CODE::errImageEmpty;
    }

    cv::Mat avgImage = AverageSliceImage(inImg);
    CTBatteryInspectorResult avgResult;
    ERR_CODE resultCode = Inspect(avgImage, avgResult);
    if (resultCode == ERR_CODE::OK && mSliceNo != 1)
    {
        std::vector<std::pair<int, double>> ctfList;
        for (int i = 0; i < inImg.size(); i++)
        {
            cv::Rect ctfROI = avgResult.mPoleRegionRoi;
            ctfROI.y += ctfROI.height * 3 / 4;
            ctfROI.height = ctfROI.height / 4;
            ctfROI = CreateROI(ctfROI, inImg[i].size());

            auto ctf = CalculateCTF(inImg[i](ctfROI), mIspPole.mPolesProminenceThreshold, mIspPole.mPolesDistanceRange.GetLower(), 3);
            ctfList.emplace_back(i, ctf);
        }

        std::sort(ctfList.begin(), ctfList.end(), [](auto& lsh, auto& rsh) { return lsh.second > rsh.second; });

        std::vector<cv::Mat> insImg;
        for (int idx = 0; idx < ctfList.size() && idx < mAvgSliceNo; idx++)
        {
            insImg.push_back(inImg[ctfList[idx].first]);
        }
        //avgResult.pole
        cv::Mat avgIspImage = AverageSliceImage(insImg);
        resultCode = Inspect(avgIspImage, BIresult);

    }
    else
    {
        BIresult = avgResult;
    }

    if (resultCode != ERR_CODE::OK || resultCode != ERR_CODE::NG || resultCode != ERR_CODE::NA)
    {
        BIresult.resImg = avgImage;
        ConvertRGB(BIresult.resImg, BIresult.resImg);

        cv::Point topLeft(mIspBase.mRoi.x, mIspBase.mRoi.y);
        cv::Point bottomRight(mIspBase.mRoi.x + mIspBase.mRoi.width, mIspBase.mRoi.y + mIspBase.mRoi.height);
        if (topLeft.x < 0) topLeft.x = 0;
        if (topLeft.y < 0) topLeft.y = 0;
        int imgWidth = BIresult.resImg.cols;
        int imgHeight = BIresult.resImg.rows;
        if (bottomRight.x > imgWidth - 1) bottomRight.x = imgWidth - 1;
        if (bottomRight.y > imgHeight - 1) bottomRight.y = imgHeight - 1;
        cv::rectangle(BIresult.resImg, topLeft, bottomRight, cv::Scalar(0, 225, 255), 2);
    }
    return resultCode;
}

auto CTBatteryInspector::FindEndpointCathodeAuto(const cv::Mat& inputImg,
                                                 VecPeak& polePeak,
                                                 bool enableReference,
                                                 int thicknessPole,
                                                 int disVerifi,
                                                 float numVariance
) const -> VecPoint
{
    VecPoint Results;
    if (!inputImg.empty())
    {
        int rangeP = (int)floor(thicknessPole / 2);
        const size_t sizeWid = inputImg.cols;

        std::vector<int> countValley(sizeWid, 0);
        std::vector<int> endCount(sizeWid, 0);
        std::vector<int> tmp(sizeWid, 0);
        std::vector<int> startThreshold(sizeWid, 0);

        for (int k = inputImg.rows - 1; k > 0; k--)
        {
            VecFloat reduceVec = ReduceBlackBackgroundMat(inputImg.row(k), 0, 1);
            // Determine the pole position from accumulation graph
            FindPeaks findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
            findPeaks.Process(reduceVec);
            auto peakResult = findPeaks.GetPeakResult(0.01, mIspPole.mPolesDistanceRange.GetLower());
            int sizePeak = peakResult.size();

            FindPeaks findValleys = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
            findValleys.Process(reduceVec);
            auto valleyResult = findValleys.GetPeakResult(0.01, mIspPole.mPolesDistanceRange.GetLower());
            int sizeValley = valleyResult.size();
            
            auto maxPro = std::max_element(polePeak.begin(), polePeak.end(),
                [](const Peak& a, const Peak& b) {
                    return a.prominence < b.prominence;
                }
            );

            for (auto& pole : polePeak)
            {
                int verifyDis = disVerifi;
                std::vector<bool> coutinue(sizeWid, 0);
                for (int i = 0; i < sizePeak; i++)
                {
                    if (peakResult[i].index - rangeP <= pole.index && pole.index <= peakResult[i].index + rangeP)
                    {
                        coutinue[pole.index] = true;
                        countValley[pole.index] = 0;
                    }
                }
                if (coutinue[pole.index] != true) {
                    countValley[pole.index] ++;
                    if (countValley[pole.index] == 1)
                    {
                        tmp[pole.index] = k ;
                        verifyDis = disVerifi * abs(1 - pole.prominence/maxPro->prominence);
                    }
                }
                if (countValley[pole.index] > verifyDis && endCount[pole.index] < 1)
                {
                    startThreshold[pole.index] = tmp[pole.index];
                    endCount[pole.index]++;
                }
            }
        }
        bool minPeakinLeft = true;
        float minPromiPeak = polePeak[0].prominence;
        for (auto& p : polePeak)
        {
            if (p.prominence < minPromiPeak)
            {
                minPromiPeak = p.prominence;
                if (p.index > inputImg.cols / 2)
                {
                    minPeakinLeft = false;
                }
            }
            Results.push_back(cv::Point(p.index, startThreshold[p.index]));
        }

        //Reference Position Pole
        if (enableReference)
        {
            int middleIndex;
            int numRes = Results.size();
            if (Results.size() % 2 != 0)
            {
                middleIndex = Results.size() / 2;
            }
            else
            {
                middleIndex = -1;
            }

            float mean;
            int startLoop;
            int endLoop;
            if (minPeakinLeft)
            {
                startLoop = 0;
                endLoop = numRes;
            }
            else
            {
                startLoop = numRes - 1;
                endLoop = -1;
            }
            int i = startLoop;
            while (true)
            {
                if (i == middleIndex)
                {
                    mean = (float)(Results[i - 1].y + Results[i + 1].y) / 2.0;
                }
                else if (i == 0)
                {
                    mean = (float)(Results[i].y + Results[i + 1].y + Results[numRes - 1].y + Results[numRes - 2].y) / 4.0;
                }
                else if (i == numRes - 1)
                {
                    mean = (float)(Results[i].y + Results[i - 1].y + Results[numRes - i].y + Results[numRes - (i - 1)].y) / 4.0;
                }
                else
                {
                    mean = (float)(Results[i - 1].y + Results[i + 1].y + Results[numRes - 1 - i].y + Results[numRes - 1 - (i - 1)].y + Results[numRes - 1 - (i + 1)].y) / 5.0;
                }

                if (abs(Results[i].y - mean) / mean > numVariance)
                {
                    Results[i].y = mean;
                }
                if (startLoop > endLoop)
                {
                    i--;
                    if (i <= endLoop)
                    {
                        break;
                    }
                }
                else
                {
                    i++;
                    if (i >= endLoop)
                    {
                        break;
                    }
                }
            }
        }
    }

    return Results;
}

auto CTBatteryInspector::FindMode(VecInt numVector) const -> int
{
    int mode = -1;
    if (!numVector.empty())
    {
        size_t n = *std::max_element(numVector.begin(), numVector.end());
        std::vector<int> histogram(n + 1, 0);
        for (int i = 0; i < numVector.size(); ++i)
            ++histogram[numVector[i]];
        mode = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
    }
    return mode;
}

auto CTBatteryInspector::GetPeakResult(FindPeaks& findPeaks, VecFloat reduceVec, float prominenceTh, float& minDis, float& maxDis, float alpha) const ->VecPeak
{
    std::vector<Peak> peakResult;

    if (!reduceVec.empty())
    {
        double mean = 0.0, stdDev = 0.0;
        findPeaks.Process(reduceVec);
        //float use_threshold = mIspPole.mPolesProminenceThreshold < 1e-3 ? prominenceTh : mIspPole.mPolesProminenceThreshold;
        bool use_min = (mIspPole.mPolesDistanceRange.GetLower() < 1e-3);
        if (use_min)
        {
            peakResult = findPeaks.GetPeakResult(prominenceTh);
            VecInt vtDistance;
            vtDistance.reserve(peakResult.size() - 1);
            for (int i = 1; i < peakResult.size(); i++)
            {
                vtDistance.push_back(abs(peakResult[i].index - peakResult[i - 1].index));
            }

            if (!vtDistance.empty())
            {
                int mode = FindMode(vtDistance);
                if (mode > 0)
                {
                    float beta = 0.3;
                    int minRange = mode - round(mode * beta);
                    int maxRange = mode + round(mode * beta);
                    for (int i = vtDistance.size() - 1; i >= 0; i--)
                    {
                        if (vtDistance[i] > maxRange || vtDistance[i] < minRange)
                            vtDistance.erase(vtDistance.begin() + i);
                    }
                }
            }

            if (!vtDistance.empty())
            {
                std::tie(mean, stdDev) = MeanStdDevLite(vtDistance);

                float minStdDev = 1.0;
                if (stdDev > minStdDev)
                {
                    minDis = mean - alpha * stdDev;
                    maxDis = mean + alpha * stdDev;
                }
                else
                {
                    minDis = *std::min_element(vtDistance.begin(), vtDistance.end()) - alpha * stdDev;
                    maxDis = *std::max_element(vtDistance.begin(), vtDistance.end()) + alpha * stdDev;
                }

                peakResult = findPeaks.GetPeakResult(prominenceTh, minDis);
            }
        }
        else
        {
            minDis = mIspPole.mPolesDistanceRange.GetLower();
            maxDis = mIspPole.mPolesDistanceRange.GetUpper();

            peakResult = findPeaks.GetPeakResult(prominenceTh, mIspPole.mPolesDistanceRange.GetLower());
        }
    }

    return peakResult;
}

auto CTBatteryInspector::FindCathodeXPositionAuto(const VecFloat & signal, Rangei & polesDistanceRange, int centerNeglectionWidth, PeakType type) const -> VecPeak
{
    std::vector<Peak> vtXPositions;
    int centerWidthNeglection = std::max<int>(signal.size() / 20, std::min<int>(signal.size() / 2, centerNeglectionWidth));
    if (!signal.empty())
    {
        const size_t size = signal.size();
        const int midIdx = std::floor(size / 2.0);
        const float ratioCenter = 0.7;
        float haftCenterNeglectionWidth = centerWidthNeglection / 2.0;
        const int centerLeftIdx = std::max<int>(0, midIdx - ratioCenter * haftCenterNeglectionWidth);
        const int centerRightIdx = std::min<int>(size - 1, midIdx + ratioCenter * haftCenterNeglectionWidth);
        const double alpha = 3.0;

        VecFloat leftReduceVec, rightReduceVec, centerNeglectReduceVec;
        leftReduceVec.insert(leftReduceVec.begin(), signal.begin(), signal.begin() + centerLeftIdx);
        rightReduceVec.insert(rightReduceVec.begin(), signal.begin() + centerRightIdx, signal.end());
        centerNeglectReduceVec.insert(centerNeglectReduceVec.begin(), signal.begin() + centerLeftIdx, signal.begin() + centerRightIdx);

        // Determine the pole position from accumulation graph
        FindPeaks findPeaks = FindPeaks(type, PeakFindingMethod::Prominence);
        findPeaks.Process(centerNeglectReduceVec);
        auto peakResult = findPeaks.GetPeakResult(0);

        VecFloat vtProminence, vtDistance;
        vtProminence.reserve(peakResult.size());
        for (auto it = peakResult.begin(); it != peakResult.end(); it++)
        {
            vtProminence.push_back(abs(it->prominence));
        }

        if (!vtProminence.empty() && !rightReduceVec.empty() && !leftReduceVec.empty())
        {
            double mean, stdDev;
            std::tie(mean, stdDev) = MeanStdDevLite(vtProminence);
            vtProminence.clear();
#if _DEBUG_CATHODE_X_AUTO
            int imageHeight = 200;
            cv::Mat ImgMat = cv::Mat::zeros(cv::Size(size, imageHeight), CV_8U);
            cv::Mat debugMat = cv::Mat::zeros(ImgMat.size(), CV_8UC3);
            cv::Rect centerROI = cv::Rect(centerLeftIdx, 0, centerRightIdx - centerLeftIdx, imageHeight);
            cv::Point textPos(centerROI.width / 3, 15);
            Drawing drawTool;
            drawTool.padding_Y = 15;
            drawTool.yTopOrigin = false;
            drawTool.color = COLOR_CV_YELLOW;
            cv::Mat drawPeakMat = DrawPeakInfo(ImgMat(centerROI), centerNeglectReduceVec, peakResult, drawTool, 0);
            drawTool.color = COLOR_CV_RED;
            drawTool.Plot(drawPeakMat, "m: " + std::to_string(mean), textPos, 1, true);
            drawTool.Plot(drawPeakMat, "s: " + std::to_string(stdDev), textPos + cv::Point(0, 15), 1, true);
            drawPeakMat.copyTo(debugMat(centerROI));
#endif // _DEBUG_CATHODE_X_AUTO
            float prominenceTh = (mIspCathode.mProminence < 1e-3) ? mean : mIspCathode.mProminence;

            float leftMinPolesDistance = 0.0, leftMaxPolesDistance = 0.0;
            auto leftPeakResult = GetPeakResult(findPeaks, leftReduceVec, prominenceTh, leftMinPolesDistance, leftMaxPolesDistance, alpha);

            float rightMinPolesDistance = 0.0, rightMaxPolesDistance = 0.0;
            auto rightPeakResult = GetPeakResult(findPeaks, rightReduceVec, prominenceTh, rightMinPolesDistance, rightMaxPolesDistance, alpha);

            if (leftPeakResult.size() != 0 && rightPeakResult.size() != 0
                && leftMinPolesDistance > 0 && rightMinPolesDistance > 0
                && leftMaxPolesDistance > 0 && rightMaxPolesDistance > 0)
            {
                vtXPositions.reserve(leftPeakResult.size() + rightPeakResult.size());
                vtXPositions.insert(vtXPositions.begin(), leftPeakResult.begin(), leftPeakResult.end());

                for (int i = 0; i < rightPeakResult.size(); i++)
                {
                    auto xPos = rightPeakResult[i];
                    xPos.index += centerRightIdx;
                    vtXPositions.push_back(std::move(xPos));
                }

#if _DEBUG_CATHODE_X_AUTO
                drawTool.color = COLOR_CV_YELLOW;
                cv::Rect leftROI = cv::Rect(0, 0, centerLeftIdx, imageHeight);
                drawPeakMat = DrawPeakInfo(ImgMat(leftROI), leftReduceVec, leftPeakResult, drawTool, 0);
                textPos = cv::Size(leftROI.width / 2, 15);
                drawTool.color = COLOR_CV_RED;
                drawTool.Plot(drawPeakMat, "mimDis: " + std::to_string(leftMinPolesDistance), textPos, 1, true);
                drawTool.Plot(drawPeakMat, "maxDis: " + std::to_string(leftMaxPolesDistance), textPos + cv::Point(0, 15), 1, true);
                drawPeakMat.copyTo(debugMat(leftROI));
                drawTool.color = COLOR_CV_YELLOW;
                cv::Rect rightROI = cv::Rect(centerRightIdx, 0, size - centerRightIdx, imageHeight);
                drawPeakMat = DrawPeakInfo(ImgMat(rightROI), rightReduceVec, rightPeakResult, drawTool, 0);
                textPos = cv::Size(rightROI.width / 2, 15);
                drawTool.color = COLOR_CV_RED;
                drawTool.Plot(drawPeakMat, "mimDis: " + std::to_string(rightMinPolesDistance), textPos, 1, true);
                drawTool.Plot(drawPeakMat, "maxDis: " + std::to_string(rightMaxPolesDistance), textPos + cv::Point(0, 15), 1, true);
                drawPeakMat.copyTo(debugMat(rightROI));
                drawPeakMat.release();
                cv::rectangle(debugMat, centerROI, COLOR_CV_RED, 2);
                cv::line(debugMat, cv::Point(midIdx - haftCenterNeglectionWidth), cv::Point(midIdx - haftCenterNeglectionWidth, imageHeight), COLOR_CV_GREEN, 2);
                cv::line(debugMat, cv::Point(midIdx + haftCenterNeglectionWidth), cv::Point(midIdx + haftCenterNeglectionWidth, imageHeight), COLOR_CV_GREEN, 2);
                // show_window("Find Cathode X Position Auto", debugMat);
#endif // _DEBUG_CATHODE_X_AUTO
                polesDistanceRange = Rangei((leftMinPolesDistance + rightMinPolesDistance) / 2.0, (leftMaxPolesDistance + rightMaxPolesDistance) / 2.0);
            }
        }
    }
    return vtXPositions;
}

auto CTBatteryInspector::FindCathodePos(cv::Mat const& src) const -> int
{
    int kSize = 11;

    if (src.channels() != 1 || src.cols <= kSize || src.rows <= kSize) {
        return 0;
    }

    int w = src.cols / 2;
    cv::Rect roi = CreateROI(10, 0, w / 2, src.rows, src.size());
    cv::Mat reduce;
    cv::GaussianBlur(src(roi), reduce, cv::Size(kSize, kSize),0);
    cv::normalize(reduce, reduce, 0, 255, cv::NormTypes::NORM_MINMAX);
    cv::reduce(reduce, reduce, 1, cv::REDUCE_AVG, CV_32FC1);
    std::vector<float> signal = reduce;

    xvt::FindPeaks findPeaks0 = xvt::FindPeaks(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
    findPeaks0.Process(signal);
    auto signalResult = findPeaks0.GetPeakResult(10, 5);

    int valley = signalResult.empty() ? 0 : signalResult.back().index;

    std::vector<float> diffVec(reduce.rows, 0);
    for (int i = std::max(valley, 1), n = reduce.rows - 1; i < n; i++)
    {
        diffVec[i] = signal[i + 1] - signal[i - 1];
    }
    xvt::FindPeaks findPeaks = xvt::FindPeaks(xvt::PeakType::Peak, xvt::PeakFindingMethod::Prominence);
    findPeaks.Process(diffVec);
    auto mean = std::accumulate(diffVec.begin(), diffVec.end(), 0.0) / diffVec.size();
    auto pro = std::max(2*mean, 5.0);
    auto peakResult = findPeaks.GetPeakResult(pro, 5);

    // Get the first peak from the bottom
    int peak1 = peakResult.empty() ? 0 : peakResult.back().index;
    // Get the max peak from the bottom
    /*auto maxPeakPtr = std::max_element(peakResult.begin(), peakResult.end(), xvt::SmallerProminence);
    int peak1 = maxPeakPtr != peakResult.end()?maxPeakPtr->index:0;*/
    
#if _DEBUG_CATHODE_X
    cv::Mat colorImg;
    ConvertRGB(src, colorImg);
    cv::Mat colorImg2 = colorImg(roi);
    Drawing drawTool;
    drawTool.Plot(colorImg2, diffVec, 0, roi.height, false);
    auto meanPos = drawTool.GetDrawPoint(mean, colorImg2.rows / 2);
    auto proPos = drawTool.GetDrawPoint(pro, colorImg2.rows / 2);
    cv::line(colorImg, cv::Point(meanPos.x, 0), cv::Point(meanPos.x, colorImg2.rows), cv::Scalar(0, 255, 0));
    cv::line(colorImg, cv::Point(proPos.x, 0), cv::Point(proPos.x, colorImg2.rows), cv::Scalar(0, 0, 255));

    drawTool.color = cv::Scalar(0, 0, 255);
    drawTool.Plot(colorImg2, signal, 0, roi.height, false);
    cv::line(colorImg, cv::Point(0, peak1), cv::Point(colorImg.cols, peak1), cv::Scalar(0, 255, 0));
    cv::line(colorImg, cv::Point(0, valley), cv::Point(colorImg.cols, valley), cv::Scalar(255, 0, 0));
#endif // _DEBUG_CATHODE_X

    return peak1;
}

auto CTBatteryInspector::FindTopReferenceLine(const cv::Mat& src, const cv::Rect& batteryROI, cv::Rect& batteryCenter) const -> InspectionResult
{
    auto finalResult = InspectionResult(EResult::ER, "FindTopReferenceLine: Image Empty");
    auto w = std::max(mIspJR.mCenterNeglectionWidth>>3, 15);
    if (!src.empty() && !batteryROI.empty())
    {
        int yROIPos = 0;
        yROIPos = FindCathodePos(src(batteryROI));

        batteryCenter.y = yROIPos + batteryROI.y;
        
        int heightJR = batteryROI.br().y - batteryCenter.y;

        cv::Rect jrROI = CreateROI(batteryROI.x + batteryROI.width / 4, batteryCenter.y+ heightJR*(1.0/8.0), batteryROI.width / 2, heightJR/2, batteryROI);
        if (!jrROI.empty() && jrROI.height > 10)
        {
            cv::Mat thresholdImg;
            cv::Mat reduceThresholdImg;
            int kSize = 11;
            cv::normalize(src(jrROI), reduceThresholdImg, 0, 255, cv::NormTypes::NORM_MINMAX);
            cv::reduce(reduceThresholdImg, reduceThresholdImg, 0, cv::REDUCE_AVG, CV_8U);
            cv::blur(reduceThresholdImg, reduceThresholdImg, cv::Size(w, 1));
            auto m = cv::mean(reduceThresholdImg);
            auto thd = std::max(m[0]*0.92, 40.0);
            cv::threshold(reduceThresholdImg, thresholdImg, thd, 255, cv::THRESH_BINARY_INV);

            auto kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_CROSS, cv::Size(15, 1));
            cv::morphologyEx(thresholdImg, thresholdImg, cv::MorphTypes::MORPH_CLOSE, kernel);

            VecPoint contour2 = FindMaxContour(thresholdImg, CompareWidth, cv::CHAIN_APPROX_NONE);
            cv::Rect centerROI = cv::boundingRect(contour2);
            batteryCenter.x = centerROI.x + jrROI.x;
            batteryCenter.width = centerROI.width;
            batteryCenter.height = heightJR;

#if _DEBUG_TOP_REF_LINE
            cv::Mat colorImg;
            ConvertRGB(src, colorImg);
            cv::Mat colorImg2 = colorImg(jrROI);

            VecFloat signal = reduceThresholdImg;
            std::vector<float> diffVec(signal.size(), 0);
            for (int i = 1, n = signal.size() - 1; i < n; i++)
            {
                diffVec[i] = signal[i + 1] - signal[i - 1];
            }

            Drawing drawTool;
            drawTool.color = cv::Scalar(0, 0, 255);
            drawTool.Plot(colorImg2, 0, jrROI.width, signal,false);
            auto meanPos = drawTool.GetDrawPoint(10, m[0]);
            auto proPos = drawTool.GetDrawPoint(10, thd);
            cv::line(colorImg2, cv::Point(0, meanPos.y), cv::Point(colorImg2.cols, meanPos.y), cv::Scalar(0, 255, 0));
            cv::line(colorImg2, cv::Point(0, proPos.y), cv::Point(colorImg2.cols, proPos.y), cv::Scalar(0, 0, 255));

            drawTool.color = cv::Scalar(0, 255, 255);
            drawTool.Plot(colorImg2, 0, colorImg2.cols, diffVec,  false);

            cv::line(colorImg, cv::Point(batteryCenter.x, jrROI.y), cv::Point(batteryCenter.x, jrROI.y + jrROI.height), cv::Scalar(0, 255, 0));
            cv::line(colorImg, cv::Point(batteryCenter.x+batteryCenter.width, jrROI.y), cv::Point(batteryCenter.x + batteryCenter.width, jrROI.y + jrROI.height), cv::Scalar(0, 255, 0));

#endif // _DEBUG_TOP_REF_LINE

            finalResult(EResult::OK, "");
        }
        else
        {
            finalResult(EResult::ER, "FindTopReferenceLine: Cannot find Center Point.");
        }
    }
    return finalResult;
}

auto GetMeanInstensity(cv::Mat const& img, cv::Point ca, int h, bool topPart) ->float
{
    int const w = 3;
    cv::Rect roi = topPart ? CreateROI(ca.x - w / 2, ca.y - h, w, h, img.size()) : CreateROI(ca.x - w / 2, ca.y, w, h, img.size());

    auto m = 0.0;
    if (!roi.empty()) {
        m = cv::mean(img(roi))[0];
    }

#if 0
    //cv::Mat cloneImg = img.clone();
    cv::rectangle(img, roi, cv::Scalar(255));
#endif

    return m;
}

auto CTBatteryInspector::VerifyParameters(BatteryInspectionResult& BIresult) const -> ERR_CODE
{
    ERR_CODE result = ERR_CODE::OK;
    std::string message = "";

    // Check pixel size parameter setting
    if (mPixelSize <= 0)
    {
        message = "Setting: Pixel size can't set to 0";
        result = ERR_CODE::errPixelSize;
    }

    // Check outer roi parameter setting
    if (mIspBase.mRoi.empty())
    {
        message = "Setting: Outer ROI can't empty";
        result = ERR_CODE::errBatteryROI;
    }

    BIresult.Description = message;
    return result;
}

auto CTBatteryInspector::FindAnodeByLineTracing(const cv::Mat& inputImg,
                                                VecPoint& anodes,
                                                cv::Point startPoint,
                                                int limitVal,
                                                int stepHorizontal,
                                                int stepVertical,
                                                float breakThreshold,
                                                int borderDistance,
                                                bool restrictMove,
                                                int moveAllow,
                                                cv::Point offset,
                                                int borderCheck
) const -> InspectionResult
{
    cv::Mat drawOuput;
    /*stepVertical = 7;
    stepHorizontal = 3;*/
    bool writeDebugPole = drawOuput.empty() ? false : true;

    //if (writeDebugPole) { cv::circle(drawOuput, startPoint, 1, cv::Scalar(0, 255, 0)); }

    int stepSkip = 1;
    int stepFrog = 1;
    int stopCheck = stepVertical + stepFrog + 1;
    int borderOffset = 1;

    cv::Rect moveRect;

    moveRect.width = 2 * stepHorizontal + 1;
    moveRect.height = stepVertical + 1;

    //REPOSITION OF STARTING POINT BASED ON THE LOWEST INTENSITY POINT NEARBY//
    /*cv::Rect rectStart(startPoint.x - stepHorizontal, startPoint.y - stepVertical, moveRect.width, stepVertical);
    RefineROI(rectStart, inputImg.size());
    double minValue, maxValue;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(inputImg(rectStart), &minValue, &maxValue, &minLoc, &maxLoc);
    startPoint = minLoc + rectStart.tl();*/

    int const& startPointX = startPoint.x - offset.x;
    int const& startPointY = startPoint.y - offset.y;

    if (borderCheck == -1)
    {
        if (2 * borderDistance < startPointX)
        {
            //borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (borderDistance < startPointX && borderDistance > 2)
        {
            borderOffset = 2;
        }
        else
        {
            borderOffset = 0;
        }

    }
    else if (borderCheck == 1)
    {
        if (startPointX < inputImg.cols - 2 * borderDistance)
        {
            // borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (startPointX < inputImg.cols - borderDistance && borderDistance > 2)
        {
            borderOffset = 2;
        }
        else
        {
            borderOffset = 0;
        }
    }
    //--------------END REPOSITION-------------------------------------------//
    ///////////////////////////////////////////////////////////////////////////
    //cv::circle(drawOuput, startPoint, 1, cv::Scalar(255, 255, 0));

    cv::Point currentPoint(startPointX, startPointY);
    int numberOfBlankLine = 5;


    //cv::circle(ROI4Draw, startPoint, 1, cv::Scalar(0, 255, 0));
    /*int CathodeOffsetX = batteryROI.x + BatInsp.borderPadding;
    int CathodeOffsetY = yA2 + jrPos.first;*/
    cv::Point prePoint = currentPoint;
    //cv::Point offset(batteryROI.x + BatInsp.borderPadding, yA2 + jrPos.first);
    int stepCheck = 0;
    int imgColsHalf = inputImg.cols / 2;
    cv::Mat reduceRes;
    //LINE TRACING FROM STARTING POINT TO END OF ROI //
    int it = 0;
    int leftBorderX = startPointX - borderOffset; //when checking the left border pole
    int rightBorderX = startPointX + borderOffset; //when checking the right border pole

    int leftLimit = stepHorizontal;
    int rightLimit = inputImg.cols - stepHorizontal;

    assert(leftLimit > 0 && rightLimit > 0);
    cv::Point outResult = currentPoint;

    for (it = currentPoint.y - stepSkip; it > stopCheck; it -= stepFrog)
    {
        int currentPointXCord = currentPoint.x;

        moveRect.x = currentPointXCord - stepHorizontal;
        moveRect.y = it - stepVertical;
        moveRect.width = 2 * stepHorizontal + 1;
        if (borderCheck == -1)
        {
            //if (borderDistance < startPointX)
            //Point is out of the left border
            if (leftBorderX >= moveRect.x)
            {
                moveRect.width = moveRect.br().x - leftBorderX;
                moveRect.x = leftBorderX;
            }
        }
        else if (borderCheck == 1)
        {
            if (moveRect.br().x >= rightBorderX)
            {
                moveRect.width = rightBorderX - moveRect.x + 1;
            }
        }

        float avgVal = 0.0;
        RefineROI(moveRect, inputImg.size());
        cv::Mat tmpMat = inputImg(moveRect);

        if (!tmpMat.empty())
        {
            //Vertical projection
            cv::reduce(tmpMat, reduceRes, 0, cv::REDUCE_AVG, CV_32FC1);
            int reduceCol = reduceRes.cols;

        #if 0
            //CENTER WEIGHT
            for (int w = 0; w < reduceCol; w++)
            {
                float distance = abs((float)(reduceCol / 2.0) - w);
                float scaleWeight = 1;
                reduceRes.at<float>(0, w) -= scaleWeight / distance;
            }

            //DIRECTIONAL WEIGHT
            float smallWeight = 0.5;
            if (currentPointXCord < imgColsHalf)
            {
                reduceRes.at<float>(0, stepHorizontal + 1) -= smallWeight;
            }
            else
            {
                reduceRes.at<float>(0, stepHorizontal - 1) -= smallWeight;
            }
        #endif
            auto maxPos = std::max_element((float*)reduceRes.ptr(0), (float*)reduceRes.ptr(0) + reduceRes.cols);
            avgVal = *maxPos;
            currentPoint = cv::Point(moveRect.x + std::distance((float*)reduceRes.ptr(0), maxPos), it);

            //Right border
            if (borderCheck == 1 && currentPoint.x >= rightBorderX)
            {
                currentPoint.x = rightBorderX;
            }

            //Left border
            if (borderCheck == -1 && leftBorderX >= currentPoint.x)
            {
                currentPoint.x = leftBorderX;
            }

            //restrict movement
            if (restrictMove)
            {
                if (currentPoint.x < imgColsHalf && (currentPoint.x - startPointX) < -moveAllow)
                {
                    currentPoint.x = startPointX;
                }
                if (currentPoint.x > imgColsHalf && (currentPoint.x - startPointX) > moveAllow)
                {
                    currentPoint.x = startPointX;
                }
            }

            if (currentPoint.y >= limitVal)
            {
                if (avgVal < breakThreshold)
                {
                    stepCheck++;
                    if (stepCheck >= numberOfBlankLine)
                    {
                        if (writeDebugPole)
                        {
                            //cv::line(drawOuput, currentPoint, prePoint, cv::Scalar(255, 200, 0));
                            prePoint = currentPoint;
                        }
                        //return(currentPointYCord);
                        break;
                    }
                }
                else
                {
                    anodes.push_back(currentPoint + offset);
                    stepCheck = 0;
                }
            }
            else
            {
                break;
            }
        }
        else // Move out of image
        {
            //Use the currentPoint as the output
            //assert(false);
            break;
        }
    }
    //---------END LINE TRACING --------------------------//
    ///////////////////////////////////////////////////////

    //currentPoint.y = currentPoint.y + stepCheck * stepFrog;
    reduceRes.release();
    return InspectionResult();
}

auto CTBatteryInspector::FindAnodeByProminence(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode) const->InspectionResult
{
    cv::Mat Fimg;
    auto anodeResultProminence = InspectionResult();
    if (!inputImg.empty())
    {
        inputImg.convertTo(Fimg, CV_32F);

        float thresholdValue = 8.0;
        int limitPoint = 100;
        VecPoint oldStations;
        VecPoint nextStations;

        // The points to be considered for arrival
        nextStations.push_back(cv::Point(startAnode.x, startAnode.y));
        while (!nextStations.empty() || oldStations.size() > limitPoint)
        {
            float medianInten = 0.0;
            // Sub Image
            int startRectX = nextStations[0].x - 1;
            int startRectY = nextStations[0].y - 1;
            cv::Rect subRect = cv::Rect(startRectX, startRectY, nextStations.back().x - nextStations[0].x + 3, 3);
            RefineROI(subRect, Fimg.size());
            cv::Mat searchSubImg = Fimg(subRect);

            // Priority Matrix
            cv::Mat oneMatrix = cv::Mat::ones(1, subRect.width, CV_32F);
            cv::Mat templateMatrix = (cv::Mat_<float>(3, 1) << 4.0, 0.0, 0.0);
            cv::Mat priorityMatrix = templateMatrix * oneMatrix;

            //// Enhance Image
            cv::Mat enhanMatrix;
            //enhanMatrix = searchSubImg.mul(priorityMatrix);
            cv::add(searchSubImg, priorityMatrix, enhanMatrix);

            // The points that have been traversed
            oldStations.insert(oldStations.end(), nextStations.begin(), nextStations.end());
            nextStations.clear();

            for (auto old : oldStations)
            {
                medianInten += Fimg.at<float>(old.y, old.x);
            }
            medianInten = medianInten / (float)oldStations.size();

            //To check the next position or element in nextStations
            auto pointEquals = [](const cv::Point& p1, const cv::Point& p2) {
                return p1 == p2;
            };

            // Find max in Enhance Matrix
            float maxVal = enhanMatrix.at<float>(0, 0);
            VecPoint maxLocs;
            for (int i = 0; i < enhanMatrix.rows; i++)
            {
                for (int j = 0; j < enhanMatrix.cols; j++)
                {
                    bool checkPoint = false;
                    for (cv::Point& p : oldStations)
                    {
                        if (p.y == (i + startRectY) && abs(p.x - (j + startRectX)) < 3)
                        {
                            checkPoint = true;
                        }
                    }
                    if (enhanMatrix.at<float>(i, j) == maxVal && !checkPoint)
                    {
                        maxLocs.push_back(cv::Point(j, i));
                    }
                    else if (enhanMatrix.at<float>(i, j) > maxVal && !checkPoint)
                    {
                        maxVal = enhanMatrix.at<float>(i, j);
                        maxLocs.clear();
                        maxLocs.push_back(cv::Point(j, i));
                    }
                }
            }
            if (!maxLocs.empty())
            {
                if (medianInten - Fimg.at<float>(maxLocs[0].y + startRectY, maxLocs[0].x + startRectX) > thresholdValue)
                {
                    maxLocs.clear();
                }
            }

            // To convert subImage matrix coordinates to srcImage coordinates
            for (auto maxLoc : maxLocs)
            {
                cv::Point maxPoint;
                maxPoint = cv::Point(maxLoc.x + startRectX, maxLoc.y + startRectY);
                if (nextStations.empty() || nextStations[0].x < maxPoint.x)
                {
                    nextStations.push_back(maxPoint);
                }
                else
                {
                    nextStations.insert(nextStations.begin(), maxPoint);
                }
            }

            //To add an element to a nextStations matrix only if it has the same brightness (intensity)
            if (!nextStations.empty())
            {
                cv::Point prePoint, nextPoint;
                prePoint.x = (nextStations[0].x - 1) < 0 ? -1 : nextStations[0].x - 1;
                prePoint.y = nextStations[0].y;
                nextPoint.x = (nextStations.back().x + 1) >= inputImg.cols ? -1 : (nextStations.back().x + 1);
                nextPoint.y = nextStations.back().y;
                while (prePoint.x != -1 && nextPoint.x != -1
                       && ((Fimg.at<float>(prePoint.y, prePoint.x) == Fimg.at<float>(nextStations[0].y, nextStations[0].x))
                           || (Fimg.at<float>(nextPoint.y, nextPoint.x) == Fimg.at<float>(nextStations[0].y, nextStations[0].x))))
                {
                    if (Fimg.at<float>(prePoint.y, prePoint.x) == Fimg.at<float>(nextStations[0].y, nextStations[0].x))
                    {
                        cv::Point maxPoint;
                        maxPoint = cv::Point(prePoint.x, prePoint.y);
                        nextStations.insert(nextStations.begin(), maxPoint);
                    }
                    if (Fimg.at<float>(nextPoint.y, nextPoint.x) == Fimg.at<float>(nextStations[0].y, nextStations[0].x))
                    {
                        cv::Point maxPoint;
                        maxPoint = cv::Point(nextPoint.x, nextPoint.y);
                        nextStations.push_back(maxPoint);
                    }
                    prePoint.x = (nextStations[0].x - 1) < 0 ? -1 : nextStations[0].x - 1;
                    prePoint.y = nextStations[0].y;
                    nextPoint.x = (nextStations.back().x + 1) >= inputImg.cols ? -1 : (nextStations.back().x + 1);
                    nextPoint.y = nextStations.back().y;
                }
            }
        }
        anode = oldStations;
        anodeResultProminence(EResult::OK, "");
    }
    else
    {
        anodeResultProminence(EResult::ER, "No Image in Anode - Algorithm Prominence!");
    }
    return anodeResultProminence;
}

auto CTBatteryInspector::FindAnodeByShape(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode, cv::Rect& subAnoRoi, int disconnectLimit, int widthAnode, int startRegion) const->InspectionResult
{
    auto anodeResultShape = InspectionResult();

    int disconnectState = 0;
    std::vector<pointValue> verifiAnode;
    std::vector<pointValue> temporAnode;
    std::vector<pointValue> backgroundLeft;
    std::vector<pointValue> backgroundRight;
    VecFloat boundaryIns;
    float boundaryInsMedian = 0;

    if (!inputImg.empty())
    {
        if (!subAnoRoi.empty())
        {

            cv::Mat subAno = inputImg(subAnoRoi);
            cv::Rect ispRoi = cv::Rect(cv::Point(0, subAno.rows - startRegion), cv::Size(subAno.cols, startRegion));

            bool outRange = false;
            int count = 0;
            while (disconnectState < disconnectLimit && (!outRange))
            {
                VecFloat findIspVec;
                cv::reduce(subAno(ispRoi), findIspVec, 0, cv::REDUCE_AVG, CV_32FC1);

                // Find Peak and Valley of Start Region Signal
                FindPeaks findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
                findPeaks.Process(findIspVec);
                auto peakResult = findPeaks.GetPeakResult(0.05);
                FindPeaks findValleys = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
                findValleys.Process(findIspVec);
                auto valleyResult = findValleys.GetPeakResult(0.05);

                if (subAnoRoi.y + ispRoi.y == 631)
                {
                    int oke = 1;
                }

                if (peakResult.size() > 1)
                {
                    for (int i = 0; i < peakResult.size(); i++)
                    {
                        if (!verifiAnode.empty() && abs(peakResult[i].index - verifiAnode.back().position.x + subAnoRoi.x) >= 2)
                        {
                            peakResult.erase(peakResult.begin() + i);
                        }
                    }
                }

                if (valleyResult.size() == 1 && peakResult.size() == 1)
                {
                    if (valleyResult[0].index > peakResult[0].index)
                    {
                        for (int i = peakResult[0].index; i >= 0; i--)
                        {
                            if (findIspVec[i] <= valleyResult[0].value)
                            {
                                Peak left;
                                left.index = i;
                                left.prominence = 0.05;
                                left.value = findIspVec[i];
                                valleyResult.emplace_back(left);
                                break;
                            }
                        }
                    }
                    else
                    {
                        for (int i = peakResult[0].index; i < findIspVec.size(); i++)
                        {
                            if (findIspVec[i] <= valleyResult[0].value)
                            {
                                Peak right;
                                right.index = i;
                                right.prominence = 0.05;
                                right.value = findIspVec[i];
                                valleyResult.emplace_back(right);
                                break;
                            }
                        }
                    }
                }

                Peak valMin;
                Peak seValMin;
                if (valleyResult.size() >= 2)
                {
                    valMin = valleyResult[0];
                    seValMin = valleyResult.back();
                    for (auto point : valleyResult)
                    {
                        if (point.value < valMin.value)
                        {
                            seValMin = valMin;
                            valMin = point;
                        }
                        else if (point.value < seValMin.value && point.index != valMin.index)
                        {
                            seValMin = point;
                        }
                    }

                    auto leftValley = (valMin.index < seValMin.index) ? valMin : seValMin;
                    auto rightValley = (valMin.index < seValMin.index) ? seValMin : valMin;
                    backgroundLeft.emplace_back(leftValley.value, cv::Point(subAnoRoi.x + leftValley.index, subAnoRoi.y + ispRoi.y));
                    backgroundRight.emplace_back(rightValley.value, cv::Point(subAnoRoi.x + rightValley.index, subAnoRoi.y + ispRoi.y));

                    auto valMax = findIspVec[leftValley.index];
                    auto seValMax = findIspVec[rightValley.index];
                    int indexValMax = leftValley.index;
                    int indexSeValMax = rightValley.index;
                    for (int i = leftValley.index; i <= rightValley.index; i++)
                    {
                        if (findIspVec[i] > valMax)
                        {
                            seValMax = valMax;
                            indexSeValMax = indexValMax;
                            valMax = findIspVec[i];
                            indexValMax = i;
                        }
                        else if (findIspVec[i] > seValMax && i != indexValMax)
                        {
                            seValMax = findIspVec[i];
                            indexSeValMax = i;
                        }
                    }
                    count++;
                    for (int i = leftValley.index; i <= rightValley.index; i++)
                    {
                        if (i != indexValMax && i != indexSeValMax)
                        {
                            if (count > 1)
                            {
                                //boundaryInsMedian = (!boundaryIns.empty())? (boundaryInsMedian * boundaryIns.size() + findIspVec[i])/(boundaryIns.size() + 1.0) : findIspVec[i];
                                boundaryIns.emplace_back(findIspVec[i]);
                                if (boundaryIns.size() > 20)
                                {
                                    boundaryIns.erase(boundaryIns.begin());
                                }
                                auto maxElement = std::max_element(boundaryIns.begin(), boundaryIns.end());
                                if (maxElement != boundaryIns.end())
                                {
                                    // Retrieve the maximum value
                                    boundaryInsMedian = *maxElement;
                                }
                            }
                        }
                    }

                    if (!boundaryIns.empty() && (seValMax <= boundaryInsMedian || valMax <= boundaryInsMedian))
                    {
                        disconnectState += 1;
                        startRegion = startRegion + 1;
                        ispRoi = cv::Rect(cv::Point(0, subAno.rows - startRegion), cv::Size(subAno.cols, 1));
                    }
                    else
                    {
                        verifiAnode.emplace_back(valMax, cv::Point(subAnoRoi.x + indexValMax, subAnoRoi.y + ispRoi.y));
                        verifiAnode.emplace_back(seValMax, cv::Point(subAnoRoi.x + indexSeValMax, subAnoRoi.y + ispRoi.y));
                        startRegion = startRegion + 1;
                        ispRoi = cv::Rect(cv::Point(0, subAno.rows - startRegion), cv::Size(subAno.cols, 1));
                        disconnectState = 0;
                    }
                }
                else
                {
                    disconnectState += 1;
                    startRegion = startRegion + 1;
                    ispRoi = cv::Rect(cv::Point(0, subAno.rows - startRegion), cv::Size(subAno.cols, 1));
                }
                if (startRegion >= subAno.rows - 5)
                {
                    outRange = true;
                }
            }

            VecPoint oldStations;
            for (auto point : verifiAnode)
            {
                oldStations.emplace_back(point.position);
            }
            anode = oldStations;
            anodeResultShape(EResult::OK, "");
        }
        else
        {
            anodeResultShape(EResult::ER, "Empty ROI in Anode - Algorithm Shape!");
        }
    }
    else
    {
        anodeResultShape(EResult::ER, "No Image in Anode - Algorithm Shape!");
    }
    return anodeResultShape;
}

void CTBatteryInspector::FindEndpointCathodeThreshold(const cv::Mat& img, VecPeak& polePeak, cv::Rect& subROI, CTBatteryInspectorResult& ispResult, int kSize) const
{
    if (img.empty() || subROI.empty()) return;
    cv::Mat src = img(subROI);

    bool isWhitePole = true;//if pole has white or dark intensity

    int const catWindowSize = (mIspCathode.mCathodeLineWindowSize > 0 ? mIspCathode.mCathodeLineWindowSize / 2 : 0) * 2 + 1;
    int const halfWindowSize = catWindowSize / 2;
    auto caRect = cv::Rect(-halfWindowSize, -subROI.height, catWindowSize, subROI.height);

    VecInt errorIdx;
    auto midPos = VecPoint(polePeak.size(), cv::Point(0, 0));
    for (int i = 0; i < polePeak.size(); i++)
    {
        int& ca_x = polePeak[i].index;
        float threshold = 0.0f;
        int regionIdx = CheckRegion(ca_x, 6, subROI.width, ispResult.mCenterWidth);
        switch (regionIdx)
        {
            case 1:
                threshold = mIspCathode.mCathodeLineThresholdOuter;
                break;
            case 2:
                threshold = mIspCathode.mCathodeLineThresholdMiddle;
                break;
            case 3:
                threshold = mIspCathode.mCathodeLineThresholdInner;
                break;
            default:
                threshold = mIspCathode.mCathodeLineThresholdInner;
        }

        cv::Rect poleROI = xvt::CreateROI(ca_x - halfWindowSize, 0, catWindowSize, subROI.height, src.size());

        cv::Mat poleImg = src(poleROI);
        cv::Mat thresholdImg;
        //cv::threshold(poleImg, thresholdImg, 0, 255, (isWhitePole ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV) | cv::THRESH_OTSU);
        cv::reduce(poleImg, thresholdImg, 1, cv::REDUCE_AVG, CV_32FC1);
        cv::GaussianBlur(thresholdImg, thresholdImg, cv::Size(1, 7), 0);

        VecFloat reduceVec = ReduceBlackBackgroundMat(thresholdImg, 1, kSize);
        thresholdImg.release();
        VecFloat diffVec(reduceVec.size(), 0);
        for (int i = 1; i < reduceVec.size() - 1; i++)
        {
            diffVec[i] = reduceVec[i + 1] - reduceVec[i - 1];
        }

        FindPeaks findPeaks = FindPeaks(isWhitePole ? PeakType::Peak : PeakType::Valley , PeakFindingMethod::Prominence);
        findPeaks.Process(diffVec);
        auto peakResult = findPeaks.GetPeakResult(threshold, 5);

        std::sort(peakResult.begin(), peakResult.end(), [](Peak& a, Peak& b) {return abs(a.index) > abs(b.index); });

        if (peakResult.size() > 0)
        {
            midPos[i] = cv::Point(ca_x, poleROI.y + peakResult[0].index);
        }
        else
        {
            errorIdx.push_back(i);
            midPos[i] = cv::Point(ca_x, -1);
        }
    }

    auto centerRoi = CreateROI(ispResult.mCenter.x - subROI.x - ispResult.mCenterWidth / 2, 0, ispResult.mCenterWidth, subROI.height, subROI.size());
    auto cathodeLine = RefineCathodeLine(src, centerRoi, midPos);

    #if 0
    for (auto idx : errorIdx)
    {
        if (idx == 0) midPos[idx] = midPos[idx + 1];
        else if (idx == midPos.size()-1) midPos[idx] = midPos[idx - 1];
        else midPos[idx] = (midPos[idx + 1] + midPos[idx - 1])/2;
    }
    #endif

    for (auto& p:midPos)
    {
        if (p.x < centerRoi.x || p.x > centerRoi.br().x)
        {
            ispResult.mCathodePos.push_back(p + subROI.tl());
        }
    }
    ispResult.mAnodePoles.mCathodeLine = xvt::Transform(cathodeLine, subROI.tl());

#if _DEBUG_CATHODE
    cv::Mat drawImg;
    xvt::ConvertRGB(src, drawImg);

    for (auto& p : midPos)
    {
        DrawPlusSign(drawImg, p, COLOR_CV_GREEN);
    }
    for (auto idx : errorIdx)
    {
        DrawPlusSign(drawImg, midPos[idx], COLOR_CV_RED);
    }
#endif
}

VecInt CTBatteryInspector::CheckDominantPoles(const VecFloat& avgInt, float firstThresh, float lastThresh, int limitIntensity) const
{
    VecInt resultDelete;
    if (avgInt.size() < 2) return resultDelete;

    int nPole = 5;
    int noPoleCheck = avgInt.size() < nPole * 2 ? avgInt.size() / 2 - 1 : nPole;

    auto avgVecbegin = avgInt.begin();
    int first = 0;
    int last = avgInt.size() - 1;

    //Calculate sum intensity of nearby respective poles
    float sumFirst = std::accumulate(avgVecbegin + 1, avgVecbegin + noPoleCheck + 1, 0.0);
    float sumLast = std::accumulate(avgVecbegin + (last - noPoleCheck), avgVecbegin + last, 0.0);

    //Calculate average intensity of nearby respective poles
    float avgFirst = sumFirst / (float)noPoleCheck;
    float avgLast = sumLast / (float)noPoleCheck;

    //Calculate intensity of respective poles
    float roiFirst = avgInt[first];
    float roiLast = avgInt[last];

    if (roiFirst / avgFirst > firstThresh && roiFirst > limitIntensity) {
        resultDelete.push_back(first);
    }
    if (roiLast / avgLast > lastThresh && roiLast > limitIntensity) {
        resultDelete.push_back(last);
    }

    return resultDelete;
}

void CTBatteryInspector::RemovePoles(const cv::Mat& img, CTBatteryInspectorResult& ispResult) const
{
    auto poleRegionROI = ispResult.mPoleRegionRoi;
    int borderLeft = poleRegionROI.x + mIspPole.mSkipPolesDistance;
    int borderRight = poleRegionROI.x + poleRegionROI.width - mIspPole.mSkipPolesDistance;
    int centerLeft = ispResult.mCenter.x - ispResult.mCenterWidth / 2;
    int centerRight = ispResult.mCenter.x + ispResult.mCenterWidth / 2;

    VecInt idxAnodes;
    ispResult.mAnodePoles.mLeftNoPole = 0;
    int cathodeSize = ispResult.mCathodePos.size();
    int anodeSize = ispResult.mAnodePoles.mPoles.size();
    cv::Size imgSize = img.size();
    for (int i = anodeSize - 1; i >= 0; i--)
    {

        auto xPole = ispResult.mAnodePoles.mPoles[i].mAnode.x;
        if (xPole < borderLeft
            || (xPole > centerLeft && xPole < centerRight)
            || xPole > borderRight)
        {
            idxAnodes.push_back(i);
        }
        else
        {
            if(xPole < ispResult.mCenter.x)
                ispResult.mAnodePoles.mLeftNoPole++;
        }
    }

    VecInt idxCathodes;
    for (int i = cathodeSize - 1; i >= 0; i--)
    {
        auto xPole = ispResult.mCathodePos[i].x;

        if (xPole < borderLeft
            || (xPole <= centerLeft && xPole >= centerRight)
            || xPole > borderRight)
        {
            idxCathodes.push_back(i);
        }
        else
        {
            if (xPole < ispResult.mCenter.x)
                ispResult.nNoCathodeLeft++;
        }
    }

    bool enableRemoveTabPoles = mIspJR.mJROffsetY < 0;
    if (enableRemoveTabPoles)
    {
        VecFloat vtCathodeIntensityAvg;
        cv::Rect offset(-1, -1, 3, ispResult.mPoleRegionRoi.height);
        for (int i = 0; i < cathodeSize; i++)
        {
            cv::Rect cathodeRect(offset.tl() + ispResult.mCathodePos[i], offset.size());
            //cv::Rect cathodeRect = CreateROI(cv::Rect(offset.tl() + ispResult.mCathodePos[i], offset.size()), ispResult.mPoleRegionRoi);
            if (RefineROI(cathodeRect, ispResult.mPoleRegionRoi))
            {
                double avg = cv::sum(img(cathodeRect))[0];
                vtCathodeIntensityAvg.push_back(avg / cathodeRect.area());
            }
        }

        auto cathodeRemove = CheckDominantPoles(vtCathodeIntensityAvg, 1.5, 1.5, 200);
        for (auto idx : cathodeRemove)
        {
            if (std::find(idxCathodes.begin(), idxCathodes.end(), idx) == idxCathodes.end())
                idxCathodes.push_back(idx);
        }

        VecFloat vtAnodeIntensityAvg;
        for (int i = 0; i < anodeSize; i++)
        {
            auto anodePoits = ispResult.mAnodePoles.mPoles[i].mPoints;

            if (!anodePoits.empty())
            {
                int aSize = anodePoits.size();
                if (aSize > 1)
                {
                    double avg = 0.0;
                    for (int j = 0; j < aSize; j++)
                    {
                        avg += (double)img.at<uchar>(anodePoits[j]);
                    }
                    vtAnodeIntensityAvg.push_back(avg / aSize);
                }
            }
        }

        auto anodeRemove = CheckDominantPoles(vtAnodeIntensityAvg, 2, 2, 200);
        for (auto idx : anodeRemove)
        {
            if (std::find(idxAnodes.begin(), idxAnodes.end(), idx) == idxAnodes.end())
                idxAnodes.push_back(idx);
        }
    }

    std::sort(idxCathodes.begin(), idxCathodes.end(), std::greater<int>());
    std::sort(idxAnodes.begin(), idxAnodes.end(), std::greater<int>());
    for (auto idx : idxCathodes)
    {
        ispResult.mCathodePos.erase(ispResult.mCathodePos.begin() + idx);
    }

    for (auto idx : idxAnodes)
    {
        ispResult.mAnodePoles.mPoles.erase(ispResult.mAnodePoles.mPoles.begin() + idx);
    }
}

auto CTBatteryInspector::RefineCathodeLine(const cv::Mat& img, cv::Rect centerROI, VecPoint& ispResult) const -> VecPoint
{
    VecPoint listCathode;
    if (ispResult.empty() || img.empty())
        return listCathode;

    int ymin = centerROI.height;
    for (int i = 0; i < ispResult.size(); i++)
    {
        if (ispResult[i].y > 0 && ispResult[i].y < ymin)
        {
            ymin = ispResult[i].y;
        }

    }

    ymin = std::max(0, ymin - 10);
    cv::Rect subROIRefine = CreateROI(0, ymin, img.cols, img.rows - ymin, img.size());

    cv::Mat mainsubFilter;
    cv::Mat kernelImg = cv::Mat::zeros(cv::Size(7, 7), CV_32F);
    kernelImg.row(0).setTo(-0.5);
    kernelImg.row(1).setTo(-1);
    kernelImg.row(2).setTo(-0.5);
    kernelImg.row(4).setTo(0.5);
    kernelImg.row(5).setTo(1);
    kernelImg.row(6).setTo(0.5);
    kernelImg = kernelImg / 7.0f;

    cv::filter2D(img(subROIRefine), mainsubFilter, CV_32F, kernelImg);
    double maxValue = 255;
    for (auto r : ispResult)
    {
        if(r.x>0 && r.y>0)
        {
            cv::Point currentPos = r - subROIRefine.tl();
            cv::Rect subROI = CreateROI(cv::Rect(currentPos - cv::Point(2, 1), cv::Size(5, 3)), mainsubFilter.size());
            mainsubFilter(subROI) += maxValue;
        }
    }

    cv::Rect neglectRoi = CreateROI(centerROI, subROIRefine.size());
    if (!neglectRoi.empty())
        mainsubFilter(neglectRoi).setTo(0);

    listCathode = FindPathByAStar(mainsubFilter, mIspPole.mPolesDistanceRange.GetUpper());

    listCathode = xvt::Transform(listCathode, subROIRefine.tl());
    {
#if _DEBUG_REFINE_CATHODE_LINE
        cv::Mat debugImg;
        mainsubFilter.convertTo(debugImg, CV_8UC1);
        ConvertRGB(debugImg, debugImg);
        DrawPoints(debugImg, listCathode, cv::Scalar(0, 0, 255), -subROIRefine.tl());
        DrawPoints(debugImg, ispResult, cv::Scalar(255, 0, 0), -subROIRefine.tl());
#endif // _DEBUG_REFINE_CATHODE_LINE
        int cols = mainsubFilter.cols;
        int cathodeUnstableSize = 0;// cols / 15;
        for (int i = 0; i < ispResult.size(); i++)
        {
            int idx = ispResult[i].x;
            if ((idx > cathodeUnstableSize && idx < cols - cathodeUnstableSize) || ispResult[i].y == 0)
            {
                ispResult[i] = listCathode[idx];
            }
        }
    }

    return listCathode;
}

void CTBatteryInspector::RefineAnodePoles(CTBatteryInspectorResult& ispResult) const
{
    int anodeSize = ispResult.mAnodePoles.mPoles.size();
    bool enableRefineAnodePoles = mIspJR.mJROffsetY > 0;
    if (anodeSize > 10 && enableRefineAnodePoles)
    {
        VecFloat vtAnodeLeght;
        for (int i = 0; i < anodeSize; i++)
        {
            int lenght = ispResult.mAnodePoles.mPoles[i].mPoints.size();
            vtAnodeLeght.push_back(lenght);
        }

        auto anodeRefine = CheckDominantPoles(vtAnodeLeght, 1.5, 1.5, 5);
        for (auto idx : anodeRefine)
        {
            int refIdx = idx > 0 ? idx - 1 : idx + 1;
            auto& tmpPole = ispResult.mAnodePoles.mPoles[idx];
            int refLeght = vtAnodeLeght[refIdx];
            if (vtAnodeLeght[idx] > refLeght)
            {
                tmpPole.mAnode = tmpPole.mPoints[refLeght];
                tmpPole.mPoints.erase(tmpPole.mPoints.begin() + refLeght, tmpPole.mPoints.end());
            }
        }
    }
}

void CTBatteryInspector::GetCathodeLeftRight(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const
{
    std::vector<int> lstPolePosXAll;
    for (const auto& p : IspResult.mAnodePoles.mPoles)
    {
        lstPolePosXAll.push_back(p.mCathode.x);
    }
    bool const isUseNearest = mIspPole.mEnableCathodeNearestRef;
    int const cWidth = IspResult.mCenterDetected.width * 0.8;
    int const lstPoleSize = lstPolePosXAll.size();
    int const cSize = IspResult.mCathodePos.size();

    IspResult.vtCathodesLR = std::vector<std::pair<cv::Point, cv::Point>>(lstPoleSize, std::make_pair(cv::Point(0, 0), cv::Point(0, 0)) );

    int ca_Idx_l = 0;
    int ca_Idx_r = 0;
    for (int an_Idx = 0; an_Idx < lstPoleSize; an_Idx++)
    {
        auto& lr = IspResult.vtCathodesLR[an_Idx];
        auto const& a_x = lstPolePosXAll[an_Idx];

        for (ca_Idx_r = ca_Idx_l; ca_Idx_r < cSize; ca_Idx_r++)
        {
            auto const& c = IspResult.mCathodePos[ca_Idx_r];
            if (a_x < c.x)
            {
                break;
            }
            else
            {
                ca_Idx_l = ca_Idx_r;
            }
        }

        if (ca_Idx_r == cSize) ca_Idx_r--;

        auto& ca_left  = IspResult.mCathodePos[ca_Idx_l];
        auto& ca_right = IspResult.mCathodePos[ca_Idx_r];

        int d_l = a_x - ca_left.x;
        int d_r = ca_right.x - a_x;

        if (d_l > 0 && d_l < cWidth)
        {
            if (!isUseNearest && an_Idx > 0)
            {
                if(lstPolePosXAll[an_Idx - 1] < ca_left.x)
                    lr.first = ca_left;
            }
            else {
                lr.first = ca_left;
            }
        }

        if (d_r> 0 && d_r < cWidth)
        {
            if (!isUseNearest && an_Idx < lstPoleSize - 1)
            {
                if (lstPolePosXAll[an_Idx + 1] > ca_right.x)
                    lr.second = ca_right;
            }
            else {
                lr.second = ca_right;
            }
        }

        if (mIspBase.mDirection == 1)
        {
            std::swap(lr.first, lr.second);
        }
    }

#if _DEBUG_GET_CATHODE_LR
    cv::Mat debugAnodeCathode;
    xvt::ConvertRGB(img, debugAnodeCathode);
    for (int i = 0; i < lstPoleSize; i++)
    {
        const auto& pole = IspResult.mAnodePoles.mPoles[i];
        auto aPos = cv::Point(pole.mCathode.x, pole.mAnode.y);

        if (IspResult.vtCathodesLR[i].first != cv::Point(0, 0))
            DrawLine(debugAnodeCathode, IspResult.vtCathodesLR[i].first, aPos, CVPen(COLOR_CV_ORANGE, 2));
        if (IspResult.vtCathodesLR[i].second != cv::Point(0, 0))
            DrawLine(debugAnodeCathode, IspResult.vtCathodesLR[i].second, aPos, CVPen(COLOR_CV_YELLOW));
        DrawPlusSign(debugAnodeCathode, aPos, CVPen(COLOR_CV_BLUE, 2));
    }

    for(auto & a: IspResult.mCathodePos)
        DrawPlusSign(debugAnodeCathode, a, CVPen(COLOR_CV_RED, 2));
#endif // _DEBUG_GET_CATHODE_LR
}

void CTBatteryInspector::RemoveCathodeCenterPoles(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const
{
    if (IspResult.vtCathodesLR.size() > 5 && !img.empty())
    {
        auto imgSize = img.size();
        int noPoleL = IspResult.mAnodePoles.mLeftNoPole;
        //cv::Rect offsetRoi(-1, -1, 3, 10);
        auto& firstAnodeR = IspResult.vtCathodesLR[noPoleL];
        cv::Point& firstRefR = mIspBase.mDirection == 0 ? firstAnodeR.first : firstAnodeR.second;
        if (firstRefR != cv::Point(0, 0))
        {
            auto findCathode = std::find(IspResult.mCathodePos.begin(), IspResult.mCathodePos.end(), firstRefR);
            //auto refCathode = findCathode + 1;
            if (findCathode != IspResult.mCathodePos.end())
            {
                int idx = std::distance(IspResult.mCathodePos.begin(), findCathode);
                firstRefR = cv::Point(0, 0);
                if (idx < IspResult.nNoCathodeLeft)
                    IspResult.nNoCathodeLeft--;
                IspResult.mCathodePos.erase(findCathode);
            }
        }

        auto& firstAnodeL = IspResult.vtCathodesLR[noPoleL - 1];
        auto& firstRefL = mIspBase.mDirection == 0 ? firstAnodeL.second : firstAnodeL.first;
        if (firstRefL != cv::Point(0, 0))
        {
            auto findCathode = std::find(IspResult.mCathodePos.begin(), IspResult.mCathodePos.end(), firstRefL);
            if (findCathode != IspResult.mCathodePos.end())
            {
                int idx = std::distance(IspResult.mCathodePos.begin(), findCathode);
                firstRefL = cv::Point(0, 0);
                if(idx < IspResult.nNoCathodeLeft)
                    IspResult.nNoCathodeLeft--;
                IspResult.mCathodePos.erase(findCathode);
            }
        }
    }
}

auto IsRealCathode(cv::Mat const& img, cv::Point p0, cv::Point p1, cv::Point br) ->bool
{
    auto I0 = GetMeanInstensity(img, p0, (br.y - p0.y) / 2, false);
    auto I1 = GetMeanInstensity(img, p1, (br.y - p1.y) / 2, false) * 0.8;

    auto I_Top = GetMeanInstensity(img, p0 + cv::Point(0, -5), 8, true);
    auto I_Bot = GetMeanInstensity(img, p0 + cv::Point(0, 5), 8, false) * 0.8;

    return (I0 > I1 && I_Bot > I_Top);
}

void CTBatteryInspector::AddMissingAnodeEndingPoles(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const
{
    if (IspResult.vtCathodesLR.size() > 2 && !img.empty())
    {
        auto imgSize = img.size();
        auto firstAnode = IspResult.vtCathodesLR.front();
        auto lastAnode = IspResult.vtCathodesLR.back();

        #if _DEBUG_REMOVE_CATHODE
        VecPoint addAnode;
        VecPoint removeCathode;
        #endif

        cv::Point firstRef = mIspBase.mDirection == 0 ? firstAnode.first : firstAnode.second;
        cv::Point secondRef = mIspBase.mDirection == 0 ? lastAnode.second : lastAnode.first;

        if (firstRef != cv::Point(0, 0))
        {
            cv::Point secondCathode = cv::Point(0, 0);
            for (int i = 1; i < IspResult.vtCathodesLR.size(); i++)
            {
                auto ref = mIspBase.mDirection == 0 ? IspResult.vtCathodesLR[i].first : IspResult.vtCathodesLR[i].second;
                if (ref != cv::Point(0, 0))
                {
                    secondCathode = ref;
                    break;
                }
            }

            auto isCathode = IsRealCathode(img, firstRef, secondCathode, IspResult.mOuterRoi.br());
            if (isCathode)//add onemore anode
            {
                auto refBeginPole = IspResult.mAnodePoles.mPoles.front();
                cv::Point offsetX = cv::Point(refBeginPole.mCathode.x - firstRef.x, 0);
                PoleInfo tmpPole;
                tmpPole.mAnode = refBeginPole.mAnode - offsetX * 2;
                tmpPole.mCathode = refBeginPole.mCathode - offsetX * 2;

                if (tmpPole.mAnode.x > 0 && tmpPole.mAnode.x < imgSize.width
                    && tmpPole.mAnode.y > 0 && tmpPole.mAnode.y < imgSize.height
                    && img.at<uchar>(tmpPole.mCathode) > img.at<uchar>(refBeginPole.mCathode) / 2)
                {
                    IspResult.vtCathodesLR.insert(IspResult.vtCathodesLR.begin(), std::make_pair(cv::Point(0, 0), firstRef));

                    IspResult.mAnodePoles.mPoles.insert(IspResult.mAnodePoles.mPoles.begin(), tmpPole);
                    IspResult.mAnodePoles.mLeftNoPole++;
                }
            }
            else // Remove cathode
            {
                auto& cathodes = IspResult.mCathodePos;
                auto findCathode = std::find(cathodes.begin(), cathodes.end(), firstRef);
                //auto refCathode = findCathode + 1;
                if (findCathode != cathodes.end())
                {
                    int idx = std::distance(cathodes.begin(), findCathode)+1;
                    IspResult.nNoCathodeLeft -= idx;
                    firstRef = cv::Point(0, 0);
                    IspResult.mCathodePos.erase(cathodes.begin(), findCathode);
                }
            }
        }

        if (secondRef != cv::Point(0, 0))
        {
            auto lastCathodeLR = IspResult.vtCathodesLR.end() - 2;
            cv::Point secondCathode = cv::Point(0, 0);
            for (int i = IspResult.vtCathodesLR.size() - 1; i >= 0; i--)
            {
                auto ref = mIspBase.mDirection == 0 ? IspResult.vtCathodesLR[i].first : IspResult.vtCathodesLR[i].second;
                if (ref != cv::Point(0, 0))
                {
                    secondCathode = ref;
                    break;
                }
            }

            auto isCathode = IsRealCathode(img, secondRef, secondCathode, IspResult.mOuterRoi.br());
            if (isCathode)//add one more anode
            {
                auto refBeginPole = IspResult.mAnodePoles.mPoles.back();
                cv::Point offsetX = cv::Point(secondRef.x - refBeginPole.mCathode.x, 0);
                PoleInfo tmpPole;
                tmpPole.mAnode = refBeginPole.mAnode + offsetX * 2;
                tmpPole.mCathode = refBeginPole.mCathode + offsetX * 2;
                if (tmpPole.mAnode.x > 0 && tmpPole.mAnode.x < imgSize.width
                    && tmpPole.mAnode.y > 0 && tmpPole.mAnode.y < imgSize.height
                    && img.at<uchar>(tmpPole.mCathode) > img.at<uchar>(refBeginPole.mCathode) / 2)
                {
                    IspResult.vtCathodesLR.insert(IspResult.vtCathodesLR.end(), std::make_pair(secondRef, cv::Point(0, 0)));
                    IspResult.mAnodePoles.mPoles.insert(IspResult.mAnodePoles.mPoles.end(), tmpPole);

                #if _DEBUG_REMOVE_CATHODE
                    addAnode.push_back(tmpPole.mAnode);
                    IspResult.AddMsg("Add one more anode");
                #endif
                }
            }
            else //remove cathode
            {
                auto& cathodes = IspResult.mCathodePos;
                auto findCathode = std::find(cathodes.begin(), cathodes.end(), secondRef);
                //auto refCathode = findCathode + 1;
                if (findCathode != cathodes.end())
                {
                #if _DEBUG_REMOVE_CATHODE
                    removeCathode.push_back(*findCathode);
                    IspResult.AddMsg("Remove Anode");
                #endif

                    //int idx = std::distance(cathodes.begin(), findCathode)+1;
                    //IspResult.nNoCathodeLeft -= idx;
                    secondRef = cv::Point(0, 0);
                    IspResult.mCathodePos.erase(findCathode, cathodes.end());
                }
            }
        }

    #if _DEBUG_REMOVE_CATHODE
        auto& anodes = IspResult.mAnodePoles.mPoles;
        auto& cathodes = IspResult.mCathodePos;

        auto rect1 = cv::Rect(0, 0, abs(secondRef.x - firstRef.x), 100);

        cv::Mat drawImg;
        xvt::ConvertRGB(img, drawImg);

        auto p0 = -rect1.tl() + cv::Point(15, 80);


        for (auto& p : cathodes)
        {
            DrawPlusSign(drawImg, p, COLOR_CV_GREEN);
        }

        for (auto& p : removeCathode)
        {
            DrawPlusSign(drawImg, p, COLOR_CV_RED);
        }
        for (auto& p : addAnode)
        {
            DrawPlusSign(drawImg, p, COLOR_CV_YELLOW);
        }
    #endif // _DEBUG_REMOVE_CATHODE

    }

}

void CTBatteryInspector::RemoveNoiseAnodeEndingPoles(cv::Mat const& img, CTBatteryInspectorResult& IspResult) const
{
    int poleSize = IspResult.mAnodePoles.mPoles.size();
    if (IspResult.vtCathodesLR.size() > 5 && !img.empty() && poleSize > 5)
    {
        VecInt listRemoveIdxs;

        auto imgSize = img.size();
        auto firstAnode = IspResult.vtCathodesLR.front();
        auto lastAnode = IspResult.vtCathodesLR.back();
        auto beginPole = IspResult.mAnodePoles.mPoles.front();
        auto endPole = IspResult.mAnodePoles.mPoles.back();
        auto noLeftPole = IspResult.mAnodePoles.mLeftNoPole;
        float thresholdRate = 0.8;
        cv::Rect offset(-1, -1, 3, 3);

        int refLeftIdx = -1;
        for (int i = 0; i < noLeftPole; i++)
        {
            auto tmpCathode = IspResult.vtCathodesLR[i];
            if (tmpCathode.first != cv::Point(0, 0) || tmpCathode.second != cv::Point(0, 0))
            {
                refLeftIdx = i;
                break;
            }
        }

        if (refLeftIdx != -1)
        {
            float refBeginPole = IspResult.mAnodePoles.mPoles[refLeftIdx].mPeakValue.value * thresholdRate;
            for (int i = 0; i < refLeftIdx; i++)
            {
                auto curPole = IspResult.mAnodePoles.mPoles[i].mPeakValue.value;
                if(curPole < refBeginPole)
                    listRemoveIdxs.push_back(i);
            }
        }
        
        int refRightIdx = -1;
        for (int i = poleSize - 1; i >= noLeftPole; i--)
        {
            auto tmpCathode = IspResult.vtCathodesLR[i];
            if (tmpCathode.first != cv::Point(0, 0) || tmpCathode.second != cv::Point(0, 0))
            {
                refRightIdx = i;
                break;
            }
        }

        if (refRightIdx != -1)
        {
            float refEndPole = IspResult.mAnodePoles.mPoles[refRightIdx].mPeakValue.value * thresholdRate;
            for (int i = poleSize - 1; i > refRightIdx; i--)
            {
                auto curPole = IspResult.mAnodePoles.mPoles[i].mPeakValue.value;
                if (curPole < refEndPole)
                    listRemoveIdxs.push_back(i);
            }
        }

        std::sort(listRemoveIdxs.begin(), listRemoveIdxs.end());
        for(int i = listRemoveIdxs.size() - 1; i >= 0; i--)
        {
            auto idx = listRemoveIdxs[i];
            if (idx < IspResult.mAnodePoles.mLeftNoPole)
            {
                IspResult.mAnodePoles.mLeftNoPole--;
            }
            IspResult.vtCathodesLR.erase(IspResult.vtCathodesLR.begin() + idx);
            IspResult.mAnodePoles.mPoles.erase(IspResult.mAnodePoles.mPoles.begin() + idx);
        }
    }
}

auto CTBatteryInspector::RemoveTab(cv::Mat & img, int tabThd) const -> xvt::VecVecPoint
{
    xvt::VecVecPoint tabContour;
    if (img.empty()) return tabContour;

    if (tabThd == 0)
    {
        double minVal, maxVal;
        cv::Point minloc, maxloc;
        cv::minMaxLoc(img, &minVal, &maxVal, &minloc, &maxloc);
        tabThd = maxVal * 0.9;
    }

    cv::Mat binImg;
    xvt::threshold::Threshold(img, binImg, tabThd, 255, cv::THRESH_BINARY);

    tabContour = xvt::FindContour(binImg,
                     [h = binImg.rows*0.9](auto& c)
                     {
                         auto rect = cv::boundingRect(c);
                         return (rect.height > h);
                     }
    );

    if(tabContour.size() == 1)
        cv::drawContours(img, tabContour, -1, cv::Scalar::all(tabThd/4), -1);

    return tabContour;
}

void CTBatteryInspector::RemoveCathodeByAnode(CTBatteryInspectorResult& ispResult, float d) const
{
    auto& anodes = ispResult.mAnodePoles.mPoles;
    auto& cathodes = ispResult.mCathodePos;

    if (anodes.empty() || cathodes.empty() || d < 0) return;

#if _DEBUG_REMOVE_CATHODE
    auto caList = cathodes;
#endif // _DEBUG_REMOVE_CATHODE

    VecInt caIdx;
    int aIdx = 0;
    int sizeCa = cathodes.size();
    int removeSize = 3;
    for (int i = 0; i < sizeCa; i++)
    {
        if (i < removeSize
            || i >= sizeCa - removeSize
            || (i >= ispResult.nNoCathodeLeft - removeSize && i < ispResult.nNoCathodeLeft + removeSize))
        {
            auto& cPole = cathodes[i];
            for (; aIdx < anodes.size(); aIdx++)
            {
                auto& aPole = anodes[aIdx].mCathode;
                if (abs(aPole.x - cPole.x) <= d)
                {
                    caIdx.push_back(i);
                }

                if (aPole.x > cPole.x)
                {
                    break;
                }
            }
        }
    }

    //Remove cathode pos
    for (auto ptr = caIdx.rbegin(); ptr <caIdx.rend(); ptr++)
    {
        if (*ptr < ispResult.nNoCathodeLeft)
            ispResult.nNoCathodeLeft--;

        cathodes.erase(cathodes.begin() + *ptr);
    }

#if _DEBUG_REMOVE_CATHODE
    auto rect1 = cv::boundingRect(cathodes);
    cv::Mat img = cv::Mat(rect1.height + 100, rect1.width + 30, CV_8UC3, cv::Scalar::all(125));
    auto p0 = -rect1.tl() + cv::Point(15, 80);
    for (auto& p : anodes)
    {
        DrawPlusSign(img, p.mCathode + p0, COLOR_CV_YELLOW, 1, 2);
    }

    for (auto& p : caList)
    {
        DrawPlusSign(img, p + p0, COLOR_CV_RED);
    }

    for (auto& p : cathodes)
    {
        DrawPlusSign(img, p + p0, COLOR_CV_GREEN);
    }
#endif // _DEBUG_REMOVE_CATHODE

}

void CTBatteryInspector::RemoveCathodeByIntensity(cv::Mat const& img, CTBatteryInspectorResult& ispResult, float d) const
{
    auto& anodes = ispResult.mAnodePoles.mPoles;
    auto& cathodes = ispResult.mCathodePos;

    if (anodes.empty() || cathodes.empty() || d < 0) return;

#if _DEBUG_REMOVE_CATHODE
    auto caList = cathodes;
#endif // _DEBUG_REMOVE_CATHODE

    VecInt caIdx;
    int sizeCa = cathodes.size();
    int removeSize = 1;
    for (int i = 0; i < sizeCa; i++)
    {
        if (i < removeSize
            || i >= sizeCa - removeSize
            //|| (i >= ispResult.nNoCathodeLeft - removeSize && i < ispResult.nNoCathodeLeft + removeSize)
            )
        {
            auto& cPole = cathodes[i];
            auto& refPole = i < sizeCa - 1 ? cathodes[i+1] : cathodes[i - 1];
            auto isRealCathode = IsRealCathode(img, cPole, refPole, ispResult.mOuterRoi.br());
            if (!isRealCathode)
            {
                caIdx.push_back(i);
            }
        }
    }

    //Remove cathode pos
    for (auto ptr = caIdx.rbegin(); ptr < caIdx.rend(); ptr++)
    {
        if (*ptr < ispResult.nNoCathodeLeft)
            ispResult.nNoCathodeLeft--;

        cathodes.erase(cathodes.begin() + *ptr);
    }

#if _DEBUG_REMOVE_CATHODE
    auto rect1 = cv::boundingRect(cathodes);
    cv::Mat drawImg;
    xvt::ConvertRGB(img, drawImg);

    auto p0 = -rect1.tl() + cv::Point(15, 80);


    for (auto& p : caList)
    {
        DrawPlusSign(drawImg, p, COLOR_CV_RED);
    }

    for (auto& p : cathodes)
    {
        DrawPlusSign(drawImg, p, COLOR_CV_GREEN);
    }
#endif // _DEBUG_REMOVE_CATHODE
}

#ifdef _DEBUG_CODE
cv::Mat GetCDFImage(const cv::Mat& inputImg)
{
    auto size = inputImg.size();
    cv::Mat result = cv::Mat::zeros(size, inputImg.type());
    for (int i = 0; i < size.width; i++)
    {
        cv::Mat col = inputImg.col(i);

        cv::Mat valleyImg, peakImg;
        cv::threshold(col, valleyImg, 0, 255, cv::THRESH_TOZERO_INV);
        cv::threshold(col, peakImg, 0, 255, cv::THRESH_TOZERO);

        int noNonZeroValley = cv::countNonZero(valleyImg);
        if (noNonZeroValley > 0) valleyImg = cv::abs(valleyImg) / noNonZeroValley;

        int noNonZeroPeak = cv::countNonZero(peakImg);
        if (noNonZeroPeak > 0) peakImg = peakImg / noNonZeroPeak;

        double totalValley = cv::sum(valleyImg)[0];
        double totalPeak = cv::sum(peakImg)[0];
        float ratio = 0.55;

        for (int j = 0; j < size.height; j++)
        {
            float sumValley = totalValley > 0 ? cv::sum(valleyImg.rowRange(0, j + 1))[0] / totalValley * ratio : 0.0;
            if (noNonZeroValley == 0) sumValley = ratio;

            float sumPeak = totalPeak > 0 ? cv::sum(peakImg.rowRange(j, size.height))[0] / totalPeak * (1 - ratio) : 0.0;
            if (noNonZeroPeak == 0) sumPeak = 1 - ratio;

            float sum = sumValley + sumPeak;
            result.at<float>(j, i) = sum;
        }
    }
    return result;
}

//enum PoleType
//{
//    ANODE,
//    CATHODE,
//    NOISE
//};

struct ColMapResult
{
    int idx;
    VecDouble values;

    int GetPeakSize() {
        int size = 0;
        for (auto v : values)
            if (v > 0) size++;

        return size;
    }

    int GetValleySize() {
        int size = 0;
        for (auto v : values)
            if (v < 0) size++;

        return size;
    }
};

VecPeak convertToVecPeak(VecDouble values)
{
    VecPeak result;

    for (int i = 0; i < values.size(); i++)
    {
        //result.emplace_back(i, 0, values[i]);
    }
    return result;
}

void DebugAlgorithm2(const cv::Mat& inputImg)
{
    std::vector<ColMapResult> colRes;

    for (int i = 0; i < inputImg.cols; i++)
    {
        ColMapResult col;
        col.idx = i;
        VecDouble colValue = inputImg.col(i);
        colRes.push_back(col);
    }
}
VecPoint DebugAlgorithm(const cv::Mat& inputImg, VecPeak& polePeak, cv::Rect center, cv::Point c)
{
    VecPoint Results;
    if (!inputImg.empty())
    {
        cv::Mat debug;
        auto peakMat0 = GetPeakAndValleyImage(inputImg, 0.01, 3);
        peakMat0(center).setTo(0);

        auto size = inputImg.size();
        cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
        for (int j = 0; j < polePeak.size(); j++)
        {
            cv::Rect roi = CreateROI(polePeak[j].index - 1, 0, 3, size.height, size);
            mask(roi).setTo(1);
        }

        cv::Mat peakMat = cv::Mat::zeros(size, peakMat0.type());;
        cv::Mat peakMat2 = cv::abs(peakMat0);
        DebugAlgorithm2(peakMat2);
        cv::Mat peakMat3, peakMat4;
        cv::threshold(peakMat0, peakMat3, 0, 255, cv::THRESH_TOZERO_INV);
        peakMat3 = cv::abs(peakMat3);

        cv::Mat mainsubFilter;
        cv::Mat kernelImg = cv::Mat::zeros(cv::Size(7, 7), CV_32F);
        kernelImg.row(0).setTo(-0.5);
        kernelImg.row(1).setTo(-1);
        kernelImg.row(2).setTo(-0.5);
        kernelImg.row(4).setTo(0.5);
        kernelImg.row(5).setTo(1);
        kernelImg.row(6).setTo(0.5);
        kernelImg = kernelImg / 7.0f;

        cv::filter2D(inputImg, mainsubFilter, CV_32F, kernelImg);
        cv::rotate(mainsubFilter, mainsubFilter, cv::ROTATE_90_CLOCKWISE);
        auto peakMat5 = GetPeakAndValleyImage(mainsubFilter, 0.001, 3);
        cv::rotate(mainsubFilter, mainsubFilter, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(peakMat5, peakMat5, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::threshold(peakMat5, peakMat5, 0, 255, cv::THRESH_TOZERO);

        peakMat4 = peakMat2 + peakMat5;

        peakMat0.copyTo(peakMat, mask);
        cv::Mat negative = -peakMat0;
        cv::Mat invert = 1 - mask;
        negative.copyTo(peakMat, invert);
        //cv::threshold(peakMat, peakMat, 125, 255, cv::THRESH_TOZERO_INV);
        //cv::threshold(peakMat, peakMat, -125, 255, cv::THRESH_TOZERO);
        //cv::bitwise_and(peakMat0, peakMat0, peakMat, mask);
        xvt::ConvertRGB(inputImg, debug);

        cv::Mat cdfPeakMat = cv::Mat::zeros(size, peakMat.type());
        {
            peakMat.row(size.height - 1).copyTo(cdfPeakMat.row(size.height - 1));
            for (int i = size.height - 2; i >= 0; i--)
            {
                auto mat1 = peakMat.row(i);
                auto mat2 = cdfPeakMat.row(i + 1);
                cv::Mat mat;
                cv::add(mat1, mat2, mat);
                mat.copyTo(cdfPeakMat.row(i));
            }

            cv::Mat threshold;
            cv::threshold(peakMat, threshold, 0.01, 255, cv::THRESH_BINARY);
            // Define the structuring element (kernel size)
            // Adjust the size according to the characteristics of your image
            int kernelWidth = 50;  // Example width
            int kernelHeight = 10; // Example height
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelWidth, kernelHeight));

            // Apply morphological opening (erosion followed by dilation)
            cv::Mat morphedImage;
            cv::morphologyEx(threshold, morphedImage, cv::MORPH_OPEN, kernel);
            //cv::blur(peakMat, peakMat, cv::Size(7, 7));
            cdfPeakMat = GetCDFImage(peakMat);
            auto listCathode = FindPathByAStar(cdfPeakMat, 50);

            for (auto& p : polePeak)
            {
                Results.push_back(cv::Point(p.index, listCathode[p.index].y));
            }

            DrawPoints(debug, listCathode, cv::Vec3b(0, 255, 0));
        }

        cv::Mat debug2;
        xvt::ConvertRGB(peakMat2, debug2);
        cv::circle(debug2, c, 2, cv::Scalar(0, 255, 0));
        DrawPoints(debug2, Results, cv::Vec3b(0, 0, 255));
    }
    return Results;
}
#endif

auto CTBatteryInspector::InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorResult& ispResult) const->InspectionResult
{
    auto cathodeInpection = InspectionResult();
    auto yCathode = ispResult.mCenterDetected.y - subROI.y;
    if (!src.empty() && !subROI.empty() && subROI.height - yCathode > 3)
    {
        cv::Mat mainsubImg = src(subROI).clone();


        int kSize = 3;
        /*cv::Mat mat = (cv::Mat_<float>(5, 5) << 0, 0, 3, 0, 0,
                                                0, 2, 3, 2, 0,
                                                1, 2, 3, 2, 1,
                                                0, 2, 3, 2, 0,
                                                0, 0, 3, 0, 0);
        mat = mat / 29.0;*/

        cv::Mat mat = (cv::Mat_<float>(5, 5) << 0, 0, 2, 0, 0,
                                                0, 1, 3, 1, 0,
                                                0, 2, 3, 2, 0,
                                                0, 1, 3, 1, 0,
                                                0, 0, 2, 0, 0);
        mat = mat / 21.0;

        ispResult.mPoleRegionRoi = subROI;
        auto cathodeROIX_start = 0;
        auto cathodeROIX_end = subROI.width;
        auto cathodeROIY = ispResult.mCenterDetected.y - subROI.y + 10;

        if (mIspJR.mEnableRemoveTab)
        {
            auto tabContour = RemoveTab(mainsubImg, mIspJR.mTabThreshold);
            if (tabContour.size() == 1)
            {
                xvt::VecPoint tabs;
                for (auto const& p : tabContour[0])
                {
                    if (p.y > cathodeROIY)
                    {
                        tabs.push_back(p);
                    }
                }
                auto rect = cv::boundingRect(tabs);
                auto tmpx= rect.x + rect.width / 2;
                if (tmpx < subROI.width / 2)
                    cathodeROIX_start = rect.x + rect.width/2;
                else
                    cathodeROIX_end = rect.x + rect.width / 2;
            }
        }

        auto h = (subROI.height - cathodeROIY) / 2;
        cv::Rect subROI2 = CreateROI(cathodeROIX_start, cathodeROIY + h, cathodeROIX_end - cathodeROIX_start, h, subROI.size());

        cv::Mat mainsubFilter;
        cv::filter2D(mainsubImg, mainsubFilter, -1, mat);
        // Create a 10x1 Gaussian kernel
        cv::Mat gaussianKernel = cv::getGaussianKernel(mCathodeKernelSize, -1, CV_32F);
        cv::filter2D(mainsubFilter, mainsubFilter, CV_32F, gaussianKernel);

        cv::Mat subImgFilter;
        cv::filter2D(mainsubFilter(subROI2), subImgFilter, -1, mat);
        VecFloat reduceVec = ReduceBlackBackgroundMat(subImgFilter, 0, 3);
        
        xvt::Rangei distanceRange;
        VecPeak polePeak = FindCathodeXPositionAuto(reduceVec, distanceRange, ispResult.mCenterWidth, xvt::PeakType::Peak);
        if (polePeak.size() > 5)
        {
            std::vector<cv::Point> startPos = {};
            bool checkCathodeOK = true;
            for (auto& p : polePeak)
            {
                p.index += subROI2.x;
                startPos.push_back(cv::Point(p.index + subROI.x, subROI.y + subROI2.y));
            }

            if (mIspCathode.mCathodeLineThresholdOuter < 1e-3 && mIspCathode.mCathodeLineThresholdMiddle < 1e-3 && mIspCathode.mCathodeLineThresholdInner < 1e-3)
            {
                cv::Rect centerROI = CreateROI(ispResult.mCenter.x - subROI.x - ispResult.mCenterWidth/2, 0, ispResult.mCenterWidth, subROI.height, subROI.size());
                //mainsubFilter(centerROI).setTo(0);
                VecPoint endCathodeAuto = FindEndpointCathodeAuto(mainsubFilter, polePeak, mCathodeEnableReference, mCathodeThicknessPole, mCathodeDistanceVerifi, mCathodeNumVariance);
                
                VecPoint cathodeLine = RefineCathodeLine(mainsubFilter, centerROI, endCathodeAuto);
                //endCathodeAuto = DebugAlgorithm(mainsubFilter, polePeak, centerROI, ispResult.mCenterDetected.tl() - subROI.tl() + cv::Point(ispResult.mCenterDetected.size()/2));
                for (int i = 0; i < endCathodeAuto.size(); i++)
                {
                    if ((startPos[i].x < ispResult.mCenter.x - ispResult.mCenterWidth / 2
                        || startPos[i].x > ispResult.mCenter.x + ispResult.mCenterWidth / 2)
                        /*&& vtCathodePeakRef[endCathodeAuto[i].x].prominence < 0*/)
                    {
                        ispResult.mCathodePos.push_back(cv::Point(endCathodeAuto[i].x + subROI.x, endCathodeAuto[i].y + subROI.y));

                        //if ((endCathodeAuto[i].y < 1) || (endCathodeAuto[i].y > subROI.height - 1)) checkCathodeOK = false;
                    }
                }

                ispResult.mAnodePoles.mCathodeLine = xvt::Transform(cathodeLine, subROI.tl());
#if _DEBUG_CATHODE
                cv::cvtColor(mainsubImg, mainsubImg, cv::COLOR_GRAY2BGR);
                for (auto& pCat : endCathodeAuto)
                {
                    cv::circle(mainsubImg, cv::Point(pCat.x, pCat.y), 1, cv::Scalar(255, 0, 0));//pCat.index + subROI2.x, subROI2.y ||  
                }
                for (auto& pCat : startPos)
                {
                    cv::circle(mainsubImg, pCat - subROI.tl(), 1, cv::Scalar(0, 255, 0));//pCat.index + subROI2.x, subROI2.y ||  
                }
#endif //_DEBUG_CATHODE
            }
            else
            {
                FindEndpointCathodeThreshold(src, polePeak, subROI, ispResult, kSize);
            }

            if (checkCathodeOK)
            {
                //VecFloat lstCathodeLine;
                //for(auto&c : cathodePoints) lstCathodeLine.push_back(c.y);
                //cv::Mat matRow(1, lstCathodeLine.size(), CV_32F, lstCathodeLine.data());
                //cv::medianBlur(matRow, matRow, 5);
                //std::vector<float> dataVector;
                //matRow.reshape(1, 1).copyTo(dataVector);
                //int thresholdMedian = 3;
                //for(int i = 0; i < lstCathodeLine.size(); i++)
                //{
                //    if(abs(lstCathodeLine[i] - dataVector[i]) > 5)
                //        lstCathodeLine[i] = dataVector[i];
                //}
                //cv::GaussianBlur(lstCathodeLine, lstCathodeLine, cv::Size(5, 1), 0, 0, cv::BORDER_REFLECT);
                //for(int i = 0; i < lstCathodeLine.size(); i++) cathodePoints[i].y = lstCathodeLine[i];
                cathodeInpection(EResult::OK, "");
            }
            else
            {
                cathodeInpection(EResult::ER, "Cannot inspect Cathode, Check ROI pole!");
            }
        }
        else
        {
            cathodeInpection(EResult::ER, "Peak Cathode size is not enough!");
        }
    }
    else
    {
        cathodeInpection(EResult::ER, "No Image in Cathode!");
    }
    return cathodeInpection;
}

auto CTBatteryInspector::InspectAnode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorResult& ispResult) const->InspectionResult
{
    auto anodeInspection = InspectionResult();
    if (!src.empty() && !subROI.empty())
    {
        if (ispResult.mCathodePos.size() > 5 && !ispResult.mAnodePoles.mCathodeLine.empty())
        {
            cv::Mat subImg = src(subROI);

            cv::Mat mat = (cv::Mat_<float>(5, 5) << 0, 0, 2, 0, 0,
                0, 1, 3, 1, 0,
                0, 2, 3, 2, 0,
                0, 1, 3, 1, 0,
                0, 0, 2, 0, 0);
            mat = mat / 21.0;
            cv::Mat mainsubFilter;
            cv::filter2D(subImg, mainsubFilter, -1, mat);
            cv::Mat gaussianKernel = cv::getGaussianKernel(3, -1, CV_32F);

            cv::Mat blurImg;
            cv::filter2D(mainsubFilter, blurImg, CV_32F, gaussianKernel);

            int count = 0;
            int XPaddingAno = 0;
            cv::Point offset = subROI.tl();
            
            cv::blur(subImg, blurImg, cv::Size(1, 3));

            auto peakMat0 = GetPeakAndValleyImage(subImg, 0.1, 2, 1);
            cv::GaussianBlur(peakMat0, peakMat0, cv::Size(5, 5), 0);

            blurImg.convertTo(blurImg, peakMat0.type());
            if (mIspAnode.mAnodeEnhanceScale > 1e-6)
            {
                auto scalePeakImg = peakMat0 * mIspAnode.mAnodeEnhanceScale;
                blurImg += scalePeakImg;
            }
            auto listCathode = xvt::Transform(ispResult.mAnodePoles.mCathodeLine, -offset);
            cv::Size size = subROI.size();
            auto mask = cv::Mat(size, peakMat0.type(), cv::Scalar(0));
            int offsetY = 0;
            cv::Rect tmpRect2(0, -mIspPole.mPoleHeight, 1, mIspPole.mPoleHeight - offsetY);

            cv::Mat dst2;
            for (auto p : listCathode)
            {
                cv::Rect rect = CreateROI(tmpRect2 + p, size);
                mask(rect).setTo(1);
            }
            
            cv::multiply(blurImg, mask, dst2);

            VecFloat reduceVec;// = ReduceBlackBackgroundMat(dst2, 0, 3);
            cv::reduce(dst2, reduceVec, 0, cv::REDUCE_SUM, CV_32FC1);
            for (auto& r : reduceVec) r /= mIspPole.mPoleHeight;
            //cv::GaussianBlur(reduceVec, reduceVec, cv::Size(3, 1), 0);

            FindPeaks findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
            findPeaks.Process(reduceVec);
            auto peakResult = findPeaks.GetPeakResult(mIspPole.mPolesProminenceThreshold, mIspPole.mPolesDistanceRange.GetLower());
            float const centLimitLeft  = ispResult.mCenter.x - ispResult.mCenterWidth / 2.0f;
            float const centLimitRight = ispResult.mCenter.x - ispResult.mCenterWidth / 2.0f;
            int avgAnodeThd = std::accumulate(reduceVec.begin(), reduceVec.end(), 0.0) / reduceVec.size();
            float const anodePoleThd = mIspPole.mPoleThreshold > 1e-6 ? mIspPole.mPoleThreshold : avgAnodeThd;
            for (auto p : peakResult)
            {
                PoleInfo currentPole;
                currentPole.mCathode = ispResult.mAnodePoles.mCathodeLine[p.index];
                currentPole.mPeakValue = p;
                currentPole.mPoints.push_back(currentPole.mCathode);
                if (p.value > anodePoleThd && (currentPole.mCathode.x < centLimitLeft || currentPole.mCathode.x > centLimitRight))
                {
                    ispResult.mAnodePoles.mPoles.push_back(currentPole);
                }
            }

            for (auto& pole : ispResult.mAnodePoles.mPoles)
            {
                auto AnodeResult = InspectionResult();
                //if (pole.mCathode.x < ispResult.mCenter.x) ispResult.mAnodePoles.mLeftNoPole++;
                if (mChooseAnodeAlgorithm != 1 && mChooseAnodeAlgorithm != 2)
                {
                    int regionIdx = CheckRegion(pole.mPoints[0].x - subROI.x, 14, subROI.width, ispResult.mCenterWidth);
                    float aThreshold = 0.0;
                    switch (regionIdx)
                    {
                        case 1:
                            aThreshold = mIspAnode.mAnodeThresholdOuter;
                            break;
                        case 2:
                            aThreshold = (mIspAnode.mAnodeThresholdOuter + mIspAnode.mAnodeThresholdMiddle) / 2.0;
                            break;
                        case 3:
                            aThreshold = mIspAnode.mAnodeThresholdMiddle;
                            break;
                        case 4:
                        case 5:
                        case 6:
                            aThreshold = (mIspAnode.mAnodeThresholdMiddle + mIspAnode.mAnodeThresholdInner) / 2.0;
                        case 7:
                            aThreshold = mIspAnode.mAnodeThresholdInner;
                            break;
                        default:
                            aThreshold = mIspAnode.mAnodeThresholdInner;
                    }
                    //Check if pole is within outer region to restrict movement
                    bool ifRestrict = false;
                    if (regionIdx == 1)
                    {
                        ifRestrict = true;
                    }
                    int limitYAnode = 0;// subROI.y;// std::max(ispResult.mCenter.y, subROI.y);
                    AnodeResult = FindAnodeByLineTracing(blurImg, pole.mPoints, pole.mPoints[0], limitYAnode, 2, 2, aThreshold * 10, 0, ifRestrict, 0, offset);
                }
                else
                {
                    //Find Anode SubRoi
                    count = count + 1;
                    int xSubAnoRect = ispResult.mCathodePos[count - 1].x - XPaddingAno;
                    int ySubAnoRect = ispResult.mPoleRegionRoi.y;
                    int widSubAnoRect = ispResult.mCathodePos[count].x - ispResult.mCathodePos[count - 1].x + 2 * XPaddingAno;
                    int heiSubAnoRect = (ispResult.mCathodePos[count - 1].y > ispResult.mCathodePos[count].y) ? ispResult.mCathodePos[count - 1].y - ySubAnoRect : ispResult.mCathodePos[count].y - ySubAnoRect;
                    cv::Rect subAnoRoi = cv::Rect(cv::Point(xSubAnoRect, ySubAnoRect), cv::Size(widSubAnoRect, heiSubAnoRect));

                    VecPoint anodePole = ispResult.mCathodePos;
                    if (mChooseAnodeAlgorithm == 1)
                    {
                        AnodeResult = FindAnodeByProminence(src, pole.mPoints, pole.mPoints[0]);
                    }
                    else
                    {
                        AnodeResult = FindAnodeByShape(src, pole.mPoints, pole.mPoints[0], subAnoRoi);
                    }
                }
                ispResult.CombineResult(&AnodeResult);
                if (!pole.mPoints.empty())
                {
                    pole.mAnode = pole.mPoints.back();
                }
                else
                {
                    pole.mAnode = pole.mCathode;
                }
            }
#if _DEBUG_ANODE
            {
                if (!ispResult.mAnodePoles.mPoles.empty())
                {
                    //peakMat0(center).setTo(0);

                    cv::Mat debugAnodeImg;
                    ConvertRGB(blurImg, debugAnodeImg);
                    VecPoint anodes;
                    cv::Mat maskImg = cv::Mat::zeros(subROI.size(), CV_8UC1);

                    for (auto& pole : ispResult.mAnodePoles.mPoles)
                    {
                        for (auto& p : pole.mPoints)
                        {
                            cv::Point curPos = p - offset;
                            anodes.emplace_back(curPos);
                            maskImg.at<uchar>(curPos) = 1;
                        }
                    }
                    DrawPoints(debugAnodeImg, anodes, cv::Scalar(0, 255, 0));
                    //cv::Mat anodeImg;
                    //blurImg.convertTo(anodeImg, peakMat0.type());
                    //anodeImg = anodeImg + peakMat0*3;
                    //cv::bitwise_and(peakMat0, maskImg, anodeImg);
                }
            }
#endif // _DEBUG_ANODE
#if USE_POLE_REFINEMENT
            // Refine incorrect measurement by using symmetry property
            VecInt lstPolePosXAll, anodePos, cathodePos;
            for (auto& pole : ispResult.mAnodePoles.mPoles)
            {
                lstPolePosXAll.emplace_back(pole.mCathode.x - offset.x);
                anodePos.emplace_back(pole.mAnode.y - offset.y);
                cathodePos.emplace_back(pole.mCathode.y - offset.y);
            }
            std::string errMes;
            PoleLengthRefinement(lstPolePosXAll, anodePos, cathodePos, subROI.width, errMes, cv::Mat());
            if (!errMes.empty())
            {
                for (int i = 0; i < anodePos.size(); i++)
                {
                    int realAnodeYPos = anodePos[i] + offset.y;
                    auto& lstPoles = ispResult.mAnodePoles.mPoles;
                    if (realAnodeYPos != lstPoles[i].mAnode.y && !lstPoles.empty())
                    {
                        int poleIdx = abs(lstPoles[i].mCathode.y - realAnodeYPos);
                        if (poleIdx >= lstPoles[i].mPoints.size())
                        {
                            lstPoles[i].mPoints.emplace_back(lstPoles[i].mAnode.x, realAnodeYPos);
                        }
                        else if (poleIdx > 0)
                        {
                            lstPoles[i].mPoints.erase(lstPoles[i].mPoints.begin() + poleIdx, lstPoles[i].mPoints.end());
                        }
                        lstPoles[i].mAnode = lstPoles[i].mPoints.back();
                    }
                }
            }
#endif // USE_POLE_LENGTH_REFINE

            //ispResult.mAnodePoles.mPixelSize = mPixelSize;
            //ispResult.mAnodePoles.mValidAnode2CaseRange = mIspPole.mValidAnode2CaseRange;
            //ispResult.mAnodePoles.mValidCathode2AnodeRange = mIspPole.mValidCathode2AnodeRange;
            //ispResult.mAnodePoles.mValidCathode2CaseRange = mIspPole.mValidCathode2CaseRange;
#if _DEBUG_ANODE

            cv::Mat debugAnodeImg;
            ConvertRGB(src, debugAnodeImg);
            VecPoint anodes;
            for (auto& pole : ispResult.mAnodePoles.mPoles)
            {
                anodes.emplace_back(pole.mCathode.x, pole.mAnode.y);
            }
            DrawPoints(debugAnodeImg, anodes, cv::Scalar(0, 0, 255));
#endif // _DEBUG_ANODE

            for (auto& pole : ispResult.mAnodePoles.mPoles)
            {
                ispResult.mAnodePoles.mXPoles.emplace_back(abs(ispResult.mOuterRoi.x - pole.mCathode.x) * mPixelSize);

                bool Cathode2AnodeNG = !mIspPole.mValidCathode2AnodeRange.IsInRange(pole.length() * mPixelSize);
                pole.SetResult((Cathode2AnodeNG) ? EResult::NG : EResult::OK);
                ispResult.CombineResult(pole.GetResult());
            }

            for (int i = 0; i < ispResult.mAnodePoles.Size(); i++)
            {
                bool Cathode2AnodeNG = !mIspPole.mValidCathode2AnodeRange.IsInRange(ispResult.mAnodePoles.mPoles[i].length() * mPixelSize);
                ispResult.mAnodePoles.mPoles[i].SetResult((Cathode2AnodeNG) ? EResult::NG : EResult::OK);
                ispResult.CombineResult(ispResult.mAnodePoles.mPoles[i].GetResult());
            }
            anodeInspection(EResult::OK, "");
        }
        else
        {
            anodeInspection(EResult::ER, "Peak Anode size is not enough!");
        }
    }
    else
    {
        anodeInspection(EResult::ER, "No Image in Anode - Lower!");
    }
    return anodeInspection;
}

void CTBatteryInspectorResult::UpdateTableData() const
{
    int sizeCathode = mCathodePos.size();
    int sizeC2BDis = sCathode2Case.size();
    int sizeC2BDec = vtCathode2CaseDecision.size();
    int sizeXCatPos = mXCathodePos.size();
    int sizeAnode = mAnodePoles.mPoles.size();
    int sizeXAnoPos = mAnodePoles.mXPoles.size();
    int sizeA2CLR = mAnodePoles.mCathodeLR2Anode.size();
    int sizeA2B = mAnodePoles.mAnode2Case.size();

    if (!(sizeC2BDis > 0 && (sizeC2BDis == sizeC2BDec) && (sizeC2BDis == sizeXCatPos) && (sizeC2BDis == sizeCathode)
        && sizeAnode > 0 && (sizeAnode == sizeXAnoPos) && (sizeAnode == sizeA2CLR) && (sizeAnode == sizeA2B)))
        return;

    CVPen pen;
    pen.mSpace = 15;
    pen.mFontScale = 0.6;
    pen.mThickness = 2;
    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    bool enableA2C = mAnodePoles.mValidCathode2AnodeRange.mIsEnable;
    bool enableA2B = mAnodePoles.mValidAnode2CaseRange.mIsEnable;
    bool enableC2B = mAnodePoles.mValidCathode2CaseRange.mIsEnable && (!enableA2C && !enableA2B);
    int col_width = 75;
    int row_height = 30;
    bool drawBorder = 0;
    int heightLimit = 100;
    float thickness = pen.mThickness;
    float precision = mDecimalNo;
    float font_scale = pen.mFontScale;
    auto penBorder = drawBorder ? std::make_shared<xvt::CVPen>(COLOR_CV_WHITE, 1, font_scale) : nullptr;
    auto pen0 = std::make_shared<xvt::CVPen>(COLOR_CV_WHITE, thickness, font_scale);
    auto penOK = pen0;// std::make_shared<xvt::CVPen>(COLOR_CV_YELLOW, thickness, font_scale);
    auto penNG = std::make_shared<xvt::CVPen>(COLOR_CV_RED, thickness, font_scale);

    if(mPoleTableR.GetSize().empty() && mPoleTableL.GetSize().empty())
    {
        Table drawingL(col_width, row_height, pen0, penBorder);
        Table drawingR(col_width, row_height, pen0, penBorder);
        VecString headerString = { "No.", "Pos", "A-C(L)", "A-C(R)", "A-B", "C-B" };
        drawingL.AddRow(headerString);
        drawingR.AddRow(headerString);
        std::string idStr = enableC2B ? "C" : "A";
        std::string tmpStr = "-";
        if (enableC2B)
        {
            for (int i = 0; i < sizeCathode; i++)
            {
                bool left = i < nNoCathodeLeft;
                int idx = left ? abs(i - (nNoCathodeLeft - 1)) : i;

                bool isCathode2BaseError = !vtCathode2CaseDecision[idx];
                VecString Cat;
                {
                    Cat.push_back(xvt::ToString(mXCathodePos[idx], precision));
                    Cat.push_back(tmpStr);
                    Cat.push_back(tmpStr);
                    Cat.push_back(tmpStr);
                    Cat.push_back(xvt::ToString(sCathode2Case[idx], precision));
                }

                if (left)
                {
                    Cat.insert(Cat.begin(), idStr + "L" + xvt::ToString(nNoCathodeLeft - idx));

                    drawingL.AddRow(Cat);
                    auto current_row = drawingL.GetRows() - 1;
                    drawingL.SetCellColor(current_row, 0, isCathode2BaseError ? penNG : penOK, penBorder);
                    drawingL.SetCellColor(current_row, 2, penOK, penBorder);
                    drawingL.SetCellColor(current_row, 3, penOK, penBorder);
                    drawingL.SetCellColor(current_row, 4, penOK, penBorder);
                    drawingL.SetCellColor(current_row, 5, isCathode2BaseError ? penNG : penOK, penBorder);
                }
                else
                {
                    Cat.insert(Cat.begin(), idStr + "R" + xvt::ToString(i + 1 - nNoCathodeLeft));

                    drawingR.AddRow(Cat);
                    auto current_row = drawingR.GetRows() - 1;
                    drawingR.SetCellColor(current_row, 0, isCathode2BaseError ? penNG : penOK, penBorder);
                    drawingR.SetCellColor(current_row, 2, penOK, penBorder);
                    drawingR.SetCellColor(current_row, 3, penOK, penBorder);
                    drawingR.SetCellColor(current_row, 4, penOK, penBorder);
                    drawingR.SetCellColor(current_row, 5, isCathode2BaseError ? penNG : penOK, penBorder);
                }
            }
        }
        else
        {
            for (int i = 0; i < sizeAnode; i++)
            {
                bool left = i < mAnodePoles.mLeftNoPole;
                int idx = left ? abs(i - (mAnodePoles.mLeftNoPole - 1)) : i;

                bool isA2CLError = false;
                bool isA2CRError = false;
                bool isA2BError = false;
                auto const& c2aLR = mAnodePoles.mCathodeLR2Anode[idx];
                VecString Ano;
                {
                    Ano.push_back(xvt::ToString(mAnodePoles.mXPoles[idx], precision));
                    if (mAnodePoles.mValidCathode2AnodeRange.mIsEnable)
                    {
                        isA2CLError = c2aLR.first.result == EResult::NG;
                        isA2CRError = c2aLR.second.result == EResult::NG;
                        auto lStr = c2aLR.first.value < 0 ? tmpStr : xvt::ToString(c2aLR.first.value, precision);
                        auto rStr = c2aLR.second.value < 0 ? tmpStr : xvt::ToString(c2aLR.second.value, precision);
                        Ano.push_back(lStr);
                        Ano.push_back(rStr);
                    }
                    else
                    {
                        Ano.push_back(tmpStr);
                        Ano.push_back(tmpStr);
                    }

                    if (mAnodePoles.mValidAnode2CaseRange.mIsEnable)
                    {
                        //auto value = mAnodePoles.mPoles[idx].mPeakValue.value;
                        //isAnode2BaseError = value < 20 || mAnodePoles.mPoles[idx].mPeakValue.prominence<30;// mAnodePoles.mAnode2Case[idx].result == EResult::NG;
                        //Ano.push_back(xvt::ToString(value, precision));
                        isA2BError = mAnodePoles.mAnode2Case[idx].result == EResult::NG;
                        Ano.push_back(xvt::ToString(mAnodePoles.mAnode2Case[idx].value, precision));
                    }
                    else
                    {
                        Ano.push_back(tmpStr);
                    }

                    {
                        //Ano.push_back(xvt::ToString(mAnodePoles.mPoles[idx].mPeakValue.prominence, precision));
                        Ano.push_back(tmpStr);
                    }
                }

                bool isPoleError = isA2BError || isA2CLError || isA2CRError;
                if (left)
                {
                    Ano.insert(Ano.begin(), idStr + "L" + xvt::ToString(mAnodePoles.mLeftNoPole - idx));
                    drawingL.AddRow(Ano);
                    auto current_row = drawingL.GetRows() - 1;
                    drawingL.SetCellColor(current_row, 0, isPoleError ? penNG : penOK, penBorder);
                    drawingL.SetCellColor(current_row, 2, isA2CLError ? penNG : penOK, penBorder);
                    drawingL.SetCellColor(current_row, 3, isA2CRError ? penNG : penOK, penBorder);
                    drawingL.SetCellColor(current_row, 4, isA2BError ? penNG : penOK, penBorder);
                    drawingL.SetCellColor(current_row, 5, penOK, penBorder);
                }
                else
                {
                    Ano.insert(Ano.begin(), idStr + "R" + xvt::ToString(i + 1 - mAnodePoles.mLeftNoPole));
                    drawingR.AddRow(Ano);
                    auto current_row = drawingR.GetRows() - 1;
                    drawingR.SetCellColor(current_row, 0, isPoleError ? penNG : penOK, penBorder);
                    drawingR.SetCellColor(current_row, 2, isA2CLError ? penNG : penOK, penBorder);
                    drawingR.SetCellColor(current_row, 3, isA2CRError ? penNG : penOK, penBorder);
                    drawingR.SetCellColor(current_row, 4, isA2BError ? penNG : penOK, penBorder);
                    drawingR.SetCellColor(current_row, 5, penOK, penBorder);
                }
            }
        }

        mPoleTableL = drawingL;
        mPoleTableR = drawingR;
    }

    if (mPoleNoTableL.GetSize().empty() && mPoleNoTableR.GetSize().empty())
    {
        // Draw text pole no info
        std::stringstream leftText, rightText;
        int widthPoleCol = col_width * 1.5;
        Table drawingPoleNoL(col_width * 1.5, row_height, pen0, nullptr);
        Table drawingPoleNoR(col_width * 1.5, row_height, pen0, nullptr);

        std::vector<Table::ColumnData> poleNoColLeft, poleNoColRight;
        bool enabeAnode = enableA2C || enableA2B;
        if (enabeAnode)
        {
            poleNoColLeft.push_back({ std::make_shared<Cell>("No. AL:", pen0, nullptr) });
            poleNoColLeft.push_back({ std::make_shared<Cell>(xvt::ToString(mAnodePoles.mLeftNoPole), mAnodePoles.mLeftNoPole >= mOneSidePoleNo ? penOK : penNG, nullptr) });
            poleNoColLeft[0][0]->mTextAlign = TextAlign::RIGHT;
            poleNoColLeft[1][0]->mTextAlign = TextAlign::CENTER;

            poleNoColRight.push_back({ std::make_shared<Cell>("No. AR:", pen0, nullptr) });
            int noRight = sizeAnode - mAnodePoles.mLeftNoPole;
            poleNoColRight.push_back({ std::make_shared<Cell>(xvt::ToString(noRight), noRight >= mOneSidePoleNo ? penOK : penNG, nullptr) });
            poleNoColRight[0][0]->mTextAlign = TextAlign::RIGHT;
            poleNoColRight[1][0]->mTextAlign = TextAlign::CENTER;
        }
        if (enableC2B || enabeAnode)
        {
            auto colEx = std::make_shared<int>(enabeAnode ? col_width * 2 : widthPoleCol);
            poleNoColLeft.push_back({ std::make_shared<Cell>("No. CL:", pen0, nullptr) });
            poleNoColLeft.push_back({ std::make_shared<Cell>(xvt::ToString(nNoCathodeLeft), nNoCathodeLeft >= mOneSidePoleNo ? penOK : penNG) });
            int lastIdx = poleNoColLeft.size() - 1;
            poleNoColLeft[lastIdx - 1][0]->mTextAlign = TextAlign::RIGHT;
            poleNoColLeft[lastIdx][0]->mTextAlign = TextAlign::CENTER;

            int noRight = sizeCathode - nNoCathodeLeft;
            poleNoColRight.push_back({ std::make_shared<Cell>("No. CR:", pen0, nullptr) });
            poleNoColRight.push_back({ std::make_shared<Cell>(xvt::ToString(noRight), noRight >= mOneSidePoleNo ? penOK : penNG) });
            poleNoColRight[lastIdx - 1][0]->mTextAlign = TextAlign::RIGHT;
            poleNoColRight[lastIdx][0]->mTextAlign = TextAlign::CENTER;
        }

        for (auto col : poleNoColLeft)
            drawingPoleNoL.AddColumn(col);

        for (auto col : poleNoColRight)
            drawingPoleNoR.AddColumn(col);

        float scaleWidth = 1.8;
        if (drawingPoleNoL.GetColumns() == 4)
        {
            drawingPoleNoL.GetColumn(1)[0]->mBorderRight->SetStyle(pen0);
            drawingPoleNoL.SetColumnWidth(0, col_width * scaleWidth);
            drawingPoleNoL.SetColumnWidth(1, col_width);
            drawingPoleNoL.SetColumnWidth(2, col_width * scaleWidth);
            drawingPoleNoL.SetColumnWidth(3, col_width);
        }

        if (drawingPoleNoR.GetColumns() == 4)
        {
            drawingPoleNoR.SetColumnWidth(0, col_width * scaleWidth);
            drawingPoleNoR.SetColumnWidth(1, col_width);
            drawingPoleNoR.SetColumnWidth(2, col_width * scaleWidth);
            drawingPoleNoR.SetColumnWidth(3, col_width);
            drawingPoleNoR.GetColumn(1)[0]->mBorderRight->SetStyle(pen0);
        }


        mPoleNoTableL = drawingPoleNoL;
        mPoleNoTableR = drawingPoleNoR;
    }
}

void CTBatteryInspectorResult::DrawPoleTextResult(cv::Mat& resImg, CVPen const& pen, cv::Point const& offset) const
{
    UpdateTableData();
#if 0
// "A-CL", "A-CR", "A-B"
    Table::ColumnData cNo, cA2CL, cA2CR, cA2B, cC2B, cPosAnode, cPosCathode;
    std::string tmp = "-";
    auto emplty = std::make_shared<xvt::Cell>(tmp, pen0);
    int anodeSize = mAnodePoles.mPoles.size();
    int cathodeSize = mCathodePos.size();
    int noCathodeLeft = nNoCathodeLeft == 0 ? cathodeSize / 2 : nNoCathodeLeft;
    int noLeftRow = std::max(mAnodePoles.mLeftNoPole, nNoCathodeLeft);
    int noRightRow = std::max(anodeSize - mAnodePoles.mLeftNoPole, cathodeSize - noCathodeLeft);
    int totalRows = noLeftRow + noRightRow;
    int noLeftAnodeAdded = noLeftRow - mAnodePoles.mLeftNoPole;
    int noRightAnodeAdded = noRightRow - (anodeSize - mAnodePoles.mLeftNoPole);


    for (int i = 0; i < anodeSize; i++)
    {
        cPosAnode.push_back(std::make_shared<xvt::Cell>(xvt::ToString(mAnodePoles.mXPoles[i], 3), pen0));
        if (enableA2C)
        {
            const auto& curC2ALR = mAnodePoles.mCathodeLR2Anode[i];
            cA2CL.push_back(std::make_shared<xvt::Cell>(xvt::ToString(curC2ALR.first.value, 3), curC2ALR.first.result == EResult::NG ? penNG : penOK));
            cA2CR.push_back(std::make_shared<xvt::Cell>(xvt::ToString(curC2ALR.second.value, 3), curC2ALR.second.result == EResult::NG ? penNG : penOK));
        }
        else
        {
            cA2CL.push_back(emplty);
            cA2CR.push_back(emplty);
        }


        if(enableA2B)
            cA2B.push_back(std::make_shared<xvt::Cell>(xvt::ToString(mAnodePoles.mAnode2Case[i].value, 3), mAnodePoles.mAnode2Case[i].result == EResult::NG ? penNG : penOK));
        else
            cA2B.push_back(emplty);

        if (i == mAnodePoles.mLeftNoPole - 1 && noLeftAnodeAdded != 0)
        {
            for (int j = 0; j < noLeftAnodeAdded; j++)
            {
                cPosAnode.push_back(emplty);
                cA2CL.push_back(emplty);
                cA2CR.push_back(emplty);
                cA2B.push_back(emplty);
            }
        }
    }

    if (noRightAnodeAdded != 0)
    {
        for (int j = 0; j < noRightAnodeAdded; j++)
        {
            cPosAnode.push_back(emplty);
            cA2CL.push_back(emplty);
            cA2CR.push_back(emplty);
            cA2B.push_back(emplty);
        }
    }

    int noLeftCathodeAdded = noLeftRow - noCathodeLeft;
    int noRightCathodeAdded = noRightRow - (cathodeSize - noCathodeLeft);
    for (int i = 0; i < cathodeSize; i++)
    {
        if (enableC2B)
        {
            cPosCathode.push_back(std::make_shared<xvt::Cell>(xvt::ToString(mXCathodePos[i], 3), pen0));
            cC2B.push_back(std::make_shared<xvt::Cell>(xvt::ToString(sCathode2Case[i], 3), !vtCathode2CaseDecision[i] ? penNG : penOK));
        }
        else
        {
            cPosCathode.push_back(emplty);
            cC2B.push_back(emplty);
        }


        if (i == noCathodeLeft - 1 && noLeftCathodeAdded != 0)
        {
            for (int j = 0; j < noLeftCathodeAdded; j++)
            {
                cPosCathode.push_back(emplty);
                cC2B.push_back(emplty);
            }
        }
    }

    if (noRightCathodeAdded != 0)
    {
        for (int j = 0; j < noRightCathodeAdded; j++)
        {
            cPosCathode.push_back(emplty);
            cC2B.push_back(emplty);
        }
    }

    if (cA2CL.size() == totalRows
        || cA2CR.size() == totalRows
        || cA2B.size() == totalRows
        || cC2B.size() == totalRows
        || cPosAnode.size() == totalRows
        || cPosCathode.size() == totalRows)
    {
        int count = 1;
        int noLeftPoleAdded = noLeftRow - (enableC2B ? noCathodeLeft : mAnodePoles.mLeftNoPole);
        int noRightPoleAdded = noRightRow - (enableC2B ? (cathodeSize - noCathodeLeft) : (anodeSize - mAnodePoles.mLeftNoPole));
        int endLeft = noLeftRow - noLeftPoleAdded;
        int endRight = totalRows - noRightPoleAdded;
        for (int i = 0; i < totalRows; i++)
        {
            if (i > endLeft && i < noLeftRow
                || i > endRight && i < totalRows)
            {
                cNo.push_back(emplty);
            }
            else
            {
                bool checkNG = cA2CL[i]->mFontStyle->mColor == COLOR_CV_RED
                                || cA2CR[i]->mFontStyle->mColor == COLOR_CV_RED
                                || cA2B[i]->mFontStyle->mColor == COLOR_CV_RED
                                || cC2B[i]->mFontStyle->mColor == COLOR_CV_RED;
            
                std::string str = enableC2B ? "C" : "A";
                cNo.push_back(std::make_shared<xvt::Cell>(str + xvt::ToString(count), checkNG ? penNG : penOK));
                count++;
            }
        }

        drawingL.AddColumn(Table::ColumnData(cNo.begin(), cNo.begin() + endLeft));
        if(enableC2B)
            drawingL.AddColumn(Table::ColumnData(cPosCathode.begin(), cPosCathode.begin() + endLeft));
        else
            drawingL.AddColumn(Table::ColumnData(cPosAnode.begin(), cPosAnode.begin() + endLeft));

        drawingL.AddColumn(Table::ColumnData(cA2CL.begin(), cA2CL.begin() + endLeft));
        drawingL.AddColumn(Table::ColumnData(cA2CR.begin(), cA2CR.begin() + endLeft));
        drawingL.AddColumn(Table::ColumnData(cA2B.begin(), cA2B.begin() + endLeft));
        drawingL.AddColumn(Table::ColumnData(cC2B.begin(), cC2B.begin() + endLeft));
        
        drawingR.AddColumn(Table::ColumnData(cNo.begin() + noRightRow, cNo.begin() + endRight));
        if (enableC2B)
            drawingR.AddColumn(Table::ColumnData(cPosCathode.begin() + noRightRow, cPosCathode.begin() + endRight));
        else
            drawingR.AddColumn(Table::ColumnData(cPosAnode.begin() + noRightRow, cPosAnode.begin() + endRight));
        drawingR.AddColumn(Table::ColumnData(cA2CL.begin() + noRightRow, cA2CL.begin() + endRight));
        drawingR.AddColumn(Table::ColumnData(cA2CR.begin() + noRightRow, cA2CR.begin() + endRight));
        drawingR.AddColumn(Table::ColumnData(cA2B.begin() + noRightRow, cA2B.begin() + endRight));
        drawingR.AddColumn(Table::ColumnData(cC2B.begin() + noRightRow, cC2B.begin() + endRight));

        VecString headerString = { "No", "Pos", "A-CL", "A-CR", "A-B", "C-B" };
    }

#endif
    auto l_size = mPoleTableL.GetSize();
    auto r_size = mPoleTableR.GetSize();

    int offset_x = 30;
    cv::Point draw_pos = cv::Point(offset_x, offset_x);
    int expand_height = 0;

    if (expand_height <= 0) expand_height = std::max(l_size.height, r_size.height) + draw_pos.y * 2;

    cv::Mat tableImg = cv::Mat(resImg.rows + expand_height, resImg.cols, resImg.type(), cv::Scalar::all(0));
    draw_pos.y += resImg.rows;
    draw_pos.x = (resImg.cols - l_size.width * 2) / 3;
    resImg.copyTo(tableImg(cv::Rect(0, 0, resImg.cols, resImg.rows)));
    mPoleTableL.Draw(tableImg, draw_pos);

    cv::Point drawRPos = cv::Point(resImg.cols - draw_pos.x - r_size.width, draw_pos.y);
    mPoleTableR.Draw(tableImg, drawRPos);
    mPoleNoTableL.Draw(tableImg, draw_pos + cv::Point((l_size.width - mPoleNoTableL.GetSize().width) / 2, -60));
    mPoleNoTableR.Draw(tableImg, drawRPos + cv::Point((r_size.width - mPoleNoTableR.GetSize().width) / 2, -60));
    resImg = std::move(tableImg);
}

void CTBatteryInspectorResult::SaveDetailsCSV(std::wstring path)
{
    UpdateTableData();
    if (mPoleTableL.GetColumns() < 2
        && mPoleTableL.GetRows() < 2)
        return;

    auto l_rows = mPoleTableL.GetRows();
    auto l_cols = mPoleTableL.GetColumns();
    auto r_rows = mPoleTableR.GetRows();
    auto r_cols = mPoleTableR.GetColumns();

    std::vector<CSVOutput> mCSVPoleDetails;
    CSVOutput tmpCSV;
    tmpCSV.emplace_back("No.","");
    tmpCSV.emplace_back("Pos","");
    tmpCSV.emplace_back("A-C(L)","");
    tmpCSV.emplace_back("A-C(R)","");
    tmpCSV.emplace_back("A-B","");
    tmpCSV.emplace_back("C-B","");
    for (int i = 1; i < l_rows; i++)
    {
        auto tCSV = tmpCSV;
        auto const& row = mPoleTableL.GetRow(i);
        for (int j = 0; j < l_cols; j++)
        {
            tCSV[j].second = row[j]->mText;
        }
        mCSVPoleDetails.push_back(tCSV);
    }

    for (int i = 1; i < r_rows; i++)
    {
        auto tCSV = tmpCSV;
        auto const& row = mPoleTableR.GetRow(i);
        for (int j = 0; j < r_cols; j++)
        {
            tCSV[j].second = row[j]->mText;
        }
        mCSVPoleDetails.push_back(tCSV);
    }

    if (mCSVPoleDetails.empty())
        mCSVPoleDetails.push_back(tmpCSV);
    
    if(mCSVPoleDetails.size() > 0) CSV::Save(path, mCSVPoleDetails.front(), true);

    for(int i = 1; i < mCSVPoleDetails.size(); i++)
        CSV::Save(path, mCSVPoleDetails[i], false);
}

void CTBatteryInspectorResult::DrawValidCaseLineResult(cv::Mat& img, CVPen pen) const
{
    int sign = mDirection == 0 ? -1 : 1;
    auto batteryROI = mOuterRoi;
    cv::Scalar minColor = cv::Scalar(220, 220, 0);
    cv::Scalar avgColor = cv::Scalar(255, 150, 0);
    cv::Scalar whiteColor = cv::Scalar(255, 255, 255);

    //if (Description.empty() && (mDisplayMode & DisplayMode::TEXT) == DisplayMode::TEXT)

    if(0)
    {
        
        if (mAnodePoles.mValidCathode2CaseRange.mIsEnable)
        {
            int xCathode2Base1 = batteryROI.x;
            int xCathode2Base2 = batteryROI.x + batteryROI.width / 5;
            int yCathode2Base1 = mAnodePoles.mCaseRef.y - sign * mAnodePoles.mValidCathode2CaseRange.GetLower() / mPixelSize;
            int yCathode2Base2 = mAnodePoles.mCaseRef.y - sign * avgCathode2Case / mPixelSize;
            int yCathode2Base3 = mAnodePoles.mCaseRef.y - sign * mAnodePoles.mValidCathode2CaseRange.GetUpper() / mPixelSize;
            Drawing::DrawDashedLine(img, cv::Point(xCathode2Base1, yCathode2Base1), cv::Point(xCathode2Base2, yCathode2Base1), minColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xCathode2Base1, yCathode2Base2), cv::Point(xCathode2Base2, yCathode2Base2), avgColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xCathode2Base1, yCathode2Base3), cv::Point(xCathode2Base2, yCathode2Base3), cv::Scalar(0, 0, 255), 2, "", 10);
        }

        if (mAnodePoles.mValidAnode2CaseRange.mIsEnable)
        {
            int xAnode2Base1 = batteryROI.x + batteryROI.width * 2 / 5;
            int xAnode2Base2 = batteryROI.x + batteryROI.width * 3 / 5;
            int yAnode2Base1 = mAnodePoles.mCaseRef.y - sign * mAnodePoles.mValidAnode2CaseRange.GetLower() / mPixelSize;
            int yAnode2Base2 = mAnodePoles.mCaseRef.y - sign * avgAnode2Case / mPixelSize;
            int yAnode2Base3 = mAnodePoles.mCaseRef.y - sign * mAnodePoles.mValidAnode2CaseRange.GetUpper() / mPixelSize;
            Drawing::DrawDashedLine(img, cv::Point(xAnode2Base1, yAnode2Base1), cv::Point(xAnode2Base2, yAnode2Base1), minColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xAnode2Base1, yAnode2Base2), cv::Point(xAnode2Base2, yAnode2Base2), avgColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xAnode2Base1, yAnode2Base3), cv::Point(xAnode2Base2, yAnode2Base3), cv::Scalar(0, 0, 255), 2, "", 10);
        }

        if (mAnodePoles.mValidCathode2AnodeRange.mIsEnable)
        {
            int xCathode2Anode1 = batteryROI.x + batteryROI.width * 4 / 5;
            int xCathode2Anode2 = batteryROI.x + batteryROI.width;
            int avgCathode2Anode = (mAnodePoles.mValidCathode2AnodeRange.GetUpper() - mAnodePoles.mValidCathode2AnodeRange.GetLower()) / (mPixelSize * 2.0);
            int yCathode2Anode2 = mAnodePoles.mCaseRef.y - sign * (avgCathode2Case + avgAnode2Case) / (mPixelSize * 2);
            int yCathode2Anode1 = yCathode2Anode2 + sign * avgCathode2Anode;
            int yCathode2Anode3 = yCathode2Anode2 - sign * avgCathode2Anode;
            Drawing::DrawDashedLine(img, cv::Point(xCathode2Anode1, yCathode2Anode1), cv::Point(xCathode2Anode2, yCathode2Anode1), minColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xCathode2Anode1, yCathode2Anode2), cv::Point(xCathode2Anode2, yCathode2Anode2), avgColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xCathode2Anode1, yCathode2Anode3), cv::Point(xCathode2Anode2, yCathode2Anode3), cv::Scalar(0, 0, 255), 2, "", 10);
        }

        if (mAnodePoles.mValidCathode2CaseRange.mIsEnable
            || mAnodePoles.mValidAnode2CaseRange.mIsEnable
            || mAnodePoles.mValidCathode2AnodeRange.mIsEnable)
        {
            // draw line info
            //Draw the setting ROI
            const int xLineInfo1 = 100;
            const int xLineInfo2 = xLineInfo1 + 100;
            const int yLineInfo1 = 130;
            const int yLineInfoOffset = 25;
            Drawing::DrawDashedLine(img, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset), cv::Point(xLineInfo2, yLineInfo1 + yLineInfoOffset), cv::Scalar(0, 0, 255), 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 2), cv::Point(xLineInfo2, yLineInfo1 + yLineInfoOffset * 2), avgColor, 2, "", 10);
            Drawing::DrawDashedLine(img, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 3), cv::Point(xLineInfo2, yLineInfo1 + yLineInfoOffset * 3), minColor, 2, "", 10);
            cv::line(img, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 4), cv::Point(xLineInfo2 - 10, yLineInfo1 + yLineInfoOffset * 4), cv::Scalar(0, 255, 0), 3);
            pen.mColor = COLOR_CV_WHITE;
            cv::Point offsetDraw = cv::Point(-15, 0);
            xvt::DrawText(img, "Max", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset), TextAlign::MIDDLE_RIGHT, pen, offsetDraw);
            xvt::DrawText(img, "Avg", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 2), TextAlign::MIDDLE_RIGHT, pen, offsetDraw);
            xvt::DrawText(img, "Min", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 3), TextAlign::MIDDLE_RIGHT, pen, offsetDraw);
            xvt::DrawText(img, "Base", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 4), TextAlign::MIDDLE_RIGHT, pen, offsetDraw);
        }
    }
    else
    {
        auto const& vC2BRange = mAnodePoles.mValidCathode2CaseRange;
        auto const& vA2BRange = mAnodePoles.mValidAnode2CaseRange;
        int offset = 3;
        int xBase1 = mCenter.x - mCenterWidth / 2 + offset;
        int xBase2 = mCenter.x + mCenterWidth / 2;
        int yCathode2Base2 = mAnodePoles.mCaseRef.y - sign * (vC2BRange.GetLower() + vC2BRange.GetUpper()) / (mPixelSize * 2);
        int yAnode2Base2 = mAnodePoles.mCaseRef.y - sign * (vA2BRange.GetLower() + vA2BRange.GetUpper()) / (mPixelSize * 2);
        pen.mFontScale = 0.5;
        pen.mColor = whiteColor;
        pen.mThickness = 2;
        auto textPen = pen;
        //textPen.mColor = COLOR_CV_YELLOW;

        if (mAnodePoles.mValidCathode2AnodeRange.mIsEnable
            && minLenghtIdx != -1
            && maxLenghtIdx != -1
            && mAnodePoles.mPoles.size() > 0)
        {
            auto poleMin = mAnodePoles.mPoles[minLenghtIdx];
            auto poleMax = mAnodePoles.mPoles[maxLenghtIdx];
            xvt::DrawLine(img, cv::Point(poleMin.mCathode.x, poleMin.mCathode.y), cv::Point(poleMin.mCathode.x, poleMin.mAnode.y), pen);
            xvt::DrawText(img, "Min", cv::Point(poleMin.mCathode.x + 3, (poleMin.mCathode.y + poleMin.mAnode.y) / 2), TextAlign::MIDDLE_LEFT, textPen);
            xvt::DrawLine(img, cv::Point(poleMax.mCathode.x, poleMax.mCathode.y), cv::Point(poleMax.mCathode.x, poleMax.mAnode.y), pen);
            xvt::DrawText(img, "Max", cv::Point(poleMax.mCathode.x + 3, (poleMax.mCathode.y + poleMax.mAnode.y) / 2), TextAlign::MIDDLE_LEFT, textPen);
        }

        if (vC2BRange.mIsEnable)
        {
            xvt::DrawText(img, "Ca.", cv::Point((xBase1 + xBase2) / 2, yCathode2Base2 - offset), xvt::TextAlign::CENTER, textPen);
            Drawing::DrawDashedLine(img, cv::Point(xBase1, yCathode2Base2), cv::Point(xBase2, yCathode2Base2), whiteColor, 2, "", 5);
        }

        if (vA2BRange.mIsEnable)
        {
            xvt::DrawText(img, "An.", cv::Point((xBase1 + xBase2) / 2, yAnode2Base2 - offset), xvt::TextAlign::CENTER, textPen);
            Drawing::DrawDashedLine(img, cv::Point(xBase1, yAnode2Base2), cv::Point(xBase2, yAnode2Base2), whiteColor, 2, "", 5);
        }
    }
}

void CTBatteryInspectorResult::DrawPoleResult(cv::Mat& img) const
{
    bool drawCircle = mDisplayMode != DisplayMode::POLE;
    bool simpleColor = true;
    int sign = mDirection == 0 ? -1 : 1;
    const auto enableC2A = mAnodePoles.mValidCathode2AnodeRange.mIsEnable;
    const auto enableA2B = mAnodePoles.mValidAnode2CaseRange.mIsEnable;
    const auto enableC2B = mAnodePoles.mValidCathode2CaseRange.mIsEnable;
    cv::Scalar cathodeTextColor = simpleColor ? COLOR_CV_YELLOW : cv::Scalar(0, 255, 0);
    cv::Scalar anodeTextColor = simpleColor ? COLOR_CV_YELLOW : cv::Scalar(255, 255, 0);
    cv::Scalar cathodeColor = simpleColor ? COLOR_CV_YELLOW : cv::Scalar(255, 255, 0);
    cv::Scalar anodeColor = simpleColor ? COLOR_CV_YELLOW : cv::Scalar(255, 255, 150);
    cv::Scalar ngColor = COLOR_CV_RED;
    float fontScale = 0.45;
    int noDrawText = 10;

    if ((enableC2A || enableA2B)
        && mAnodePoles.mPoles.size() == mAnodePoles.mAnode2Case.size()
        && mAnodePoles.mPoles.size() == mAnodePoles.mCathode2Anode.size())
    {

        for (int i = 0; i < mAnodePoles.mPoles.size(); i++)
        {
            bool left = i < mAnodePoles.mLeftNoPole;
            int idx = left ? abs(i - (mAnodePoles.mLeftNoPole - 1)) : i;
            
            //draw Anode
            cv::Point textOffset = cv::Point(-15, 15 * sign);
            auto anodePos = cv::Point(mAnodePoles.mPoles[idx].mCathode.x, mAnodePoles.mPoles[idx].mAnode.y);
            bool anodeOK = vtAno2CaseDecision[idx] && (vtAno2CathodLRDecision[idx].first && vtAno2CathodLRDecision[idx].second);

            cv::Scalar iColor = anodeOK ? anodeColor : ngColor;
            int curIdx = left ? mAnodePoles.mLeftNoPole - idx - 1 : idx - mAnodePoles.mLeftNoPole;
            bool checkDrawText = curIdx == 0 || curIdx % noDrawText == 0;
            if (checkDrawText)
            {
                std::string textAno = "A" + std::string(left ? "L" : "R") + std::to_string(curIdx + 1);
                cv::putText(img, textAno, anodePos + textOffset, cv::FONT_HERSHEY_SIMPLEX, fontScale, anodeTextColor, 2);
            }
            if (drawCircle)
            {
                cv::circle(img, anodePos, 2, iColor, (checkDrawText || !anodeOK) ? -1 : 1);
            }
            else
            {
                xvt::DrawPlusSign(img, anodePos, iColor, 1, checkDrawText ? 3 : 1);
            }
        }
    }

    //draw Cathode
    if (enableC2A || enableC2B)
    {
        bool checkCathode = enableC2B && (!vtCathode2CaseDecision.empty() && vtCathode2CaseDecision.size() == mCathodePos.size());
        for (int i = 0; i < mCathodePos.size(); i++)
        {
            bool cathodeLeftSide = i < nNoCathodeLeft;
            int idx = cathodeLeftSide ? abs(i - (nNoCathodeLeft - 1)) : i;
            
            cv::Point textOffset = cv::Point(15, 15 * sign);
            bool cathodeOK = true;
            if (checkCathode) cathodeOK = vtCathode2CaseDecision[idx];

            cv::Scalar iColor = cathodeOK ? cathodeColor : ngColor;

            int curIdx = cathodeLeftSide ? nNoCathodeLeft - 1 - idx : idx - nNoCathodeLeft;
            bool checkDrawText = curIdx == 0 || curIdx % noDrawText == 0;
            if (checkDrawText)
            {
                std::string textCat = "C" + std::string(cathodeLeftSide ? "L" : "R") + std::to_string(curIdx + 1);
                cv::putText(img, textCat, mCathodePos[idx] - textOffset, cv::FONT_HERSHEY_SIMPLEX, fontScale, cathodeTextColor, 2);
            }

            if (drawCircle)
            {
                cv::circle(img, mCathodePos[idx], 2, iColor, (checkDrawText || !cathodeOK) ? -1 : 1);
            }
            else
            {
                cv::Point lineOffset = cv::Point(2, 0);
                //xvt::DrawPlusSign(img, mCathodePos[i], iColor, checkDrawText ? 2 : 1, 2);
                cv::line(img, mCathodePos[idx] - lineOffset, mCathodePos[idx] + lineOffset, iColor, checkDrawText ? 2 : 1);
            }
        }
    }
}

void CTBatteryInspectorResult::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3)
    {
        xvt::enhance::GammaCorrector gamma;
        gamma.SetGamma(mGamma);
        gamma.Apply(img, img);
        ConvertRGB(img, img);
    }

    //Draw the setting ROI
    auto size = img.size();

    pen.mSpace = 15;
    pen.mFontScale = 0.6;
    pen.mThickness = 2;
    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    pen.mColor = GetResultColor();
    cv::Rect subRoi = !mOuterRoi.empty() ? mOuterRoi : cv::Rect(cv::Point(0, 0), size);
    cv::Scalar color(150, 150, 0);
    cv::rectangle(img, GetROI(), (mErrMsg.empty() && finalDecision == "OK") ? COLOR_CV_WHITE : COLOR_CV_RED, 2);
    if (IsER() && !mPoints.empty() && !subRoi.empty())
    {
        DrawPoints(img, mPoints, pen.mColor);
    }
    //cv::rectangle(img, mPoleRegionRoi, cv::Scalar(106, 113, 220), 2);

    DrawValidCaseLineResult(img, pen);

    {
        // draw center neglection
        int xCenterROI1 = mCenter.x - mCenterWidth / 2;
        int xCenterROI2 = mCenter.x + mCenterWidth / 2;
        int yPoleROI1 = mPoleRegionRoi.y;
        int yPoleROI2 = mPoleRegionRoi.br().y;
        Drawing::DrawDashedLine(img, cv::Point(xCenterROI1, yPoleROI1), cv::Point(xCenterROI1, yPoleROI2), COLOR_CV_YELLOW, 1, "", 5);
        Drawing::DrawDashedLine(img, cv::Point(xCenterROI2, yPoleROI1), cv::Point(xCenterROI2, yPoleROI2), COLOR_CV_YELLOW, 1, "", 5);

        // draw pole roi
        int xPoleROI1 = mPoleRegionRoi.x;
        int xPoleROI2 = mPoleRegionRoi.br().x;
        Drawing::DrawDashedLine(img, cv::Point(xPoleROI1, yPoleROI1), cv::Point(xPoleROI1, yPoleROI2), COLOR_CV_YELLOW, 1, "", 5);
        Drawing::DrawDashedLine(img, cv::Point(xPoleROI2, yPoleROI1), cv::Point(xPoleROI2, yPoleROI2), COLOR_CV_YELLOW, 1, "", 5);
    }
    
    if (!mOuterRoi.empty())
    {
        auto tl = mOuterRoi.tl();
        auto br = mOuterRoi.br();
        pen.mColor = COLOR_CV_YELLOW;
        //std::vector<cv::Point> points = { tl, br, cv::Point(tl.x, br.y), cv::Point(br.x, tl.y) };
        for (auto p : mCornerPoint) DrawPlusSign(img, p, pen, 7);
    }

    DrawPoleResult(img);

    if (mAnodePoles.mValidCathode2CaseRange.mIsEnable
        || mAnodePoles.mValidAnode2CaseRange.mIsEnable)
    {
        cv::line(img, cv::Point(mPoleRegionRoi.x, mAnodePoles.mCaseRef.y), cv::Point(mPoleRegionRoi.x + mPoleRegionRoi.width, mAnodePoles.mCaseRef.y), COLOR_CV_YELLOW, 2);
    }

    // center draw
    if(1)
    {
        cv::Point lineStartLeft = mCenterDetected.tl();
        cv::Point lineStartRight(mCenterDetected.br().x, mCenterDetected.y);

        cv::Point lineEndLeft(mCenterDetected.x, mCenterDetected.br().y);
        cv::Point lineEndRight = mCenterDetected.br();
        
        cv::Point ofsetDrawCenter = cv::Point(0, mCenterDetected.height / 2);
        if (mCenterDetected.y >= mCenter.y)
        {
            lineStartLeft += ofsetDrawCenter;
            lineStartRight += ofsetDrawCenter;
        }
        else
        {
            lineEndLeft -= ofsetDrawCenter;
            lineEndRight -= ofsetDrawCenter;
        }
        cv::line(img, lineStartLeft, lineEndLeft, COLOR_CV_WHITE, 1);
        cv::line(img, lineStartRight, lineEndRight, COLOR_CV_WHITE, 1);
    }

    cv::circle(img, mCenter, 4, COLOR_CV_YELLOW, -1);

    DrawResultStr(img, "", pen, cv::Point(size.width / 2, 0) - cv::Point(25));
}

auto CTBatteryInspectorResult::DrawResultStr(cv::Mat& img,
                                                  std::string const& name,
                                                  CVPen const& pen,
                                                  cv::Point const& offset,
                                                  bool isDrawStrResult) const -> cv::Point
{
    cv::Point txtPos;
    if (img.empty())
        return cv::Point(offset.x, txtPos.y);
    if (img.channels() != 3 && !Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    CVPen tmpPen = pen;
    int rowHeight = 30;
    int noPrecision = mDecimalNo;
    auto size = img.size();

    std::string decisionStr = finalDecision.empty() ? "NG" : finalDecision;
    cv::putText(img, decisionStr, cv::Point(img.cols * 0.85, 200), cv::FONT_HERSHEY_SIMPLEX, 2,
        decisionStr == "OK" ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255), 3);

    VecString msgErr = mErrMsg;
    msgErr.push_back(Description);

    tmpPen.mFontScale = 0.8;
    tmpPen.mColor = COLOR_CV_RED;
    Table tableErr(100, 25, std::make_shared<xvt::CVPen>(tmpPen), nullptr);
    tableErr.AddColumn(msgErr);
    tableErr.SetColumnAlign(0, TextAlign::MIDDLE_CENTER);
    auto draw_size = tableErr.GetSize();
    cv::Point draw_pos;
    draw_pos.y = 50;
    draw_pos.x = (img.cols - draw_size.width) / 2;
    tableErr.Draw(img, draw_pos);

    const auto validC2A = mAnodePoles.mValidCathode2AnodeRange;
    const auto validA2B = mAnodePoles.mValidAnode2CaseRange;
    const auto validC2B = mAnodePoles.mValidCathode2CaseRange;
    const bool checkVarA2B = !(mAnodePoles.mVariationAnode2Case < 1e-9) && validA2B.mIsEnable;

    // draw base to anode
    if (checkVarA2B)
    {
        std::ostringstream tmpStrVariation;
        tmpStrVariation << std::fixed << std::setprecision(3) << mAnodePoles.mVariationAnode2Case;
        std::string isNGA2CaseStr = "Base->An Variation [" + tmpStrVariation.str() + "] : " + Anode2CaseVariationDecision;

        cv::Size textSizeVariation = pen.GetTextSize(isNGA2CaseStr);
        int tmpPosX = img.cols / 2 - textSizeVariation.width / 2;
        int tmpPosY = 180;

        cv::putText(img, isNGA2CaseStr, cv::Point(tmpPosX, tmpPosY), pen.mFontFace, pen.mFontScale, (Anode2CaseVariationDecision == "NG") ? COLOR_CV_RED : cv::Scalar(255, 255, 255), pen.mThickness);
        std::ostringstream tmpStrMax;
        tmpStrMax << std::fixed << std::setprecision(3) << abs(maxAnode2Case - minAnode2Case);
        cv::putText(img, "Max - Min : " + tmpStrMax.str() + " [mm]", cv::Point(tmpPosX, tmpPosY + 20), pen.mFontFace, pen.mFontScale, cv::Scalar(255, 255, 255), pen.mThickness);
    }

    std::stringstream leftText;
    
    //procesText << "Pixel Size: " << xvt::ToString(mPixelSize, 3) << "[mm]";
    /*if (validC2A.mIsEnable || validA2B.mIsEnable) procesText << " | No. Anode: " << vtAnodes.size();
    if (validC2A.mIsEnable || validC2B.mIsEnable) procesText << " | No. Cathode: " << mCathodePos.size();*/

    auto drawPos = cv::Point(std::min(size.width - 1, 50), std::max(0, size.height - 80));
    tmpPen.mColor = COLOR_CV_GRAY(255);
    tmpPen.mFontScale = 0.6;
    DrawText(img, cTime, drawPos, tmpPen);

    if (!IsER() && (mDisplayMode & DisplayMode::TEXT) == DisplayMode::TEXT)
    {
        int cathod2BaseIdx = 0;
        int anode2BaseIdx = 3;
        int cathode2AnodeIdx = 6;
        VecString rangeRow(9);
        {
            rangeRow[cathod2BaseIdx] = "Cathode-Base (" + xvt::ToString(validC2B.GetLower(), noPrecision) + "~" + xvt::ToString(validC2B.GetUpper(), noPrecision) + ")";
            rangeRow[anode2BaseIdx] = "Anode-Base (" + xvt::ToString(validA2B.GetLower(), noPrecision) + "~" + xvt::ToString(validA2B.GetUpper(), noPrecision) + ")";
            rangeRow[cathode2AnodeIdx] = "Anode-Cathode (" + xvt::ToString(validC2A.GetLower(), noPrecision) + "~" + xvt::ToString(validC2A.GetUpper(), noPrecision) + ")";
        }

        static const VecString Row1 = { "Min.", "Max.", "Ave.", "Min.", "Max.", "Ave.", "Min.", "Max.", "Ave." };

        auto row1Color = std::make_shared<CVPen>(cv::Scalar(255, 255, 255), tmpPen.mThickness, tmpPen.mFontScale);
        auto row2Color = std::make_shared<CVPen>(COLOR_CV_YELLOW, tmpPen.mThickness, tmpPen.mFontScale);
        auto ngColor = std::make_shared<CVPen>(COLOR_CV_RED, tmpPen.mThickness, tmpPen.mFontScale);
        Table drawing(100, 30, row1Color, row1Color);
        Table::RowData Row2;
        {
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(minCathode2Case, noPrecision)
                                                    ,(validC2B.mIsEnable && minCathode2Case < validC2B.GetLower()) ? ngColor : row2Color
                                                    ,drawing.mBorderStyle));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(maxCathode2Case, noPrecision)
                                                    ,(validC2B.mIsEnable && maxCathode2Case > validC2B.GetUpper())? ngColor : row2Color
                                                    , drawing.mBorderStyle));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(avgCathode2Case, noPrecision), row2Color, drawing.mBorderStyle));

            Row2.push_back(std::make_shared<Cell>(xvt::ToString(minAnode2Case, noPrecision)
                                                    , (validA2B.mIsEnable && minAnode2Case < validA2B.GetLower()) ? ngColor : row2Color
                                                    , drawing.mBorderStyle));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(maxAnode2Case, noPrecision)
                                                    , (validA2B.mIsEnable && maxAnode2Case > validA2B.GetUpper()) ? ngColor : row2Color
                                                    , drawing.mBorderStyle));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(avgAnode2Case, noPrecision), row2Color, drawing.mBorderStyle));

            Row2.push_back(std::make_shared<Cell>(xvt::ToString(minCathode2Anode, noPrecision)
                                                    , (validC2A.mIsEnable && minCathode2Anode < validC2A.GetLower()) ? ngColor : row2Color
                                                    , drawing.mBorderStyle));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(maxCathode2Anode, noPrecision)
                                                    , (validC2A.mIsEnable && maxCathode2Anode > validC2A.GetUpper()) ? ngColor : row2Color
                                                    , drawing.mBorderStyle));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(avgCathode2Anode, noPrecision), row2Color, drawing.mBorderStyle));
        }

        drawing.AddRow(rangeRow);
        drawing.AddRow(Row1);
        drawing.AddRow(Row2);
        //drawing.SetRowColor(2, row2Color, drawing.mBorderStyle);
        drawing.MergeCells(0, cathod2BaseIdx, 0, 2);
        drawing.MergeCells(0, anode2BaseIdx, 0, 5);
        drawing.MergeCells(0, cathode2AnodeIdx, 0, 8);
        int noCol = drawing.GetColumns();
        for (auto i = 0; i < noCol; i++)
        {
            if (i < anode2BaseIdx && !validC2B.mIsEnable
                || (i >= anode2BaseIdx && i < cathode2AnodeIdx && !validA2B.mIsEnable)
                || i >= cathode2AnodeIdx && !validC2A.mIsEnable)
            {
                drawing.SetColumnWidth(i, 0);
            }
        }

        auto s = drawing.GetSize();

        drawing.Draw(img, cv::Point((img.cols - s.width) / 2, 560));

        DrawPoleTextResult(img, pen);
    }

    return cv::Point(offset.x, txtPos.y);
}

auto CTBatteryInspectorResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
    out.emplace_back("Result", finalDecision);
    std::string errMsg = "";
    for (auto& str : mErrMsg) errMsg += str + ".";
    out.emplace_back("ErrMsg", errMsg);
    out.emplace_back("Time", cTime);
    out.emplace_back("ProTime", xvt::ToString(mProTime, mDecimalNo) + "ms");
    out.emplace_back("Version", Battery_VersionInfo.GetVersionInfo(false));
    out.emplace_back("No.AL", xvt::ToString(mAnodePoles.mLeftNoPole));
    out.emplace_back("No.CL", xvt::ToString(nNoCathodeLeft));
    out.emplace_back("No.AR", xvt::ToString((int)mAnodePoles.mPoles.size() - mAnodePoles.mLeftNoPole));
    out.emplace_back("No.CR", xvt::ToString((int)mCathodePos.size() - nNoCathodeLeft));
    out.emplace_back("A2C.Min", xvt::ToString(minCathode2Anode, mDecimalNo));
    out.emplace_back("A2C.Max", xvt::ToString(maxCathode2Anode, mDecimalNo));
    out.emplace_back("A2C.Avg", xvt::ToString(avgCathode2Anode, mDecimalNo));
    out.emplace_back("A2B.Min", xvt::ToString(minAnode2Case, mDecimalNo));
    out.emplace_back("A2B.Max", xvt::ToString(maxAnode2Case, mDecimalNo));
    out.emplace_back("A2B.Avg", xvt::ToString(avgAnode2Case, mDecimalNo));
    out.emplace_back("C2B.Min", xvt::ToString(minCathode2Case, mDecimalNo));
    out.emplace_back("C2B.Max", xvt::ToString(maxCathode2Case, mDecimalNo));
    out.emplace_back("C2B.Avg", xvt::ToString(avgCathode2Case, mDecimalNo));
}

}// battery namespace
}// xvt namespace