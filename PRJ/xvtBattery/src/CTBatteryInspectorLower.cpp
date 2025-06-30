#include "xvtBattery/CTBatteryInspectorLower.h"
#include "xvtBattery/CylinderBatteryLower.h"
#include "xvtCV/Polyfit.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/Contour.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Utils.h"
#include "xvtCV/Peak.h"
#include "xvtCV/Thinning.h"
#include "xvtCV/GammaCorrector.h"
#include "xvtCV/xvtTable.h"
#include "xvtCV/Utils.h"
#include "xvtCV/AStar.h"
#if 0
namespace xvt {
namespace battery
{
CTBatteryInspectorLower::CTBatteryInspectorLower()
{
    mIspBase.mIsInvert = false;
}

//#define PoleTracking
//#define DEBUGCathodeLine
auto CTBatteryInspectorLower::Inspect(cv::Mat const &img) const -> std::unique_ptr<xvt::IInspectionResult>
{
    auto ispResult = std::make_unique<CTBatteryInspectorLowerResult>();
    ispResult->mEnable = mEnable;

    if (!ispResult->mEnable)
    {
        ispResult->operator()(EResult::ER, "Disabled!");
        return ispResult;
    }

    ScopeTimer t("Cylinder Battery CT Lower");

    cv::Mat src;
    if (!Convert8Bits(img, src, false))
    {
        std::array<cv::Point, 4> connerPoint;
        cv::Rect batteryBoundary;

        auto imgSize = img.size();
        auto settingROI = mIspBase.GetInspectROI(imgSize);

        double subROIY = 0;//YA2
        auto& beadingRes = ispResult->mBeadingResult;
        if(mIspBeading.mEnable)
        {
            auto baseResult = mIspBase.Inspect(src);
            connerPoint = baseResult.GetCornerPoint();
            if (!baseResult.IsER() && !baseResult.IsUC() && !connerPoint.empty())
            {
                batteryBoundary = cv::boundingRect(baseResult.mPoints);
            }
            ispResult->CombineResult(&baseResult);

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
            ispResult->mOuterRoi = CreateROI(batteryBoundary, img.size());
        }
        else
        {
            auto findBatteryResult = FindBattery(src(settingROI), mIspBase.mThreshold, connerPoint);
            if (findBatteryResult.IsOK())
            {
                for (auto& c : connerPoint) c += settingROI.tl();

                batteryBoundary = cv::boundingRect(connerPoint);
            }
            ispResult->CombineResult(&findBatteryResult);

            if (!findBatteryResult.mPoints.empty())
                ispResult->mPoints = xvt::Transform(findBatteryResult.mPoints, settingROI.tl());

            cv::Point tl = connerPoint[0];
            cv::Point tr = connerPoint[1];
            int yPos = round((tl.y + tr.y) / 2.0f);
            ispResult->mOuterRoi = CreateROI(batteryBoundary.x, yPos, batteryBoundary.width, batteryBoundary.height + batteryBoundary.y - yPos, img.size());
            subROIY = ispResult->mOuterRoi.y;
        }

        if (ispResult->IsOK())
        {
            auto oX = mIspJR.mJROffsetX;
            auto oY = mIspJR.mJROffsetY;
            auto JRROI = xvt::CreateROI(ispResult->mOuterRoi.x + oX,
                                        subROIY + oY,
                                        ispResult->mOuterRoi.width - 2 * oX,
                                        ispResult->mOuterRoi.height - oY,
                                        settingROI
            );

            auto findTopROIResult = FindTopReferenceLine(src, JRROI, ispResult->mCenterDetected);
            ispResult->mCenterWidth = mIspJR.mCenterNeglectionWidth != 0 ? mIspJR.mCenterNeglectionWidth : ispResult->mCenterDetected.width;
            if(findTopROIResult.IsOK() && !ispResult->mCenterDetected.empty())
            {
                ispResult->mCenter.x = ispResult->mCenterDetected.x + ispResult->mCenterDetected.width / 2;
            }
            else
            {
                ispResult->mCenter.x = mIspBase.mRoi.x + mIspBase.mRoi.width/2;
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

            cv::Rect poleROI = xvt::CreateROI(JRROI.x - difCenter,
                                              subROIY + oY,
                                              JRROI.width,
                                              mIspJR.mHeight - oY,
                                              settingROI
            );

            if (!poleROI.empty() && ispResult->mCenterWidth != 0)
            {
                auto cathodeIns = InspectCathode(src, poleROI, *ispResult);
                auto anodeIns   = InspectAnode(src, poleROI, *ispResult);
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
    auto tmp = t.Stop();
    ispResult->mProTime = tmp.count();
    ispResult->SetROI(mIspBase.mRoi);
    ispResult->mPixelSize = mPixelSize;
    ispResult->mDirection = mIspBase.mDirection;
    return ispResult;
}

auto CTBatteryInspectorLower::Inspect(const cv::Mat& inImg, CTBatteryInspectorLowerResult& IspResult) const -> ERR_CODE
{
    cv::Mat src;
    cv::Mat rotatedImg;

    IspResult.SetROI(mIspBase.mRoi);
    auto res = Convert8Bits(inImg, src, true);
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
    if (false == RefineROI(settingROI, cv::Size(inImg.cols, inImg.rows)))
    {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errROIRefinement;
    }

    if (IspResult.resImg.empty()
        || IspResult.resImg.size() != src.size()
        || IspResult.resImg.type() != CV_8UC3)
    {
        xvt::enhance::GammaCorrector gamma;
        gamma.SetGamma(mGamma);
        gamma.Apply(src, IspResult.resImg);
        ConvertRGB(IspResult.resImg, IspResult.resImg);
    }

    if (mIspBase.mDirection == 1) {
        cv::rotate(inImg, rotatedImg, cv::ROTATE_180);
    }
    else {
        rotatedImg = inImg.clone();
    }

    if (mPixelSize == 0) {
        IspResult.Description = "Pixel size cannot set to 0";
        return ERR_CODE::errPixelSize;
    }

    auto tmp = Inspect(rotatedImg);
    auto tmp2 = xvt::DynamicCastUniquePtr<CTBatteryInspectorLowerResult, xvt::IInspectionResult>(std::move(tmp));
    IspResult = *tmp2;
    xvt::enhance::GammaCorrector gamma;
    gamma.SetGamma(mGamma);
    gamma.Apply(src, IspResult.resImg);
    
    if(IspResult.IsER())
    {
        IspResult.Description = IspResult.GetMsg();
        return ERR_CODE::errInspection;
    }

    cv::Rect batteryROI = IspResult.mOuterRoi;

    if (batteryROI.width > 0 && batteryROI.height > 0)
    {
        //Check the width of battery is in the range of masterJigCellSize
        auto batteryWidth_mm = batteryROI.width * mPixelSize;

        if (!mIspBase.mValidBatteryWidthRange.IsInRange(round_f(batteryWidth_mm, 2)))
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
    if (mIspBase.mDirection == 1) {
        roiBtmYPosition = src.rows - settingROI.y;
    }

    if (batteryROI.y >= roiBtmYPosition) {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }

    // find all poles position
    std::vector<int> lstPolePosXAll, anodePos, cathodePos;
    for(const auto& p : IspResult.mAnodePoles.mPoles)
    {
        lstPolePosXAll.push_back(p.mCathode.x);
        cathodePos.push_back(p.mCathode.y);
        anodePos.push_back(p.mAnode.y);
    }

    if(lstPolePosXAll.empty()
	    || cathodePos.size() != lstPolePosXAll.size()
	    || anodePos.size() != lstPolePosXAll.size()
	    //|| (IspResult.mCathodePos.size() - lstPolePosXAll.size() != 2)
        )
	{
	    IspResult.Description = "Cannot find any poles, please check again outer ROI Setting or input image";
	    return ERR_CODE::errPoleXDetection5;
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
    //Check disappear poles
    std::vector<std::pair<int, int>> lstPolePositionErr;
    ERR_CODE errBlackRegion = ERR_CODE::OK;

    bool isAnode2CaseOK = true;
    bool isAnode2CathodeOK = true;
    bool isCathode2CaseOK = true;
    bool isCathodeLeft2CaseOK = true;
    bool isCathodeRight2CaseOK = true;

    //===============================================making Decisions==================================================//
    
    int poleIdx = 0;
    int missPoleIdx = 0;
    auto poleRegionROI = IspResult.mPoleRegionRoi;
    std::vector<int> leftXList;
    std::vector<int> leftAnodeList;
    std::vector<int> leftCathodeList;
    std::vector<int> rightXList;
    std::vector<int> rightAnodeList;
    std::vector<int> rightCathodeList;
    int borderLeft = poleRegionROI.x + mIspPole.mSkipPolesDistance;
    int borderRight = poleRegionROI.x + poleRegionROI.width - mIspPole.mSkipPolesDistance;
    int centerLeft = IspResult.mCenter.x - IspResult.mCenterWidth / 2;
    int centerRight = IspResult.mCenter.x + IspResult.mCenterWidth / 2;
    for (auto pole = 0; pole < lstPolePosXAll.size(); pole++) {
        if (lstPolePosXAll[pole] >= borderLeft
            && lstPolePosXAll[pole] <= centerLeft) {
            nPoleLeft++;
            leftXList.push_back(lstPolePosXAll[pole]);
            leftAnodeList.push_back(anodePos[pole]);
            leftCathodeList.push_back(cathodePos[pole]);
            //IspResult.vtCathodesLR.push_back(std::make_pair(IspResult.mCathodePos[pole], IspResult.mCathodePos[pole + 1]));
            /*if (mIspPole.mIsCheckPoleNo && nPoleLeft >= mIspJR.mOneSidePoleNumber) {
                break;
            }*/
        }
    }

    int poleRightIdx = lstPolePosXAll.size() - 1;
    for (auto pole = poleRightIdx; pole >= 0; pole--) {
        if (lstPolePosXAll[pole] >= centerRight
            && lstPolePosXAll[pole] <= borderRight) {
            nPoleRight++;
            rightXList.push_back(lstPolePosXAll[pole]);
            rightAnodeList.push_back(anodePos[pole]);
            rightCathodeList.push_back(cathodePos[pole]);
            //IspResult.vtCathodesLR.push_back(std::make_pair(IspResult.mCathodePos[pole + 1], IspResult.mCathodePos[pole + 2]));
            /*if (mIspPole.mIsCheckPoleNo && (nPoleRight) >= mIspJR.mOneSidePoleNumber)
                break;*/

        }
    }

    if(rightAnodeList.empty()
        || leftAnodeList.empty()
        || rightCathodeList.empty()
        || leftCathodeList.empty()
        || rightXList.empty()
        || leftXList.empty())
    {
        IspResult.Description = "Cannot find any poles, please check again outer ROI Setting or input image";
        return ERR_CODE::errPoleXDetection5;
    }

    anodePos.clear();
    cathodePos.clear();
    lstPolePosXAll.clear();

    anodePos = leftAnodeList;
    anodePos.insert(anodePos.end(), rightAnodeList.rbegin(), rightAnodeList.rend());

    cathodePos = leftCathodeList;
    cathodePos.insert(cathodePos.end(), rightCathodeList.rbegin(), rightCathodeList.rend());

    lstPolePosXAll = leftXList;
    lstPolePosXAll.insert(lstPolePosXAll.end(), rightXList.rbegin(), rightXList.rend());

    int lstPoleSize = lstPolePosXAll.size();

    // For preventing anode position out of range
    std::for_each(anodePos.begin(), anodePos.end(), [](int& ap) {ap = ap < 0 ? 0 : ap; });

    std::vector<double> lstAnodeDouble;
    std::for_each(anodePos.begin(), anodePos.end(), [&lstAnodeDouble](int& ap) {lstAnodeDouble.push_back(ap); });
    cv::blur(lstAnodeDouble, lstAnodeDouble, cv::Size(5, 1));
    //lstAnodeDouble = weightedAvgSmoothen(lstAnodeDouble);
    anodePos.clear();
    std::for_each(lstAnodeDouble.begin(), lstAnodeDouble.end(), [&anodePos](double& ap) {anodePos.push_back(ap); });

    assert(cathodePos.size() == lstPoleSize && anodePos.size() == lstPoleSize);

    int startIdx = 0;
    int maxIdx = lstPoleSize - 1;
    int cWidth = IspResult.mCenterWidth;
    IspResult.vtCathodesLR = std::vector<std::pair<cv::Point, cv::Point>>(lstPoleSize);
    auto cSize = IspResult.mCathodePos.size();

    int an_Idx = 0, ca_Idx = 0, ca_Idx_l = -1, ca_Idx_r = -1;
    bool isUpdate = false;
	while (an_Idx < lstPoleSize)
    {
        auto const& a_x = lstPolePosXAll[an_Idx];
        auto &       lr = IspResult.vtCathodesLR[an_Idx];
        auto const&   c = IspResult.mCathodePos[ca_Idx];
        auto dis = abs(a_x - c.x);

        if (a_x > c.x)
        {
			ca_Idx_l = dis < cWidth ? ca_Idx :- 1;

			if (ca_Idx < cSize - 1)
                ca_Idx++;
            else {
                isUpdate = true;
            }
        }
        else
        {
            ca_Idx_r = dis < cWidth ? ca_Idx:-1;

            //if (ca_Idx < cSize - 1)
            //    ca_Idx++;
            isUpdate = true;
        }

        if (isUpdate) {
            if (ca_Idx_l < 0) ca_Idx_l = ca_Idx_r;
            if (ca_Idx_r < 0) ca_Idx_r = ca_Idx_l;

            lr.first = IspResult.mCathodePos[ca_Idx_l];
            lr.second = IspResult.mCathodePos[ca_Idx_r];
            ca_Idx_l = ca_Idx_r;

            an_Idx++;
            isUpdate = false;
        }
	}

    /*cv::Mat debug;
    xvt::ConvertRGB(rotatedImg, debug);
    DrawPoints(debug, IspResult.mCathodePos, COLOR_CV_RED);
    for (int i = 0; i < lstPoleSize; i++)
    {
        DrawPlusSign(debug, IspResult.mAnodePoles.mPoles[i].mAnode, CVPen(COLOR_CV_BLUE));
        if(IspResult.vtCathodesLR[i].first != cv::Point(0,0))
            DrawLine(debug, IspResult.vtCathodesLR[i].first, IspResult.mAnodePoles.mPoles[i].mAnode, CVPen(COLOR_CV_ORANGE));
        if(IspResult.vtCathodesLR[i].second != cv::Point(0,0))
            DrawLine(debug, IspResult.vtCathodesLR[i].second, IspResult.mAnodePoles.mPoles[i].mAnode, CVPen(COLOR_CV_YELLOW));
    }*/
    //=============================================Making Decisions===================================
    //For calculate the anode to case distance
    int caseLine  = IspResult.mCenter.y + mIspJR.mBaseLineOffset;
    const auto C2AValid = mIspPole.mValidCathode2AnodeRange;
    const auto A2BValid = mIspPole.mValidAnode2CaseRange;
    const auto C2BValid = mIspPole.mValidCathode2CaseRange;
    int decimal_places = 3;
    for (int poleIdx = 0; poleIdx < lstPoleSize; poleIdx++)
    {
        //Update the original position on the Image
        auto mAnode = cv::Point(lstPolePosXAll[poleIdx], anodePos[poleIdx]);
        auto mCathode = cv::Point(lstPolePosXAll[poleIdx], cathodePos[poleIdx]);

        //Get the distance from the pole to the left battery roi
        listPolePos.push_back(abs(mAnode.x - batteryROI.br().x) * mPixelSize);

        //Get the anode to case result
        double mAnode2CasePixel = abs(mAnode.y - caseLine) + mIspPole.mAnode2CaseOffset;
        double mAnode2Case_mm = mAnode2CasePixel * mPixelSize;
        auto mAnode2CaseResult = A2BValid.IsInRange(round_f(mAnode2Case_mm, decimal_places));

        double mAnode2Cathode_mm1 = mIspPole.ToMilimet(abs(mAnode.y - mCathode.y) + mIspPole.mCathode2AnodeOffset);
        //Get the anode to cathode result
        double mAnode2CathodePixel = abs(mAnode.y - mCathode.y) + mIspPole.mCathode2AnodeOffset;
        double mAnode2Cathode_mm = mAnode2CathodePixel * mPixelSize;
        auto mAnode2CathodeResult = A2BValid.IsInRange(round_f(mAnode2Cathode_mm, decimal_places));

        double mAnode2CathodeLPixel = abs(mAnode.y - IspResult.vtCathodesLR[poleIdx].first.y) + mIspPole.mCathode2AnodeOffset;
        double mAnode2CathodeL_mm = mAnode2CathodeLPixel * mPixelSize;
        auto mAnode2CathodeLResult = C2AValid.IsInRange(round_f(mAnode2CathodeL_mm, decimal_places));

        double mAnode2CathodeRPixel = abs(mAnode.y - IspResult.vtCathodesLR[poleIdx].second.y) + mIspPole.mCathode2AnodeOffset;
        double mAnode2CathodeR_mm = mAnode2CathodeRPixel * mPixelSize;
        auto mAnode2CathodeRResult = C2AValid.IsInRange(round_f(mAnode2CathodeR_mm, decimal_places));

        //Get the anode to cathode result
		double mCathode2CasePixel = abs(mCathode.y - caseLine) + mIspPole.mCathode2CaseOffset;
		double mCathode2Case_mm = mCathode2CasePixel * mPixelSize;
		auto mCathode2CaseResult = C2BValid.IsInRange(round_f(mCathode2Case_mm, decimal_places));

        isAnode2CaseOK &= mAnode2CaseResult;
        isCathode2CaseOK &= mCathode2CaseResult;
        isAnode2CathodeOK &= (mAnode2CathodeLResult && mAnode2CathodeRResult);

        listAno2CaseDecision.push_back(mAnode2CaseResult);
        listAno2CathodDecision.push_back(mAnode2CathodeResult);
        listAno2CathodLRDecision.push_back(std::make_pair(mAnode2CathodeLResult, mAnode2CathodeRResult));
        listCathode2CaseDecision.push_back(mCathode2CaseResult);

        listAnode2CaseDistance.push_back(mAnode2Case_mm);
        listAnode2CathodeDistance.push_back(mAnode2Cathode_mm);
        listAnode2CathodeLRDistance.push_back(std::make_pair(mAnode2CathodeL_mm, mAnode2CathodeR_mm));
        listCathode2CaseDistance.push_back(mCathode2Case_mm);

        listAnodeOri.push_back(mAnode);
        listCathodeOri.push_back(mCathode);
    }

    
    cv::Point lineStartLeft = IspResult.mCenterDetected.tl();
    cv::Point lineStartRight(IspResult.mCenterDetected.br().x, IspResult.mCenterDetected.y);

    cv::Point lineEndLeft(IspResult.mCenterDetected.x, IspResult.mCenterDetected.br().y);
    cv::Point lineEndRight = IspResult.mCenterDetected.br();

    std::vector<std::string> errMsg;
    if (mIspBase.mDirection == 1) {
        auto imgSize = src.size();
        //cv::rotate(res, res, cv::ROTATE_180);
		//cv::Point ImgCenter = cv::Point((src.cols - 1) / 2, (src.rows - 1) / 2);
        listAnodeOri = rotate180vtPoint(listAnodeOri, imgSize);
        listCathodeOri = rotate180vtPoint(listCathodeOri, imgSize);
        IspResult.mCathodePos = rotate180vtPoint(IspResult.mCathodePos, imgSize);
        //std::reverse(IspResult.mCathodePos.begin(), IspResult.mCathodePos.end());

        std::reverse(listPolePos.begin(), listPolePos.end());
        std::reverse(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
        std::reverse(listAno2CaseDecision.begin(), listAno2CaseDecision.end());
        std::reverse(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
        std::reverse(listAno2CathodDecision.begin(), listAno2CathodDecision.end());
        std::reverse(listAno2CathodLRDecision.begin(), listAno2CathodLRDecision.end());
        std::reverse(listAnode2CathodeLRDistance.begin(), listAnode2CathodeLRDistance.end());
        std::reverse(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
        std::reverse(listCathode2CaseDecision.begin(), listCathode2CaseDecision.end());
        
        // Update the rectangle coordinates after rotation
        batteryROI.x = IspResult.resImg.cols - batteryROI.x - batteryROI.width;
        batteryROI.y = IspResult.resImg.rows - batteryROI.y - batteryROI.height;
        caseLine = IspResult.resImg.rows - caseLine;

        IspResult.mCenterDetected.x = IspResult.resImg.cols - IspResult.mCenterDetected.x - IspResult.mCenterDetected.width;
        IspResult.mCenterDetected.y = IspResult.resImg.rows - IspResult.mCenterDetected.y - IspResult.mCenterDetected.height;

        IspResult.mPoleRegionRoi.x = IspResult.resImg.cols - IspResult.mPoleRegionRoi.x - IspResult.mPoleRegionRoi.width;
        IspResult.mPoleRegionRoi.y = IspResult.resImg.rows - IspResult.mPoleRegionRoi.y - IspResult.mPoleRegionRoi.height;

        IspResult.mCenter.x = IspResult.resImg.cols - IspResult.mCenter.x;
        IspResult.mCenter.y = IspResult.resImg.rows - IspResult.mCenter.y;
        IspResult.nLeftPole = nPoleRight;
    }
    else
    {
        IspResult.nLeftPole = nPoleLeft;
    }

    IspResult.mOuterRoi = batteryROI;
    IspResult.mAnodePoles.ClearAll();
    for (int i = 0; i < listAnodeOri.size(); i++)
    {
        IspResult.mAnodePoles.mPoles.emplace_back(listAnodeOri[i], listCathodeOri[i]);
        IspResult.mAnodePoles.mCathode2Anode.emplace_back(listAno2CathodDecision[i], listAnode2CathodeDistance[i]);
        auto lResult = BResult(listAno2CathodLRDecision[i].first, listAnode2CathodeLRDistance[i].first);
        auto rResult = BResult(listAno2CathodLRDecision[i].second, listAnode2CathodeLRDistance[i].second);
        IspResult.mAnodePoles.mCathodeLR2Anode.emplace_back(lResult, rResult);
        IspResult.mAnodePoles.mCathode2Case.emplace_back(listCathode2CaseDecision[i], listCathode2CaseDistance[i]);
        IspResult.mAnodePoles.mAnode2Case.emplace_back(listAno2CaseDecision[i], listAnode2CaseDistance[i]);
    }
    IspResult.mAnodePoles.mXPoles = listPolePos;
    IspResult.mAnodePoles.mCaseRef = cv::Point(batteryROI.x, caseLine);
    IspResult.vtAno2CathodLRDecision = listAno2CathodLRDecision;
    IspResult.sCathode2AnodeLR = listAnode2CathodeLRDistance;
    //===============================================PLoting results in the 180 rotated image=================================================//

    std::string str = " ,maxB=" + std::to_string(maxB) + " ,minB=" + std::to_string(minB) + " ,averageB=" + std::to_string(averageB);
    IspResult.sData += str.c_str();
    if (!listAnode2CaseDistance.empty()) {
        IspResult.maxAnode2Case = *max_element(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
        IspResult.minAnode2Case = *min_element(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end());
        IspResult.avgAnode2Case = std::accumulate(listAnode2CaseDistance.begin(), listAnode2CaseDistance.end(), 0.0) / listAnode2CaseDistance.size();
    }

    if (listAnode2CathodeDistance.size() != 0) {
        IspResult.minCathode2Anode = *min_element(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
        IspResult.maxCathode2Anode = *max_element(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end());
        IspResult.avgCathode2Anode = std::accumulate(listAnode2CathodeDistance.begin(), listAnode2CathodeDistance.end(), 0.0) / listAnode2CathodeDistance.size();
    }

    if (listCathode2CaseDistance.size() != 0) {
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
    CVPen pen;
    pen.mSpace = 15;
    pen.mFontScale = 0.6;
    pen.mThickness = 2;
    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;

    if ((lstPolePositionErr.size() > 0
        || (mIspPole.mIsCheckPoleNo && (nPoleLeft < mIspJR.mOneSidePoleNumber || nPoleRight < mIspJR.mOneSidePoleNumber)))
        && (A2BValid.mIsEnable || C2AValid.mIsEnable || C2BValid.mIsEnable))
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

    if ((A2BValid.mIsEnable || C2AValid.mIsEnable)
        && mInspectingItems.ANODE_TO_CASE_VARIATION)
    {
        bool isNGA2Case = abs(IspResult.maxAnode2Case - IspResult.minAnode2Case) > mIspPole.mVariationAnode2Case;
        std::ostringstream tmpStrVariation;
        tmpStrVariation << std::fixed << std::setprecision(3) << mIspPole.mVariationAnode2Case;
        std::string isNGA2CaseStr = "Base->An Variation [" + tmpStrVariation.str() + "] : " + (isNGA2Case ? "NG" : "OK");

        cv::Size textSizeVariation = pen.GetTextSize(isNGA2CaseStr);
        int tmpPosX = IspResult.resImg.cols/2 - textSizeVariation.width/2;
        int tmpPosY = 130;

        cv::putText(IspResult.resImg, isNGA2CaseStr, cv::Point(tmpPosX, tmpPosY), pen.mFontFace, pen.mFontScale, isNGA2Case ? COLOR_CV_RED : cv::Scalar(255, 255, 0), pen.mThickness);
        std::ostringstream tmpStrMax;
        tmpStrMax << std::fixed << std::setprecision(3) << abs(IspResult.maxAnode2Case - IspResult.minAnode2Case);
        cv::putText(IspResult.resImg, "Max - Min : " + tmpStrMax.str() + " [mm]", cv::Point(tmpPosX, tmpPosY + 20), pen.mFontFace, pen.mFontScale, cv::Scalar(255, 255, 0), pen.mThickness);
        
        IspResult.Anode2CaseVariationDecision = isNGA2Case ? "OK" : "NG";
        if (isNGA2Case) {
            FinalDecision = "NG";
        }
    }

    if( mInspectingItems.CHECK_CENTER_PIN && !IspResult.isPinExist)
    {
        FinalDecision = "NG";
        errMsg.push_back("The Pin not exist");
    }

    IspResult.finalDecision = FinalDecision;
    IspResult.mAnodePoles.mValidAnode2CaseRange = mIspPole.mValidAnode2CaseRange;
    IspResult.mAnodePoles.mValidCathode2AnodeRange = mIspPole.mValidCathode2AnodeRange;
    IspResult.mAnodePoles.mValidCathode2CaseRange = mIspPole.mValidCathode2CaseRange;

    IspResult.cTime = GetStrCurrentTime();
    IspResult.mErrMsg = errMsg;

    if ((mDisplayMode & DisplayMode::POLE) == DisplayMode::POLE) {
        IspResult.DrawResult(IspResult.resImg, cv::Point(), pen);
    }

    return ERR_CODE::OK;
}

ERR_CODE CTBatteryInspectorLower::Inspect(const std::vector<cv::Mat>& inImg, CTBatteryInspectorLowerResult& BIresult) const
{
    if(inImg.empty())
    {
        BIresult.Description = "Image empty or not support!";
        return ERR_CODE::errImageEmpty;
    }

    cv::Mat avgImage = AverageSliceImage(inImg);
    CTBatteryInspectorLowerResult avgResult;
    ERR_CODE resultCode = Inspect(avgImage, avgResult);
    if(resultCode == ERR_CODE::OK && mSliceNo != 1)
    {
        std::vector<std::pair<int, double>> ctfList;
        for(int i = 0; i < inImg.size(); i++)
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
        for(int idx = 0; idx < ctfList.size() && idx < mAvgSliceNo; idx++)
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

auto CTBatteryInspectorLower::FindAnodeByLineTracing(const cv::Mat & inputImg, VecPoint& anodes, cv::Point startPoint, int limitVal, int stepHorizontal, int stepVertical, float breakThreshold,
    int borderDistance, bool restrictMove, int moveAllow, int borderCheck) const -> InspectionResult
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

    int const& startPointX = startPoint.x;
    int const& startPointY = startPoint.y;

    if (borderCheck == -1) {
        if (2 * borderDistance < startPointX) {
            //borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (borderDistance < startPointX && borderDistance > 2) {
            borderOffset = 2;
        }
        else {
            borderOffset = 0;
        }

    }
    else if (borderCheck == 1) {
        if (startPointX < inputImg.cols - 2 * borderDistance) {
            // borderCheck = 0;
            borderOffset = borderDistance;
        }
        else if (startPointX < inputImg.cols - borderDistance && borderDistance > 2) {
            borderOffset = 2;
        }
        else {
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

    for (it = currentPoint.y - stepSkip; it > stopCheck; it -= stepFrog) {
        int currentPointXCord = currentPoint.x;

        moveRect.x = currentPointXCord - stepHorizontal;
        moveRect.y = it - stepVertical;
        moveRect.width = 2 * stepHorizontal + 1;
        if (borderCheck == -1) {
            //if (borderDistance < startPointX)
            //Point is out of the left border
            if (leftBorderX >= moveRect.x)
            {
                moveRect.width = moveRect.br().x - leftBorderX;
                moveRect.x = leftBorderX;
            }
        }
        else if (borderCheck == 1) {
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
            for (int w = 0; w < reduceCol; w++) {
                float distance = abs((float)(reduceCol / 2.0) - w);
                float scaleWeight = 1;
                reduceRes.at<float>(0, w) -= scaleWeight / distance;
            }

            //DIRECTIONAL WEIGHT
            float smallWeight = 0.5;
            if (currentPointXCord < imgColsHalf) {
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
                    anodes.push_back(currentPoint);
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

void CTBatteryInspectorLowerResult::DrawPoleTextResult(cv::Mat& resImg, CVPen const& pen, cv::Point const& offset) const
{
    int expand_height = 0;
    int row_height = 25;
    int col_width = 70;
    int heightLimit = 100;
    int offset_x = 20;
    float thickness = pen.mThickness;
    cv::Point draw_pos = cv::Point(offset_x, offset_x);
    float precision = 3;
    float font_scale = pen.mFontScale;
    auto pen0 = std::make_shared<xvt::CVPen>(cv::Scalar(255, 180, 66), thickness, font_scale);
    auto penOK = std::make_shared<xvt::CVPen>(cv::Scalar(0, 255, 0), thickness, font_scale);
    auto penNG = std::make_shared<xvt::CVPen>(COLOR_CV_RED, thickness, font_scale);
    Table drawingL(col_width, row_height, pen0, nullptr);
    Table drawingR(col_width, row_height, pen0, nullptr);

    VecString headerString = { "No", "Pos", "A-CL", "A-CR", "A-B", "C-B" };
    drawingL.AddRow(headerString);
    drawingR.AddRow(headerString);

    for (int i = 0; i < mAnodePoles.mPoles.size(); i++)
    {
        bool isAnode2CaseLError = false;
        bool isAnode2CaseRError = false;
        bool isAnode2BaseError = false;
        VecString Cat;
        {
            Cat.push_back("A" + xvt::ToString(i + 1));
            Cat.push_back(xvt::ToString(mAnodePoles.mXPoles[i], precision));
            if (mAnodePoles.mValidCathode2AnodeRange.mIsEnable)
            {
                isAnode2CaseLError = mAnodePoles.mCathodeLR2Anode[i].first.result == EResult::NG;
                isAnode2CaseRError = mAnodePoles.mCathodeLR2Anode[i].second.result == EResult::NG;
                Cat.push_back(xvt::ToString(mAnodePoles.mCathodeLR2Anode[i].first.value, precision));
                Cat.push_back(xvt::ToString(mAnodePoles.mCathodeLR2Anode[i].second.value, precision));
            }
            else
            {
                Cat.push_back("-");
                Cat.push_back("-");
            }

            if (mAnodePoles.mValidAnode2CaseRange.mIsEnable)
            {
                isAnode2BaseError = mAnodePoles.mAnode2Case[i].result == EResult::NG;
                Cat.push_back(xvt::ToString(mAnodePoles.mAnode2Case[i].value, precision));
            }
            else
            {
                Cat.push_back("-");
            }

            {
                Cat.push_back("-");
            }
        }

        bool isPoleError = isAnode2BaseError || isAnode2CaseLError || isAnode2CaseRError;
        if (i < mAnodePoles.mLeftNoPole)
        {
            drawingL.AddRow(Cat);
            auto current_row = drawingL.GetRows() - 1;
            drawingL.SetCellColor(current_row, 0, isPoleError ? penNG : penOK, nullptr);
            drawingL.SetCellColor(current_row, 2, isAnode2CaseLError ? penNG : penOK, nullptr);
            drawingL.SetCellColor(current_row, 3, isAnode2CaseRError ? penNG : penOK, nullptr);
            drawingL.SetCellColor(current_row, 4, isAnode2BaseError ? penNG : penOK, nullptr);
        }
        else
        {
            drawingR.AddRow(Cat);
            auto current_row = drawingR.GetRows() - 1;
            drawingR.SetCellColor(current_row, 0, isPoleError ? penNG : penOK, nullptr);
            drawingR.SetCellColor(current_row, 2, isAnode2CaseLError ? penNG : penOK, nullptr);
            drawingR.SetCellColor(current_row, 3, isAnode2CaseRError ? penNG : penOK, nullptr);
            drawingR.SetCellColor(current_row, 4, isAnode2BaseError ? penNG : penOK, nullptr);
        }
    }

    auto l_size = drawingL.GetSize();
    auto r_size = drawingR.GetSize();

    if(expand_height <= 0) expand_height = std::max(l_size.height, r_size.height) + draw_pos.y * 2;

	cv::Mat tableImg = cv::Mat(resImg.rows + expand_height, resImg.cols, resImg.type(),cv::Scalar::all(0));
    draw_pos.y += resImg.rows;
    draw_pos.x = (resImg.cols - l_size.width*2) / 3;

    resImg.copyTo(tableImg(cv::Rect(0, 0, resImg.cols, resImg.rows)));
    drawingL.SetColumnAlign(0, TextAlign::LEFT);
    drawingL.Draw(tableImg, draw_pos);

    drawingR.SetColumnAlign(0, TextAlign::LEFT);
    drawingR.Draw(tableImg, cv::Point(resImg.cols - draw_pos.x - r_size.width, draw_pos.y));
    resImg = std::move(tableImg);
}

void CTBatteryInspectorLowerResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    //Draw the setting ROI
    cv::rectangle(img, GetROI(), cv::Scalar(0, 255, 255), 2);
    auto size = img.size();

    pen.mSpace = 15;
    pen.mFontScale = 0.6;
    pen.mThickness = 2;
    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    pen.mColor = GetResultColor();
    cv::Rect subRoi = !mOuterRoi.empty() ? mOuterRoi : cv::Rect(cv::Point(0, 0), size);
    cv::Scalar color(150, 150, 0);
    if (IsER() && !mPoints.empty() && !subRoi.empty())
    {
        DrawPoints(img, mPoints, pen.mColor);
    }
    cv::rectangle(img, mPoleRegionRoi, cv::Scalar(106, 113, 220), 2);

    if (!IsER() && (mDisplayMode & DisplayMode::TEXT) == DisplayMode::TEXT)
    {
        int sign = mDirection == 0 ? -1 : 1;
        auto batteryROI = mOuterRoi;
        cv::Scalar minColor = cv::Scalar(220, 220, 0);
        cv::Scalar avgColor = cv::Scalar(255, 150, 0);
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
            int yCathode2Anode1 = yCathode2Anode2        + sign * avgCathode2Anode;
            int yCathode2Anode3 = yCathode2Anode2        - sign * avgCathode2Anode;
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
            const int yLineInfo1 = 75;
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

    cv::line(img, cv::Point(mCenter.x - mCenterWidth/2, mPoleRegionRoi.y), cv::Point(mCenter.x - mCenterWidth/2, mPoleRegionRoi.br().y), cv::Scalar(0, 255, 0), 1);
    cv::line(img, cv::Point(mCenter.x + mCenterWidth/2, mPoleRegionRoi.y), cv::Point(mCenter.x + mCenterWidth/2, mPoleRegionRoi.br().y), cv::Scalar(0, 255, 0), 1);

    if (!mOuterRoi.empty())
    {
        auto tl = mOuterRoi.tl();
        auto br = mOuterRoi.br();
        pen.mColor = COLOR_CV_ORANGE;
        std::vector<cv::Point> points = { tl, br, cv::Point(tl.x, br.y), cv::Point(br.x, tl.y)};
        for (auto p : points) DrawPlusSign(img, p, pen, 7);
    }

    if (mAnodePoles.mPoles.size() == mAnodePoles.mAnode2Case.size()
        && mAnodePoles.mPoles.size() == mAnodePoles.mCathode2Anode.size()
        && mAnodePoles.mPoles.size() == mAnodePoles.mCathode2Case.size())
    {
        if (1) {

            int sign = mDirection == 0 ? -1 : 1;
            if (mAnodePoles.mValidAnode2CaseRange.mIsEnable
                || mAnodePoles.mValidCathode2AnodeRange.mIsEnable)
            {

                for (int i = 0; i < mAnodePoles.mPoles.size(); i++)
                {
                    //draw Anode
                    cv::Point textAnoPosition;
                    textAnoPosition.y = mAnodePoles.mPoles[i].mAnode.y + 15 * sign;
                    textAnoPosition.x = mAnodePoles.mPoles[i].mAnode.x - 5;

                    bool Ano2Case = vtAno2CaseDecision[i];
                    bool Ano2Cat = (vtAno2CathodLRDecision[i].first && vtAno2CathodLRDecision[i].second);

                    cv::Scalar textColor(0, 0, 0);
                    if (Ano2Case == true && Ano2Cat == true) {
                        textColor[0] = 255;
                        textColor[1] = 255;
                        textColor[2] = 150;
                    }
                    else {
                        textColor[0] = 150;
                        textColor[1] = 150;
                        textColor[2] = 255;
                    }

                    if (i % 5 == 0) {
                        textColor[0] -= (textColor[0] > 250) ? 0 : 150;
                        textColor[1] -= (textColor[1] > 250) ? 0 : 150;
                        textColor[2] -= (textColor[2] > 250) ? 0 : 150;

                        std::string textAno = "A" + std::to_string(i + 1);
                        if (Ano2Case == true && Ano2Cat == true) {
                            textColor = cv::Scalar(255, 255, 0);
                        }
                        else {
                            textColor = cv::Scalar(0, 0, 255);
                        }
                        cv::putText(img, textAno, textAnoPosition, cv::FONT_HERSHEY_SIMPLEX, 0.4, textColor, 1);
                    }
                    xvt::DrawPlusSign(img, mAnodePoles.mPoles[i].mAnode, textColor, 2);
                }
            }
            //draw Cathode

            for (int i = 0; i < mCathodePos.size(); i++) {
                cv::Point textCatPosition;
                textCatPosition.y = mCathodePos[i].y - 15 * sign;
                textCatPosition.x = mCathodePos[i].x - 5;

                cv::Scalar textColor(0, 0, 0);
                if (true) {
                    textColor[0] = 200;
                    textColor[1] = 255;
                    textColor[2] = 200;
                }
                else {
                    textColor[0] = 200;
                    textColor[1] = 200;
                    textColor[2] = 255;
                }

                if (i % 5 == 0) {
                    textColor[0] -= (textColor[0] > 250) ? 0 : 200;
                    textColor[1] -= (textColor[1] > 250) ? 0 : 200;
                    textColor[2] -= (textColor[2] > 250) ? 0 : 200;

                    std::string textCat = "C" + std::to_string(i + 1);
                    if (true) {
                        textColor = cv::Scalar(0, 255, 0);
                    }
                    else {
                        textColor = cv::Scalar(0, 0, 255);
                    }
                    cv::putText(img, textCat, textCatPosition, cv::FONT_HERSHEY_SIMPLEX, 0.4, textColor, 1);
                }
                xvt::DrawPlusSign(img, mCathodePos[i], textColor, 2);
            }
        }
        else {
            for (int i = 0; i < vtCathodesLR.size(); i++) {
                if (mAnodePoles.mAnode2Case[i].result == EResult::OK
                    && mAnodePoles.mCathode2Anode[i].result == EResult::OK)
                {
                    xvt::DrawPlusSign(img, mAnodePoles.mPoles[i].mAnode, cv::Scalar(0, 125, 255), 2);
                }
                else
                {
                    cv::circle(img, mAnodePoles.mPoles[i].mAnode, 1, cv::Scalar(0, 0, 255), 2);
                }

                if (mAnodePoles.mCathode2Case[i].result == EResult::OK)
                {
                    xvt::DrawPlusSign(img, mAnodePoles.mPoles[i].mCathode, cv::Scalar(255, 100, 0), 2);
                }
                else
                {
                    cv::circle(img, mAnodePoles.mPoles[i].mCathode, 1, cv::Scalar(0, 0, 255), 2);
                }
            }

        }
    }

    if (mAnodePoles.mValidCathode2CaseRange.mIsEnable
        || mAnodePoles.mValidAnode2CaseRange.mIsEnable)
    {
        cv::line(img, cv::Point(mPoleRegionRoi.x, mAnodePoles.mCaseRef.y), cv::Point(mPoleRegionRoi.x + mPoleRegionRoi.width, mAnodePoles.mCaseRef.y), cv::Scalar(0, 255, 0), 3);
    }

    // center draw
    {
        cv::Point lineStartLeft = mCenterDetected.tl();
        cv::Point lineStartRight(mCenterDetected.br().x, mCenterDetected.y);

        cv::Point lineEndLeft(mCenterDetected.x, mCenterDetected.br().y);
        cv::Point lineEndRight = mCenterDetected.br();

        cv::line(img, lineStartLeft, lineEndLeft, cv::Scalar(0, 255, 255), 1);
        cv::line(img, lineStartRight, lineEndRight, cv::Scalar(0, 255, 255), 1);
    }

    cv::circle(img, mCenter, 2, cv::Scalar(0, 0, 255), 2);

    DrawResultStr(img, "", pen, cv::Point(size.width / 2, 0) - cv::Point(25));
}

auto CTBatteryInspectorLowerResult::DrawResultStr(cv::Mat &img,
                                            std::string const &name,
                                            CVPen const &pen,
                                            cv::Point const &offset,
                                            bool isDrawStrResult) const -> cv::Point
{
    cv::Point txtPos;
    if (img.empty())
        return cv::Point(offset.x, txtPos.y);
    if (img.channels() != 3 && !Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    CVPen tmpPen = pen;
    int rowHeight = 30;
    constexpr int noPrecision = 3;
    auto size = img.size();

    std::string decisionStr = finalDecision.empty() ? "NG" : finalDecision;
    cv::putText(img, "Result: " + decisionStr, cv::Point(img.cols / 2.5, 50), cv::FONT_HERSHEY_SIMPLEX, 1,
        decisionStr == "OK" ? cv::Scalar(245, 164, 66) : cv::Scalar(0, 0, 255), tmpPen.mThickness);

    VecString msgErr = mErrMsg;
    if (IsER()) msgErr.push_back(GetMsg());

    tmpPen.mFontScale = 0.8;
    tmpPen.mColor = COLOR_CV_RED;
    Table tableErr(100, 25, std::make_shared<xvt::CVPen>(tmpPen), nullptr);
    tableErr.AddColumn(msgErr);
    tableErr.SetColumnAlign(0, TextAlign::MIDDLE_CENTER);
    auto draw_size = tableErr.GetSize();
    cv::Point draw_pos;
    draw_pos.y = 85;
    draw_pos.x = (img.cols - draw_size.width) / 2;
    tableErr.Draw(img, draw_pos);

    const auto validC2A = mAnodePoles.mValidCathode2AnodeRange;
    const auto validA2B = mAnodePoles.mValidAnode2CaseRange;
    const auto validC2B = mAnodePoles.mValidCathode2CaseRange;

    std::stringstream procesText;
    procesText << "Pixel Size: " << xvt::ToString(mPixelSize, 3) << "[mm]";
    if (validC2A.mIsEnable || validA2B.mIsEnable) procesText << " | No. Anode: " << vtAnodes.size();
    if (validC2A.mIsEnable || validC2B.mIsEnable) procesText << " | No. Cathode: " << mCathodePos.size();

    auto drawPos = cv::Point(std::min(size.width - 1, 50), std::max(0, size.height - 50));
    auto color1 = img.at<cv::Vec3b>(drawPos);
    int grayColor = abs(color1(0) - 125) < 50 ? 250 : (255 - color1(0));
    tmpPen.mColor = COLOR_CV_GRAY(250);
    tmpPen.mFontScale = 0.6;
    DrawText(img, procesText.str(), drawPos, tmpPen);

    if (!IsER() && (mDisplayMode & DisplayMode::TEXT) == DisplayMode::TEXT)
    {
        int cathod2BaseIdx = 0;
        int anode2BaseIdx = 3;
        int cathode2AnodeIdx = 6;
        VecString rangeRow(9);
        {
            rangeRow[cathod2BaseIdx] = "Cathode-Base [" + xvt::ToString(validC2B.GetLower(), noPrecision) + "~" + xvt::ToString(validC2B.GetUpper(), noPrecision) + "]";
            rangeRow[anode2BaseIdx] = "Anode-Base [" + xvt::ToString(validA2B.GetLower(), noPrecision) + "~" + xvt::ToString(validA2B.GetUpper(), noPrecision) + "]";
            rangeRow[cathode2AnodeIdx] = "Anode-Cathode [" + xvt::ToString(validC2A.GetLower(), noPrecision) + "~" + xvt::ToString(validC2A.GetUpper(), noPrecision) + "]";
        }

        static const VecString Row1 = { "Min", "Max", "Average", "Min", "Max", "Average", "Min", "Max", "Average" };

        auto row1Color = std::make_shared<CVPen>(cv::Scalar(0, 255, 0), tmpPen.mThickness, tmpPen.mFontScale);
        auto row2Color = std::make_shared<CVPen>(cv::Scalar(2, 165, 249), tmpPen.mThickness, tmpPen.mFontScale);
        Table drawing(110, 30, row1Color, row1Color);
        Table::RowData Row2;
        {
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(minCathode2Case, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(maxCathode2Case, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(avgCathode2Case, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(minAnode2Case, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(maxAnode2Case, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(avgAnode2Case, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(minCathode2Anode, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(maxCathode2Anode, noPrecision)));
            Row2.push_back(std::make_shared<Cell>(xvt::ToString(avgCathode2Anode, noPrecision)));
        }

        drawing.AddRow(rangeRow);
        drawing.AddRow(Row1);
        drawing.AddRow(Row2);
        drawing.SetRowColor(2, row2Color, drawing.mBorderStyle);
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

auto CTBatteryInspectorLowerResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
    const auto& listLenght = mAnodePoles.GetLenghts();
    for (int i = 0; i < 140; i++)
    {
        float leght = (i < mAnodePoles.Size()) ? listLenght[i] : -1.0f;
        out.emplace_back("NP" + std::to_string(i), xvt::ToString(leght, 3));
    }
}

auto CTBatteryInspectorLower::FindAnodeByProminence(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode) const->InspectionResult
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
            for (int i = 0; i < enhanMatrix.rows; i++) {
                for (int j = 0; j < enhanMatrix.cols; j++) {
                    bool checkPoint = false;
                    for (cv::Point& p : oldStations)
                    {
                        if (p.y == (i + startRectY) && abs(p.x - (j + startRectX)) < 3) {
                            checkPoint = true;
                        }
                    }
                    if (enhanMatrix.at<float>(i, j) == maxVal && !checkPoint) {
                        maxLocs.push_back(cv::Point(j, i));
                    }
                    else if (enhanMatrix.at<float>(i, j) > maxVal && !checkPoint) {
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

auto CTBatteryInspectorLower::FindAnodeByShape(const cv::Mat& inputImg, VecPoint& anode, cv::Point& startAnode, cv::Rect& subAnoRoi, int disconnectLimit, int widthAnode, int startRegion) const->InspectionResult
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
            while (disconnectState < disconnectLimit && (!outRange)) {
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
                            if (count > 1) {
                                //boundaryInsMedian = (!boundaryIns.empty())? (boundaryInsMedian * boundaryIns.size() + findIspVec[i])/(boundaryIns.size() + 1.0) : findIspVec[i];
                                boundaryIns.emplace_back(findIspVec[i]);
                                if (boundaryIns.size() > 20)
                                {
                                    boundaryIns.erase(boundaryIns.begin());
                                }
                                auto maxElement = std::max_element(boundaryIns.begin(), boundaryIns.end());
                                if (maxElement != boundaryIns.end()) {
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

void CTBatteryInspectorLower::FindEndpointCathodeThreshold(const cv::Mat& src, VecPeak& polePeak, cv::Rect& subROI, CTBatteryInspectorLowerResult& ispResult, VecPoint& midPos, int kSize) const
{
    VecPoint startPos = {};
    for (auto& p : polePeak)
    {
        startPos.push_back(cv::Point(p.index + subROI.x, subROI.y + subROI.height));
    }

    int catWindowSize = (mIspCathode.mCathodeLineWindowSize > 0 ? mIspCathode.mCathodeLineWindowSize / 2 : 0) * 2 + 1;
    for (int i = 0; i < startPos.size(); i++)
    {
        cv::Rect poleROI = cv::Rect(startPos[i].x - catWindowSize / 2, subROI.y, catWindowSize, startPos[i].y - subROI.y);
        RefineROI(poleROI, src.size());
        cv::Mat poleImg = src(poleROI);
        cv::reduce(poleImg, poleImg, 1, cv::REDUCE_AVG, CV_32FC1);
        float threshold = 0.0f;
        int regionIdx = CheckRegion(startPos[i].x - subROI.x, 6, subROI.width, ispResult.mCenterWidth);
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

        cv::Mat thresholdImg;
        cv::threshold(poleImg, thresholdImg, threshold, 255, cv::THRESH_BINARY_INV);

        VecFloat reduceVec = ReduceBlackBackgroundMat(thresholdImg, 1, kSize);
        thresholdImg.release();
        VecFloat diffVec(reduceVec.size(), 0);
        for (int i = 1; i < reduceVec.size(); i++)
        {
            diffVec[i] = reduceVec[i] - reduceVec[i - 1];
        }

        FindPeaks findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
        findPeaks.Process(diffVec);
        auto peakResult = findPeaks.GetPeakResult(0);

        std::sort(peakResult.begin(), peakResult.end(), [](Peak& a, Peak& b) {return abs(a.index) > abs(b.index); });

        if (peakResult.size() > 0)
        {
            midPos.push_back(cv::Point(startPos[i].x, poleROI.y + peakResult[0].index));
        }
        else
        {
            midPos.push_back(cv::Point(startPos[i].x, poleROI.y + poleROI.height / 2));
        }
    }

    for (int i = 0; i < startPos.size(); i++)
    {
        if (startPos[i].x < ispResult.mCenter.x - ispResult.mCenterWidth / 2 || startPos[i].x > ispResult.mCenter.x + ispResult.mCenterWidth / 2)
        {
            ispResult.mCathodePos.push_back(midPos[i]);
        }
    }
}

//#define DEBUG_CATHODE
#ifdef DEBUG_CATHODE
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

            float sumPeak = totalPeak > 0 ? cv::sum(peakImg.rowRange(j, size.height))[0] / totalPeak * ( 1- ratio) : 0.0;
            if (noNonZeroPeak == 0) sumPeak = 1 - ratio;

            float sum = sumValley + sumPeak;
            result.at<float>(j, i) = sum;
        }
    }
    return result;
}

VecPoint DebugAlgorithm(const cv::Mat& inputImg, VecPeak& polePeak, cv::Rect center)
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
        xvt::ConvertRGB(cdfPeakMat, debug2);
        DrawPoints(debug2, Results, cv::Vec3b(0, 0, 255));
    }
    return Results;
}
#endif

auto CTBatteryInspectorLower::InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorLowerResult& ispResult) const->InspectionResult
{
    auto cathodeInpection = InspectionResult();
    if (!src.empty() && !subROI.empty())
    {
        cv::Mat mainsubImg = src(subROI);
        int kSize = 5;
        cv::Mat mat = (cv::Mat_<float>(5, 5) << 0, 0, 3, 0, 0,
                                                0, 2, 3, 2, 0,
                                                1, 2, 3, 2, 1,
                                                0, 2, 3, 2, 0,
                                                0, 0, 3, 0, 0);
        mat = mat / 29.0;

        ispResult.mPoleRegionRoi = subROI;
        cv::Rect subROI2 = subROI;
        subROI2.y = subROI.y + subROI.height * 0.6;
        subROI2.height = subROI.height * 0.4;
        RefineROI(subROI2, src.size());

        cv::Mat mainsubFilter;
        cv::filter2D(mainsubImg, mainsubFilter, -1, mat);

        cv::Mat subImgFilter;
        cv::filter2D(src(subROI2), subImgFilter, -1, mat);
        VecFloat reduceVec = ReduceBlackBackgroundMat(subImgFilter, 0, kSize);
        xvt::Rangei distanceRange;

        VecPeak polePeak = FindCathodeXPositionAuto(reduceVec, distanceRange, ispResult.mCenterWidth, xvt::PeakType::Peak);
        if (polePeak.size() > 5)
        {
            std::vector<cv::Point> startPos = {};
            for (auto& p : polePeak)
            {
                startPos.push_back(cv::Point(p.index + subROI.x, subROI.y + subROI.height));
            }

            if(mIspCathode.mCathodeLineThresholdOuter < 1e-3 && mIspCathode.mCathodeLineThresholdMiddle < 1e-3 && mIspCathode.mCathodeLineThresholdInner < 1e-3)
            {
                cv::Rect centerROI = CreateROI(ispResult.mCenter.x - subROI.x - ispResult.mCenterWidth/2, 0, ispResult.mCenterWidth, subROI.height, subROI.size());
                //mainsubFilter(centerROI).setTo(0);
                VecPoint endCathodeAuto = FindEndpointCathodeAuto(mainsubFilter, polePeak, mCathodeEnableReference, mCathodeKernelSize, mCathodeThicknessPole, mCathodeDistanceVerifi, mCathodeNumVariance);
                //endCathodeAuto = DebugAlgorithm(mainsubFilter, polePeak, centerROI);
                
                for (int i = 0; i < endCathodeAuto.size(); i++)
                {
                    if (startPos[i].x < ispResult.mCenter.x - ispResult.mCenterWidth / 2 || startPos[i].x > ispResult.mCenter.x + ispResult.mCenterWidth / 2)
                    {
                        ispResult.mCathodePos.push_back(cv::Point(endCathodeAuto[i].x + subROI.x, endCathodeAuto[i].y + subROI.y));
                    }
                }
    #if 0 //def DEBUGCathodeLine
                cv::cvtColor(mainsubImg, mainsubImg, cv::COLOR_GRAY2BGR);
                for (auto& pCat : endCathodeAuto)
                {
                    cv::circle(mainsubImg, cv::Point(pCat.x, pCat.y), 1, cv::Scalar(255, 0, 0));//pCat.index + subROI2.x, subROI2.y ||  
                }
                for (auto& pCat : polePeak)
                {
                    cv::circle(mainsubImg, cv::Point(pCat.index, 100), 1, cv::Scalar(0, 255, 0));//pCat.index + subROI2.x, subROI2.y ||  
                }
    #endif //DEBUGCathodeLine
            }
            else
            {
                FindEndpointCathodeThreshold(src, polePeak, subROI, ispResult, ispResult.mCathodePos, kSize);
            }

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
            cathodeInpection(EResult::OK,"");
        }
        else
        {
            cathodeInpection(EResult::ER, "Peak Cathode size is not enough!");
        }
    }
    else
    {
        cathodeInpection(EResult::ER, "No Image in Cathode - Lower!");
    }
    return cathodeInpection;
}

VecPoint FindAnodeStartingPosition(const cv::Mat& src, VecPoint vPoints)
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
    for (auto p: peakResult)
    {
        vtResult.push_back(listCathode[p.index]);
    }

    for (auto p : vtResult)
    {
        dst2.at<uchar>(p) = (255);
    }

    return vtResult;
}

auto CTBatteryInspectorLower::InspectAnode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorLowerResult& ispResult) const->InspectionResult
{
    auto anodeInspection = InspectionResult();
    if (!src.empty() && !subROI.empty())
    {
        if(ispResult.mCathodePos.size() > 5)
    {
            VecPoint vtResult;
            {
                auto vPoints = xvt::Transform(ispResult.mCathodePos, cv::Point(-subROI.x, -subROI.y));
                auto dst = src(subROI).clone();
                auto size = subROI.size();
                cv::Rect tmpRect1(-1, 0, 3, 1);

                for (auto p : vPoints)
                {
                    cv::Rect rect = CreateROI(tmpRect1 + p, size);
                    dst(rect).setTo(255);
                }

                auto listCathode = xvt::FindPathByAStar(dst, 9);

                auto mask = cv::Mat(subROI.size(), src.type(), cv::Scalar(0));
                cv::Rect tmpRect2(0, -mIspPole.mPoleHeight, 1, mIspPole.mPoleHeight);

                for (auto p : listCathode)
                {
                    cv::Rect rect = CreateROI(tmpRect2 + p, size);
                    mask(rect).setTo(255);
                }
                cv::Mat dst2;
                cv::bitwise_and(src(subROI), mask, dst2);

                VecFloat reduceVec = ReduceBlackBackgroundMat(dst2, 0, 3);
                FindPeaks findPeaks = FindPeaks(PeakType::Peak, PeakFindingMethod::Prominence);
                findPeaks.Process(reduceVec);
                auto peakResult = findPeaks.GetPeakResult(mIspPole.mPolesProminenceThreshold, mIspPole.mPolesDistanceRange.GetLower());
                for (auto p : peakResult)
                {
                    vtResult.push_back(listCathode[p.index]);
                }
            }

            for (auto p : vtResult)
            {
                PoleInfo currentPole;
                currentPole.mCathode = p + subROI.tl();

                currentPole.mPoints.push_back(currentPole.mCathode);
                if (currentPole.mCathode.x < ispResult.mCenter.x - ispResult.mCenterWidth / 2 || currentPole.mCathode.x > ispResult.mCenter.x + ispResult.mCenterWidth / 2)
                {
                    ispResult.mAnodePoles.mPoles.push_back(currentPole);
                }
            }

            int count = 0;
            int XPaddingAno = 0;
            cv::Mat kernelImg = cv::Mat::zeros(cv::Size(mIspCathode.mCathodeLineWindowSize, 3), CV_32F);
            kernelImg.row(0).setTo(-1);
            kernelImg.row(2).setTo(1);
            //kernelImg /= 6.0;

            //cv::Mat sobelY;// = xvt::Sobel(src(subROI), 0, 1);
            //cv::filter2D(src(subROI), sobelY, CV_32F, kernelImg);
            cv::Point offset = subROI.tl();
            for (auto& pole : ispResult.mAnodePoles.mPoles)
            {
                auto AnodeResult = InspectionResult();
                if (pole.mCathode.x < ispResult.mCenter.x) ispResult.mAnodePoles.mLeftNoPole++;
                if (mChooseAnodeAlgorithm != 1 && mChooseAnodeAlgorithm != 2)
                {
                    int regionIdx = CheckRegion(pole.mPoints[0].x - subROI.x, 6, subROI.width, ispResult.mCenterWidth);
                    float aThreshold = 0.0;
                    switch (regionIdx)
                    {
                    case 1:
                        aThreshold = mIspAnode.mAnodeThresholdOuter;
                        break;
                    case 2:
                        aThreshold = mIspAnode.mAnodeThresholdMiddle;
                        break;
                    case 3:
                        aThreshold = mIspAnode.mAnodeThresholdInner;
                        break;
                    default:
                        aThreshold = mIspAnode.mAnodeThresholdInner;
                    }
                    //Check if pole is within outer region to restrict movement
                    bool ifRestrict = false;
                    if (regionIdx == 1) {
                        ifRestrict = true;
                    }
                    int limitYAnode = subROI.y;// std::max(ispResult.mCenter.y, subROI.y);
                    AnodeResult = FindAnodeByLineTracing(src, pole.mPoints, pole.mPoints[0], limitYAnode, 2, 2, aThreshold * 10, 0, ifRestrict, 0);
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
                    
                    /*int maxIdx = -1;
                    double maxSum = -1;
                    for(int j = 0; j < pole.mPoints.size(); j++)
                    {
                        cv::Point ref = pole.mPoints[j] - offset - cv::Point(1, 1);
                        cv::Rect roi = CreateROI(ref, cv::Size(3, 3), sobelY.size());
                        if (!roi.empty())
                        {
                            auto sum = cv::sum(sobelY(roi))[0];

                            if(maxSum < sum)
                            {
                                maxSum = sum;
                                maxIdx = j;
                            }
                        }
                    }*/

                    /*if(maxIdx > 0 && maxSum > 0)
                        pole.mPoints.erase(pole.mPoints.begin()+ maxIdx, pole.mPoints.end());*/

                    pole.mAnode = pole.mPoints.back();
                }
                else
                {
                    pole.mAnode = pole.mCathode;
                }
            }

#if USE_POLE_REFINEMENT
            {
                ispResult.mAnodePoles.mPoles.size();

            }
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
            if(!errMes.empty())
            {
                for(int i = 0; i < anodePos.size(); i++)
                {
                    int realAnodeYPos = anodePos[i] + offset.y;
                    auto& lstPoles = ispResult.mAnodePoles.mPoles;
                    if(realAnodeYPos != lstPoles[i].mAnode.y && !lstPoles.empty())
                    {
                        int poleIdx = abs(lstPoles[i].mCathode.y - realAnodeYPos);
                        if(poleIdx >= lstPoles[i].mPoints.size())
                        {
                            lstPoles[i].mPoints.emplace_back(lstPoles[i].mAnode.x, realAnodeYPos);
                        }
                        else if(poleIdx > 0)
                        {
                            lstPoles[i].mPoints.erase(lstPoles[i].mPoints.begin()+ poleIdx, lstPoles[i].mPoints.end());
                        }
                        lstPoles[i].mAnode = lstPoles[i].mPoints.back();
                    }
                }
            }
#endif // USE_POLE_LENGTH_REFINE

            ispResult.mAnodePoles.mPixelSize = mPixelSize;

            ispResult.mAnodePoles.mValidAnode2CaseRange = mIspPole.mValidAnode2CaseRange;
            ispResult.mAnodePoles.mValidCathode2AnodeRange = mIspPole.mValidCathode2AnodeRange;
            ispResult.mAnodePoles.mValidCathode2CaseRange = mIspPole.mValidCathode2CaseRange;

            for (auto& pole : ispResult.mAnodePoles.mPoles)
            {
                ispResult.mAnodePoles.mXPoles.emplace_back(abs(ispResult.mOuterRoi.x - pole.mCathode.x) * mPixelSize);

                bool Cathode2AnodeNG = mIspPole.mEnableValidCathode2Anode && !mIspPole.mValidCathode2AnodeRange.IsInRange(pole.length() * mPixelSize);
                pole.SetResult((Cathode2AnodeNG) ? EResult::NG : EResult::OK);
                ispResult.CombineResult(pole.GetResult());
            }

            for (int i = 0; i < ispResult.mAnodePoles.Size(); i++)
            {
                bool Cathode2AnodeNG = mIspPole.mEnableValidCathode2Anode && !mIspPole.mValidCathode2AnodeRange.IsInRange(ispResult.mAnodePoles.mPoles[i].length() * mPixelSize);
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
} // namespace battery
}
#endif