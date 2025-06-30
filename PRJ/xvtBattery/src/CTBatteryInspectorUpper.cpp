#include "xvtBattery/CTBatteryInspectorUpper.h"
#include "xvtBattery/CylinderBatteryUpper.h"
#include "xvtCV/Polyfit.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/Contour.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Utils.h"
#include "xvtCV/Peak.h"
#include "xvtCV/AStar.h"
#include "xvtCV/Thinning.h"
#include "xvtCV/xvtTable.h"
#if 0
namespace xvt {
namespace battery
{
CTBatteryInspectorUpper::CTBatteryInspectorUpper()
{
    mIspBase.mIsInvert = false;
}

auto CTBatteryInspectorUpper::Inspect(cv::Mat const &img) const -> CTBatteryInspectorUpperResult
{
    auto ispResult = CTBatteryInspectorUpperResult();
    ScopeTimer t("Cylinder Battery CT Upper");

    cv::Mat src;
    if (!xvt::Convert8Bits(img, src, false))
    {
        cv::Rect batteryBoundary;
        auto baseResult = mIspBase.Inspect(src);
        
        if (!baseResult.IsER() && !baseResult.IsUC() && !baseResult.GetCornerPoint().empty())
        {
            batteryBoundary = cv::boundingRect(baseResult.mPoints);
        }
        ispResult.CombineResult(&baseResult);
        ispResult.mPoints = xvt::Transform(baseResult.mPoints, -mIspBase.mRoi.tl());

        if (ispResult.IsOK())
        {
            ispResult.mOuterRoi = CreateROI(batteryBoundary, img.size());

            double subROIY = 0;//YA2

            // find beading
            cv::Mat drawCountourImg = cv::Mat::zeros(mIspBase.mRoi.size(), src.type());
            cv::drawContours(drawCountourImg, VecVecPoint({ispResult.mPoints}), -1, cv::Scalar(255), cv::FILLED);
            ispResult.mBeadingResult = mIspBeading.Inspect(drawCountourImg, ispResult.mPoints);
            if (!ispResult.mBeadingResult.IsER())
            {
                ispResult.mBeadingResult.mGrooveLeftContour += mIspBase.mRoi.tl();
                ispResult.mBeadingResult.mGrooveRightContour += mIspBase.mRoi.tl();
                ispResult.mBeadingResult.mBottomLeftContour += mIspBase.mRoi.tl();
                ispResult.mBeadingResult.mBottomRightContour += mIspBase.mRoi.tl();
            }
            if (ispResult.mBeadingResult.IsOK())
            {
                subROIY = (ispResult.mBeadingResult.mBottomLeftContour.y + ispResult.mBeadingResult.mBottomRightContour.y) / 2.0;
            }
            else
            {
                subROIY = ispResult.mOuterRoi.y;
            }

            auto oX = mIspJR.mJROffsetX;
            auto oY = mIspJR.mJROffsetY;
            auto JRROI = xvt::CreateROI(ispResult.mOuterRoi.x + oX,
                                        subROIY + oY,
                                        ispResult.mOuterRoi.width - 2*oX,
                                        ispResult.mOuterRoi.height - oY,
                                        baseResult.mRoiSetting
            );

            auto findTopROIResult = FindTopReferenceLine(src, JRROI, ispResult.mCenterDetected);
            ispResult.mCenterWidth = mIspJR.mCenterNeglectionWidth != 0 ? mIspJR.mCenterNeglectionWidth : ispResult.mCenterDetected.width;
            ispResult.AddMsg(findTopROIResult.GetMsg());

            if(findTopROIResult.IsOK() && !ispResult.mCenterDetected.empty())
            {
                ispResult.mCenter.x = ispResult.mCenterDetected.x + ispResult.mCenterDetected.width / 2;
            }
            else
            {
                ispResult.mCenter.x = mIspBase.mRoi.x + mIspBase.mRoi.width/2;
            }

            if (ispResult.mBeadingResult.IsOK())
            {
                ispResult.mCenter.y = (ispResult.mBeadingResult.mGrooveLeftContour.y + ispResult.mBeadingResult.mGrooveRightContour.y) / 2;
            }
            else
            {

                if (ispResult.mCenterDetected.y > 0 && mIspJR.mJROffsetY < 0)
                {
                    subROIY = ispResult.mCenterDetected.y;
                }
                else
                {
                    subROIY = ispResult.mOuterRoi.y;
                }
                ispResult.mCenter.y = subROIY;
            }

            int difCenter = ispResult.mOuterRoi.x + ispResult.mOuterRoi.width / 2 - ispResult.mCenter.x;

            cv::Rect poleROI = xvt::CreateROI(JRROI.x - difCenter,
                                              subROIY + oY,
                                              JRROI.width,
                                              mIspJR.mHeight - oY,
                                              baseResult.mRoiSetting
            );
            
            if (!poleROI.empty() && ispResult.mCenterWidth != 0)
            {
                auto cathodeIns = InspectCathode(src, poleROI, ispResult);
                ispResult.CombineResult(&cathodeIns);
            }
            else
            {
                ispResult.CombineResult(&findTopROIResult);
            }
        }
        else
        {
            //ispResult.CombineResult(&baseResult);
        }
    }
    else
    {
        ispResult(EResult::ER, "Image type is not supported!");
    }
    ispResult.mPoints = xvt::Transform(ispResult.mPoints, mIspBase.mRoi.tl());
    ispResult.mDisplayMode = mDisplayMode;
    auto tmp = t.Stop();
    ispResult.SetROI(mIspBase.mRoi);
    ispResult.mProTime = tmp.count();
    return ispResult;
}

ERR_CODE CTBatteryInspectorUpper::Inspect(const cv::Mat& inImg, CTBatteryInspectorUpperResult& IspResult) const
{
    // Convert input image to 8 bit image
    cv::Mat Img8bit;
    auto res = xvt::Convert8Bits(inImg, Img8bit, true);
    if (res)
    {
        IspResult.Description = "Image empty or not support!";
        return res == 1 ? ERR_CODE::errImageEmpty : ERR_CODE::errImageFormat;
    }

    ERR_CODE verifyERR = VerifyParameters(IspResult);
    if(verifyERR != ERR_CODE::OK)
    {
        return verifyERR;
    }

    cv::Rect settingROI = CreateROI(mIspBase.mRoi, Img8bit.size());
    // check predefined setting 
    if (settingROI.empty())
    {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errROIRefinement;
    }
    
    std::string FinalDecision;
    // check Battery Flip
    if (mBatteryDirection == 1) {
        cv::flip(Img8bit, Img8bit, 0);
        //mIspBase.mRoi.y = Img8bit.cols - mIspBase.mRoi.y - mIspBase.mRoi.height;
    }

    IspResult = Inspect(Img8bit);
    IspResult.mPixelSize = mPixelSize;

    xvt::enhance::GammaCorrector gamma(mGamma);
    gamma.Apply(Img8bit, IspResult.resImg);
    ConvertRGB(IspResult.resImg, IspResult.resImg);

    if(IspResult.IsER())
    {
        IspResult.Description = IspResult.GetMsg();
        return ERR_CODE::errInspection;
    }

    cv::Rect batteryROI = IspResult.mOuterRoi;

    if (batteryROI.width > 0 && batteryROI.height > 0)
    {
        //Check the width of battery is in the range of masterJigCellSize
        if (!mIspBase.mValidBatteryWidthRange.IsInRange(round_f(batteryROI.width * mPixelSize, 2)))
        {
            IspResult.Description = "Error: Width of battery is out of range 2 : " + std::to_string(batteryROI.width * mPixelSize);
            return ERR_CODE::errWidthBattery;
        }
    }
    else
    {
        IspResult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errBatterySize;
    }

    std::vector<double> beadingInsResult = { -1,-1,-1,-1,-1 };

    int yA2 = IspResult.mCenter.y;
    if (yA2 >= (batteryROI.y + batteryROI.height)) {
        IspResult.Description = "The pole region height is out of Image ROI, please check Pole Region Height parameter in setting";
        return ERR_CODE::errReferenceCase;
    }
    cv::Rect poleRegionROI = IspResult.mPoleRegionRoi;

    // find all poles position
    std::vector<int> lstPolePosXAll, cathodePos;
    std::vector<bool> listCathode2CaseDecision;
    std::vector<double> listCathode2CaseDistance;
    //pixel distance to the battery ROI
    std::vector<double> listPolePos;
    bool isCathode2CaseOK = true;
    // poleLeftIdx is number of pole in left side
    //Check disappear poles
    std::vector<std::pair<int, int>> lstPolePositionErr;
    int nPoleLeft = 0;
    int nPoleRight = 0;
    //For calculate the anode to case distance
    const int caseLine = yA2 + mIspJR.mBaseLineOffset;

    if (1)
    {   
        
        for(const auto& p : IspResult.mCathodePos)
        {
            lstPolePosXAll.push_back(p.x);
            cathodePos.push_back(p.y);
        }

        if(lstPolePosXAll.empty()
	        || cathodePos.size() != lstPolePosXAll.size())
	    {
	        IspResult.Description = "Cannot find any poles, please check again outer ROI Setting or input image";
	        return ERR_CODE::errPoleXDetection5;
	    }
        //===============================================making Decisions==================================================//

        std::vector<int> leftXList;
        std::vector<int> leftCathodeList;
        std::vector<int> rightXList;
        std::vector<int> rightCathodeList;
        int borderLeft = poleRegionROI.x + mIspPole.mSkipPolesDistance;
        int borderRight = poleRegionROI.x + poleRegionROI.width - mIspPole.mSkipPolesDistance;
        int centerLeft = poleRegionROI.x + (poleRegionROI.width - mIspJR.mCenterNeglectionWidth) / 2;
        int centerRight = poleRegionROI.x + (poleRegionROI.width + mIspJR.mCenterNeglectionWidth) / 2;
        for (auto pole = 0; pole < lstPolePosXAll.size(); pole++) {
            if (lstPolePosXAll[pole] >= borderLeft
                && lstPolePosXAll[pole] <= centerLeft) {
                nPoleLeft++;
                leftXList.push_back(lstPolePosXAll[pole]);
                leftCathodeList.push_back(cathodePos[pole]);
                if (mIspPole.mIsCheckPoleNo && nPoleLeft >= mIspJR.mOneSidePoleNumber) {
                    break;
                }
            }
        }

        int poleRightIdx = lstPolePosXAll.size() - 1;
        for (auto pole = poleRightIdx; pole >= 0; pole--) {
            if (lstPolePosXAll[pole] >= centerRight
                && lstPolePosXAll[pole] <= borderRight) {
                nPoleRight++;
                rightXList.push_back(lstPolePosXAll[pole]);
                rightCathodeList.push_back(cathodePos[pole]);
                if (mIspPole.mIsCheckPoleNo && (nPoleRight) >= mIspJR.mOneSidePoleNumber)
                    break;
            }
        }
        
        if(rightCathodeList.empty()
	        || leftCathodeList.empty()
	        || rightXList.empty()
	        || leftXList.empty())
	    {
	        IspResult.Description = "Cannot find any poles, please check again outer ROI Setting or input image";
	        return ERR_CODE::errPoleXDetection5;
	    }
        cathodePos.clear();
        lstPolePosXAll.clear();
        cathodePos = leftCathodeList;
        cathodePos.insert(cathodePos.end(), rightCathodeList.rbegin(), rightCathodeList.rend());
        lstPolePosXAll = leftXList;
        lstPolePosXAll.insert(lstPolePosXAll.end(), rightXList.rbegin(), rightXList.rend());

        const int detectedPoleNumb = lstPolePosXAll.size();

        assert(cathodePos.size() == detectedPoleNumb);

        int decimal_places = 3;
        
        auto cathodeRange = mIspPole.mValidCathode2CaseRange;
        for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
        {
            //Update the original position on the Image
            auto mCathode = cv::Point(lstPolePosXAll[poleIdx], cathodePos[poleIdx]);

            //Get the distance from the pole to the left battery roi
            listPolePos.push_back(abs(lstPolePosXAll[poleIdx] - batteryROI.x) * mPixelSize);

            //Get the anode to cathode result
			double mCathode2CasePixel = abs(mCathode.y - caseLine) + mIspPole.mCathode2CaseOffset;
			double mCathode2Case_mm = mCathode2CasePixel * mPixelSize;
			auto mCathode2CaseResult = cathodeRange.IsInRange(round_f(mCathode2Case_mm, decimal_places));
            isCathode2CaseOK &= mCathode2CaseResult;
            listCathode2CaseDecision.push_back(mCathode2CaseResult);

            listCathode2CaseDistance.push_back(mCathode2Case_mm);
            IspResult.vtCathodes.push_back(mCathode);
        }

        // Plot the skip pole region
        //cv::line(BIresult.resImg, cv::Point(poleRegionROI.x + borderLeft, poleRegionROI.y), cv::Point(poleRegionROI.x + borderLeft, poleRegionROI.y + mPoleRegionHeight), CV_RGB(0xc2, 0xa6, 0x09));
        //cv::line(BIresult.resImg, cv::Point(poleRegionROI.x + poleRegionROI.width - borderLeft, poleRegionROI.y), cv::Point(poleRegionROI.x + poleRegionROI.width - borderLeft, poleRegionROI.y + mPoleRegionHeight), CV_RGB(0xc2, 0xa6, 0x09));
        //Print reference case line
        cv::line(IspResult.resImg, cv::Point(poleRegionROI.x, caseLine), cv::Point(poleRegionROI.x + poleRegionROI.width, caseLine), cv::Scalar(0, 255, 0), 3);

        //Draw 2 yellow line define center of battery
        cv::Point lineStartLeft = IspResult.mCenterDetected.tl();
        cv::Point lineStartRight(IspResult.mCenterDetected.br().x, IspResult.mCenterDetected.y);

        cv::Point lineEndLeft(IspResult.mCenterDetected.x, IspResult.mCenterDetected.br().y);
        cv::Point lineEndRight = IspResult.mCenterDetected.br();

        cv::line(IspResult.resImg, lineStartLeft, lineEndLeft, cv::Scalar(0, 255, 255), 1);
        cv::line(IspResult.resImg, lineStartRight, lineEndRight, cv::Scalar(0, 255, 255), 1);
        IspResult.sXPos = listPolePos;
        IspResult.nLeftPole = nPoleLeft;
        IspResult.mCaseRef = cv::Point(poleRegionROI.x, caseLine);
    }
    //===============================================PLoting results=================================================//
    int fontWeight = 2;

    IspResult.yA2 = yA2;
    // Beading Inspection
    IspResult.corverHeight = beadingInsResult[0];
    IspResult.corverDiameter = beadingInsResult[1];
    IspResult.outerDiameter = beadingInsResult[2];
    IspResult.grooveDepth = beadingInsResult[3];
    IspResult.grooveHeight = beadingInsResult[4];
    IspResult.nLeftPole = nPoleLeft;

    if (listCathode2CaseDistance.size() != 0) {
        IspResult.minCathode2Case = *min_element(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
        IspResult.maxCathode2Case = *max_element(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end());
        IspResult.avgCathode2Case = std::accumulate(listCathode2CaseDistance.begin(), listCathode2CaseDistance.end(), 0.0) / listCathode2CaseDistance.size();
    }

    //Draw setting ROI
    if (mBatteryDirection == 1) {
        IspResult.yA2 = Img8bit.rows - IspResult.yA2;
        batteryROI.y = Img8bit.rows - batteryROI.y - batteryROI.height;
        for (int poleIdx = 0; poleIdx < lstPolePosXAll.size(); poleIdx++) {
            IspResult.vtCathodes[poleIdx].y = IspResult.resImg.rows - IspResult.vtCathodes[poleIdx].y;
        }
    }

    IspResult.sCathode2Anode = {};
    IspResult.sAnode2Case = {};
    IspResult.sCathode2Case = std::move(listCathode2CaseDistance);
    IspResult.vtCathode2CaseDecision = std::move(listCathode2CaseDecision);
    IspResult.sXPos = std::move(listPolePos);
    IspResult.vtAno2CathodDecision = {};
    IspResult.vtAno2CaseDecision = {};
    IspResult.outerROI = batteryROI;
    IspResult.Anodes2CaseDecision = "";
    IspResult.Anode2CathodeDecision = "";
    IspResult.Cathode2CaseDecision = isCathode2CaseOK ? "OK" : "NG";
    IspResult.mValidCathode2Case = mIspPole.mValidCathode2CaseRange;
    FinalDecision = (isCathode2CaseOK) ? "OK" : "NG";
    
    std::vector<std::string> errMsg;

    if ((mIspPole.mIsCheckPoleNo && (nPoleLeft < mIspJR.mOneSidePoleNumber || nPoleRight < mIspJR.mOneSidePoleNumber))
        && mInspectingItems.CATHODE_TO_CASE_GAP)
    {
        FinalDecision = "NG";
        errMsg.push_back("Number of detected poles is not sufficient");
    }

    if (IspResult.Cathode2CaseDecision == "NG"
        && mInspectingItems.CATHODE_TO_CASE_GAP) {
        FinalDecision = "NG";
        errMsg.push_back("Base->Cathode Distance NG");
    }

    if (mInspectingItems.COVER_HEIGHT)
    {
        IspResult.BeadingCoverDiameterDecision = mIspBeading.mValidCoverHeightRange.IsInRange(round_f(IspResult.corverHeight, 2)) ? "OK" : "NG";
        
        if (IspResult.BeadingCoverDiameterDecision == "NG")
        {
            FinalDecision = "NG";
            errMsg.push_back("Beading Cover Height NG");
        }
    }

    if (mInspectingItems.GROOVE_HEIGHT)
    {
        IspResult.BeadingGrooveHeightDecision = mIspBeading.mValidGrooveHeightRange.IsInRange(round_f(IspResult.grooveHeight, 2)) ? "OK" : "NG";
        
        if (IspResult.BeadingGrooveHeightDecision == "NG")
        {
            FinalDecision = "NG";
            errMsg.push_back("Beading Groove Height NG");
        }
    }

    if (mInspectingItems.GROOVE_DEPTH)
    {
        IspResult.BeadingGrooveDepthDecision = mIspBeading.mValidGrooveDepthRange.IsInRange(round_f(IspResult.grooveDepth, 2)) ? "OK" : "NG";
        
        if (IspResult.BeadingGrooveDepthDecision == "NG")
        {
            FinalDecision = "NG";
            errMsg.push_back("Beading Groove Depth NG");
        }
    }

    if (mInspectingItems.INNER_DIAMETER)
    {
        IspResult.BeadingInnerDiameterDecision = mIspBeading.mValidInnerDiameterRange.IsInRange(round_f(IspResult.corverDiameter, 2)) ? "OK" : "NG";
        
        if (IspResult.BeadingInnerDiameterDecision == "NG")
        {
            FinalDecision = "NG";
            errMsg.push_back("Beading Inner Diameter NG");
        }
    }
    
    IspResult.finalDecision = FinalDecision;
    IspResult.mPoles.mValidAnode2CaseRange = mIspPole.mValidAnode2CaseRange;
    IspResult.mPoles.mValidCathode2AnodeRange = mIspPole.mValidCathode2AnodeRange;
    IspResult.mPoles.mValidCathode2CaseRange = mIspPole.mValidCathode2CaseRange;
    IspResult.mErrMsg = errMsg;
    if ((mDisplayMode & DisplayMode::POLE) == DisplayMode::POLE)
    {
        IspResult.DrawResult(IspResult.resImg);
    }

    IspResult.cTime = GetStrCurrentTime();
    return ERR_CODE::OK;
}

ERR_CODE CTBatteryInspectorUpper::Inspect(const std::vector<cv::Mat>& inImg, CTBatteryInspectorUpperResult& BIresult) const
{
    if(inImg.empty())
    {
        BIresult.Description = "Image empty or not support!";
        return ERR_CODE::errImageEmpty;
    }

    cv::Mat avgImage = AverageSliceImage(inImg);
    CTBatteryInspectorUpperResult avgResult;
    ERR_CODE resultCode = Inspect(avgImage, avgResult);
    if(resultCode == ERR_CODE::OK && mSliceNo != 1)
    {
        std::vector<std::pair<int, double>> ctfList;
        
        for(int i = 0; i < inImg.size(); i++)
        {
            cv::Rect ctfROI = avgResult.mPoleRegionRoi;
            
            ctfROI.y += ctfROI.height * 3/4;
            ctfROI.height = ctfROI.height/4;
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
        //mIspResult.mCTFs = std::move(ctfList);
        //mIspResult.sliceIdex = maxIdx;
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
        if (bottomRight.x > imgWidth-1) bottomRight.x = imgWidth-1;
        if (bottomRight.y > imgHeight-1) bottomRight.y = imgHeight-1;
        cv::rectangle(BIresult.resImg, topLeft, bottomRight, cv::Scalar(0, 225, 255), 2);
    }
    return resultCode;
}

auto CTBatteryInspectorUpper::InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorUpperResult& ispResult) const -> InspectionResult
{
    auto cathodeInpection = InspectionResult();
    int kSize = 3;
    cv::Mat mat = (cv::Mat_<float>(7, 5) << 0, 0, 3, 0, 0,
        0, 2, 3, 2, 0,
        1, 2, 3, 2, 1,
        1, 2, 3, 2, 1,
        1, 2, 3, 2, 1,
        0, 2, 3, 2, 0,
        0, 0, 3, 0, 0);
    mat = mat / 47.0f;
    cv::Rect subROI2 = subROI;
    subROI2.y = subROI.y + subROI.height * 2 / 3;
    subROI2.height = subROI.height / 3;
    RefineROI(subROI2, src.size());
    ispResult.mPoleRegionRoi = subROI;

    if(!src.empty() && !subROI2.empty())
    {
        cv::Mat subImgFilter;
        cv::filter2D(src(subROI2), subImgFilter, -1, mat);
        VecFloat reduceVec = ReduceBlackBackgroundMat(subImgFilter, 0, kSize);
        xvt::Rangei distanceRange;
        VecPeak polePeak = FindCathodeXPositionAuto(reduceVec, distanceRange, ispResult.mCenterWidth, xvt::PeakType::Peak);
        if(polePeak.size() > 5)
        {
            std::vector<cv::Point> startPos = {};
            for (auto& p : polePeak)
            {
                startPos.push_back(cv::Point(p.index + subROI.x, subROI.y + subROI.height));
            }

            std::vector<cv::Point> midPos = {};
            int catWindowSize = (mIspCathode.mCathodeLineWindowSize > 0 ? mIspCathode.mCathodeLineWindowSize / 2 : 0) * 2 + 1;
            bool findCathodeAuto = (mIspCathode.mCathodeLineThresholdInner < 1e-3) && (mIspCathode.mCathodeLineThresholdMiddle < 1e-3) && (mIspCathode.mCathodeLineThresholdOuter < 1e-3);
            if(findCathodeAuto)
            {
                cv::Mat mainsubFilter;
                cv::filter2D(src(subROI), mainsubFilter, -1 , mat);

                VecPoint endCathodeAuto = FindEndpointCathodeAuto(mainsubFilter, polePeak, mCathodeEnableReference, mCathodeKernelSize, mCathodeThicknessPole, mCathodeDistanceVerifi, mCathodeNumVariance);
                for (int i = 0; i < endCathodeAuto.size(); i++)
                {
                    if (startPos[i].x < ispResult.mCenter.x - ispResult.mCenterWidth / 2 || startPos[i].x > ispResult.mCenter.x + ispResult.mCenterWidth / 2)
                    {
                        ispResult.mCathodePos.push_back(cv::Point(endCathodeAuto[i].x + subROI.x, endCathodeAuto[i].y + subROI.y));
                    }
                }
            }
            else
            {
                for(int i = 0; i < startPos.size(); i++)
                {
                    cv::Rect poleROI = cv::Rect(startPos[i].x - catWindowSize / 2, subROI.y, catWindowSize, startPos[i].y - subROI.y);
                    RefineROI(poleROI, src.size());
                    cv::Mat poleImg = src(poleROI);
                    cv::reduce(poleImg, poleImg, 1, cv::REDUCE_AVG, CV_32FC1);
                    cv::Mat thresholdImg;
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
                    cv::threshold(poleImg, thresholdImg, threshold, 255, cv::THRESH_BINARY_INV);

                    std::vector<float> reduceVec = ReduceBlackBackgroundMat(thresholdImg, 1, kSize);
                    std::vector<float> diffVec(reduceVec.size(), 0);
                    for (int i = 1; i < reduceVec.size(); i++)
                    {
                        diffVec[i] = reduceVec[i] - reduceVec[i - 1];
                    }

                    xvt::FindPeaks findPeaks = xvt::FindPeaks(xvt::PeakType::Peak, xvt::PeakFindingMethod::Prominence);
                    findPeaks.Process(diffVec);
                    auto peakResult = findPeaks.GetPeakResult(0);

                    std::sort(peakResult.begin(), peakResult.end(), [](xvt::Peak& a, xvt::Peak& b) {return abs(a.index) > abs(b.index); });

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
            
            cathodeInpection(EResult::OK, "");
        }
        else
        {
            cathodeInpection(EResult::ER, "Find Cathode X Position - Peak size is not enough!");
        }
    }
    else
    {
        cathodeInpection(EResult::ER, "No Image in Cathode - Lower!");
    }
    return cathodeInpection;
}

void CTBatteryInspectorUpperResult::DrawPoleTextResult(cv::Mat& resImg, CVPen const& pen, cv::Point const& offset) const
{
    int expand_height = 0;
    int row_height = 25;
    int col_width = 70;
    int offset_x = 20;
    float thickness = pen.mThickness;
    cv::Point draw_pos = cv::Point(offset_x, offset_x);
    float precision = 3;
    float font_scale = pen.mFontScale;

    //Draw the setting ROI
    {
        const int xCathode2Base1 = mOuterRoi.x + mOuterRoi.width * 2 / 5;
        const int xCathode2Base2 = mOuterRoi.x + mOuterRoi.width * 3 / 5;
        const int yCathode2Base1 = mCaseRef.y + mValidCathode2Case.GetLower() / mPixelSize;
        const int yCathode2Base2 = mCaseRef.y + avgCathode2Case / mPixelSize;
        const int yCathode2Base3 = mCaseRef.y + mValidCathode2Case.GetUpper() / mPixelSize;
        Drawing::DrawDashedLine(resImg, cv::Point(xCathode2Base1, yCathode2Base1), cv::Point(xCathode2Base2, yCathode2Base1), cv::Scalar(180, 180, 0), 2, "", 10);
        Drawing::DrawDashedLine(resImg, cv::Point(xCathode2Base1, yCathode2Base2), cv::Point(xCathode2Base2, yCathode2Base2), cv::Scalar(227, 162, 0), 2, "", 10);
        Drawing::DrawDashedLine(resImg, cv::Point(xCathode2Base1, yCathode2Base3), cv::Point(xCathode2Base2, yCathode2Base3), cv::Scalar(0, 0, 255), 2, "", 10);

        // draw line info
        //Draw the setting ROI
        const int xLineInfo1 = 100;
        const int xLineInfo2 = xLineInfo1 + 100;
        const int yLineInfo1 = resImg.rows - 230;
        const int yLineInfoOffset = 25;
        cv::line(resImg, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset), cv::Point(xLineInfo2 - 10, yLineInfo1 + yLineInfoOffset), cv::Scalar(0, 255, 0), 3);
        Drawing::DrawDashedLine(resImg, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 2), cv::Point(xLineInfo2, yLineInfo1 + yLineInfoOffset * 2), cv::Scalar(180, 180, 0), 2, "", 10);
        Drawing::DrawDashedLine(resImg, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 3), cv::Point(xLineInfo2, yLineInfo1 + yLineInfoOffset * 3), cv::Scalar(227, 162, 0), 2, "", 10);
        Drawing::DrawDashedLine(resImg, cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 4), cv::Point(xLineInfo2, yLineInfo1 + yLineInfoOffset * 4), cv::Scalar(0, 0, 255), 2, "", 10);
        CVPen temp = pen;
        temp.mColor = COLOR_CV_WHITE;
        cv::Point offsetDraw = cv::Point(-15, 0);
        xvt::DrawText(resImg, "Base", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset), xvt::TextAlign::MIDDLE_RIGHT, temp, offsetDraw);
        xvt::DrawText(resImg, "Min", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 2), xvt::TextAlign::MIDDLE_RIGHT, temp, offsetDraw);
        xvt::DrawText(resImg, "Avg", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 3), xvt::TextAlign::MIDDLE_RIGHT, temp, offsetDraw);
        xvt::DrawText(resImg, "Max", cv::Point(xLineInfo1, yLineInfo1 + yLineInfoOffset * 4), xvt::TextAlign::MIDDLE_RIGHT, temp, offsetDraw);
    }
    auto pen0 = std::make_shared<xvt::CVPen>(cv::Scalar(255, 180, 66), thickness, font_scale);
    auto penOK = std::make_shared<xvt::CVPen>(cv::Scalar(0, 255, 0), thickness, font_scale);
    auto penNG = std::make_shared<xvt::CVPen>(COLOR_CV_RED, thickness, font_scale);
    Table drawingL(col_width, row_height, pen0, nullptr);
    Table drawingR(col_width, row_height, pen0, nullptr);

    VecString headerString = { "No", "Pos", "A-CL", "A-CR", "A-B", "C-B" };
    drawingL.AddRow(headerString);
    drawingL.SetColumnWidth(0, 50);

    drawingR.AddRow(headerString);
    drawingR.SetColumnWidth(0, 50);

    for (int i = 0; i < vtCathodes.size(); i++)
    {
        bool isCathode2BaseError = !vtCathode2CaseDecision[i];
        VecString Cat;
        {
            Cat.push_back("A" + xvt::ToString(i + 1));
            Cat.push_back(xvt::ToString(sXPos[i], precision));
            Cat.push_back("-");
            Cat.push_back("-");
            Cat.push_back("-");
            Cat.push_back(xvt::ToString(sCathode2Case[i], precision));
        }

        if (i < nLeftPole)
        {
            drawingL.AddRow(Cat);
            auto current_row = drawingL.GetRows() - 1;
            drawingL.SetCellColor(current_row, 0, isCathode2BaseError ? penNG : penOK, nullptr);
            drawingL.SetCellColor(current_row, 5, isCathode2BaseError ? penNG : penOK, nullptr);
        }
        else
        {
            drawingR.AddRow(Cat);
            auto current_row = drawingR.GetRows() - 1;
            drawingR.SetCellColor(current_row, 0, isCathode2BaseError ? penNG : penOK, nullptr);
            drawingR.SetCellColor(current_row, 5, isCathode2BaseError ? penNG : penOK, nullptr);
        }
    }

    auto l_size = drawingL.GetSize();
    auto r_size = drawingR.GetSize();

    if (expand_height <= 0) expand_height = std::max(l_size.height, r_size.height) + draw_pos.y * 2;

    cv::Mat tableImg = cv::Mat(resImg.rows + expand_height, resImg.cols, resImg.type(), cv::Scalar::all(0));
    draw_pos.y += resImg.rows;
    draw_pos.x = (resImg.cols - l_size.width * 2) / 3;

    resImg.copyTo(tableImg(cv::Rect(0, 0, resImg.cols, resImg.rows)));

    drawingL.SetColumnAlign(0, TextAlign::LEFT);
    drawingL.Draw(tableImg, draw_pos);

    drawingR.SetColumnAlign(0, TextAlign::LEFT);
    drawingR.Draw(tableImg, cv::Point(resImg.cols - draw_pos.x - r_size.width, draw_pos.y));

    resImg = std::move(tableImg);
}

void CTBatteryInspectorUpperResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    pen.mSpace = 15;
    pen.mFontScale = 0.6;
    pen.mThickness = 2;
    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    pen.mColor = GetResultColor();
    cv::rectangle(img, GetROI(), cv::Scalar(0, 255, 255), 2);
    cv::Scalar color(150, 150, 0);
    if (IsER())
    {
        mBeadingResult.DrawResult(img, cv::Point(), pen);

        if (!mPoints.empty())
            xvt::DrawPoints(img, mPoints, pen.mColor);
    }
    else
    {
        for (int i = 0; i < vtCathodes.size(); i++) {
            cv::Point textCatPosition;
            textCatPosition.y = vtCathodes[i].y + 15;
            textCatPosition.x = vtCathodes[i].x - 5;

            cv::Scalar textColor(0, 0, 0);
            if (vtCathode2CaseDecision[i] == true) {
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
                if (vtCathode2CaseDecision[i] == true) {
                    textColor = cv::Scalar(0, 255, 0);
                }
                else {
                    textColor = cv::Scalar(0, 0, 255);
                }
                cv::putText(img, textCat, textCatPosition, cv::FONT_HERSHEY_SIMPLEX, 0.4, textColor, 1);
            }
            xvt::DrawPlusSign(img, vtCathodes[i], textColor, 2);
        }
    }


    if (!mOuterRoi.empty())
    {
        auto tl = mOuterRoi.tl();
        auto br = mOuterRoi.br();
        pen.mColor = COLOR_CV_ORANGE;
        std::vector<cv::Point> points = { tl, br, cv::Point(tl.x, br.y), cv::Point(br.x, tl.y) };
        for (auto p : points) DrawPlusSign(img, p, pen, 7);
    }

    cv::circle(img, mCenter, 2, cv::Scalar(0, 0, 255), 2);
    
    cv::line(img, cv::Point(mCenter.x - mCenterWidth / 2, mPoleRegionRoi.y), cv::Point(mCenter.x - mCenterWidth / 2, mPoleRegionRoi.br().y), cv::Scalar(0, 255, 0), 1);
    cv::line(img, cv::Point(mCenter.x + mCenterWidth / 2, mPoleRegionRoi.y), cv::Point(mCenter.x + mCenterWidth / 2, mPoleRegionRoi.br().y), cv::Scalar(0, 255, 0), 1);

    if (!mPoleRegionRoi.empty())
        cv::rectangle(img, mPoleRegionRoi, cv::Scalar(106, 113, 220), 2);

    DrawResultStr(img, "", pen, cv::Point());
}

auto CTBatteryInspectorUpperResult::DrawResultStr(cv::Mat &img,
                                            std::string const &name,
                                            CVPen const &pen,
                                            cv::Point const &offset,
                                            bool isDrawOKResult) const -> cv::Point
{
    auto tmpPen = pen;
    tmpPen.mFontScale = 0.55;
    tmpPen.mThickness = 2;
    tmpPen.mSpace = 20;
    auto size = img.size();
    std::string decisionStr = finalDecision.empty() ? "NG" : finalDecision;
    if (IsER())
    {
        cv::putText(img, GetMsg(), cv::Point(size.width / 3.1, 85 + 25), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), tmpPen.mThickness);
    }
    else
    {
        for (int i = 0; i < mErrMsg.size(); i++) {
            cv::putText(img, mErrMsg[i], cv::Point(size.width / 3.1, 85 + i * 25), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), tmpPen.mThickness);
        }
    }
    cv::putText(img, "Result: " + decisionStr, cv::Point(size.width / 2.5, 50), cv::FONT_HERSHEY_SIMPLEX, 1,
        decisionStr == "OK" ? cv::Scalar(245, 164, 66) : cv::Scalar(0, 0, 255), tmpPen.mThickness);

    auto startPos = cv::Point(img.cols / 2, 0) - cv::Point(25);
    //auto txtPos = InspectionResult::DrawResultStr(img, name, tmpPen, startPos, 0);
#if 1
    std::stringstream ss;
    ss << "Pixel Size: " << xvt::ToString(mPixelSize, 3) << "[mm]" << " | No. Cathode: " << vtCathodes.size();
    auto drawPos = cv::Point(std::min(size.width - 1, 50), std::max(0, size.height - 50));
    auto color1 = img.at<cv::Vec3b>(drawPos);
    int grayColor = abs(color1(0) - 125) < 50 ? 250 : (255 - color1(0));
    tmpPen.mColor = COLOR_CV_GRAY(250);
    DrawText(img, ss.str(), drawPos, tmpPen);
#endif // _DEBUG

    if ((mDisplayMode & DisplayMode::TEXT) == DisplayMode::TEXT && !IsER())
    {
        auto row1Color = std::make_shared<CVPen>(cv::Scalar(0, 255, 0), tmpPen.mThickness, tmpPen.mFontScale);
        auto row2Color = std::make_shared<CVPen>(cv::Scalar(2, 165, 249), tmpPen.mThickness, tmpPen.mFontScale);
        VecString rangeRow(3);
        rangeRow[0] = "Base->Cathode [" + xvt::ToString(mPoles.mValidCathode2CaseRange.GetLower(), 3) + " ~ " + xvt::ToString(mPoles.mValidCathode2CaseRange.GetUpper(), 3) + "]";

        VecString Row1 = { "Min", "Max", "Average" };
        VecString Row2;
        {
            Row2.push_back(xvt::ToString(minCathode2Case, 3));
            Row2.push_back(xvt::ToString(maxCathode2Case, 3));
            Row2.push_back(xvt::ToString(avgCathode2Case, 3));
        }

        Table drawingTable(120, 30, row1Color, row1Color);
        drawingTable.AddRow(rangeRow);
        drawingTable.AddRow(Row1);
        drawingTable.AddRow(Row2);
        drawingTable.SetRowColor(2, row2Color, drawingTable.mBorderStyle);

        drawingTable.MergeCells(0, 0, 0, 2);

        auto s = drawingTable.GetSize();
        drawingTable.Draw(img, cv::Point((size.width - s.width) / 2, 560));

        DrawPoleTextResult(img, pen);
    }

    return cv::Point(offset.x, startPos.y);
}

auto CTBatteryInspectorUpperResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
    /*const auto& listLenght = mAnodePoles.GetLenghts();
    for (int i = 0; i < 140; i++)
    {
        float leght = (i < mAnodePoles.Size()) ? listLenght[i] : -1.0f;
        out.emplace_back("NP" + std::to_string(i), xvt::ToString(leght, 3));
    }*/
}

} // namespace battery
}
#endif