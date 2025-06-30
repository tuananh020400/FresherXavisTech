#include "xvtBattery/JRBatteryInspector.h"
#include "xvtBattery/JRUtils.h"
#include "xvtCV/Utils.h"
#include "xvtCV/xvtConvert.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/Peak.h"
#include "xvtCV/ScopeTimer.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <CvPlot/cvplot.h>

namespace xvt
{
namespace battery
{

JRBatteryInspector::JRBatteryInspector()
{
    mIspBase.mValidBatteryWidthRange.Set(0, 10000);

    mIspJR.mCenterNeglectionWidth = 0;
    mIspJR.mEnableCenterNeglection = false;

    mIspCathode.mIsApplyEnhanced = false;
}
#define DEBUG_MEASURE
auto JRBatteryInspector::Inspect(const cv::Mat &inImg) -> JRBatteryInspectResult
{
    JRBatteryInspectResult ispResult;

    ScopeTimer t("JR Battery");

    cv::Mat src;

    if (!xvt::Convert8Bits(inImg, src, mIsRotated ? true : false))
    {
        if (mIsRotated)
        {
            /*cv::Point tl = mIspBase.mRoi.tl();
            cv::Point br = mIspBase.mRoi.br();
            tl = TranformPoint(tl, inImg.size());
            br = TranformPoint(br, inImg.size());*/

            mIspBase.mRoi = TranformRect(mIspBase.mRoi, inImg.size());

            cv::transpose(src, src);
            cv::flip(src, src, 1);
        }

        t.GetElapsedTime();
        auto baseResult = mIspBase.Inspect(src);
        baseResult.mProTime = t.GetElapsedTime().count();

        auto connerPoint = baseResult.GetCornerPoint();
        if (!baseResult.IsER() && !baseResult.IsUC() && !connerPoint.empty())
        {
            cv::Point tl = connerPoint[0];
            cv::Point tr = connerPoint[1];
            //cv::Point br = connerPoint[2];
            //cv::Point bl = connerPoint[3];

            int yMinAnode = tl.y > tr.y ? tr.y : tl.y;
            int heightSubROI = yMinAnode + abs(mIspPole.mPoleHeight) + baseResult.GetROI().tl().y;
            cv::Rect subLineRoi = CreateROI(baseResult.GetROI().tl() + cv::Point(mIspJR.mJROffsetX, mIspJR.mJROffsetY),
                                            cv::Size(baseResult.GetROI().width - mIspJR.mJROffsetX * 2, heightSubROI - baseResult.GetROI().tl().y),
                                            inImg.size());           
            ispResult.SetBaseResult(std::move(baseResult));
            
            cv::Rect subRoi = subLineRoi;
            subRoi.height += mIspJR.mHeight;
            RefineROI(subRoi, src.size());
            auto cathodeResult = mIspCathode.Inspect(src, subLineRoi, ispResult.mAnodePoleResult.mPoles);
            cathodeResult.mProTime = t.GetElapsedTime().count();
            ispResult.SetCathodeResult(std::move(cathodeResult));
            auto &cathodePoles = ispResult.mCathodePoleResult.mPoles;
            auto &anodePoles = ispResult.mAnodePoleResult.mPoles;
            int isize = anodePoles.size();          
            cv::Mat srcEqua = LocalHistogramEqualization(src(subRoi), mIspJR.mNoRegion);
            for (int i = 1, size = isize; i < size; i++)
            {
                int pre = i - 1;
                cv::Rect midSubROI = CreateROI(anodePoles[pre].mCathode + cv::Point(mOffset, 0),
                                   cv::Size(abs(anodePoles[i].mCathode.x - anodePoles[pre].mCathode.x) - 2 * mOffset,
                                            abs(subRoi.br().y - anodePoles[pre].mCathode.y)), subRoi.size());
                PoleInfo cathodePos;
                auto pointResult = FindStartPointCathode(srcEqua, midSubROI, cathodePos.mAnode);
                if (pointResult.IsOK())
                    cathodePoles.push_back(cathodePos);
            }

            ispResult.mCathodePoleResult.mPixelSize = mPixelSize;
            ispResult.mCathodePoleResult.SetROI(subRoi);
            ispResult.mCathodePoleResult.mProTime = t.GetElapsedTime().count();
            ispResult.mAnodePoleResult.mPixelSize = mPixelSize;
            ispResult.mAnodePoleResult.SetROI(subRoi);
            if ((anodePoles.size() - cathodePoles.size()) == 1 && anodePoles.size() > 2)
            {
                anodePoles[0].mCathode.y = cathodePoles[0].mAnode.y;
                for (int i = 1; i < isize; i++)
                {
                    anodePoles[i].mCathode.y = cathodePoles[i - 1].mAnode.y;
                }

                auto anodeResult = mIspAnode.Inspect(src, subRoi, ispResult.mAnodePoleResult.mPoles);
                anodeResult.mProTime = t.GetElapsedTime().count();
                ispResult.SetAnodeResult(std::move(anodeResult));
                
                if (!anodePoles.empty())
                {
                    //Check Anode Position in Battery
                    int minPosAno = ispResult.mBaseResult.GetROI().y - ispResult.mAnodePoleResult.GetROI().y;
                    //auto anoResult = CheckAnodeInBattery(anodePoles, minPosAno);
                    bool direction = BatteryPositionIsLeft(src, subLineRoi);
                    //Check direction
                    if (!direction)
                    {
                        std::reverse(cathodePoles.begin(), cathodePoles.end());
                        std::reverse(anodePoles.begin(), anodePoles.end());

                        for (const auto& a : anodePoles)
                        {
                            ispResult.mAnodePoleResult.mXPoles.emplace_back(abs(subRoi.width - a.mAnode.x) * mPixelSize);
                        }
                    }
                    else
                    {
                        for (const auto& a : anodePoles)
                        {
                            ispResult.mAnodePoleResult.mXPoles.emplace_back(a.mAnode.x * mPixelSize);
                        }
                    }

                    if (anodePoles.size() > mIspPole.mMaximumNumberPoles)
                    {
                        anodePoles.erase(anodePoles.begin() + mIspPole.mMaximumNumberPoles, anodePoles.end());
                        cathodePoles.erase(cathodePoles.begin() + mIspPole.mMaximumNumberPoles - 1, cathodePoles.end());
                    }

                    ispResult.mAnodePoleResult.mValidAnode2CaseRange = mIspPole.mValidAnode2CaseRange;
                    ispResult.mAnodePoleResult.mValidCathode2AnodeRange = mIspPole.mValidCathode2AnodeRange;
                    ispResult.mAnodePoleResult.mValidCathode2CaseRange = mIspPole.mValidCathode2CaseRange;
                    

                    for(int i = 1; i <= (mIspPole.mMaximumNumberPoles); i++)
                    {
                        if (i <= (mIspPole.mMaximumNumberPoles - mIspPole.mMinimumNumberPoles)) {
                            ispResult.mAnodePoleResult.mPoles[mIspPole.mMaximumNumberPoles - i].mType = PoleType::Incorrect;
                            ispResult.mAnodePoleResult.mPoles[mIspPole.mMaximumNumberPoles - i].SetResult(EResult::OK);
                            ispResult.CombineResult(ispResult.mAnodePoleResult.mPoles[mIspPole.mMaximumNumberPoles - i].GetResult());
                        }
                        else {
                            bool Cathode2AnodeNG = mIspPole.mEnableValidCathode2Anode && !mIspPole.mValidCathode2AnodeRange.IsInRange(ispResult.mAnodePoleResult.mPoles[mIspPole.mMaximumNumberPoles - i].length() * mPixelSize);
                            ispResult.mAnodePoleResult.mPoles[mIspPole.mMaximumNumberPoles - i].SetResult((Cathode2AnodeNG) ? EResult::NG : EResult::OK);
                            ispResult.CombineResult(ispResult.mAnodePoleResult.mPoles[mIspPole.mMaximumNumberPoles - i].GetResult());
                        }
                    }
                    
                    //Check certain and uncertain region
                    cv::Point uncertainPoint = cv::Point(0, 0);
                    int offset = 100;
                    int index = -1;

                    for(auto& p : ispResult.mAnodePoleResult.mPoles)
                    {
                        if (abs(p.mPeakValue.prominence) < mProminenceValue) {
                            if (index < 0) {
                                uncertainPoint = p.mAnode;
                                index += 1;
                            }
                        }
                    }

                    if(uncertainPoint != cv::Point(0, 0))
                    {
                        if (!direction)
                        {
                            int xUncertainR = ispResult.mAnodePoleResult.mPoles.back().mAnode.x - offset;
                            int yUncertainR = uncertainPoint.y - offset;
                            ispResult.mUncertainRegion = CreateROI(cv::Point(xUncertainR, yUncertainR), cv::Size(uncertainPoint.x - xUncertainR + 20, 300), srcEqua.size());
                            int xCertainR = uncertainPoint.x + 20;
                            int yCertainR = uncertainPoint.y - offset;
                            ispResult.mCertainRegion   = CreateROI(cv::Point(xCertainR, yCertainR), cv::Size(ispResult.mAnodePoleResult.mPoles[0].mAnode.x - xCertainR + 100, 300), srcEqua.size());
                        }
                        else 
                        {
                            int xCertainR = ispResult.mAnodePoleResult.mPoles[0].mAnode.x - offset;
                            int yCertainR = uncertainPoint.y - offset;
                            ispResult.mCertainRegion   = CreateROI(cv::Point(xCertainR, yCertainR), cv::Size(uncertainPoint.x  - xCertainR - 20, 300), srcEqua.size());
                            int xUnCertainR = uncertainPoint.x - 20;
                            int yUnCertainR = uncertainPoint.y - offset;
                            ispResult.mUncertainRegion = CreateROI(cv::Point(xUnCertainR, yUnCertainR), cv::Size(ispResult.mAnodePoleResult.mPoles.back().mAnode.x - xUnCertainR + 100, 300), srcEqua.size());
                        }
                    }
                    else 
                    {
                        if (!direction)
                        {
                            int xCertainR = ispResult.mAnodePoleResult.mPoles.back().mAnode.x - 20;
                            int yCertainR = ispResult.mAnodePoleResult.mPoles[0].mAnode.y - offset;
                            int wCertainR = ispResult.mAnodePoleResult.mPoles[0].mAnode.x - xCertainR + offset;
                            int hCertainR = 300;
                            ispResult.mCertainRegion = CreateROI(cv::Point(xCertainR, yCertainR), cv::Size(wCertainR, hCertainR), srcEqua.size());
                        }
                        else
                        {
                            int xCertainR = ispResult.mAnodePoleResult.mPoles[0].mAnode.x - 20;
                            int yCertainR = ispResult.mAnodePoleResult.mPoles[0].mAnode.y - offset;
                            int wCertainR = ispResult.mAnodePoleResult.mPoles.back().mAnode.x - xCertainR + offset;
                            int hCertainR = 300;
                            ispResult.mCertainRegion = CreateROI(cv::Point(xCertainR, yCertainR), cv::Size(wCertainR, hCertainR), srcEqua.size());
                        }
                    }

#ifdef DEBUG_MEASURE
                    cv::Mat tmpDst = src(subRoi).clone(); // = src.clone();
                    cv::Rect roiRect = cv::boundingRect(ispResult.mCathodeResult.mContour);
                    cv::Mat testImg = tmpDst(roiRect).clone();
                    CalculateSNR(src(subRoi), ispResult.mAnodePoleResult ,ispResult.snrValRoi, 15, 100);
#endif //DEBUG_MEASURE
                }
            }
            else
            {
                ispResult(EResult::ER, "Inspect: Anode and Cathode pole number is not correct!");
            }

            auto verifyResult = VerifyResult(ispResult, src.size());
            ispResult.CombineResult(&verifyResult);
            ispResult.SetROI(subRoi);

        }
        else
        {

            ispResult.SetROI(baseResult.GetROI());
        }
    }
    else
    {
        ispResult(EResult::ER, "Inspect: Image type is not supported!");
    }

    auto tmp = t.GetTotalElapsedTime();
    ispResult.mProTime = tmp.count();
    return ispResult;
}

auto JRBatteryInspector::CheckAnodeInBattery(std::vector<PoleInfo>& anoPos, int minPosAno) const -> InspectionResult
{
    InspectionResult anoResult;
    if (minPosAno < 0)
    {
        minPosAno = 0;
    }

    if (!anoPos.empty()) 
    {
        for( auto& a : anoPos)
        {
            a.mAnode.y = (a.mAnode.y < minPosAno) ? minPosAno : a.mAnode.y;
        }
    }
    else
    {
        anoResult(EResult::ER, "Check Anode In Battery: Anode Empty");
    }

    return anoResult;
}

auto JRBatteryInspector::CalculateSNR(const cv::Mat& img, PoleResult anodeInfo, std::vector<float>& snrValRoi, int stepWid, int stepHei) const -> InspectionResult
{
    InspectionResult finalResult;
    // Find rectMaster
    int xRectMas = anodeInfo.mPoles[0].mAnode.x - stepWid;
    int yRectMas = anodeInfo.mPoles[0].mAnode.y - stepHei;
    int widthMas = 2 * stepWid;
    int heightMas= 2 * stepHei;
    cv::Rect subROI = CreateROI(cv::Point(xRectMas, yRectMas), cv::Size(widthMas, heightMas), img.size());
    cv::Mat masterImg = img(subROI).clone();
    // P master
    float signalPower = 0.0;
    for (int y = 0; y < masterImg.rows; ++y) {
        for (int x = 0; x < masterImg.cols; ++x) {
            float originalPixel = masterImg.at<uchar>(y, x);
            signalPower += originalPixel * originalPixel;
        }
    }
    
    for (auto p : anodeInfo.mPoles) 
    {
        int xRect = p.mAnode.x - stepWid;
        int yRect = p.mAnode.y - stepHei;
        int width = 2 * stepWid;
        int height = 2 * stepHei;
        cv::Rect subROI = CreateROI(cv::Point(xRect, yRect), cv::Size(width, height), img.size());
        cv::Mat imgSub = img(subROI).clone();

        float noisePower = 0.0;
        for (int y = 0; y < imgSub.rows; ++y) {
            for (int x = 0; x < imgSub.cols; ++x) {
                float originalPixel = masterImg.at<uchar>(y, x);
                float noisyPixel = imgSub.at<uchar>(y, x);
                float noise = originalPixel - noisyPixel;
                noisePower += noise * noise;
            }
        }
        float snr = 10 * log10(signalPower / noisePower);
        snrValRoi.emplace_back(snr);
    }
    finalResult(EResult::OK, "");
    return finalResult;
}

//auto JRBatteryInspector::CalculateCTF(const cv::Mat& img, std::vector<float>& ctfValRoi, int sampleWid) const ->InspectionResult
//{
//
//}

auto JRBatteryInspector::FindStartPointCathode(const cv::Mat& src, const cv::Rect& roi, cv::Point& catPos) const -> InspectionResult
{
    InspectionResult finalResult;
    cv::Rect subRoi = CreateROI(roi.tl(), roi.size(), src.size());
    if (!src.empty() && !subRoi.empty())
    {
        std::vector<float> tempReduceVec;
        cv::reduce(src(subRoi), tempReduceVec, 0, cv::REDUCE_AVG);
        FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
        findPeaks.Process(tempReduceVec);
        std::vector<Peak> peaks = findPeaks.GetPeakResult(0);

        int idx = tempReduceVec.size() / 2;
        if (peaks.size() > 0)
        {
            auto minMax = std::minmax_element(begin(peaks), end(peaks), [](const Peak& lsh, const Peak& rhs) {
                return abs(lsh.prominence) > abs(rhs.prominence);
                });

            idx = minMax.second->index;
        }
        //auto maxPos = std::min_element(tempReduceVec.begin(), tempReduceVec.end());
        catPos.x = subRoi.tl().x + idx; //std::distance(tempReduceVec.begin(), maxPos);

        if (mAlgo == FindStartAnode::findPeak)
        {
            finalResult = FindStartPointByPeak(src, subRoi, catPos);
        }
        else
        {
            cv::Mat thdImg, reduceThresholdImg;
            cv::threshold(src(subRoi), thdImg, mManullyThreshold, 1, mAlgo == FindStartAnode::thresholdManuly ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
            cv::reduce(thdImg, reduceThresholdImg, 1, cv::REDUCE_SUM, CV_32FC1);
            std::vector<float> tempreduceThresholdVec = cv::Mat_<float>(reduceThresholdImg.col(0));
            int thresholdCathode = (subRoi.width < mThresholdCathodeDefault) ? subRoi.width : mThresholdCathodeDefault;
            auto firstPos = std::find_if(tempreduceThresholdVec.begin(), tempreduceThresholdVec.end(), [&thresholdCathode](const float& t) {
                return t > thresholdCathode;
                });

            catPos.y = subRoi.tl().y + std::distance(tempreduceThresholdVec.begin(), firstPos);
            finalResult(EResult::OK, "");
        }
        catPos.y = catPos.y + mOffsetAno;
    }
    else
    {
        finalResult(EResult::ER, "FindCathodeLineByAStar: Image Empty");
    }

    return finalResult;
}

//#define DEBUG_PLOT
auto JRBatteryInspector::FindStartPointByPeak(const cv::Mat& src, cv::Rect subRoi, cv::Point &catPos) const -> InspectionResult
{
    constexpr int rangeSkip = 20;
    constexpr int rangeSkip2 = 10;
    InspectionResult peakResult;
    cv::Mat imgRoiF = xvt::GetImage(src, subRoi);
    if (!imgRoiF.empty() && imgRoiF.rows > 2 * rangeSkip + 1)
    {
        // Parameters
#ifdef DEBUG_PLOT
        std::vector<float> staticGLeft;
        std::vector<float> staticGright;

        if (catPos.x >= 384) 
        {
            int d = 0;
        }
        int endCompare = imgRoiF.rows - 15;
        int startCompare = 15;
        std::vector<cv::Point2f> checkValidation(imgRoiF.rows, cv::Point2f(0.0, 0.0));
        for (int o = 250; o > startCompare; o--) {

            cv::Point sRectCheckPole = cv::Point(0, o);
            cv::Size sRangeCheckRect = cv::Size(imgRoiF.cols, 15);
            cv::Rect rectCheckPole = CreateROI(sRectCheckPole, sRangeCheckRect, imgRoiF.size());
            cv::Mat imgCheck = imgRoiF(rectCheckPole);
            std::vector<float> tempCheckReduceVec;
            cv::reduce(imgCheck, tempCheckReduceVec, 0, cv::REDUCE_AVG, CV_32FC1);
            cv::GaussianBlur(tempCheckReduceVec, tempCheckReduceVec, cv::Size(3, 1), 0);
            cv::Point2f valiPoint(0.0, 0.0);

            xvt::FindPeaks findPeaks = xvt::FindPeaks(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
            findPeaks.Process(tempCheckReduceVec);
            auto peakResult = findPeaks.GetPeakResult();

            /*for (auto pea : peakResult) {
                if (pea.index <=  && pea.index >= )
                {
                    valiPoint.x += 1.0;
                    if (pea.prominence < valiPoint.y)
                    {
                        valiPoint.y = pea.prominence * 100;
                    }
                }
            }
            checkValidation[o] = (valiPoint);*/
#ifdef DEBUG_PLOT
            cv::Mat matVali(600, 1200, CV_8UC3);
            matVali.setTo(cv::Scalar(255, 255, 255));

            CvPlot::Axes axesVali = CvPlot::makePlotAxes();
            axesVali.enableHorizontalGrid();
            axesVali.enableVerticalGrid();
            axesVali.title("ROI");

            axesVali.create<CvPlot::Series>(tempCheckReduceVec, "-r");

            int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
            axesVali.setMargins(borderLeft, borderRight, borderTop, borderBottom);
            axesVali.setXLim(std::pair<int, int>(0, 40));
            axesVali.setYLim(std::pair<int, int>(0, 255));
            axesVali.setXTight(true);
            axesVali.render(matVali);
#endif // DEBUG_PLOT
        }
#endif // DEBUG_PLOT

        //Setup find Signal Transformation Commencement. Apply Left and Right Side of Point.
        constexpr float thetaR = 210.0;
        constexpr float thetaR2 = 2.0 * thetaR * thetaR;
        constexpr float thetaL = 30.0;
        constexpr float thetaL2 = 2.0 * thetaL * thetaL;

        //Setup find Variation Among the Points.
        constexpr float thetaEval = 6.0;
        constexpr float thetaEval2 = 2.0 * thetaEval * thetaEval;
        //Magnification coefficient
        constexpr float magCoff = 100.0;
        constexpr float GSumLeftEvalThreshold = 0.7 * magCoff;
        std::vector<float> listLeftEval;
    
        //Signal
        cv::Mat anodeLine;
        cv::reduce(imgRoiF, anodeLine, 1, cv::REDUCE_AVG, CV_32FC1);
        std::vector<float> values = cv::Mat_<float>(anodeLine);
        int kSize = std::max<int>((rangeSkip2 % 2 != 0) ? rangeSkip2 : rangeSkip2 + 1, 5);
        cv::GaussianBlur(values, values, cv::Size(kSize, 1), 0);
        // Find max min intensity
        auto minmax = std::minmax_element(values.begin(), values.end());

        //Parameter for the Signal Transformation Commencement method to find the maximum value in a list.
        int index = -1;
        float maxG = -1.0f;
        float minThresholdAnode = *minmax.first + 10.0;
        float maxThresholdAnode = *minmax.second;

        //List Derivative.
        std::vector<float> listDev;
        //Sum of G_left and G_right.
        std::vector<float> listSumG;

        //Signal Transformation Commencement
        int startIdx = rangeSkip;
        int endIdx = anodeLine.rows - rangeSkip;
        for (int i = startIdx; i < endIdx; i++)
        {

            float sumDiff = 0;
            float GLeftEval = 0;
            float G = 0;

            // Caculater G_Right
            float sumDiff_R = 0;
            // Caculater G_Left
            float sumDiff_L = 0;
            // Caculater G_Left Evaluation
            float sumDiff_LEval = 0;

            if (values[i - rangeSkip] < maxThresholdAnode && values[i + rangeSkip] >= minThresholdAnode)
            {
                float Gleft = 0;
                float Gright = 0;
                for (int k = 1; k <= rangeSkip; k++)
                {
                    sumDiff_R += abs(values[i] - values[i + k]);
                    sumDiff_L += abs(values[i - k] - values[i]);
                    if (k <= rangeSkip2)
                    {
                        sumDiff_LEval += sumDiff_L;
                    }
                }

                sumDiff_R /= rangeSkip;
                sumDiff_L /= rangeSkip;
                sumDiff = sumDiff_R + sumDiff_L;
                float detha = 255.0 - sumDiff_R;
                Gright = exp(-(detha * detha) / thetaR2) * 100;
                Gleft  = exp(-(sumDiff_L * sumDiff_L) / thetaL2) * 100;
                // Sum G
                G = Gleft + Gright;
                G = G;
                sumDiff_LEval /= rangeSkip2;
                GLeftEval = exp(-(sumDiff_LEval * sumDiff_LEval) / (thetaEval2))*magCoff;

    #ifdef DEBUG_PLOT
                staticGLeft.emplace_back(Gleft);
                staticGright.emplace_back(Gright);
    #endif // DEBUG_PLOT
            }

            listDev.push_back(sumDiff);
            listLeftEval.emplace_back(GLeftEval);
            listSumG.emplace_back(G);

        }

#ifdef DEBUG_PLOT
        std::vector<float> vectorZero(rangeSkip, 0.0);
        staticGLeft.insert(staticGLeft.begin(), vectorZero.begin(), vectorZero.end());
        staticGLeft.insert(staticGLeft.end(), vectorZero.begin(), vectorZero.end());
        staticGright.insert(staticGright.begin(), vectorZero.begin(), vectorZero.end());
        staticGright.insert(staticGright.end(), vectorZero.begin(), vectorZero.end());
        listSumG.insert(listSumG.begin(), vectorZero.begin(), vectorZero.end());
        listSumG.insert(listSumG.end(), vectorZero.begin(), vectorZero.end());
        listLeftEval.insert(listLeftEval.begin(), vectorZero.begin(), vectorZero.end());
        listLeftEval.insert(listLeftEval.end(), vectorZero.begin(), vectorZero.end());

        std::vector<float> vectorZeroDev(rangeSkip, 0.0);
        listDev.insert(listDev.begin(), vectorZeroDev.begin(), vectorZeroDev.end());
        listDev.insert(listDev.end(), vectorZeroDev.begin(), vectorZeroDev.end());
#endif //DEBUG_PLOT

        if (!listSumG.empty() && !listDev.empty())
        {
            // Find max
            int kSize = std::max<int>((rangeSkip % 2 != 0) ? rangeSkip : rangeSkip + 1, 5);
            cv::GaussianBlur(listSumG, listSumG, cv::Size(kSize, 1), 0);

            auto maxElement = std::max_element(listSumG.begin(), listSumG.end());
            int maxIndex = std::distance(listSumG.begin(), maxElement);

            auto maxElementDev = std::max_element(listDev.begin(), listDev.end());
            int maxIndexDev = std::distance(listDev.begin(), maxElementDev);

#ifdef DEBUG_PLOT
            cv::Mat mat(600, 1200, CV_8UC3);
            mat.setTo(cv::Scalar(255, 255, 255));

            CvPlot::Axes axes = CvPlot::makePlotAxes();
            axes.enableHorizontalGrid();
            axes.enableVerticalGrid();
            axes.title("ROI");

            axes.create<CvPlot::Series>(values, "-r");
            //axes.create<CvPlot::Series>(valuesFilter, "-b");
            //axes.create<CvPlot::Series>(valuesFilter2, "-y");
            axes.create<CvPlot::Series>(staticGLeft, "-c");
            axes.create<CvPlot::Series>(staticGright, "-g");
            axes.create<CvPlot::Series>(listSumG, "-b");
            axes.create<CvPlot::Series>(listDev, "-r");
            axes.create<CvPlot::Series>(listLeftEval, "-g");
#endif //DEBUG_PLOT

            //Signal with a transformation point?
            bool turningPoint = false;
            for (int k = maxIndex - 1; k >= 0; k--)
            {
                if (listLeftEval[k] > GSumLeftEvalThreshold)
                {
                    turningPoint = true;
                    break;
                }
            }

            if (turningPoint)
            {
                catPos.y = subRoi.tl().y + maxIndex;
            }
            else
            {
                catPos.y = subRoi.tl().y + maxIndexDev;
            }

#ifndef DEBUG_PLOT
            catPos.y += rangeSkip;
#endif

#ifdef DEBUG_PLOT
            axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ { (float)(maxIndex), anodeLine.at<float>(maxIndex, 0) }, { (float)(maxIndex), anodeLine.at<float>(maxIndex, 0) }}, "-ob");
            axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ { (float)(maxIndexDev), anodeLine.at<float>(maxIndexDev, 0) }, { (float)(maxIndexDev), anodeLine.at<float>(maxIndexDev, 0) }}, "-or");
            if (turningPoint)
            {
                axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ { (float)(maxIndex), anodeLine.at<float>(maxIndex, 0) }, { (float)(maxIndex), anodeLine.at<float>(maxIndex, 0) }}, "-og");
            }
            else
            {
                axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ { (float)(maxIndexDev), anodeLine.at<float>(maxIndexDev, 0) }, { (float)(maxIndexDev), anodeLine.at<float>(maxIndexDev, 0) }}, "-og");
            }

            int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
            axes.setMargins(borderLeft, borderRight, borderTop, borderBottom);
            axes.setXLim(std::pair<int, int>(0, anodeLine.rows));
            axes.setXTight(true);
            axes.render(mat);
#endif // DEBUG_PLOT
            peakResult(EResult::OK, "");
        }
        else
        {
            peakResult(EResult::ER, "Invalid reduce list");
        }
    }
    else 
    {
        peakResult(EResult::ER, "FindStartPointByPeak: Invalid Image");
    }
    return peakResult;
}

auto JRBatteryInspector::VerifyResult(JRBatteryInspectResult &result, cv::Size imgSize) -> InspectionResult
{
    InspectionResult finalResult;
    if (mIsRotated)
    {
        auto &anodes = result.mAnodePoleResult;
        auto &cathodes = result.mCathodePoleResult;
        cv::Point offset = result.mCathodeResult.GetROI().tl();
        for (auto &p : anodes.mPoles)
        {
            p.mCathode += offset;
            p.mAnode += offset;


            p.mCathode = TranformPointInvert(p.mCathode, imgSize);
            p.mAnode = TranformPointInvert(p.mAnode, imgSize);
            p.mPoints = TranformPointsInvert(p.mPoints, imgSize, offset);
        }

        for (auto &p : cathodes.mPoles)
        {
            p.mCathode += offset;
            p.mAnode += offset;

            p.mCathode = TranformPointInvert(p.mCathode, imgSize);
            p.mAnode = TranformPointInvert(p.mAnode, imgSize);
            p.mPoints = TranformPointsInvert(p.mPoints, imgSize, offset);
        }

        anodes.SetROI(TranformRectInvert(anodes.GetROI(), imgSize));
        anodes.mLeftNoPole = anodes.Size();
        cathodes.SetROI(TranformRectInvert(cathodes.GetROI(), imgSize));
        result.mBaseResult.mRoiSetting = TranformRectInvert(result.mBaseResult.mRoiSetting, imgSize);
        result.mBaseResult.SetROI(TranformRectInvert(result.mBaseResult.GetROI(), imgSize));
        result.mCathodeResult.SetROI(TranformRectInvert(result.mCathodeResult.GetROI(), imgSize));
        result.mAnodeResult.SetROI(TranformRectInvert(result.mAnodeResult.GetROI(), imgSize));
        result.mCathodeResult.mPoints = TranformPointsInvert(result.mCathodeResult.mPoints, imgSize, offset);
        result.mCathodeResult.mContour = TranformPointsInvert(result.mCathodeResult.mContour, imgSize, offset);
        result.mBaseResult.mPoints = TranformPointsInvert(result.mBaseResult.mPoints, imgSize);
        if (!result.mUncertainRegion.empty())
        {
            result.mUncertainRegion = cv::Rect(result.mUncertainRegion.tl() + offset, cv::Size(result.mUncertainRegion.width, result.mUncertainRegion.height));
            result.mUncertainRegion = TranformRectInvert(result.mUncertainRegion,imgSize);
        }
        if (!result.mCertainRegion.empty())
        {
            result.mCertainRegion = cv::Rect(result.mCertainRegion.tl() + offset, cv::Size(result.mCertainRegion.width, result.mCertainRegion.height));
            result.mCertainRegion = TranformRectInvert(result.mCertainRegion,imgSize);
        }
    }

    finalResult(EResult::OK, "");
    return finalResult;
}

void JRBatteryInspectResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    //if (mIsResultRotated)
    /*{
        cv::Mat src = cv::Mat(img.rows, img.cols, img.type());
        cv::transpose(img, src);
        cv::flip(src, src, 1);
        img = src;
    }*/

    pen.mThickness = 1;
    mBaseResult.DrawResult(img, cv::Point(), pen);
    //mCathodeResult.DrawResult(img, cv::Point(), pen);
    mAnodeResult.DrawResult(img, cv::Point(), pen);
    //mAnodePoleResult.DrawResult(img, cv::Point(), pen);
    //mCathodePoleResult.DrawResult(img, cv::Point(), pen);
    if (mShowUncertainRegion)
    {
        if(!mUncertainRegion.empty())
        {
            cv::rectangle(img, mUncertainRegion, COLOR_CV_VIOLET, 3);
        }

        if (!mCertainRegion.empty())
        {
            cv::rectangle(img, mCertainRegion, COLOR_CV_GREEN, 3);
        }
    }

    ////Define parameters
    //int mEndDashLine = GetROI().empty() ? img.rows : GetROI().br().y;

    cv::Scalar mCathodeColor = COLOR_CV_DARKYELLOW;

    if (img.empty())
        return;

    pen.mFontFace = cv::FONT_HERSHEY_SIMPLEX;
    pen.mFontScale = 0.5;
    pen.mColor = mCathodeColor;
    pen.mThickness = 1;
    cv::Point tl = GetROI().tl();
    for (int i = 0; i < mCathodePoleResult.mPoles.size(); i++)
    {
        xvt::DrawText(img, std::to_string(i + 1), mCathodePoleResult.mPoles[i].mAnode, TextAlign::MIDDLE_LEFT, pen, cv::Point(10, 0));
        /*xvt::Drawing::DrawDashedLine(img,
                                     cv::Point(mCathodePoleResult.mPoles[i].mAnode.x + tl.x, mEndDashLine),
                                     mCathodePoleResult.mPoles[i].mAnode + tl,
                                     mCathodeColor,
                                     1,
                                     "",
                                     10);*/
        // Draw the point on the image
        //cv::circle(img, mCathodePoleResult.mPoles[i].mAnode - cv::Point(0,-5), 2, pen.mColor, 3);
        //cv::circle(img, mCathodePoleResult.mPoles[i].mAnodeOld, 2, COLOR_CV_GREEN, 3);
        cv::line(img, mCathodePoleResult.mPoles[i].mAnode + cv::Point(0, 5), mCathodePoleResult.mPoles[i].mAnode - cv::Point(0, 5),pen.mColor, 3);
    }
    int poleUncertain = 0;
    //Draw result Anode
    for (int i = 0; i < mAnodePoleResult.mPoles.size(); i++)
    {
        if (mAnodePoleResult.mPoles[i].mType == PoleType::Incorrect){
            pen.mColor = COLOR_CV_ORANGE;
            xvt::DrawText(img,
                std::to_string(i + 1),
                mAnodePoleResult.mPoles[i].mAnode,
                TextAlign::MIDDLE_RIGHT,
                pen,
                cv::Point(-10, 0));
            xvt::Drawing::DrawDashedLine(img, mAnodePoleResult.mPoles[i].mCathode, mAnodePoleResult.mPoles[i].mAnode, pen.mColor, 1, "", 1);
            cv::circle(img, mAnodePoleResult.mPoles[i].mAnode, 2, pen.mColor, 3);
            cv::circle(img, mAnodePoleResult.mPoles[i].mCathode, 2, pen.mColor, 3);
      
        }
        else
        {
            pen.mColor = mAnodePoleResult.mPoles[i].IsOK() ? COLOR_CV_CYAN : COLOR_CV_RED;

            xvt::DrawText(img,
                std::to_string(i + 1),
                mAnodePoleResult.mPoles[i].mAnode,
                TextAlign::MIDDLE_RIGHT,
                pen,
                cv::Point(-10, 0));
            xvt::Drawing::DrawDashedLine(img, mAnodePoleResult.mPoles[i].mCathode, mAnodePoleResult.mPoles[i].mAnode, pen.mColor, 1, "", 1);
            cv::circle(img, mAnodePoleResult.mPoles[i].mAnode, 2, pen.mColor, 3);
            cv::circle(img, mAnodePoleResult.mPoles[i].mCathode, 2, pen.mColor, 3);
        }
    }
    //mAnodePoleResult.mPoles.;
    auto stringPos = cv::Point(100 + offSetPoint.x, 100 + offSetPoint.y);
    //Draw the message
    DrawResultStr(img, "", CVPen(), stringPos);
#if 1
    auto procesText = "Total: " + std::to_string((int)mProTime) + ", B: " + std::to_string((int)mBaseResult.mProTime) +
                      ", P: " + std::to_string((int)mCathodeResult.mProTime) + ", C: " + std::to_string((int)mCathodePoleResult.mProTime) +
                      ", N: " + std::to_string((int)mAnodeResult.mProTime) + "(ms)";
    auto drawPos = cv::Point(std::min(img.cols - 1, 100), std::max(0, img.rows - 100));
    auto color = img.at<cv::Vec3b>(drawPos);
    pen.mColor = COLOR_CV_GRAY(255 - color(0));
    DrawText(img, procesText, drawPos, pen);
#endif // _DEBUG
    //img = img(mAnodeResult.GetROI());
}

auto JRBatteryInspectResult::DrawResultStr(cv::Mat &image,
                                           std::string const &name,
                                           CVPen const &pen,
                                           cv::Point const &offset,
                                           bool isDrawOKResult) const -> cv::Point
{
    auto tmpPen = pen;
    tmpPen.mFontScale = 2;
    tmpPen.mThickness = 2;
    tmpPen.mSpace = 10;
    auto startPos = offset;
    auto txtPos = InspectionResult::DrawResultStr(image, name, tmpPen, startPos, 1);
    tmpPen.mFontScale = 1.5;
    //Draw Cathode Result
    txtPos = mCathodeResult.DrawResultStr(image, "Insp Cat", tmpPen, txtPos, false);
    //Draw Anode Result
    txtPos = mAnodeResult.DrawResultStr(image, "Insp Ano", tmpPen, txtPos, false);

    tmpPen.mColor = COLOR_CV_CYAN;
    txtPos = xvt::DrawText(image, "No. Negative: " + std::to_string(mAnodePoleResult.Size()), cv::Point(tmpPen.GetTabSize(), txtPos.y + 50), tmpPen);

    tmpPen.mColor = COLOR_CV_DARKYELLOW;
    txtPos = xvt::DrawText(image, "No. Positive: " + std::to_string(mCathodePoleResult.Size()), cv::Point(tmpPen.GetTabSize(), txtPos.y + 50), tmpPen);

    tmpPen.mSpace = 60;
    txtPos = mAnodePoleResult.DrawResultStr(image, "", tmpPen, cv::Point(tmpPen.GetTabSize() / 2, txtPos.y + 200), 1);

    return cv::Point(offset.x, txtPos.y);
}

auto JRBatteryInspectResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
    if(isRecursive)
    {
        out.emplace_back("N1", xvt::ToString((int)mAnodePoleResult.Size()));
        out.emplace_back("N2", xvt::ToString((int)mCathodePoleResult.Size()));
    
        const auto& listLenght = mAnodePoleResult.GetLenghts();
        for (int i = 0; i < 25; i++)
        {
            float leght = (i < mAnodePoleResult.Size()) ? listLenght[i] : -1.0f;
            out.emplace_back("NP" + std::to_string(i), xvt::ToString(leght, 3));
        }
    }
}
} // namespace battery
} // namespace xvt