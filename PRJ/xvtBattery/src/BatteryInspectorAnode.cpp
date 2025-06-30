#include "xvtBattery/BatteryInspectorAnode.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtBattery/PoleInfo.h"
#include "xvtCV/Peak.h"
#include "xvtCV/Utils.h"
#include <xvtCV/ColorDefine.h>
#include <CvPlot/cvplot.h>
namespace xvt
{
namespace battery
{
auto BatteryInspectorAnode::Inspect(cv::Mat const &inImg, cv::Rect roi, std::vector<PoleInfo> &poles) const -> BatteryInspectorAnodeResult
{
    auto ispResult = BatteryInspectorAnodeResult();

    ispResult.mEnable = mEnable;

    if (!ispResult.mEnable)
    {
        ispResult(EResult::UC, "Disabled!");
        return ispResult;
    }

    cv::Rect subRoi = CreateROI(roi.tl(), roi.size(), inImg.size());
    cv::Mat src;
    ispResult.SetROI(subRoi);
    if (!xvt::Convert8Bits(inImg, src, false) && !subRoi.empty())
    {
        // 5: Applied Local Thresholding for determine the ending Point of Anode Poles
        cv::Mat imgROI4_0, imgCLAHE;
        cv::bilateralFilter(src(subRoi), imgROI4_0, 5, 55, 5);

        if (mIsApplyEnhanced)
        {
            cv::Ptr<cv::CLAHE> claheTool4Cathode = cv::createCLAHE(15, cv::Size(75, imgROI4_0.rows));
            claheTool4Cathode->apply(imgROI4_0, imgCLAHE);
        }
        else
        {
            imgCLAHE = imgROI4_0;
        }

        auto findAnodeResult = FindAnode(imgCLAHE, poles);
        ispResult.CombineResult(&findAnodeResult);
    }
    else
    {
        ispResult(EResult::ER, "Inspect: Image type is not supported!");
    }
    return ispResult;
}

auto BatteryInspectorAnode::FindAnode(const cv::Mat &inputImg, std::vector<PoleInfo> &poles) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!inputImg.empty() && !poles.empty())
    {
        int anodeYPos;
        int stepHorizontal = 3;
        int stepVertical = 5;
        double aThreshold = 0.0;
        int moveAlow = 3;
        anodeAlgoType algoType = mEnableAutoMode ? anodeAlgoType::Projection : anodeAlgoType::LineTracing;
        if (mAnodeThresholdInner == 0 && mAnodeThresholdMiddle == 0 && mAnodeThresholdOuter == 0)
        {
            algoType = anodeAlgoType::Edge;
        }

        size_t size = poles.size();
        for (int poleIdx = 0; poleIdx < size; poleIdx++)
        {

            auto &polePos = poles[poleIdx];
            const auto &startingPos = polePos.mCathode;
            cv::Point ending;
            switch (algoType)
            {
            case xvt::battery::anodeAlgoType::LineTracing:
            {
                int regionIdx = CheckRegion(startingPos.x, 6, inputImg.cols, GetJR().mCenterNeglectionWidth);
                switch (regionIdx)
                {
                case 1:
                    aThreshold = mAnodeThresholdOuter;
                    break;
                case 2:
                    aThreshold = mAnodeThresholdMiddle;
                    break;
                case 3:
                    aThreshold = mAnodeThresholdInner;
                    break;
                default:
                    aThreshold = mAnodeThresholdMiddle;
                }

                // 12: For each pole position, select the sub-Roi area (ROI_P1)
                // containing only Anode this pole (based on poles position in step and Anode end point in step)
                //BORDER CHECK
                int isBorder = 0;
                if (poleIdx == 0 /*|| poleIdx == 1 || poleIdx == 2*/)
                {
                    isBorder = -1;
                }
                else if (poleIdx == size - 1 /*|| poleIdx == lstPoleSize-2 || poleIdx == lstPoleSize -3*/)
                {
                    isBorder = 1;
                }

                bool isRestrictMove = (regionIdx == 1);

                auto anodeResult = FindAnodeByLineTracing(inputImg,
                                                          anodeYPos,
                                                          startingPos,
                                                          startingPos.y,
                                                          stepHorizontal,
                                                          stepVertical,
                                                          aThreshold * 10,
                                                          isBorder,
                                                          GetPole().mPolesDistanceRange.GetLower(),
                                                          isRestrictMove,
                                                          moveAlow);
            }
            break;
            case xvt::battery::anodeAlgoType::Edge:
            {
                auto anodeResult =
                    FindAnodeAutoByEdge(inputImg, anodeYPos, startingPos, startingPos.y, stepHorizontal, stepVertical, polePos.mPoints, ending);
                break;
            }
            case xvt::battery::anodeAlgoType::Projection:
            {
                cv::Mat processImage = cv::Mat::zeros(inputImg.size(), 0);
                cv::blur(inputImg, processImage, cv::Size(3, 1));
                auto anodeResult = IsPoleAnode(processImage, startingPos, anodeYPos, 15, 10);
                break;
            }
            case xvt::battery::anodeAlgoType::Kmean: 
            {

            }
            default:
                anodeYPos = startingPos.y;
                break;
            }

            anodeYPos = std::max(0, std::min(anodeYPos, startingPos.y));
            polePos.mAnode = cv::Point(startingPos.x, anodeYPos);
        }
        finalResult(EResult::OK, "");
    }
    else
    {
        finalResult(EResult::ER, "");
    }

    return finalResult;
}

auto BatteryInspectorAnode::FindAnodeByLineTracing(const cv::Mat &inputImg,
                                                   int &anode,
                                                   cv::Point startPoint,
                                                   int defaultVal,
                                                   int stepHorizontal,
                                                   int stepVertical,
                                                   float breakThreshold,
                                                   int borderCheck,
                                                   int borderDistance,
                                                   bool restrictMove,
                                                   int moveAllow) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!inputImg.empty())
    {

        int stepSkip = 1;
        int stepFrog = 1;
        int stopCheck = stepVertical + stepFrog + 1;
        int borderOffset = 1;

        cv::Rect moveRect;

        moveRect.width = 2 * stepHorizontal + 1;
        moveRect.height = stepVertical + 1;

        int const &startPointX = startPoint.x;
        int const &startPointY = startPoint.y;

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

        cv::Point currentPoint(startPointX, startPointY);
        int outputYAnode = defaultVal;
        int numberOfBlankLine = 5;

        int stepCheck = 0;
        int imgColsHalf = inputImg.cols / 2;
        cv::Mat reduceRes;
        //LINE TRACING FROM STARTING POINT TO END OF ROI //
        int it = 0;
        int leftBorderX = startPointX - borderOffset;  //when checking the left border pole
        int rightBorderX = startPointX + borderOffset; //when checking the right border pole

        int leftLimit = stepHorizontal;
        int rightLimit = inputImg.cols - stepHorizontal;

        assert(leftLimit > 0 && rightLimit > 0);

        for (it = currentPoint.y - stepSkip; it > stopCheck; it -= stepFrog)
        {
            int currentPointXCord = currentPoint.x;

            moveRect.x = currentPointXCord - stepHorizontal;
            moveRect.y = it - stepVertical;
            moveRect.width = 2 * stepHorizontal + 1;
            if (borderCheck == -1)
            {
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

                auto minPos = std::min_element((float *)reduceRes.ptr(0), (float *)reduceRes.ptr(0) + reduceRes.cols);
                avgVal = *minPos;
                currentPoint = cv::Point(moveRect.x + std::distance((float *)reduceRes.ptr(0), minPos), it);

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

                if (avgVal > breakThreshold)
                {
                    stepCheck++;
                    if (stepCheck >= numberOfBlankLine)
                    {
                        break;
                    }
                }
                else
                {
                    stepCheck = 0;
                }
            }
            else // Move out of image
            {
                //Use the currentPoint as the output
                break;
            }
        }
        //---------END LINE TRACING --------------------------//

        reduceRes.release();
        anode = currentPoint.y + stepCheck * stepFrog;
        finalResult(EResult::OK, "");
    }
    else
    {
        finalResult(EResult::ER, "");
    }
    return finalResult;
}

// Function to normalize a vector
VecFloat normalizeVector(const VecFloat& vec) {   
    float magnitude = 0.0;
    for (float val : vec) {
        magnitude += val * val;
    }
    magnitude = std::sqrt(magnitude);
    if (magnitude < 1e-6) {
        std::cerr << "Vector magnitude is too small to normalize." << std::endl;
        return vec;
    }
    VecFloat normalizedVec;
    for (float val : vec) {
        normalizedVec.push_back(val * 100 / magnitude);
    }
    return normalizedVec;
}
//#define DEBUG_PLOT
auto BatteryInspectorAnode::IsPoleAnode(const cv::Mat &inputImg, cv::Point polePos, int &endPos, int stepHorizontal, int stepVertical) const -> InspectionResult
{
    InspectionResult finalResult;
    if (!inputImg.empty() && (stepHorizontal > 0 && stepVertical > 0))
    {
        cv::Point sRectPole = cv::Point(polePos.x - stepHorizontal, 0);
        cv::Size sRangeRect = cv::Size(2* stepHorizontal, polePos.y);
        cv::Rect rectPole = CreateROI(sRectPole, sRangeRect, inputImg.size());
        cv::Mat poleImg = inputImg(rectPole);

        cv::Mat verticalProjectionImg;
        cv::reduce(poleImg, verticalProjectionImg, 1, cv::REDUCE_AVG, CV_32FC1);
        VecFloat listVerticalProjection = cv::Mat_<float>(verticalProjectionImg);
        VecFloat listVerticalProjectionGauss;
        cv::GaussianBlur(listVerticalProjection, listVerticalProjectionGauss, cv::Size(7, 1), 0);

        int imRange = 3;
        int step = 3;
        int startCompare = 0;
        int countStart = 0;
        int countEnd = 0;

        float transThresStart = 4.0 ;
        float transThresEnd = 1.0 ;
        float thresVali = 0.0;
        
        VecFloat listDevVerPro(listVerticalProjectionGauss.size(), 0.0);
        int size = listVerticalProjectionGauss.size() - step;

        for (int i = 0; i < size; i++) 
        {
            float dev = 0;
            for (int k = 1; k <= step; k++) 
            {
                dev += listVerticalProjectionGauss[i] - listVerticalProjectionGauss[i + k];
            }
            dev /= (float)(2*step);
            listDevVerPro[i] = dev;
            if (dev > transThresStart && countStart == 0) {
                countStart++;
                startCompare = i;
            }
            if (dev < transThresEnd) {
                countEnd ++;
            }
            else{
                countEnd = 0;
            }
        }

        int endCompare = (countEnd > stepVertical) ? (poleImg.rows - countEnd + (int)(stepVertical)) : (poleImg.rows - stepVertical);
        startCompare = ((startCompare - stepVertical) > 0) ? (startCompare - (int)(stepVertical)) : 0;
        VecFloat listDiffHorPro(poleImg.rows, 0.0);
        VecFloat diffMaxMinVec;
#ifdef DEBUG_PLOT
        if (polePos.x > 495) {
            int x = 0;
        }
#endif // DEBUG_PLOT
        std::vector<cv::Point2f> checkValidation(poleImg.rows, cv::Point2f(0.0,0.0));
        for (int o = endCompare; o > startCompare; o--) {
          
            cv::Point sRectCheckPole = cv::Point(polePos.x - stepHorizontal, o);
            cv::Size sRangeCheckRect = cv::Size(2 * stepHorizontal, stepVertical);
            cv::Rect rectCheckPole = CreateROI(sRectCheckPole, sRangeCheckRect, inputImg.size());
            cv::Mat imgCheck = inputImg(rectCheckPole);
            VecFloat tempCheckReduceVec;
            cv::reduce(imgCheck, tempCheckReduceVec, 0, cv::REDUCE_AVG, CV_32FC1);
            cv::GaussianBlur(tempCheckReduceVec, tempCheckReduceVec, cv::Size(3, 1), 0);
            cv::Point2f valiPoint(0.0,0.0);

            xvt::FindPeaks findPeaks = xvt::FindPeaks(xvt::PeakType::Valley, xvt::PeakFindingMethod::Prominence);
            findPeaks.Process(tempCheckReduceVec);
            auto peakResult = findPeaks.GetPeakResult();
            
            for (auto pea : peakResult){
                if (pea.index <= stepHorizontal + imRange && pea.index >= stepHorizontal - imRange)
                {
                    valiPoint.x += 1.0;
                    if (pea.prominence < valiPoint.y)
                    {
                        valiPoint.y = pea.prominence*100;
                    }
                }
            }
            checkValidation[o] = (valiPoint);
#ifdef DEBUG_PLOT
            cv::Mat mat(600, 1200, CV_8UC3);
            mat.setTo(cv::Scalar(255, 255, 255));

            CvPlot::Axes axes = CvPlot::makePlotAxes();
            axes.enableHorizontalGrid();
            axes.enableVerticalGrid();
            axes.title("ROI");

            axes.create<CvPlot::Series>(tempCheckReduceVec, "-r");

            int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
            axes.setMargins(borderLeft, borderRight, borderTop, borderBottom);
            axes.setXLim(std::pair<int, int>(0, 40));
            axes.setYLim(std::pair<int, int>(100, 120));
            axes.setXTight(true);
            axes.render(mat);
#endif // DEBUG_PLOT
        }
        VecFloat vector(poleImg.rows, 0.0);
        int endLoop = (poleImg.rows - 6 < (endCompare)) ? (poleImg.rows - 6) : endCompare;
        for(int i = startCompare + 1; i < endLoop ; i ++)
        {
            vector[i + 1] = checkValidation[i].y - checkValidation[i + 6].y;
        }
#ifdef DEBUG_PLOT
        cv::Mat matDiff(600, 1200, CV_8UC3);
        matDiff.setTo(cv::Scalar(255, 255, 255));

        CvPlot::Axes axesDiff = CvPlot::makePlotAxes();
        axesDiff.enableHorizontalGrid();
        axesDiff.enableVerticalGrid();
        axesDiff.title("ROI");

        axesDiff.create<CvPlot::Series>(listVerticalProjection, "-r");
        axesDiff.create<CvPlot::Series>(listVerticalProjectionGauss, "-b");
        //axesDiff.create<CvPlot::Series>(vector, "-or");
        //axesDiff.create<CvPlot::Series>(listDiffHorPro, "-or");
        axesDiff.create<CvPlot::Series>(listDevVerPro, "-og");
        //axesDiff.create<CvPlot::Series>(result, "-ob");

        int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
        axesDiff.setMargins(borderLeft, borderRight, borderTop, borderBottom);
        axesDiff.setXLim(std::pair<int, int>(0, listVerticalProjection.size()));
        axesDiff.setYLim(std::pair<int, int>(0, 255));
        axesDiff.setXTight(true);
        axesDiff.render(matDiff);
#endif // DEBUG_PLOT
        float maxChange = vector[endCompare];
        for (int i = endCompare; i > startCompare; i--) {
            if (checkValidation[i].y > thresVali) {
                //break;
            }
            else {
                if (vector[i] >= maxChange) {
                    maxChange = vector[i];
                    endPos = i + stepVertical;
                }
            }
        }

    }
    else 
    {
        finalResult(EResult::ER, "Invalid Image");
    }
    return finalResult;
}


auto BatteryInspectorAnode::FindAnodeAutoByEdge(const cv::Mat &inputImg,
                                                int &anode,
                                                cv::Point startPoint,
                                                int defaultVal,
                                                int stepHorizontal,
                                                int stepVertical,
                                                VecPoint &vtPoint,
                                                cv::Point &ending) const -> InspectionResult
{
    anode = defaultVal;
    InspectionResult finalResult;
    bool isValidStartPoint = (startPoint.x >= 0 && startPoint.x < inputImg.cols) && (startPoint.y > 0 && startPoint.y < inputImg.rows);
    if (!inputImg.empty() && isValidStartPoint && (stepHorizontal > 0 && stepVertical > 0))
    {
        std::vector<pointValue> listMinimumIntensity;
        cv::Mat sumAverageImage = cv::Mat::zeros(inputImg.size(), 0);
        int numberblank = 1;
        float weightSumDiff = 0.4;
        float weightSumCount = 0.6;
        cv::blur(inputImg, sumAverageImage, cv::Size(1, stepVertical));
        listMinimumIntensity.emplace_back(sumAverageImage.at<uchar>(startPoint), startPoint);
        cv::Point startingForLoop = cv::Point(startPoint.x, startPoint.y  + 6);
        //Find all minimum points from startpoint which window 1x7
        cv::Point localMinimumPoint = startingForLoop;

        for (int i = startingForLoop.y; i >= 0; i--)
        {
            //// Used when searching for critical points
            //int startIdx = std::max(localMinimumPoint.x - stepHorizontal, 0);
            //int endIdx = std::min(localMinimumPoint.x + stepHorizontal, inputImg.cols - 1);
            int startIdx = std::max(startingForLoop.x - stepHorizontal, 0);
            int endIdx = std::min(startingForLoop.x + stepHorizontal, inputImg.cols - 1);
            VecInt vtRow = cv::Mat_<uchar>(sumAverageImage.row(i));
            /*auto vtRow = (sumAverageImage.row(i).data);*/
            auto minIntensity = std::min_element(vtRow.begin()  + startIdx, vtRow.begin() + endIdx, [](const int &a, const int &b) { return a < b; });
            localMinimumPoint = cv::Point(std::distance(vtRow.begin(), minIntensity), i);

            listMinimumIntensity.emplace_back(*minIntensity, localMinimumPoint);
        }

        //Find average intensity of 5 first
        int sum = 0;
        int limitSize = numberblank * 5;
        if (listMinimumIntensity.size() > limitSize)
        {
            for (int i = 0; i < limitSize; i++)
            {
                sum = sum + listMinimumIntensity[i].intensity;
            }
        }
        int averageSum = sum / (float)limitSize + 0.5;
        int thresholdInten = averageSum * 1.5;

        std::reverse(listMinimumIntensity.begin(), listMinimumIntensity.end());
        //Put Derivative values into listIntenDev list and put counting derivative value into listCountIntenDev
        int pos = -1;
        //VecInt listIntensityAcc;
        VecInt listIntenDev;
        int sumaccumulate;
        if (listMinimumIntensity.size() > 1)
        {
            listIntenDev.emplace_back(-1);
            int size = listMinimumIntensity.size() - 1;

            for (int i = 1; i < size; i++)
            {
                int diff = listMinimumIntensity[i].intensity - listMinimumIntensity[i + 1].intensity;
                listIntenDev.emplace_back(diff);
            }

            size = listIntenDev.size() - 1;
            for (int i = 1; i < size; i++)
            {
                int pre = i - 1;
                if (listIntenDev[i] <= 1 && listIntenDev[pre] > 1 && listIntenDev[i + 1] > 1)
                {
                    listIntenDev[i] = listIntenDev[pre];
                }
            }
            listIntenDev.emplace_back(-1);

            VecInt listCountIntenDev(listIntenDev.size(), 0);
            for (int i = listIntenDev.size() - 2; i > 1; i--)
            {
                int pre = i - 1;
                int next = i + 1;
                if (listIntenDev[i] > 1 && listIntenDev[pre] > 1 && listIntenDev[next] <= 1)
                {
                    listCountIntenDev[i] = 1;
                }
                else if ((listIntenDev[i] > 1 && listIntenDev[pre] > 1 && listIntenDev[next] > 1) ||
                         (listIntenDev[i] > 1 && listIntenDev[pre] <= 1 && listIntenDev[next] > 1))
                {
                    listCountIntenDev[i] = listCountIntenDev[next] + 1;
                }
            }
            //Putting sumDiff, the center position, the first position and weighted value of each cluster into listWeightDiff
            std::vector<candiEdP> listWeightDiff;
            int posMax = -1;
            for (int i = 1; i < size; i++)
            {
                if (0 == listCountIntenDev[i - 1] && listCountIntenDev[i] >= 1 && listCountIntenDev[i + 1] >= 1)
                {
                    int sumDiff = 0;
                    for (int j = listCountIntenDev[i]; j > 0; j--)
                    {
                        sumDiff = sumDiff + listIntenDev[i + listCountIntenDev[i] - j];
                    }
                    int mean = sumDiff / listCountIntenDev[i];
                    int var = 0;
                    for (int j = listCountIntenDev[i]; j > 0; j--)
                    {
                        var += (listIntenDev[i + listCountIntenDev[i] - j] - mean) * (listIntenDev[i + listCountIntenDev[i] - j] - mean);
                    }
                    var /= listCountIntenDev[i];

                    //int std = var * var;
                    int WeightValue = (sumDiff * weightSumDiff) - (listCountIntenDev[i] * weightSumCount);
                    candiEdP t = { sumDiff, (i + (i + listCountIntenDev[i])) / 2, i + 2, listCountIntenDev[i], WeightValue, var };
                   listWeightDiff.emplace_back(t);  
                }
            }
#ifdef DEBUG_PLOT
            VecPoint pointsIns;
            cv::Mat mat1(600, 1200, CV_8UC3);
            mat1.setTo(cv::Scalar(255, 255, 255));

            CvPlot::Axes axes1 = CvPlot::makePlotAxes();
            axes1.enableHorizontalGrid();
            axes1.enableVerticalGrid();
            axes1.title("Image Intensity");

            for (auto listIns : listMinimumIntensity) {
                cv::Point point = cv::Point(listIns.position.y, listIns.intensity);
                pointsIns.emplace_back(point);
            }
            axes1.create<CvPlot::Series>(pointsIns, "-r");
            axes1.create<CvPlot::Series>(listIntenDev, "-b");
            axes1.create<CvPlot::Series>(listCountIntenDev, "-g");
           /* for (auto peak : peaks) {
                axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ {(float)peak.index, tempReduceVec[peak.index] }, { (float)peak.index, tempReduceVec[peak.index] }}, "-ob");
            }
            if (maxIndex > 0) {
                axes.create<CvPlot::Series>(std::vector<cv::Point2f>{ {(float)maxIndex, tempReduceVec[maxIndex] }, { (float)maxIndex, tempReduceVec[maxIndex] }}, "-og");
            }*/

            int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
            axes1.setMargins(borderLeft, borderRight, borderTop, borderBottom);
            axes1.setXLim(std::pair<int, int>(0, pointsIns.size()));
            axes1.setYLim(std::pair<int, int>(0, 255));
            axes1.setXTight(true);
            axes1.render(mat1);

#endif // DEBUG_PLOT

            if (0 != listWeightDiff.size())
            {
                //Check condition for each element in listWeightDiff, if sum average intensity 5 concecutive above first position - sum average intensity 5 concecutive above first position >0 -> stop at this point
                std::sort(listWeightDiff.begin(), listWeightDiff.end(), [](auto &left, auto &right) { return left.weightedSum > right.weightedSum; });
                for (int i = 0, size = listWeightDiff.size(); i < size; i++)
                {

                    int sumAverageCurrent = 0;
                    for (int k = 0, ksize = listWeightDiff[i].countDerivative - 1; k < ksize; k++)
                    {
                        sumAverageCurrent = sumAverageCurrent + listMinimumIntensity[listWeightDiff[i].firstPos + k].intensity;
                    }
                    int lastElement = listWeightDiff[i].firstPos + listWeightDiff[i].countDerivative;
                    int sumNextLast = 0;
                    if (lastElement + (numberblank * 5) < listCountIntenDev.size())
                    {
                        for (int i = lastElement, isize = lastElement + (numberblank * 5); i < isize; i++)
                        {
                            sumNextLast = sumNextLast + listMinimumIntensity[i].intensity;
                        }
                    }
                    int averageNextLast = sumNextLast / (numberblank * 5);

                    sumAverageCurrent = sumAverageCurrent / (listWeightDiff[i].countDerivative - 1);
                    bool stop = false;
                    if (averageNextLast <= thresholdInten)
                    {
                        int sum = 0;
                        int count = 0;
                        int sumaverage = 0;
                        for (int j = listWeightDiff[i].firstPos / 2, jsize = listWeightDiff[i].firstPos - 1; j < jsize; j++)
                        {
                            sum = sum + listMinimumIntensity[j].intensity;
                            count++;
                        }
                        sumaverage = count != 0 ? sum / count : -1;
                        if (sumaverage - sumAverageCurrent > 0)
                        {
                            stop = true;
                        }
                    }
                    if (stop == true)
                    {
                        posMax = listWeightDiff[i].position;
                        break;
                    }
                }
                if (-1 != posMax)
                {
                    int offset = 5;
                    if (listMinimumIntensity[posMax].position.y + offset < inputImg.rows)
                    {
                        anode = listMinimumIntensity[posMax].position.y + offset;
                        ending = listMinimumIntensity[posMax].position;
                    }
                }
                finalResult(EResult::OK, "");

                for (auto &iPoint : listMinimumIntensity)
                    vtPoint.emplace_back(iPoint.position);
            }
            else
            {
                finalResult(EResult::ER, "FindAnodeAutoByEdge: No List Weight Diff");
            }

        }
        else
        {
            finalResult(EResult::ER, "FindAnodeAutoByEdge: No List MinimumIntensity");
        }
    }
    else
    {
        finalResult(EResult::ER, "FindAnodeAutoByEdge: Invalid Image");
    }
    return finalResult;
}

void BatteryInspectorAnodeResult::DrawResult(cv::Mat &img, cv::Point offSetPoint, CVPen pen) const
{
    if (img.empty())
        return;
    if (img.channels() != 3 && !xvt::Convert8Bits(img, img))
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    /*cv::Mat drawImg = img(GetROI());
    if (!mPoints.empty())
        xvt::DrawPoints(drawImg, mPoints, cv::Vec3b(0, 0, 200), offSetPoint);*/

    if (!GetROI().empty())
        cv::rectangle(img, GetROI(), cv::Scalar(120, 120, 255), 2);
}

auto BatteryInspectorAnodeResult::DrawResultStr(cv::Mat &image,
                                                std::string const &name,
                                                CVPen const &pen,
                                                cv::Point const &offset,
                                                bool isDrawOKResult) const -> cv::Point
{
    auto tmpPos = InspectionResult::DrawResultStr(image, name, pen, offset, isDrawOKResult);

    return cv::Point(offset.x, tmpPos.y);
}

auto BatteryInspectorAnodeResult::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
}

} // namespace battery
} // namespace xvt