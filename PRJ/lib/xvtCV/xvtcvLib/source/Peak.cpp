#include "xvtCV/Peak.h"
#include "xvtCV/ColorDefine.h"
#include <numeric>
namespace xvt {

FindPeaks::FindPeaks()
{
    mSignal = {};
    mPeakType = PeakType::Peak;
    mPeakFindingMethod = PeakFindingMethod::Prominence;
    mPeakList = {};
    mPeriod = 0;
    mIncludeEndPoints = false;
}

FindPeaks::FindPeaks(  PeakType peakType
                     , PeakFindingMethod findPeaksMethod
                     , int period
                     , bool includeEndPoints
)
{
    mSignal = {};
    mPeakList = {};
    SetPeakType(peakType);
    SetPeakFindingMethod(findPeaksMethod);
    SetPeriod(period);
    SetIncludeEndPoints(includeEndPoints);
}

void FindPeaks::Process(const std::vector<float>& signal)
{
    mSignal = signal;
    if (mPeakList.size() > 0)
    {
        mPeakList.clear();
    }

    int size = (int)signal.size();
    if (size > 0)
    {
        /* https://www.mathworks.com/help/signal/ref/findpeaks.html */
        /*Step 1: Place a marker on the peak*/
        std::vector<Peak> peakMarkerList, subPeakMarkerList;
        if (mPeakType == PeakType::Peak)
        {
            this->FindPeaksCandidate(peakMarkerList, subPeakMarkerList);
        }
        else if (mPeakType == PeakType::Valley)
        {
            this->FindPeaksCandidate(subPeakMarkerList, peakMarkerList);
        }
        else if (mPeakType == PeakType::Both)
        {
            throw ("Not support PeakType Both");
        }

        int subPeakMarkerListSize = (int)subPeakMarkerList.size();
        int firstSubPeakIdx = 0;
        for (auto peak : peakMarkerList)
        {
            /*Determine the closest subPeak on the left and right*/
            Peak leftClosestSubPeak{ 0, signal[0], 0 }; //Init with left endpoints
            Peak rightClosestSubPeak{ size - 1, signal[static_cast<int64>(size) - 1], 0 }; //Init with right endpoints
            int tmpId = subPeakMarkerListSize - 1;

            /*Determine the closest subPeak on right*/
            for (int i = firstSubPeakIdx; i < subPeakMarkerListSize; i++)
            {
                if (subPeakMarkerList[i].index > peak.index)
                {
                    rightClosestSubPeak = subPeakMarkerList[i];
                    tmpId = i;
                    break;
                }
            }

            /*Determine the closest subPeak on the left*/
            for (int i = tmpId; i >= firstSubPeakIdx; i--)
            {
                if (subPeakMarkerList[i].index < peak.index)
                {
                    leftClosestSubPeak = subPeakMarkerList[i];
                    firstSubPeakIdx = i;
                    break;
                }
            }

            /*
            Step 2: Extend a horizontal line from the peak to the left and right until the line does one of the following:
            - Crosses the signal because there is a higher peak
            - Reaches the left or right end of the signal */
            std::pair<int, int> twoIntervals = this->FindTwoIntervals(peak, leftClosestSubPeak, rightClosestSubPeak);
            /*
            Step 3: Find the minimum of the signal in each of the two intervals defined in Step 2. This point is either a valley or one of the signal endpoints.
            */
            /*
             Step 4: The higher of the two interval minima specifies the reference level. The height of the peak above this level is its prominence.
            */
            std::pair<int, float> higherMinimumIntervalPoint = this->FindKeyCol(peak, twoIntervals, leftClosestSubPeak, rightClosestSubPeak);
            peak.prominence = (higherMinimumIntervalPoint.first >= 0 && higherMinimumIntervalPoint.first <= size) ? peak.value - higherMinimumIntervalPoint.second : 0;

            mPeakList.push_back(peak);
        }
    }
}

void FindPeaks::SetPeakType(PeakType peakType)
{
    if (peakType == PeakType::Valley || peakType == PeakType::Peak)
    {
        this->mPeakType = peakType;
    }
    else
    {
        throw ("Not support PeakType Both or unknown PeakType");
    }
}

void FindPeaks::SetPeakFindingMethod(PeakFindingMethod peakFindingMethod)
{
    if (peakFindingMethod == PeakFindingMethod::Prominence || peakFindingMethod == PeakFindingMethod::RelativeHeight)
    {
        this->mPeakFindingMethod = peakFindingMethod;
    }
    else
    {
        throw ("Unknown Peak Finding Method");
    }
}

void FindPeaks::SetPeriod(int period)
{
    if (period < 0)
    {
        this->mPeriod = 0;
    }
    else
    {
        this->mPeriod = period;
    }
}

void FindPeaks::SetIncludeEndPoints(bool includeEndPoints)
{
    this->mIncludeEndPoints = includeEndPoints;
}

auto FindPeaks::GetPeakType() const -> PeakType
{
    return mPeakType;
}

auto FindPeaks::GetPeakFindingMethod() const -> PeakFindingMethod
{
    return mPeakFindingMethod;
}

auto FindPeaks::GetPeriod() const->int
{
    return mPeriod;
}

auto FindPeaks::GetPeakResult(  float minProminence
                              , float minDistance
                              , float peakRefinement
) const->std::vector<Peak>
{
    std::vector<Peak> filteredPeakList = mPeakList;
    this->RemoveLowProminencePeaks(filteredPeakList, minProminence);
    this->RemoveClosePeaks(filteredPeakList, minDistance);
    this->RefinePeaks(filteredPeakList, peakRefinement);
#ifdef DEBUG_FIND_PEAK_BY_PROMINENCE
    DrawGraph(filteredPeakList);
#endif
    return filteredPeakList;
}

bool FindPeaks::GetIncludeEndPoints() const
{
    return mIncludeEndPoints;
}

void FindPeaks::FindPeaksCandidate(  std::vector<Peak>& peakCandidatesList
                                   , std::vector<Peak>& valleyCandidatesList
) const
{
    int size = (int)mSignal.size();
    if (size > 2)
    {
        int endIdx = size - 1;
        peakCandidatesList.clear();
        valleyCandidatesList.clear();
        float diffLeft1, diffRight1;
        Peak t;
        //Add the starting point of the list
        if (mIncludeEndPoints)
        {
            t = { 0,mSignal[0],0 };
            diffLeft1 = mSignal[1] - mSignal[0];
            if (diffLeft1 > 0)
            {
                valleyCandidatesList.push_back(t);
            }

            if (diffLeft1 < 0)
            {
                peakCandidatesList.push_back(t);
            }
        }

        for (int i = 1; i < endIdx; i++)
        {
            diffLeft1 = mSignal[static_cast<int64>(i) - 1] - mSignal[i];
            diffRight1 = mSignal[static_cast<int64>(i) + 1] - mSignal[i];

            if (diffLeft1 != 0 || diffRight1 != 0)
            {
                t = { i, mSignal[i], 0 };
                if (diffLeft1 <= 0 && diffRight1 <= 0)
                {
                    peakCandidatesList.push_back(t);
                }
                else if (diffLeft1 >= 0 && diffRight1 >= 0)
                {
                    valleyCandidatesList.push_back(t);
                }
            }
        }

        if (mIncludeEndPoints)
        {
            diffLeft1 = mSignal[static_cast<int64>(endIdx) - 1] - mSignal[endIdx];
            t = { endIdx, mSignal[endIdx], 0 };
            if (diffLeft1 > 0)
            {
                valleyCandidatesList.push_back(t);
            }

            if (diffLeft1 < 0)
            {
                peakCandidatesList.push_back(t);
            }
        }
    }
}

auto FindPeaks::FindTwoIntervals(  const Peak& peakPoint
                                 , const Peak& leftClosestSubPeak
                                 , const Peak& rightClosestSubPeak
) const->std::pair<int, int>
{
    /*Extend a horizontal line from the peak to the left and right until the line does one of the following:
    - Crosses the signal because there is a higher peak
    - Reaches the left or right end of the signal*/
    int size = (int)mSignal.size();
    std::pair<int, int> twoIntervals;
    if (size > 0 && peakPoint.index < size && peakPoint.index >= 0)
    {
        int leftIntervalIdx = 0;
        int rightIntervalIdx = size - 1;

        //Left Interval
        for (int iDx = leftClosestSubPeak.index; iDx >= 0; iDx--)
        {
            if (mPeakFindingMethod == PeakFindingMethod::RelativeHeight && iDx < leftClosestSubPeak.index - mPeriod)
            {
                leftIntervalIdx = leftClosestSubPeak.index - mPeriod;
                break;
            }

            if ((mPeakType == PeakType::Valley && mSignal[iDx] <= peakPoint.value) //valley
                || (mPeakType == PeakType::Peak && mSignal[iDx] >= peakPoint.value)) //peak
            {
                leftIntervalIdx = iDx;
                break;
            }
        }

        //Right Interval
        for (int iDx = rightClosestSubPeak.index; iDx < size; iDx++)
        {
            if (mPeakFindingMethod == PeakFindingMethod::RelativeHeight && iDx > rightClosestSubPeak.index + mPeriod)
            {
                rightIntervalIdx = rightClosestSubPeak.index + mPeriod;
                break;
            }

            if ((mPeakType == PeakType::Valley && mSignal[iDx] <= peakPoint.value) //valley
                || (mPeakType == PeakType::Peak && mSignal[iDx] >= peakPoint.value)) //peak
            {
                rightIntervalIdx = iDx;
                break;
            }
        }

        twoIntervals = std::make_pair(leftIntervalIdx, rightIntervalIdx);
    }
    else
    {
        twoIntervals = std::make_pair(peakPoint.index, peakPoint.index);
    }

    return twoIntervals;
}

auto FindPeaks::findTwoIntervalsRelativeHeight(  std::vector<float> const& signal
                                               , PeakType peaktype
                                               , Peak peakPoint
                                               , Peak leftClosestSubPeak
                                               , Peak rightClosestSubPeak
                                               , int period
)->std::pair<int, int>
{
    /*Extend a horizontal line from the peak to the left and right until the line does one of the following:
    - Crosses the signal because there is a higher peak
    - Reaches the left or right end of the signal
    - Reaches the limit that is [leftClosestSubPeak.index - period, rightClosestSubPeak.index + period] */
    int size = signal.size();
    std::pair<int, int> twoIntervals;
    if (size > 0 && peakPoint.index < size && peakPoint.index >= 0)
    {
        int leftIntervalIdx = 0;
        int rightIntervalIdx = size - 1;
        //Left Interval
        for (int iDx = leftClosestSubPeak.index; iDx >= 0; iDx--)
        {
            if (iDx < leftClosestSubPeak.index - period)
            {
                leftIntervalIdx = leftClosestSubPeak.index - period;
                break;
            }
            if ((peaktype == PeakType::Valley && signal[iDx] <= peakPoint.value) //valley
                || (peaktype == PeakType::Peak && signal[iDx] >= peakPoint.value)) //peak
            {
                leftIntervalIdx = iDx;
                break;
            }
        }

        //Right Interval
        for (int iDx = rightClosestSubPeak.index; iDx < size; iDx++)
        {
            if (iDx > rightClosestSubPeak.index + period)
            {
                rightIntervalIdx = rightClosestSubPeak.index + period;
                break;
            }
            if ((peaktype == PeakType::Valley && signal[iDx] <= peakPoint.value) //valley
                || (peaktype == PeakType::Peak && signal[iDx] >= peakPoint.value)) //peak
            {
                rightIntervalIdx = iDx;
                break;
            }
        }
        twoIntervals = std::make_pair(leftIntervalIdx, rightIntervalIdx);
    }
    else
    {
        twoIntervals = std::make_pair(peakPoint.index, peakPoint.index);
    }

    return twoIntervals;
}

auto FindPeaks::FindKeyCol(  Peak const& peakPoint
                           , std::pair<int, int> twoIntervals
                           , const Peak& leftClosestSubPeak
                           , const Peak& rightClosestSubPeak
) const->std::pair<int, float>
{
    /*Step 3: Find the minimum of the signal in each of the two intervals defined in Step 2. This point is either a valley or one of the signal endpoints.*/
    std::pair<int, float> keyCol;
    size_t size = mSignal.size();
    if (size > 0 && peakPoint.index < size && peakPoint.index >= 0 && twoIntervals.first >= 0 && twoIntervals.second >= 0 && twoIntervals.second < size)
    {
        int leftKeyColIdx = peakPoint.index;
        int rightKeyColIdx = peakPoint.index;
        int keyColIdx;
        //Left Interval
        for (int iDx = leftClosestSubPeak.index; iDx >= twoIntervals.first; iDx--)
        {
            if ((mPeakType == PeakType::Valley && mSignal[iDx] >= mSignal[leftKeyColIdx]) //valley finder
                || (mPeakType == PeakType::Peak && mSignal[iDx] <= mSignal[leftKeyColIdx])) //peak finder
            {
                leftKeyColIdx = iDx;
            }
        }

        //Right Interval
        for (int iDx = rightClosestSubPeak.index; iDx <= twoIntervals.second; iDx++)
        {
            if ((mPeakType == PeakType::Valley && mSignal[iDx] >= mSignal[rightKeyColIdx]) //valley finder
                || (mPeakType == PeakType::Peak && mSignal[iDx] <= mSignal[rightKeyColIdx])) //peak finder
            {
                rightKeyColIdx = iDx;
            }
        }

        /*Step 4: The higher of the two interval (col) minima specifies the reference level. The height of the peak above this level is its prominence.*/
        if (mPeakFindingMethod == PeakFindingMethod::Prominence)
        {
            keyColIdx = (mPeakType == PeakType::Valley && mSignal[leftKeyColIdx] < mSignal[rightKeyColIdx]) //valley finder
                || (mPeakType == PeakType::Peak && mSignal[leftKeyColIdx] > mSignal[rightKeyColIdx]) //peak finder
                ? leftKeyColIdx : rightKeyColIdx;
        }
        /*Step 4: The lower of the two interval (col) minima specifies the reference level. The height of the peak above this level is its relative height.*/
        else
        {
            keyColIdx = (mPeakType == PeakType::Valley && mSignal[leftKeyColIdx] < mSignal[rightKeyColIdx]) //valley finder
                || (mPeakType == PeakType::Peak && mSignal[leftKeyColIdx] > mSignal[rightKeyColIdx]) //peak finder
                ? rightKeyColIdx : leftKeyColIdx;
        }

        keyCol = std::make_pair(keyColIdx, mSignal[keyColIdx]);
    }
    else
    {
        keyCol = std::make_pair(peakPoint.index, peakPoint.value);
    }
    return keyCol;
}

auto FindPeaks::findKeyColRelativeHeight(  std::vector<float> const& signal
                                         , PeakType peaktype
                                         , Peak peakPoint
                                         , std::pair<int, int> twoIntervals
                                         , Peak leftClosestSubPeak
                                         , Peak rightClosestSubPeak
)->std::pair<int, float>
{
    /*Step 3: Find the minimum of the signal in each of the two intervals defined in Step 2. This point is either a valley or one of the signal endpoints.*/
    int a = 0;
    std::pair<int, float> keyCol;
    int size = signal.size();
    if (size > 0 && peakPoint.index < size && peakPoint.index >= 0 && twoIntervals.first >= 0 && twoIntervals.second >= 0 && twoIntervals.second < size)
    {
        int leftKeyColIdx = peakPoint.index;
        int rightKeyColIdx = peakPoint.index;
        int keyColIdx;
        //Left Interval
        for (int iDx = leftClosestSubPeak.index; iDx >= twoIntervals.first; iDx--)
        {
            if ((peaktype == PeakType::Valley && signal[iDx] >= signal[leftKeyColIdx]) //valley finder
                || (peaktype == PeakType::Peak && signal[iDx] <= signal[leftKeyColIdx])) //peak finder
            {
                leftKeyColIdx = iDx;
            }
        }

        //Right Interval
        for (int iDx = rightClosestSubPeak.index; iDx <= twoIntervals.second; iDx++)
        {
            if ((peaktype == PeakType::Valley && signal[iDx] >= signal[rightKeyColIdx]) //valley finder
                || (peaktype == PeakType::Peak && signal[iDx] <= signal[rightKeyColIdx])) //peak finder
            {
                rightKeyColIdx = iDx;
            }
        }

        /*Step 4: The lower of the two interval (col) minima specifies the reference level. The height of the peak above this level is its relative height.*/

        keyColIdx = (peaktype == PeakType::Valley && signal[leftKeyColIdx] < signal[rightKeyColIdx]) //valley finder
            || (peaktype == PeakType::Peak && signal[leftKeyColIdx] > signal[rightKeyColIdx]) //peak finder
            ? rightKeyColIdx : leftKeyColIdx;

        keyCol = std::make_pair(keyColIdx, signal[keyColIdx]);
    }
    else
    {
        keyCol = std::make_pair(peakPoint.index, peakPoint.index);
    }
    return keyCol;

}

void FindPeaks::RemoveClosePeaks(std::vector<Peak>& filteredPeakList, float minDistance) const
{
    /*Algorithm: https://www.mathworks.com/matlabcentral/answers/156809-how-does-minpeakdistance-work */
    /*Find the largest peaks first and then searches the surrounding neighborhood around it (i.e. +/- meanpeakdistance) for any peaks then deletes those.
    The neighborhood around the deleted peaks are not searched. And as mentioned, this is done from largest to smallest peak.*/

    //std::tuple<int, float, float> = <iDx, peakValue, prominence>
    std::vector<Peak> selectedPeakList = {};
    size_t peakSize = filteredPeakList.size();

    if (minDistance > 0 && peakSize > 0)
    {
        std::vector<Peak> peakCandidateTemp = filteredPeakList;

        /*Sort the list in descending order of prominence in case finding peak, or ascending order in case of finding valley (because prominence in this case is negative value) */
        if (mPeakType == PeakType::Peak)
            std::sort(peakCandidateTemp.begin(), peakCandidateTemp.end(), [](const Peak& a, const Peak& b) { return ((a.prominence) > (b.prominence)); });
        else if (mPeakType == PeakType::Valley)
            std::sort(peakCandidateTemp.begin(), peakCandidateTemp.end(), [](const Peak& a, const Peak& b) { return ((a.prominence) < (b.prominence)); });
        else
        {
            throw "Not support Both type";
        }
        /*Vector determine peaks will be removed or not. true = delete, false = keep*/
        std::vector<bool> iDelete(peakSize, false);

        for (int i = 0; i < peakSize; i++)
        {
            if (!iDelete[i])
            {
                int currentPeakIdx = peakCandidateTemp[i].index;
                int lowerIdx = currentPeakIdx - (int)minDistance;
                int upperIdx = currentPeakIdx + (int)minDistance;
                /*Remove all peaks in the neighborhood of current peak */
                for (int j = i + 1; j < peakSize; j++)
                {
                    int jIdx = (peakCandidateTemp[j].index);
                    iDelete[j] = iDelete[j] ||
                        (jIdx >= lowerIdx && jIdx <= upperIdx); // = true if it is in range [currentPeakIdx - minDistance, currentPeakIdx + minDistance] or has been removed by another peak
                }

                /*Keep current peak*/
                //iDelete[i] = false;

                /*Push current peak to the selected list*/
                selectedPeakList.push_back(peakCandidateTemp[i]);
            }
        }

        /*Reorder the selected list by ascending the iDx*/
        std::sort(selectedPeakList.begin(), selectedPeakList.end(), [](const Peak& a, const  Peak& b) { return ((a.index) < (b.index)); });

        filteredPeakList.clear();
        filteredPeakList = selectedPeakList;
    }
    else
    {
        //Do nothing
    }
}

void FindPeaks::RemoveLowProminencePeaks(std::vector<Peak>& filteredPeakList, float minProminence) const
{
    size_t peakSize = filteredPeakList.size();
    if (minProminence > 0 && peakSize > 0)
    {
        std::vector<Peak> selectedPeakList;
        selectedPeakList.reserve(peakSize);

        for (auto peak : filteredPeakList)
        {
            if (abs(peak.prominence) >= abs(minProminence))
            {
                selectedPeakList.push_back(peak);
            }
        }

        std::swap(filteredPeakList, selectedPeakList);
    }
    else
    {
        //Do nothing
    }
}

void FindPeaks::RefinePeaks(std::vector<Peak>& filteredPeakList, float peakRefinement) const
{
    size_t peakSize = filteredPeakList.size();
    size_t signalSize = mSignal.size();
    if (peakRefinement > 0 && peakRefinement <= 1 && peakSize > 0 && signalSize > 0)
    {
        float refValue = 0;
        //float valleyRefValue = 0;

        //p = 1 - p;
        int leftIdx = 0;
        int rightIdx = 0;
        int currentPeakIdx = 0;
        int newPeakIdx = 0;
        for (int peakdIdx = 0; peakdIdx < peakSize; peakdIdx++)
        {
            if (mPeakType == PeakType::Valley)
            {
                refValue = (1 + peakRefinement) * (filteredPeakList[peakdIdx].value);
            }
            else
            {
                refValue = (1 - peakRefinement) * (filteredPeakList[peakdIdx].value);
            }

            leftIdx = rightIdx = currentPeakIdx = (filteredPeakList[peakdIdx].index);
            if (currentPeakIdx > 0 && currentPeakIdx < signalSize - 1)
            {
                // Find the left idx;
                for (int i = currentPeakIdx - 1; i > 0; i--)
                {
                    if ((mPeakType == PeakType::Peak) && mSignal[i] <= refValue)
                    {
                        leftIdx = i;
                        break;
                    }

                    if ((mPeakType == PeakType::Valley) && mSignal[i] >= refValue)
                    {
                        leftIdx = i;
                        break;
                    }
                }

                // Find the right idx
                for (int i = currentPeakIdx + 1; i < signalSize; i++)
                {
                    if ((mPeakType == PeakType::Peak) && mSignal[i] <= refValue)
                    {
                        rightIdx = i;
                        break;
                    }

                    if ((mPeakType == PeakType::Valley) && mSignal[i] >= refValue)
                    {
                        rightIdx = i;
                        break;
                    }
                }

                (filteredPeakList[peakdIdx].index) = newPeakIdx = (leftIdx + rightIdx) / 2;
                (filteredPeakList[peakdIdx].value) = mSignal[newPeakIdx];
                //std::get<2>(peakCandidate[peakdIdx]) += signal[newPeakIdx]-signal[currentPeakIdx];
            }
        }
    }
}

auto FindPeaks::findPeakByRelativeHeight(  std::vector<float> const& signal
                                         , PeakType peaktype
                                         , float minProminence
                                         , int minDistance
                                         , int period
                                         , float p
                                         , bool includeEndpoints
)->std::vector<Peak>
{
    std::vector<Peak> returnPeakList;
    int size = signal.size();
   
//    if (size > 0)
//    {
//        /*Step 1: Place a marker on the peak*/
//        std::vector<Peak>  peakMarkerList, subPeakMarkerList;
//        if (peaktype == PeakType::Peak) {
//            FindPeaks(signal, peakMarkerList, subPeakMarkerList, includeEndpoints);
//        }
//        else if (peaktype == PeakType::Valley) {
//            findPeaks(signal, subPeakMarkerList, peakMarkerList, includeEndpoints);
//        }
//        else if (peaktype == PeakType::Both) {
//            throw ("Not support PeakType Both");
//            return returnPeakList;
//        }
//        std::vector<Peak> peakListTuple;
//        std::vector<Peak> peakListTmp;
//
//        int subPeakMarkerListSize = subPeakMarkerList.size();
//        int firstSubPeakIdx = 0;
//        for (auto peak : peakMarkerList)
//        {
//            /*Determine the closest subPeak on the left and right*/
//            Peak leftClosestSubPeak{ 0, signal[0] }; //Init with left endpoints
//            Peak rightClosestSubPeak{ size - 1, signal[size - 1] }; //Init with right endpoints
//            int tmpId = subPeakMarkerListSize - 1;
//
//            /*Determine the closest subPeak on right*/
//            for (int i = firstSubPeakIdx; i < subPeakMarkerListSize; i++)
//            {
//                if (subPeakMarkerList[i].index > peak.index)
//                {
//                    rightClosestSubPeak = subPeakMarkerList[i];
//                    tmpId = i;
//                    break;
//                }
//            }
//
//            /*Determine the closest subPeak on the left*/
//            for (int i = tmpId; i >= firstSubPeakIdx; i--)
//            {
//                if (subPeakMarkerList[i].index < peak.index)
//                {
//                    leftClosestSubPeak = subPeakMarkerList[i];
//                    firstSubPeakIdx = i;
//                    break;
//                }
//            }
//
//            /*
//            Step 2: Extend a horizontal line from the peak to the left and right until the line does one of the following:
//            - Crosses the signal because there is a higher peak
//            - Reaches the left or right end of the signal
//            - Reaches the limit that is [leftClosestSubPeak.index - period, rightClosestSubPeak.index + period] */
//            std::pair<int, int> twoIntervals = findTwoIntervalsRelativeHeight(signal, peaktype, peak, leftClosestSubPeak, rightClosestSubPeak, period);
//
//            /*
//            Step 3: Find the minimum of the signal in each of the two intervals defined in Step 2. This point is either a valley or one of the signal endpoints.
//            */
//            /*
//             Step 4: The lower of the two interval minima specifies the reference level. The height of the peak above this level is its relative height.
//            */
//            std::pair<int, float> higherMinimumIntervalPoint = findKeyColRelativeHeight(signal, peaktype, peak, twoIntervals, leftClosestSubPeak, rightClosestSubPeak);
//            float prominence = 0;
//            if (higherMinimumIntervalPoint.first >= 0 && higherMinimumIntervalPoint.first <= size)
//            {
//                prominence = peak.value - higherMinimumIntervalPoint.second;
//                peak.prominence = prominence;
//            }
//            /*Compare prominence with minProminence to remove peak has lower prominence than this threshold*/
//            /*Actually this is not the prominence, this is relative height*/
//            if (abs(prominence) >= abs(minProminence))
//            {
//                /*peakListTemp.push_back(peak);
//                prominenceList.push_back(prominence);
//                prominenceLevelIdxList.push_back(higherMinimumIntervalPoint.first);*/
//                peakListTuple.push_back(peak);
//            }
//            peakListTmp.push_back(peak);
//        }
//
//        peakRefinement(signal, peakListTuple, p, peaktype);
//
//        /*Remove close peak by minDistance*/
//        returnPeakList = RemoveClosePeaks(signal, peakListTuple, peaktype, minDistance);
//
//#ifdef DEBUG_FIND_PEAK_BY_PROMINENCE
//        cv::Mat graph = cv::Mat(255, size, CV_8UC3);
//        graph.setTo(cv::Scalar(0, 0, 0));
//        std::vector <cv::Point2f> smoothedAccumulationPoint;
//        for (int i = 0; i < size; i++)
//        {
//            smoothedAccumulationPoint.push_back(cv::Point2f((float)i, signal[i]));
//        }
//        Utils::Drawing drawTool;
//        drawTool.yTopOrigin = false;
//        drawTool.type = Utils::DrawType::Line;
//        drawTool.color = CV_RED_LONESTAR;
//        drawTool.plot(graph, smoothedAccumulationPoint, false);
//
//        for (int i = 0; i < peakListTmp.size(); i++)
//        {
//            //Green line: prominence height
//            cv::line(graph, cv::Point2f((float)(peakListTmp[i].index), 255 - (peakListTmp[i]).value), cv::Point2f((float)(peakListTmp[i].index), 255 - ((peakListTmp[i].value) - (peakListTmp[i]).prominence)), cv::Scalar(0, 255, 0), 1, cv::LINE_8);
//            //Green color number: prominence value
//            cv::putText(graph, std::to_string(abs((int)(peakListTmp[i].prominence))), cv::Point2f((float)(peakListTmp[i].index), (float)255 - (peakListTmp[i].value) + 20), 1, 0.5, CV_ORANGE, 1, cv::LINE_8);
//            //Cyan color number: prominence value
//            cv::putText(graph, std::to_string(abs((int)(peakListTmp[i].index))), cv::Point2f((float)(peakListTmp[i].index), (float)255 - (peakListTmp[i].value) + 10), 1, 0.5, CV_LIGHTYELLOW, 1, cv::LINE_8);
//            /*Utils::Drawing::DrawDashedLine(graph, cv::Point((int)peakList[i].first, (int)(peakList[i].second - prominenceList[i])), cv::Point(prominenceLevelIdxList[i], signal[prominenceLevelIdxList[i]]), cv::Scalar(255,255,0), 1, "", 3);*/
//        }
//
//        std::vector <cv::Point2f> localMaximaPoint;
//        for (auto i : returnPeakList) {
//            localMaximaPoint.push_back((cv::Point2f((float)i.index, i.value)));
//        }
//        drawTool.type = Utils::DrawType::Circle;
//        drawTool.color = cv::Scalar(0, 0, 255);
//        drawTool.plot(graph, localMaximaPoint, false);
//        drawTool.showImg("findPeakByRelativeHeight", graph);
//#endif
//    }

    return returnPeakList;
}

void FindPeaks::DrawGraph(std::vector<Peak>& filteredPeakList) const
{
    int size = (int)mSignal.size();
    if (size > 0)
    {
        cv::Mat graph = cv::Mat(255, size, CV_8UC3);
        graph.setTo(cv::Scalar(0, 0, 0));

        std::vector <cv::Point2f> smoothedAccumulationPoint;
        for (int i = 0; i < size; i++)
        {
            smoothedAccumulationPoint.push_back(cv::Point2f((float)i, mSignal[i]));
        }
        Drawing drawTool;
        drawTool.yTopOrigin = false;
        drawTool.type = DrawType::Line;
        drawTool.color = COLOR_CV_RED_LONESTAR;
        drawTool.Plot(graph, smoothedAccumulationPoint, false);

        for (int i = 0; i < mPeakList.size(); i++)
        {
            //Green line: prominence height
            cv::line(graph, cv::Point2f((float)(mPeakList[i].index), 255 - (mPeakList[i]).value), cv::Point2f((float)(mPeakList[i].index), 255 - ((mPeakList[i].value) - (mPeakList[i]).prominence)), cv::Scalar(0, 255, 0));
            //Green color number: prominence value
            cv::putText(graph, std::to_string(abs((int)(mPeakList[i].prominence))), cv::Point2f((float)(mPeakList[i].index), (float)255 - (mPeakList[i].value) + 20), 1, 0.5, COLOR_CV_ORANGE);
            //Cyan color number: prominence value
            cv::putText(graph, std::to_string(abs((int)(mPeakList[i].index))), cv::Point2f((float)(mPeakList[i].index), (float)255 - (mPeakList[i].value) + 10), 1, 0.5, COLOR_CV_LIGHTYELLOW);
        }

        std::vector <cv::Point2f> localMaximaPoint;
        for (auto i : filteredPeakList)
        {
            localMaximaPoint.push_back((cv::Point2f((float)i.index, i.value)));
        }
        drawTool.type = DrawType::Circle;
        drawTool.color = cv::Scalar(0, 0, 255);
        drawTool.Plot(graph, localMaximaPoint, false);
        drawTool.ShowImg("findPeakByProminence", graph);
    }
}

cv::Mat DrawPeakInfo(  cv::Mat const& src
                     , std::vector<float> const& signal
                     , std::vector<Peak> const& lstPeak
                     , Drawing & drawTool
                     , int centerNeglectionWidth
)
{
    if (src.empty() || signal.empty() || lstPeak.empty() || signal.size() == 0 || lstPeak.size() == 0) return cv::Mat();
    if (src.rows <= 2 * drawTool.padding_X || src.cols <= 2 * drawTool.padding_X) throw std::invalid_argument("ERROR: img.rows <= 2 * padding_X || img.cols <= 2 * padding_X");

    const int rows = src.rows;
    const int cols = src.cols;

    cv::Mat drawPolePosImg;
    cv::cvtColor(src, drawPolePosImg, cv::COLOR_GRAY2BGR);

    std::vector<float> xList(cols);
    // Fill with 0, 1, ..., cols - 1.
#pragma warning (suppress : 4244)
    std::iota(std::begin(xList), std::end(xList), 0);

    // Draw findpeak promenence information
    // caculation the height of promenence draw image affter add padding
    int height = rows - 2 * drawTool.padding_Y;
    // caculation the wight of promenence draw image affter add padding
    int width = cols - 2 * drawTool.padding_X;
    cv::Size imgSize = cv::Size(width, height);

    // transfor point function with scale, rotation, translattion...
    // return: cv::point
    drawTool.CaculateScale(imgSize, xList, signal);

    // convert signal to vector point
    std::vector<cv::Point> vtPoint;
    for (int i = 0, _size = (int)xList.size(); i < _size; i++)
    {
        vtPoint.push_back(drawTool.GetDrawPoint(xList[i], signal[i]));
    }

    /*color = CV_YELLOW;
    plot(drawPolePosImg, xList, signal, true);*/
    for (int i = 1, _size = (int)vtPoint.size(); i < _size; i++)
    {
        // draw hight of the promenence
        cv::line(drawPolePosImg, vtPoint[static_cast<int64>(i) - 1], vtPoint[i], drawTool.color, drawTool.thickness);
    }

    float fontScale = 0.3f; // font scale set prominence text
    drawTool.color = COLOR_CV_GREEN; // color set prominence text and line
    drawTool.thickness = 1;  // thickness set prominence text and line

    // draw information of promenence each element of list peaks we found
    std::for_each(lstPeak.begin(), lstPeak.end(), [&drawPolePosImg, &drawTool, &imgSize, &fontScale](const auto& p)
                  {
                      // caculation the transfor point
                      cv::Point p1, p2;
                      p1 = drawTool.GetDrawPoint(p.index, p.value);
                      p2 = drawTool.GetDrawPoint(p.index, abs(p.value - p.prominence));

                      // draw hight of the promenence
                      cv::line(drawPolePosImg, p1, p2, drawTool.color, drawTool.thickness);

                      // using abs because we find the valey so the prominence is negative value
                      std::string strProminence = std::to_string(static_cast<int>(abs(std::round(p.prominence))));

                      // cacutation position of the text using opencv getTextSize
                      cv::Size txtSize = cv::getTextSize(strProminence, cv::FONT_HERSHEY_SIMPLEX, fontScale, drawTool.thickness, 0);
                      int xTextPos = p1.x - txtSize.width / 2;
                      int yTextPos = drawTool.yTopOrigin ? p1.y + txtSize.height : p1.y - txtSize.height ;
                      cv::Point p3 = cv::Point(xTextPos, yTextPos);

                      // draw text of the promenence value
                      cv::putText(drawPolePosImg, strProminence, p3, cv::FONT_HERSHEY_SIMPLEX, fontScale, drawTool.color, drawTool.thickness);
                  });

    // draw pole position
    std::vector<cv::Point> drawPolePos;
    std::for_each(lstPeak.begin(), lstPeak.end(), [&drawPolePos, &vtPoint](const auto& p) { drawPolePos.push_back(vtPoint[p.index]); });
    int symbolSize = drawTool.thickness * 2;
    drawTool.color = COLOR_CV_RED;
    // using draw Utils::DrawType::Star
    for (int i = 0, _size = (int)drawPolePos.size(); i < _size; i++)
    {
        cv::line(drawPolePosImg, cv::Point(drawPolePos[i].x - symbolSize, drawPolePos[i].y), cv::Point(drawPolePos[i].x + symbolSize, drawPolePos[i].y), drawTool.color, drawTool.thickness);
        cv::line(drawPolePosImg, cv::Point(drawPolePos[i].x, drawPolePos[i].y - symbolSize), cv::Point(drawPolePos[i].x, drawPolePos[i].y + symbolSize), drawTool.color, drawTool.thickness);
    }

    if (centerNeglectionWidth > 0)
    {
        // draw center neglection area
        int centerLeft = (cols - centerNeglectionWidth) / 2;
        int centerRight = (cols + centerNeglectionWidth) / 2;

        cv::line(drawPolePosImg, cv::Point(centerLeft, 0), cv::Point(centerLeft, rows), cv::Scalar(255, 255, 0), drawTool.thickness);
        cv::line(drawPolePosImg, cv::Point(centerRight, 0), cv::Point(centerRight, rows), cv::Scalar(255, 255, 0), drawTool.thickness);
    }

    //cv::namedWindow("FindPeak Pole pos", cv::WINDOW_NORMAL);
    //cv::imshow("FindPeak Pole pos", drawPolePosImg);

    return drawPolePosImg;
}

Drawing DrawPeaks(cv::Mat& img, VecFloat const& sig, VecPeak const& lstPeak, CVPen pen)
{
    Drawing drawTool;
    if (sig.empty()) return drawTool;
    int minValue = *std::min_element(sig.begin(), sig.end());
    int maxValue = *std::max_element(sig.begin(), sig.end());
    int size = sig.size();
    if (img.empty())
        img = cv::Mat::zeros((std::min)((std::max)(abs(minValue), abs(maxValue)), 200), size, CV_8UC3);

    drawTool.yTopOrigin = false;
    drawTool.color      = pen.mColor;
    drawTool.thickness  = pen.mThickness;
    drawTool.Plot(img, 0, img.cols, sig, false);

    drawTool.color = cv::Scalar(255, 255, 255) - pen.mColor;
    drawTool.type  = DrawType::Pixel;
    for (auto&& p : lstPeak)
    {
        drawTool.Plot(img, cv::Point(p.index, p.value));
    }

    return drawTool;
}

}