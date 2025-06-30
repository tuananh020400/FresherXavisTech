#include "xvtBattery/PoleInfo.h"
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/Utils.h"
#include <xvtCV/Polyfit.h>

namespace xvt {
namespace battery {

PoleCompare::PoleCompare()
{
    mLeftReferencePole = PoleInfo();
    mLeftPole = PoleInfo();
    mRightReferencePole = PoleInfo();
    mRightPole = PoleInfo();
    mType = PoleCompareStatus::Untrust;
}

PoleCompare::PoleCompare(PoleInfo &&leftPole, PoleInfo &&rightPole, double const &maxdiff)
{
    SetPole(std::move(leftPole), std::move(rightPole), maxdiff);
}

double PoleCompare::GetLengthError()
{
    return abs(mLeftPole.length() - mRightPole.length());
}

void PoleCompare::SetPole(PoleInfo &&leftPole, PoleInfo &&rightPole, double const &maxdiff)
{
    mLeftPole = leftPole;
    mRightPole = rightPole;
    mType = (GetLengthError() < maxdiff && mLeftPole.length() && mRightPole.length()) ? PoleCompareStatus::Trust : PoleCompareStatus::Untrust;
}

bool PoleCompare::AdjustUncertainType(double maxdiff)
{
    float halfDiff = maxdiff / 2.0;
    //Calculate the diff by length.
    float leftLengthError = abs(mLeftReferencePole.length() - mLeftPole.length());
    float rightLengthError = abs(mRightReferencePole.length() - mRightPole.length());

    bool isEdit = false;
    if (leftLengthError > maxdiff && rightLengthError > maxdiff)
    {
        // Khi hai pole deu sai
    }
    else
    {
        PoleInfo* pAdjPole = nullptr;
        PoleInfo* pAdjRefPole = nullptr;
        PoleInfo* pOpositePole = nullptr;
        PoleInfo* pOpositeRefPole = nullptr;

        if (leftLengthError > rightLengthError)
        {
            //Adjust pole
            pAdjPole = &mLeftPole;
            pAdjRefPole = &mLeftReferencePole;
            //Oposite pole
            pOpositePole = &mRightPole;
            pOpositeRefPole = &mRightReferencePole;
        }
        else
        {
            //Adjust pole
            pAdjPole = &mRightPole;
            pAdjRefPole = &mRightReferencePole;
            //Oposite pole
            pOpositePole = &mLeftPole;
            pOpositeRefPole = &mLeftReferencePole;
        }

        //Calculate the anode adjusting pole error (pos(pole) - pos(pole ref))
        float adjAnodePosError = abs(pAdjPole->mAnode.y - pAdjRefPole->mAnode.y);

        bool isAnodeError = adjAnodePosError > halfDiff && (pAdjPole->mType == PoleType::Real);
        bool isCathodeError = (abs(pAdjPole->mCathode.y - pAdjRefPole->mCathode.y) > halfDiff) || (abs(pAdjPole->mCathode.y - pOpositePole->mCathode.y) > halfDiff);

        //Cathode is the same, and the anode has much difference
        if (isAnodeError)
        {
            //Calculate the oposite pole length
            double opositePoleLength = pOpositePole->length();
            //Calculate the oposite pole length error (length(pole) - length(pole ref))
            float opositeRefLengthError = abs(pOpositeRefPole->length() - opositePoleLength);

            // if cathode line is trustable, copy the length from the other side
            isEdit = (!isCathodeError) && (opositeRefLengthError < maxdiff);
            if (isEdit)
            {
                //Copy the length
                pAdjPole->mAnode.y = pAdjPole->mCathode.y - opositePoleLength;
            }
            else
            {
                //if the adjust pole has big length error
                isEdit = (adjAnodePosError > halfDiff);
                if (isEdit)
                {
                    // Copy the reference anode pole position
                    pAdjPole->mAnode.y = pAdjRefPole->mAnode.y;
                }
            }
        }

        if (isEdit)
        {
            pAdjPole->mType = PoleType::Adjusted;
            mType = PoleCompareStatus::Adjusted;
        }
    }

    return isEdit;
}

PoleInfo::PoleInfo(cv::Point anode, cv::Point cathode, PoleType type)
{
    mAnode = anode;
    mCathode = cathode;
    mType = type;
}

PoleInfo::PoleInfo(Peak peakValue, cv::Point anode, cv::Point cathode, PoleType type)
{
    mPeakValue = peakValue;
    mAnode = anode;
    mCathode = cathode;
    mType = type;
}

auto PoleInfo::length() const -> double
{
    return GetLength(mCathode, mAnode);
}

PoleInfo GetRefPole(float x, PoleInfo const &trustedPoleBefore, PoleInfo const &trustedPoleAfter)
{
    PoleInfo pole{};

    float l = abs(trustedPoleAfter.mAnode.x - trustedPoleBefore.mAnode.x);
    float alpha = abs(x - trustedPoleBefore.mAnode.x) / l;
    float beta = 1.0 - alpha;

    pole.mAnode.x = x;
    pole.mAnode.y = trustedPoleAfter.mAnode.y * alpha + trustedPoleBefore.mAnode.y * beta;

    pole.mCathode.x = x;
    pole.mCathode.y = trustedPoleAfter.mCathode.y * alpha + trustedPoleBefore.mCathode.y * beta;
    pole.mType = PoleType::Inserted;
    return pole;
}

std::vector<PoleCompare> evaluatePoleForRefinement(const VecInt &lstPolePosXAll,
                                                   const VecInt &anodePos,
                                                   const VecInt &cathodePos,
                                                   int widthROI,
                                                   double maxDis,
                                                   double maxDiff,
                                                   bool refinement)
{
    //double MaxDis = 14; //Max distance using for finding the matchign poles
    //double MaxDiff = 10; // Using for deciding the untrusted poles by compare the pole length

    // For our simplicity, let�s call right poles as R(n), left poles as L(n).
    // The left pole starts from the left-most pole and the right pole from the right-most pole. n = 0, 1,�, N
    // To each pole, the coordinates of starting (bottom) and ending position
    // are written as Ls(n), and Le(n) for left poles. Rs(n) and Re(n) respectively
    VecPoint Ls, Le;
    VecPoint Rs, Re;
    VecInt xLeft, xRight;
    std::vector<PoleCompare> poleCompareList;
    for (auto it = 0; it < lstPolePosXAll.size(); it++)
    {
        if (lstPolePosXAll[it] < widthROI / 2)
        {
            Ls.push_back(cv::Point(lstPolePosXAll[it], anodePos[it]));
            Le.push_back(cv::Point(lstPolePosXAll[it], cathodePos[it]));
            xLeft.push_back(lstPolePosXAll[it]);
        }
        else
        {
            Rs.push_back(cv::Point(widthROI - lstPolePosXAll[it], anodePos[it]));
            Re.push_back(cv::Point(widthROI - lstPolePosXAll[it], cathodePos[it]));
            xRight.push_back(widthROI - lstPolePosXAll[it]);
        }
    }

    std::sort(Rs.begin(), Rs.end(), [](auto &left, auto &right) { return left.x < right.x; });
    std::sort(Re.begin(), Re.end(), [](auto &left, auto &right) { return left.x < right.x; });
    std::sort(xRight.begin(), xRight.end(), [](auto &left, auto &right) { return left < right; });

    VecDouble coeffRs;
    VecDouble coeffRe;
    VecDouble coeffLs;
    VecDouble coeffLe;
    double error = 0;
    int order = 5;
    float pInlier = 0.4;
    if (Rs.size() <= order || Re.size() <= order || Ls.size() <= order || Le.size() <= order)
    {
        return poleCompareList;
    }

    float sum = 0.0;

    int nL = Ls.size();
    std::for_each(Ls.begin(), Ls.end(), [&sum](cv::Point &p) { sum += p.y; });
    float avgLs = round(sum / nL);
    std::for_each(Ls.begin(), Ls.end(), [&avgLs](cv::Point &p) { p.y -= avgLs; });

    double rtnLs = Polyfit::LSQFit(Ls, order, coeffLs);
    std::for_each(Ls.begin(), Ls.end(), [&avgLs](cv::Point &p) { p.y += avgLs; });

    nL = Le.size();
    sum = 0.0;
    std::for_each(Le.begin(), Le.end(), [&sum](cv::Point &p) { sum += p.y; });
    float avgLe = round(sum / nL);
    std::for_each(Le.begin(), Le.end(), [&avgLe](cv::Point &p) { p.y -= avgLe; });
    double rtnLe = Polyfit::LSQFit(Le, order, coeffLe);
    std::for_each(Le.begin(), Le.end(), [&avgLe](cv::Point &p) { p.y += avgLe; });

    nL = Rs.size();
    sum = 0.0;
    std::for_each(Rs.begin(), Rs.end(), [&sum](cv::Point &p) { sum += p.y; });
    float avgRs = sum / nL;
    std::for_each(Rs.begin(), Rs.end(), [&avgRs](cv::Point &p) { p.y -= avgRs; });
    double rtnRs = Polyfit::LSQFit(Rs, order, coeffRs);
    std::for_each(Rs.begin(), Rs.end(), [&avgRs](cv::Point &p) { p.y += avgRs; });

    nL = Re.size();
    sum = 0.0;
    std::for_each(Re.begin(), Re.end(), [&sum](cv::Point &p) { sum += p.y; });
    float avgRe = sum / nL;
    std::for_each(Re.begin(), Re.end(), [&avgRe](cv::Point &p) { p.y -= avgRe; });
    double rtnRe = Polyfit::LSQFit(Re, order, coeffRe);
    std::for_each(Re.begin(), Re.end(), [&avgRe](cv::Point &p) { p.y += avgRe; });

    int matchIdx = -1;
    float cathodeFit = 0;
    float anodeFit = 0;
    auto minMaxRefList = std::minmax_element(Rs.begin(), Rs.end(), [](const cv::Point &p1, const cv::Point &p2) { return p1.x < p2.x; });
    float minX = minMaxRefList.first->x;
    float maxX = minMaxRefList.second->x;
    for (int i = 0; i < Ls.size(); i++)
    {
        int &leftX = Ls[i].x;

        PoleInfo leftPole = PoleInfo(Ls[i], Le[i], PoleType::Real);
        PoleInfo rightPole = PoleInfo();

        float xNext = i + 1 < Ls.size() ? Ls[i + 1].x : leftX + maxDis;
        matchIdx = FindClosestPoint(Rs, leftX, xNext, maxDis);

        if (matchIdx < 0)
        { // If it can not fine the left pole in the right side
            // Caculate the pole information in the right side base on x
            // Fitting the end pos
            cathodeFit = Polyfit::f(coeffRe, leftX) + avgRe;
            // Fitting the start pos
            anodeFit = Polyfit::f(coeffRs, leftX) + avgRs;

            // Assign the right pole value
            PoleType type = (minX <= leftX && leftX <= maxX) ? PoleType::Inserted : PoleType::InsertedOutRange;
            rightPole = PoleInfo(cv::Point(leftX, std::min(anodeFit, cathodeFit)), cv::Point(leftX, cathodeFit), type);
        }
        else
        { // if matched
            // Assign value
            rightPole = PoleInfo(Rs[matchIdx], Re[matchIdx], PoleType::Real);

            // Remove matched idx
            Rs.erase(Rs.begin() + matchIdx);
            Re.erase(Re.begin() + matchIdx);
        }
        PoleCompare poleCompare(std::move(leftPole), std::move(rightPole), maxDiff);

        //Add pole to the list
        poleCompareList.push_back(std::move(poleCompare));
    }
    if (refinement)
    {
        // For our simplicity, let�s call right poles as R(n), left poles as L(n).
        // The left pole starts from the left-most pole and the right pole from the right-most pole. n = 0, 1,�, N
        // To each pole, the coordinates of starting (bottom) and ending position
        // are written as Ls(n), and Le(n) for left poles. Rs(n) and Re(n) respectively
        auto minMaxLsList = std::minmax_element(Ls.begin(), Ls.end(), [](const cv::Point &p1, const cv::Point &p2) { return p1.x < p2.x; });
        // Voi nhung pole con lai o ben phai.
        for (int i = 0; i < Rs.size(); i++)
        {
            int &rightX = Rs[i].x;
            // Caculate the pole information in the left side base on x
            // Fitting the end pos
            cathodeFit = Polyfit::f(coeffLe, rightX) + avgLe;
            // Fitting the start pos
            anodeFit = Polyfit::f(coeffLs, rightX) + avgLs;
            // Assign the left pole value
            PoleType type = (minMaxLsList.first->x <= rightX && rightX <= minMaxLsList.second->x) ? PoleType::Inserted : PoleType::InsertedOutRange;
            PoleInfo leftPole = PoleInfo(cv::Point(rightX, std::min(anodeFit, cathodeFit)), cv::Point(rightX, cathodeFit), type);

            PoleInfo rightPole = PoleInfo(Rs[i], Re[i], PoleType::Real);

            PoleCompare poleCompare(std::move(leftPole), std::move(rightPole), maxDiff);

            // Find the position to insert the pole list
            auto pIdx2 = std::find_if(poleCompareList.begin(), poleCompareList.end(), [&rightX](PoleCompare p) {
                return p.mRightPole.mAnode.x > (float)rightX;
            });
            poleCompareList.insert(pIdx2, std::move(poleCompare));
        }
    }

    return poleCompareList;
}
} // namespace battery
} // namespace xvt