#pragma once
#include "xvtCV/IInspection.h"
#include "xvtCV/ScopeTimer.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>
#include <vector>
#include <numeric>
#include <random>

namespace xvt {
//! @addtogroup Utils
//! @{

#ifdef _DEBUG
    /// Predefined Ransace timeout
    #define RANSAC_TIMEOUT (1500);
#else
    /// Predefined Ransace timeout
    #define RANSAC_TIMEOUT (250);
#endif

/**
 * @brief Ransac setting parameters
*/
class XVT_EXPORTS RansacSetting
{
public:
    RansacSetting() = default;
    RansacSetting(int s, double d, int T):
          mMinModelSize{ s }
        , mModelSize{ s }
        , mDataThreshold{ d }
        , mModelThreshold{ T }
    {}

    RansacSetting(int minModelSize, int modelsize, double d, int T) :
          mMinModelSize{ minModelSize }
        , mModelSize{ std::max(minModelSize, modelsize) }
        , mDataThreshold{ d }
        , mModelThreshold{ T }
    {}
    virtual ~RansacSetting() = default;

    void SetModelSize(int s) { mModelSize = std::max(mMinModelSize, s); }
    auto GetModelSize(int s)const->int { return mModelSize; }

    auto GetMinModeSize() const->int { return mMinModelSize; };

public:
    /// Value for determining when a data point fits a model
    double mDataThreshold = 1.0f;
    /// Number of close data values required to assert that a model fits well to data(T)
    int mModelThreshold = 0;
    /// Probability that after running this algorithm mSuccessProbability% we can find atleast one set of correct model.
    double mSuccessProbability = 0.999;
    /// When running time is reached mTimeOut it will stop and raise time out error.
    int mTimeOut = RANSAC_TIMEOUT;  
    /// if true it will update/re-check the in-liers and out-liers before output with data distance=2*mDataThreshold.
    bool mIsRefineFinal = true;
    /// if true it will update/re-check the in-liers and out-liers when finding the model candidate.
    bool mIsRefineModel = false;

protected:
    /// Size of model
    int    mModelSize = 0;

private:
    /// The minimum number of data values required to fit the model
    int    mMinModelSize = 0;
};

/**
 * @brief Ransace Result template
 * @tparam ModelResult Model
*/
template<class ModelResult>
class RansacResult : public ModelResult, public RansacSetting
{
public:
    RansacResult() :RansacSetting() {}
    RansacResult(RansacSetting const& other) :RansacSetting(other) {}
    RansacResult(ModelResult const& other) :ModelResult(other) {}

    auto GetSuccessProbabily()const->double;
public:
    void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    /// Inlier probability
    double mPInlier= 0.0;
    /// Inlier points
    //xvt::VecPoint mInLiers;
    /// Outlier points
    xvt::VecPoint mOutLiers;
};

/**
 * @brief Template Ransac class that will handle the RanSAC algorithm when model is provided.
 * @tparam ModelResult Model Result
*/
template<class ModelResult>
class Ransac : public RansacSetting
{
public:
    using ModelResultType  = ModelResult;
    using RansacType       = Ransac<ModelResult>;
    using RansacResultType = RansacResult<ModelResult>;

    Ransac() = default;
    Ransac(int modelSize, double dataThreshold, int modelThreshold)
        :RansacSetting(modelSize, dataThreshold, modelThreshold) {}

    virtual auto FitRANSAC(xvt::VecPoint const& pointList) const->RansacResultType;
    virtual auto FitRANSAC(xvt::VecPoint const& pointList, xvt::VecPoint const& inliers) const->RansacResultType;

    virtual auto GetInOutLiers(xvt::VecPoint const& points
                                  , ModelResultType& tmpMod
                                  , xvt::VecPoint& inLiers
                                  , xvt::VecPoint& outLiers
    )const -> void;

    /// Estimate the model by using Least Square Error fitting methode
    virtual auto FitLSQE(xvt::VecPoint const& points) const ->ModelResultType = 0;
    /// Check if model result is valid or not, the derivative class should implement it.
    virtual auto IsValidModel(ModelResultType const& res) const->bool { return true; };

    /**
     * @brief Maximum number of iteration need to run to archive mSuccessProbability
     * @param w Inlier probability
     * @return Maximum number of iteration
    */
    auto GetMaxIteration(double w) const->int;

    /**
     * @brief Get the percentage value in range [0, 1].
     * 
     * percentage = Saturatecast(value/total, 0, 1)
     * 
     * @param value a value
     * @param total a max value
     * @return ratio of value/total that cast into the range [0, 1]
    */
    constexpr
    auto GetProbability(double value, double total) const->double;

    /// Update the ransac setting
    void Update(double d, int modelThd, int timeout, bool finnalRefine, bool modelRefine);

protected:

    /// Collect the inliers and outliers.
    auto RefineInOutLiers(ModelResultType& tmpMod
                          , xvt::VecPoint& inLiers
                          , xvt::VecPoint& outLiers
                          , double dataThreshold
    )const -> void;
};

template<class ModelResult>
inline
auto Ransac<ModelResult>::FitRANSAC(xvt::VecPoint const& pointList) const -> RansacResultType
{
    VecPoint inLiers;
    return FitRANSAC(pointList, inLiers);
}

template<class ModelResult>
auto Ransac<ModelResult>::FitRANSAC(xvt::VecPoint const& pointList
                                                            , xvt::VecPoint const& inliers
) const -> RansacResultType
{
    xvt::ScopeTimer t("Ransac");
    auto res = RansacResultType(static_cast<RansacSetting const&>(*this));
    const size_t totalSample = pointList.size();
    if (totalSample <= mModelSize && mModelSize>0) return res;


    //The probability that sample is an pInlier
    res.mPInlier = GetProbability(mModelThreshold, totalSample);

    //Calculate the probability all points are inlier.
    double w = res.GetSuccessProbabily();

    // Current model selected point size
    int curModelSize = mModelThreshold;
    res(EResult::ER,"Ransac can not find the optimal model!");
    if (w == 1.0)
    {
        static_cast<ModelResult&>(res) = FitLSQE(pointList);
        res.mPoints = pointList;
        res(EResult::OK, "");
    }
    else
    {
         //Random number generation setup
        std::random_device rd;  // Seed for the random number engine
        std::mt19937 gen(rd()); // Mersenne Twister engine seeded with rd()
        std::uniform_int_distribution<> dis(0, totalSample - 1); // Distribution for indices

        int sampleCount = 0;
        // The maximum number of iterations allowed in the algorithm
        int maxIterations = GetMaxIteration(w);

        xvt::VecPoint inLiers;
        xvt::VecPoint outLiers;
        while (maxIterations > sampleCount)
        {
            xvt::VecPoint selPoints = inliers;
            xvt::VecInt selIdx(mModelSize, -1);
            for (int i = 0; i < mModelSize; )
            {
                int t1 = dis(gen);
                bool isValidIdx = true;
                for (auto idx : selIdx)
                {
                    isValidIdx &= t1 != idx;
                    if (!isValidIdx)
                    {
                        break;
                    }
                }

                if (isValidIdx)
                {
                    selIdx[i] = t1;
                    selPoints.push_back(pointList[t1]);
                    i++;
                }
            }
            selIdx.clear();

            sampleCount++;
            auto tmpMod = FitLSQE(selPoints);
            auto rms = tmpMod.GetRMSE(selPoints);
            if (IsValidModel(tmpMod) && rms < mDataThreshold)
            {
                xvt::VecPoint curOutliers;
                //3.Count the samples are belong to the model
                GetInOutLiers(pointList, tmpMod, selPoints, curOutliers);

                if ((int)selPoints.size() > curModelSize)
                {
                    if (IsValidModel(tmpMod))
                    {
                        //4.Caculate the pInlier probability
                        res.mPInlier = GetProbability(selPoints.size(), totalSample);

                        //Probability of choosing a sample subset with no outliers
                        w = res.GetSuccessProbabily();
                        ::std::swap(outLiers, curOutliers);
                        ::std::swap(inLiers, selPoints);
                        curModelSize = inLiers.size();

                        //5.Update new maxIteration, modelThreshold
                        maxIterations = GetMaxIteration(w);

                        res(EResult::OK, "");
                    }
                }
            }

            if (mTimeOut < t.GetTotalElapsedTime(L"", false).count())
            {
                res(EResult::ER, "Ransac: Timeout!");
                break;
            }
        }

        if (!inLiers.empty())
        {
            auto oldRes = res.GetResult();
            auto oldMsg = res.GetMsg();
            static_cast<ModelResultType&>(res) = FitLSQE(inLiers);
            if (mIsRefineFinal)
            {
                RefineInOutLiers(res, inLiers, outLiers, 2 * mDataThreshold);
                static_cast<ModelResultType&>(res) = FitLSQE(inLiers);
            }
            ::std::swap(res.mPoints, inLiers);
            ::std::swap(res.mOutLiers, outLiers);
            res.Combine(oldRes, oldMsg);
        }
    }
    
    res.mProTime = t.Stop().count();
    return res;
}

template<class ModelResult>
inline
auto Ransac<ModelResult>::GetInOutLiers(xvt::VecPoint const& points
                                           , ModelResultType& tmpMod
                                           , xvt::VecPoint& inLiers
                                           , xvt::VecPoint& outLiers
)const->void
{
    int modSize = std::max(5 * GetMinModeSize(), mModelThreshold);
    inLiers.clear();
    for (auto const& p : points)
    {
        double t = tmpMod.GetRMSE(p);
        if (t < mDataThreshold)
        {
            inLiers.push_back(p);
        }
        else
        {
            outLiers.push_back(p);
        }

        if (mIsRefineModel && !inLiers.empty() &&(inLiers.size() % modSize==0))
        {
            tmpMod = FitLSQE(inLiers);

            RefineInOutLiers(tmpMod, inLiers, outLiers, mDataThreshold);
        }
    }
}

template<class ModelResult>
inline
auto Ransac<ModelResult>::GetMaxIteration(double w) const -> int
{
    auto b = log(1.0 - w);
    int n = INT_MAX;
    if(b<-DBL_EPSILON)
    {
        auto a = log(1.0 - mSuccessProbability);
        double m = ::std::ceil(a / b);
        n = m > INT_MAX ? INT_MAX : static_cast<int>(m);
    }
    return n;
}

template<class ModelResult>
constexpr
auto Ransac<ModelResult>::GetProbability(double value, double total) const -> double
{
    return xvt::SaturateCast(value / total, 0.0, 1.0);
}

template<class ModelResult>
inline
auto Ransac<ModelResult>::RefineInOutLiers(ModelResultType& tmpMod
                                           , xvt::VecPoint& inLiers
                                           , xvt::VecPoint& outLiers
                                           , double dataThreshold
) const -> void
{
    for (int i = 0; i < inLiers.size();)
    {
        double t = tmpMod.GetRMSE(inLiers[i]);
        if (t > dataThreshold)
        {
            outLiers.emplace_back(std::move(inLiers[i]));
            inLiers[i] = std::move(inLiers.back());
            inLiers.pop_back();
        }
        else
        {
            i++;
        }
    }

    for (int i=0; i <outLiers.size();)
    {
        double t = tmpMod.GetRMSE(outLiers[i]);
        if (t < dataThreshold)
        {
            inLiers.emplace_back(std::move(outLiers[i]));
            outLiers[i] = std::move(outLiers.back());
            outLiers.pop_back();
        }
        else
        {
            i++;
        }
    }
}

template<class ModelResult>
inline
void Ransac<ModelResult>::Update(double d, int modelThd, int timeout, bool finnalRefine, bool modelRefine)
{
    mDataThreshold = d;
    mModelThreshold = modelThd;
    mTimeOut = timeout;
    mIsRefineFinal = finnalRefine;
    mIsRefineModel = modelRefine;
}

template<class ModelResult>
inline
auto RansacResult<ModelResult>::GetSuccessProbabily() const -> double
{
    return pow(mPInlier, mModelSize);
}

template<class ModelResult>
void RansacResult<ModelResult>::DrawResult(cv::Mat& img, cv::Point offSetPoint, CVPen pen) const
{
    auto inLierColor  = COLOR_CV_GREEN;
    auto outLierColor = COLOR_CV_RED;
    pen.mColor        = cv::Scalar(0,125,125);

    ModelResult::DrawResult(img, offSetPoint, pen);

    //xvt::DrawPoints(img, mInLiers, inLierColor);
    xvt::DrawPoints(img, mOutLiers, outLierColor);
}

//! @} end of group Utils

}