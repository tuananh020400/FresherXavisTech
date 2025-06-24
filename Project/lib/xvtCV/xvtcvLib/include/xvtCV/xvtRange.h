#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/persistence.hpp>
#include <stdexcept>

namespace xvt {
//! @addtogroup Utils
//! @{

/**
 * @brief The range class store the lower - upper information.
 * it also support handling enable/disable checking value in the range [lower, upper]
 * @tparam T data type
*/
template <class T>
struct Range
{
private:
    /// Lower limit
    T mLower;
    /// Upper limit
    T mUpper;
public:
    /// Enable checking range. when disable xvt::Range::IsInRange() will always return true
    bool mIsEnable;

    Range()
    {
        Set(0, 0);
        mIsEnable = true;
    }

    Range(T const& l, T const& u, bool enable = true)
    {
        Set(l, u);
        mIsEnable = enable;
    }

    /**
     * @brief Set the lower/upper limit
     * while upper < lower => swap(upper, lower);
     * @param l Lower limit
     * @param u Upper limit
    */
    void Set(T const& l, T const& u)
    {
        mLower = l;
        mUpper = u;
        if (mUpper < mLower)
        {
            std::swap(mLower, mUpper);
        }
    }

    /**
     * @brief Get the upper limit
     * @return upper limit
    */
    T GetUpper() const
    {
        return mUpper;
    }

    /**
     * @brief Get the lower limit
     * @return lower limit
    */
    T GetLower() const
    {
        return mLower;
    }

    /**
     * @brief Equal comparator
     * @param other Other range
     * @return true if they are equaly.
     * @return false if they are inequaly.
    */
    bool operator==(const Range& other) const
    {
        return (mUpper == other.mUpper) && (mLower == other.mLower) && (mIsEnable == other.mIsEnable);
    }

    /**
     * @brief Check if value is in the range
     * @tparam T2 value type
     * @param value value need to check
     * @return true if value is in range [lower, upper]
     * @return false if value is not in range [lower, upper]
    */
    template<class T2>
    bool operator()(T2 const& value) const
    {
        return mIsEnable ? (value >= mLower && value <= mUpper) : true;
    }

    /**
     * @brief Check if value is in the range
     * @tparam T2 value type
     * @param value value need to check
     * @return true if value is in range [lower, upper]
     * @return false if value is not in range [lower, upper]
     */
    template<class T2>
    inline bool IsInRange(T2 const& value) const
    {
        return  operator()(value);
    }

    /**
     * @brief Write data to the cv::FileStorage file
     * @param rFs file storage
    */
    inline void Write(cv::FileStorage& rFs) const&
    {
        if (rFs.isOpened())
        {
            rFs << "mLower" << mLower << "mUpper" << mUpper << "mIsEnable" << (int)mIsEnable;
        }
        else
        {
            //Do nothing
        }
    }

    /**
     * @brief Read data from the cv::FileStorage file
     * @param node file storage node
    */
    inline void Read(cv::FileNode const& node)&
    {
        mLower = (T)node["mLower"];
        mUpper = (T)node["mUpper"];
        mIsEnable = (int)node["mIsEnable"];
    }

    /**
     * @brief Convert to string representation
    */
    inline
    std::string ToString()
    {
        return  std::to_string(mIsEnable) + "[" + std::to_string(mLower) + " " + std::to_string(mUpper) + "]";
    }
};

struct XVT_EXPORTS RotatedAngleRange
{
private:
    // mRefAngle: Reference Angle. Here we calculate how far from rotated angle to reference angle by using IsRotatedAngleInRange
    double mRefAngle;
    // mStdAngle: variable to determine shape of minAreaRect of an object (45 if square, 90 if not square).
    double mStdAngle;

    double mUpperOffsetAngle;
    double mLowerOffsetAngle;
public:
    bool mIsEnable;
    RotatedAngleRange()
    {
        Set(0, 0, 0, 90);
        this->mIsEnable = true;
    }

    RotatedAngleRange(double const& l, double const& u, double const& refAngle = 0, double const& stdAngle = 90, bool enable = true)
    {
        Set(l, u, refAngle, stdAngle);
        this->mIsEnable = enable;
    }

    void Set(double const& l, double const& u, double const& refAngle, double const& stdAngle)
    {
        if (refAngle < -stdAngle || refAngle > stdAngle || u < 0 || l < 0 ||
            u > stdAngle || l > stdAngle)
        {
            throw std::invalid_argument("Invalid Parameter");
        }
        else
        {
            this->mRefAngle = refAngle;
            this->mStdAngle = stdAngle;
            this->mUpperOffsetAngle = u;
            this->mLowerOffsetAngle = l;
        }
    }

    double GetRefAngle()
    {
        return this->mRefAngle;
    }

    double GetStdAngle()
    {
        return this->mStdAngle;
    }

    double GetUpperOffsetAngle()
    {
        return this->mUpperOffsetAngle;
    }

    double GetLowerOffsetAngle()
    {
        return this->mLowerOffsetAngle;
    }
    // Calculate angle distance between rotated angle and reference angle. 
    // Returns true if distance is within upper and lower threshold. Otherwise returns false.
    inline bool IsRotatedAngleInRange(double const& rotatedAngle) const
    {
        bool retVal = true;

        if (this->mIsEnable)
        {
            Range<double> angleRange = Range<double>(this->mRefAngle - this->mLowerOffsetAngle, this->mRefAngle + this->mUpperOffsetAngle, true);
            retVal = angleRange.IsInRange(rotatedAngle) ||
                ((rotatedAngle < 0) ? angleRange.IsInRange(rotatedAngle + mStdAngle * 2) : angleRange.IsInRange(rotatedAngle - mStdAngle * 2));
        }
        else
        {
            //Do nothing
        }

        return retVal;
    }

    inline void Write(cv::FileStorage& rFs) const&
    {
        if (rFs.isOpened())
        {
            rFs << "mLowerOffsetAngle" << mLowerOffsetAngle << "mUpperOffsetAngle" << mUpperOffsetAngle << "mIsEnable" << (int)mIsEnable;
        }
        else
        {
            //Do nothing
        }
    }

    inline void Read(cv::FileNode const& node)&
    {
        mLowerOffsetAngle = (double)node["mLowerOffsetAngle"];
        mUpperOffsetAngle = (double)node["mUpperOffsetAngle"];
        mIsEnable = (int)node["mIsEnable"];
    }
};

using Rangei = Range<int>;
using Rangef = Range<float>;
using Ranged = Range<double>;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// for convenience
#ifndef XPU_RANGE_INT
#define XPU_RANGE_INT       ::xvt::Range<int>
#endif // !XPU_RANGE_INT

#ifndef XPU_RANGE_FLOAT
#define XPU_RANGE_FLOAT     ::xvt::Range<float>
#endif // !XPU_RANGE_FLOAT

#ifndef XPU_RANGE_DOUBLE
#define XPU_RANGE_DOUBLE    ::xvt::Range<double>
#endif // !XPU_RANGE_DOUBLE


//class XVT_EXPORTS Ranged : public Range<double>
//{
//
//};

//! @} end of group Utils

}//xvt