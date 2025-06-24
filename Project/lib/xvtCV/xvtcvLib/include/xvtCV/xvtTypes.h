#pragma once
#include "xvtCV/xvtEnumTemplate.h"
#include <opencv2/core/types.hpp>
#include <vector>
#include <utility>

namespace xvt {
//! @addtogroup Utils
//! @{

using VecInt        = std::vector<int>;         //!< Vector of int
using VecFloat      = std::vector<float>;       //!< Vector of float
using VecDouble     = std::vector<double>;      //!< Vector of double

using VecString     = std::vector<std::string>; //!< Vector of std::string
using VecWString    = std::vector<std::wstring>;//!< Vector of std::wstring

using VecPoint      = std::vector<cv::Point>;   //!< Vector of cv::Point
using VecPoint2f    = std::vector<cv::Point2f>; //!< Vector of cv::Point2f
using VecPoint2d    = std::vector<cv::Point2d>; //!< Vector of cv::Point2d

using VecVecPoint   = std::vector<VecPoint>;    //!< Vector of cv::Point Vector
using VecVecPoint2f = std::vector<VecPoint2f>;  //!< Vector of cv::Point2f Vector
using VecVecPoint2d = std::vector<VecPoint2d>;  //!< Vector of cv::Point2d Vector

using VecRect       = std::vector<cv::Rect>;    //!< Vector of cv::Rect
using VecRect2f     = std::vector<cv::Rect2f>;  //!< Vector of cv::Rect2f
using VecRect2d     = std::vector<cv::Rect2d>;  //!< Vector of cv::Rect2d

using VecKeyValueStr = std::vector<std::pair<std::string, std::string>>;//!< header-data vector

/**
 * @brief Types of fitting method
*/
enum class FittingMethod : int
{
      RANSAC       = 0 //!< Random Sample Consensus
    , LEAST_SQUARE = 1 //!< Minimizing the sum of the squares of the error
};
DEFINE_CONVERT_ENUM_FUNCS(FittingMethod);

/**
 * @brief Saturate cast a value in the range [from, to]
 * 
 * if value > to: value = to
 * if value < from: value = from
 * 
 * @tparam T value type
 * @param value value to cast
 * @param from min value
 * @param to max value
 * @return new value after cast away.
*/
template<typename T>
static inline
T SaturateCast(T value, T from, T to)
{
    return (std::min)(cv::saturate_cast<T>(to), (std::max)(cv::saturate_cast<T>(from), value));
}

/**
 * @brief Get a point that should be in the rect[from, to]
 * 
 * - p.x is cast to range [from.x, to.x]
 * - p.y is cast to range [from.y, to.y]
 * 
 * @param p point to cast
 * @param from first point
 * @param to second point
 * @return new point that in the rang rect[from, to]
 * @sa xvt::SaturateCast()
*/
template<>
static inline
cv::Point SaturateCast<cv::Point>(cv::Point p, cv::Point from, cv::Point to)
{
    assert(from.x <= to.x && from.y <= to.y);
    return cv::Point(SaturateCast<int>(p.x, from.x, to.x), SaturateCast<int>(p.y, from.y, to.y));
}

/**
 * @brief Casts a std::unique_ptr<_Tx> to std::unique_ptr<_Ty>
 *
 * This function safely casts a `std::unique_ptr<_Tx>` to `std::unique_ptr<_Ty>` using `dynamic_cast`.
 * If the cast fails, a nullptr is returned. It ensures at runtime that the _Tx pointer is convertible to a _Ty pointer.
 *
 * @tparam _Ty The class type.
 * @tparam _Tx The class type.
 * @tparam _Dx The delete function of _Tx type.
 * @param otherPtr A unique pointer to an instance of the _Tx class.
 * @return std::unique_ptr<_Ty> If successful.
 * @return nullptr If the cast fails.
 */
template<class _Ty, class _Tx, class _Dx = std::default_delete<_Tx>>
auto DynamicCastUniquePtr(std::unique_ptr<_Tx, _Dx>&& otherPtr)->std::unique_ptr<_Ty>
{
    // Perform a dynamic_cast at runtime to ensure safe conversion
    const auto _Ptr = dynamic_cast<typename std::unique_ptr<_Ty>::element_type*>(otherPtr.get());
    if (_Ptr)
    {
        // Release the ownership from otherPtr and return a unique_ptr to _Ty
        otherPtr.release();  // Release ownership
        return std::unique_ptr<_Ty>(_Ptr);
    }

    // If cast fails, return nullptr
    return {};
}

/**
 * @brief Casts a std::unique_ptr<_Tx> to std::shared_ptr<_Ty>
 *
 * This function safely casts a `std::unique_ptr<_Tx>` to `std::shared_ptr<_Ty>` using `dynamic_pointer_cast`.
 * The ownership is transferred from `std::unique_ptr` to `std::shared_ptr`. If the cast fails, it returns a nullptr
 * and the `std::unique_ptr` still keep the ownership of data.
 *
 * @tparam _Tx The class type.
 * @tparam _Ty The class type.
 * @param otherPtr A unique pointer to an instance of the _Tx class.
 * @return If successful, returns a shared pointer to the _Ty type'.
 * @return If the cast fails, returns nullptr.
 */
template<class _Ty, class _Tx, class _Dx = std::default_delete<_Tx>>
auto DynamicCastSharedPtr(std::unique_ptr<_Tx, _Dx>&& otherPtr)->std::shared_ptr<_Ty>
{
    // Perform a dynamic_cast at runtime to ensure safe conversion
    const auto _Ptr = dynamic_cast<typename std::shared_ptr<_Ty>::element_type*>(otherPtr.get());
    if (_Ptr)
    {
        // Release the ownership from otherPtr and return a unique_ptr to _Ty
        otherPtr.release();  // Release ownership
        return std::shared_ptr<_Ty>(_Ptr);
    }

    // If cast fails, return nullptr
    return {};
}

//! @} end of group Utils
}
