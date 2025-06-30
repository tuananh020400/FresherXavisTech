#pragma once
#include "xvtCV/xvtDefine.h"
#include <cassert>
#include <functional>

namespace xvt {
//! @addtogroup Converting
//! @{

/**
 * @brief Convert a pixel value to millimeter units.
 *
 * This function converts a given pixel value to millimeters (mm) using the physical size of a pixel.
 * The conversion is done by multiplying the pixel value with the physical size of each pixel in millimeters.
 *
 * @param px The pixel value to convert to millimeters.
 * @param pixelSize The physical size of a single pixel in millimeters.
 *                  This represents how large one pixel is in millimeter units.
 * @param zoomScale zooming scale factor
 *
 * @return double The corresponding length in millimeters.
 *
 * @note The function is marked as `constexpr`, meaning the result is evaluated at compile time
 *       if the inputs are constant expressions.
 */
constexpr double Pixel2MM(double px, double pixelSize, double zoomScale=1.0)
{
    return px * (pixelSize / zoomScale);
}

/**
 * @brief Convert a millimeter value to pixel units.
 *
 * This function converts a length measurement in millimeters (mm) to pixel units using the
 * physical size of a pixel in millimeters. The conversion is performed by dividing the millimeter
 * value by the physical size of a pixel.
 *
 * @param mm The millimeter value to convert to pixels.
 * @param pixelSize The physical size of a single pixel in millimeters.
 *                  This represents how large one pixel is in millimeter units.
 *                  pixelSize should not equal to 0.
 * @param zoomScale zooming scale factor

 * @return double The corresponding length in pixels.
 *
 * @note The function is marked as `constexpr`, allowing for compile-time evaluation
 *       if the arguments are constant expressions.
 */
constexpr double MM2Pixel(double mm, double pixelSize, double zoomScale = 1.0)
{
    assert(pixelSize != 0);
    return mm * (zoomScale / pixelSize);
}

/**
 * @brief Class to handle pixel-related information.
 *
 * This class contains and manages three key properties of pixel information:
 * - Physical size: The size of a pixel in millimeters (provided by the manufacturer).
 * - Scale: The optical zooming scale factor of the capturing system.
 * - Optical size: The effective size of the pixel in the system, computed as:
 *   optical size = physical size / scale.
 *
 * The class also provides conversion methods between pixel units and millimeter units.
 */
class XVT_EXPORTS PixelInfo
{
public:
    PixelInfo(double px = 1.0, double s = 1.0) :mPhysicSize{ RefineInput(px) }
        , mScale{ RefineInput(s) }
    {
        mOpticalSize = mPhysicSize / mScale;
    }

    /**
     * @brief Convert pixel value to millimeter value.
     *
     * @param px The pixel value to be converted.
     * @return double The corresponding length in millimeters.
     *
     * @note The conversion is done using the optical size.
     */
    auto ToMillimeter(double const& px) const -> double { return px * mOpticalSize; }

    /**
     * @brief Convert millimeter value to pixel value.
     *
     * @param mm The millimeter value to be converted.
     * @return double The corresponding value in pixels.
     *
     * @note The conversion is done using the optical size.
     */
    auto ToPixel(double const& mm) const -> double { return mm / mOpticalSize; }

    /**
     * @brief Get the physical pixel size in milimeter
    */
    auto GetPhysicSize() const -> double { return mPhysicSize; }
    /**
     * @brief Set the physical pixel size in millimeters.
     *
     * Ensures the physical size is not less than the smallest representable positive value (epsilon).
     * Also recalculates the optical size.
     *
     * @param value The new physical size value to set. If less than epsilon, it is clamped to epsilon.
     * @return The new physical size value
     */
    auto SetPhysicSize(double value) -> double
    {
        mPhysicSize = RefineInput(value);
        UpdateOpticalSize();
        return mPhysicSize;
    }

    /**
     * @brief Get the optical scale factor.
    */
    auto GetScale() const -> double { return mScale; }
    /**
     * @brief Set the optical scale factor.
     *
     * Ensures the scale factor is smallest representable positive value (epsilon).
     * Recalculates the optical size based on the new scale factor.
     *
     * @param value The new scale value to set. If less than 0, it is clamped to epsilon.
     * @return new scale value
     */
    auto SetScale(double value) -> double
    {
        mScale = RefineInput(value);
        UpdateOpticalSize();
        return mScale;
    }

    /**
     * @brief Get the current optical size(in millimeters).
    */
    auto GetOpticalSize() const -> double { return mOpticalSize; }
    /**
     * @brief Set the optical size directly.
     *
     * When setting the optical size directly, the scale factor will be recalculated accordingly.
     * The physical size remains unchanged.
     *
     * @param value The new optical size. If less than epsilon, it is clamped to epsilon.
     * @return new optical size
     */
    auto SetOpticalSize(double value) -> double
    {
        mOpticalSize = RefineInput(value);
        mScale = mPhysicSize / mOpticalSize;
        return mOpticalSize;
    }

private:
    /**
     * @brief Refine the input value
     *
     * Ensures the value is not less than the smallest representable positive value (epsilon).
     *
     * @param value If less than epsilon, it is clamped to epsilon.
     * @return refined value
    */
    static constexpr double RefineInput(double value) { return std::max(value, std::numeric_limits<double>::epsilon()); }

    void UpdateOpticalSize() { mOpticalSize = mPhysicSize / mScale; };

private:
    static constexpr double DEFAULT_SIZE = 1.0;  //!< Default size for pixels, scale, and optical size.

    double mOpticalSize = DEFAULT_SIZE;  //!< The effective size of a pixel in the system (mm).
    double mPhysicSize = DEFAULT_SIZE;   //!< The manufacturer's physical size of a pixel (mm).
    double mScale = DEFAULT_SIZE;        //!< The scale factor applied to the pixel size.
};

/**
 * @brief Class that supports to handle pixel to milimeter conversion
 * 
 * @note This one will be removed in the future, please use PixelInfo and std::Shared_ptr<PixelInfo>
 * instead of this class.
*/
class XVT_EXPORTS PixelRef
{
public:
    explicit PixelRef(double const& vl) : mPixelSize{ std::cref(vl) } {}
    explicit PixelRef(double&& vl)=delete;
    //!< Convert from pixel to millimeter
    auto ToMilimet(double const& px) const -> double { return px * mPixelSize.get(); }
    //!< Convert from milimeter to pixel
    auto ToPixel(double const& mm) const -> double { return mm / mPixelSize.get(); }
    //!< Get the pixel size in millimeter
    auto GetPixelSize() const -> double { return mPixelSize.get(); }
public:
    //!< Pixel size in mm.
    std::reference_wrapper<const double> mPixelSize;
};

//! @} end of group Converting

}
