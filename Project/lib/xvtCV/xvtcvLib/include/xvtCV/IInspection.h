#pragma once
#include "xvtCV/xvtEnumTemplate.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTypes.h"
#include <opencv2/core/types.hpp>
#include <string>

namespace xvt {

//! @addtogroup Interface
//! @{

/**
 * @brief Inspection Result Enum
 * [Enum EResult]
*/
enum class EResult
{
    UC = -1 //!< Un-check/ not inspect yet
   ,OK =  0 //!< OK/passed/normal result
   ,NG =  1 //!< NG/failed/abnormal result
   ,ER =  2 //!< Some errors existed, process can't be finished
};
DEFINE_CONVERT_ENUM_FUNCS(EResult);
//! [Enum EResult]

/// <summary>
/// Combine two EResult variales
/// </summary>
/// 
/// Combine table:
///    |           ||   UC |OK |NG |ER   |
///    |   -------:|--:|:-:|:-:|:-:|:----|
///    |**result2**|UC |UC |OK |NG |ER   |
///    |^          |OK |OK |OK |NG |ER   |
///    |^          |NG |NG |NG |NG |ER   |
///    |^          |ER |ER |ER |ER |ER   |
///    |           ||**result1** ||||
/// 
/// <param name="result1">first variable</param>
/// <param name="result2">second variable</param>
/// <returns>Combined result as table above</returns>
inline
EResult CombineResult(EResult const& result1, EResult const& result2)
{
    //auto res = (result2 >= EResult::ER)
    //    || ((result2 == EResult::NG) && (EResult::ER != result1))
    //    || ((result2 == EResult::OK) && (EResult::UC == result1))
    //    ? result2 : result1;

    /*auto res = (result2 == EResult::ER || result1 == EResult::ER) ? EResult::ER :
        ((result2 == EResult::NG || result1 == EResult::NG) ? EResult::NG :
         (result2 == EResult::OK || result1 == EResult::OK) ? EResult::OK : EResult::UC
         );*/

    return result2 > result1 ? result2 : result1;
}

/// <summary>
/// Combine two EResult types by operator "&"
/// </summary>
/// <example>
/// <code>
/// EResult e1;
/// EResult e2;
/// EResult e3 = e1 & e2;
/// </code>
/// </example>
/// 
/// <param name="result1">first variable</param>
/// <param name="result2">second variable</param>
/// <returns>combined result</returns>
/// **See**: 
/// <see cref="CombineResult"> CombineResult</see>
inline
EResult operator&(EResult const& result1, EResult const& result2)
{
    return CombineResult(result1, result2);
}

/// <summary>
/// Combine two EResult types by operator "&="
/// </summary>
/// <example>
/// <code>
/// EResult e1;
/// EResult e2;
/// EResult e3 &= e1;
/// </code>
/// </example>
/// <param name="result1">first variable</param>
/// <param name="result2">second variable</param>
/// <returns>combined result</returns>
/// **See**: 
/// <see cref="CombineResult"> CombineResult</see>
inline
EResult operator&=(EResult& result1, EResult const& result2)
{
    result1 = CombineResult(result1, result2);
    return result1;
}

/// <summary>
/// Get the color by the EResult
/// </summary>
/// <example>
/// <code>
/// auto gray_color = ChoseColor(EResult::UC);
/// auto green_color = ChoseColor(EResult::OK);
/// auto red_color1 = ChoseColor(EResult::NG);
/// auto red_color2 = ChoseColor(EResult::ER);
/// </code>
/// </example>
/// <param name="result">input result</param>
/// <returns>color</returns>
XVT_EXPORTS
inline
cv::Scalar ChoseColor(EResult result)
{
    cv::Scalar color = cv::Scalar(125, 125, 125);;
    switch (result)
    {
    case xvt::EResult::OK:
        color = cv::Scalar(0, 255, 0);
        break;
    case xvt::EResult::UC:
        color = cv::Scalar(125, 125, 125);
        break;
    case xvt::EResult::NG:
    case xvt::EResult::ER:
    default:
        color = cv::Scalar(0, 0, 255);
        break;
    }

    return color;
}

/**
 * @brief Interface for image inspection result.
 *
 * This interface provides methods for managing inspection results, including 
 * setting and getting the result, handling messages, and drawing the results 
 * on images. It also offers utility methods for combining results and messages.
*/
class XVT_EXPORTS IInspectionResult
{
public:
    /**
     * @brief Get the inspection result.
     *
     * @return Returns `EResult::OK` if the result is successful, otherwise returns an `EResult::ER` or `EResult::OK`(No Good) status.
    */
    virtual auto GetResult() const -> EResult = 0;
    /// Set the result
    virtual void SetResult(EResult const& result) = 0;
    /// Set the result, true->OK, false->NG
    virtual void SetResult(bool const& result) = 0;

    /**
     * @brief Get the message associated with the inspection result, typically used when an error occurs.
     *
     * @return A constant reference to the message string.
    */
    virtual auto GetMsg() const& -> std::string const& = 0;
    /**
      * @brief Get the message associated with the inspection result.
      *
      * @return A string containing the message.
    */
    virtual auto GetMsg() && -> std::string = 0;
    /**
     * @brief Set the message associated with the inspection result.
     *
     * @param msg The message to be set.
    */
    virtual void SetMsg(std::string const& msg = "") = 0;
    /**
     * @brief Set the message associated with the inspection result using an rvalue reference.
     *
     * @param msg The message to be set.
    */
    virtual void SetMsg(std::string && msg) = 0;

    /**
     * @brief Draw the inspection results onto an image.
     *
     * All the figure data part should be drawn by this funtion such as:
     * line, point, circle, rectangle ...
     * 
     * @param img The image (either grayscale or BGR) on which to draw the results.
     * @param offSetPoint The offset position for drawing.
     * @param pen The drawing tool information.
     * @see xvt::CVPen
    */
    virtual void DrawResult(cv::Mat& img
                            , cv::Point offSetPoint=cv::Point()
                            , CVPen pen= CVPen()
    ) const = 0;

    /**
     * @brief Draw the text data onto the image
     * 
     * All the text data part should be drawn by this funtion such as:
     * length, OK-NG text, error message...
     * 
     * @param image An image to be drawn on
     * @param name Name of the drawing part
     * @param pen Drawing pen infromation
     * @param pos Start drawing position
     * @param isDrawOKResult whether to draw text when result is OK or not.
     * @return Bottom-right position of drawing.
    */
    virtual auto DrawResultStr(cv::Mat& image
                               , std::string const& name = ""
                               , CVPen const& pen = CVPen()
                               , cv::Point const& pos = cv::Point()
                               , bool isDrawOKResult = false
    ) const->cv::Point = 0;

    /**
     * @brief Draw the message onto the image.
     *
     * @param image The image (either grayscale or BGR) on which to draw the message.
     * @param pen The drawing tool information.
     * @param offset The offset position for drawing.
     * @return The bottom-right position after drawing.
    */
    virtual auto DrawMsg(cv::Mat& image
                         , CVPen const& pen
                         , cv::Point const& offset
    ) const->cv::Point = 0;

    /**
      * @brief Set the region of interest (ROI) for the inspection result.
      *
      * @param roi The region of interest to be set.
     */
    virtual void SetROI(cv::Rect const& roi) = 0;
    /**
     * @brief Get the region of interest (ROI) for the inspection result.
     *
     * @return The region of interest.
    */
    virtual auto GetROI() const->cv::Rect = 0;

    /**
     * @brief Virtual destructor.
     *
     * Ensures proper cleanup of derived class objects.
    */
    virtual ~IInspectionResult() = default;

    /**
     * @brief Clone the inspection result object.
     *
     * This method creates and returns a deep copy of the inspection result object.
     * 
     * @return A unique pointer to the cloned inspection result object.
    */
    virtual auto Clone() const -> std::unique_ptr<IInspectionResult> = 0;

    /**
     * @brief Convert the inspection result to a string.
     *
     * @return A string representation of the inspection result.
    */
    virtual auto GetResultStr() const->std::string;

    /**
     * @brief Convert the inspection result, including the message, to a string.
     *
     * @return A string representation of the inspection result and message.
    */
    virtual auto ToString() const->std::string;

    /**
     * @brief Save the result to file
     * 
     * File extension can be .csv, .ini, .xml or any kind of type
     * Derived classes must implement for specific file type.
     * 
     * @param path Path to the file
     * @param imgName Name of inspected image
     * @param isNewFile whether to create new file or append to existed file
     * @return 
    */
    virtual bool Save(std::wstring const& path
                      , std::wstring const& imgName = L""
                      , bool isNewFile = true
    ) const;

    ///  Check if the result is EResult::OK
    bool IsOK() const { return GetResult() == EResult::OK; }
    ///  Check if the result is EResult::NG
    bool IsNG() const { return GetResult() == EResult::NG; }
    ///  Check if the result is EResult::ER
    bool IsER() const { return GetResult() == EResult::ER; }
    ///  Check if the result is EResult::UC
    bool IsUC() const { return GetResult() == EResult::UC; }

    /// Set the result and message
    void operator()(EResult const& result, std::string const& msg)
    {
        SetResult(result);
        SetMsg(msg);
    }

    /**
     * @brief Combine the result
     * @param result Inspected result
     * @see xvt::CombineResult
    */
    void CombineResult(EResult const& result)
    {
        SetResult(xvt::CombineResult(GetResult(), result));
    }

    /**
     * @brief Combine the result when the input is a boolean, true->OK, false->NG
     * @param result Inspected result
     * @overload
    */
    void CombineResult(bool const& result)
    {
        SetResult(xvt::CombineResult(GetResult(), result ? EResult::OK : EResult::NG));
    }

    /**
     * @brief Add a message
     * @param msg Message
    */
    void CombineMsg(std::string const& msg)
    {
        auto str = GetMsg();
        auto str3 = str.empty() || msg.empty() ? "" : "\n";
        SetMsg(str + str3 + msg);
    }

    /**
     * @brief Add a message to the current message, it is same as #CombineMsg
     *
     * @param msg The message to be added.
    */
    void AddMsg(std::string const& msg)
    {
        CombineMsg(msg);
    }

    /**
     * 
     * @brief Combine the result and message from another `IInspectionResult` object.
     *
     * @param result The other `IInspectionResult` object to combine.
    */
    void CombineResult(IInspectionResult const* result)
    {
        if (result != nullptr)
        {
            CombineResult(result->GetResult());
            CombineMsg(result->GetMsg());
        }
    }

    /**
     * @brief Combine the result and message
     * @param result Inspected result
     * @param msg Inspected message
     * @see xvt::CombineResult
    */
    void Combine(EResult const& result, std::string const& msg)
    {
        CombineResult(result);
        CombineMsg(msg);
    }

    /**
     * @brief Get the result and message
     * @param result ouput result
     * @param msg output message
    */
    void GetResult(EResult& result, std::string& msg)
    {
        result = GetResult();
        msg = GetMsg();
    }

    /**
     * @brief Get the color based on the result.
     *
     * @return The color corresponding to the result.
     * @see xvt::ChoseColor
    */
    auto GetResultColor() const -> cv::Scalar { return ChoseColor(GetResult()); }

    /**
     * @brief Combine the result when the input is a boolean using the `&=` operator.
     *
     * @param other The boolean value to combine.
     * @return Reference to the current instance.
     * @see #CombineResult
    */
    IInspectionResult& operator&=(bool const& other)
    {
        CombineResult(other);
        return *this;
    }

    /**
     * @brief Combine the result with another `EResult` using the `&=` operator.
     *
     * @param other The other result to combine.
     * @return Reference to the current instance.
    */
    IInspectionResult& operator&=(EResult const& other)
    {
        CombineResult(other);
        return *this;
    }

    /**
     * @brief Combine the result with another `IInspectionResult` using the `&=` operator.
     *
     * @param other The other inspection result to combine.
     * @return Reference to the current instance.
    */
    IInspectionResult& operator&=(IInspectionResult const& other)
    {
        CombineResult(&other);
        return *this;
    }
};

/**
 * @brief Interface for image inspection.
 * 
 * This interface provides methods for inspecting single or multiple images and 
 * obtaining the inspection results. It also includes a method for cloning the inspection object.
*/
class XVT_EXPORTS IInspection
{
public:
    /**
     * @brief Virtual destructor.
     *
     * Ensures proper cleanup of derived class objects.
    */
    virtual ~IInspection() = default;

    /**
     * @brief Inspects a single image.
     *
     * This method analyzes the input image and returns a unique pointer to the inspection result.
     *
     * @param src Input image to be inspected.
     * @return A unique pointer to the inspection result.
    */
    virtual auto Inspect(cv::Mat const& src) const->std::unique_ptr<IInspectionResult> = 0;

    /**
     * @brief Inspects multiple images.
     *
     * This method analyzes a collection of input images and returns a pointer to the inspection result.
     *
     * @param srcs Vector of input images to be inspected.
     * @return A unique pointer to the inspection result.
    */
    virtual auto Inspect(std::vector<cv::Mat> const& srcs) const->std::unique_ptr<IInspectionResult> = 0;

    /**
     * @brief Load data from saved file
     * 
     * File extension can be .ini, .xml or any.
     * Derived classes must implement for specific file type.
     * 
     * @param path File path
     * @return true if loading successed otherwise false.
    */
    virtual auto Load(std::wstring const& path)&->bool = 0;

    /**
     * @brief Save data to a file
     *
     * File extension can be .ini, .xml or any.
     * Derived classes must implement for specific file type.
     *
     * @param path File path
     * @return true if saving successed otherwise false.
    */
    virtual auto Save(std::wstring const& path)const->bool = 0;

    /**
     * @brief Clones the inspection object.
     *
     * This method creates and returns a deep copy of the inspection object.
     *
     * @return A unique pointer to the cloned inspection object.
    */
    virtual auto Clone() const->std::unique_ptr<IInspection> = 0;
};

using UniquePtrInspectResult = std::unique_ptr<IInspectionResult>;
using SharedPtrInspectResult = std::shared_ptr<IInspectionResult>;

/**@}*/ //end of group DataStorage
}

