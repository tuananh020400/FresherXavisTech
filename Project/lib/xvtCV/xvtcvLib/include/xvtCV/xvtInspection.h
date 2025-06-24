#pragma once
#include "xvtCV/IInspection.h"
#include "xvtCV/xvtCSV.h"

namespace xvt {

//! @addtogroup Interface
//! @{

/**
 * @brief InspectionResult for image inspection.
 *
 * This interface provides methods for managing inspection results, including
 * setting and getting the result, handling messages, and drawing the results
 * on images. It also offers utility methods for combining results, messages
 * and save the result to CSV file.
 * @see xvt::IInspectionResult, xvt::CSV
*/
class XVT_EXPORTS InspectionResult :
      public IInspectionResult
    , public CSV
{
public:
    explicit InspectionResult(xvt::EResult result = xvt::EResult::UC, std::string const& msg = "");

    virtual void SetROI(cv::Rect const& roi) override { mROI = roi; }

    virtual auto GetROI() const -> cv::Rect override { return mROI; }

    virtual void SetResult(EResult const& result) override
    {
        mResult = result;
    }

    virtual void SetResult(bool const& result) override
    {
        mResult = result ? EResult::OK : EResult::NG;
    }

    virtual auto GetResult() const -> xvt::EResult override
    {
        return mResult;
    }
    virtual auto GetResultStr()const->std::string override;
    virtual auto GetMsg() const& -> std::string const& override
    {
        return mMsg;
    }
    virtual auto GetMsg() && -> std::string override
    {
        return std::move(mMsg);
    }
    virtual void SetMsg(std::string const& msg = "")
    {
        mMsg = msg;
    }
    virtual void SetMsg(std::string&& msg)
    {
        std::swap(mMsg, msg);
    }
    virtual auto Clone() const -> std::unique_ptr<IInspectionResult> override
    {
        return std::make_unique<InspectionResult>(*this);
    }
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    virtual auto DrawResultStr(cv::Mat& image
                               , std::string const& name = ""
                               , CVPen const& pen = CVPen()
                               , cv::Point const& pos = cv::Point()
                               , bool isDrawOKResult = false
    ) const->cv::Point;
    virtual auto DrawMsg(cv::Mat& image, CVPen const& pen, cv::Point const& pos) const->cv::Point override;

    /// CSV support functions
    virtual auto GetTitleStr()const->std::string override
    {
        return std::string("Inspection Result\n");
    }

    /**
     * @brief Save result to the file
     * 
     * Just support to save result to as CSV format
     * 
     * @param path Path to the file
     * @param imgName Name of image
     * @param isNewFile whether to create the new file or append to existed file
     * @return true if saving is success
    */
    virtual bool Save(  std::wstring const& path
                      , std::wstring const& imgName = L""
                      , bool isNewFile = true
    )const override;

    /// Transform the line base on the tranform matrix m
    virtual auto Transform(cv::Mat const& m) & ->void;

    /// Return top-left, top-right, bottom-right, bottom-left
    auto GetCornerPoint() const->std::array<cv::Point, 4>;

    /// Return vector index of 4 corner point
    auto GetCornerPointIdx() const->std::vector<int>;
public:
    /// Enable/Disbale the object function
    bool mEnable = true;

    /// Flag to draw the ROI on the result image
#ifdef _DEBUG
    bool mShowRoi = _DEBUG;
#else
    bool mShowRoi = false;
#endif // _DEBUG

    /// Inspected Points
    std::vector<cv::Point> mPoints;
    /// Procesing time in ms
    double mProTime = 0.0;
    /// Date and Time of Inspection
    std::string mDateTime = "";

private:
    /// Object ROI
    cv::Rect    mROI;
    /// Inspected Result
    EResult     mResult = EResult::UC;
    /// Information Message
    std::string mMsg;
};// Inspection class
/**@}*/ //end of group interface

}//xvt namespace