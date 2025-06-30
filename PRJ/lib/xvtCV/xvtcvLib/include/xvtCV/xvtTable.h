#pragma once
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtDefine.h"

#pragma warning(push, 0)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#pragma warning(pop)

namespace xvt {

/**
* @addtogroup Drawing
* @{
*/

/**
* @class LineSegment
* @brief Represents a line segment with direction, length, and style properties.
*
* This class allows the creation and manipulation of line segments defined by a direction, length, and style.
* It provides methods for setting the style and length, as well as calculating the endpoint and drawing the segment on an image.
*/
class XVT_EXPORTS LineSegment
{
public:

    /**
     * @brief Constructor for LineSegment.
     *
     * @param dir The direction of the line segment (unit vector).
     * @param l The length of the line (optional).
     * @param style The style of the line (optional, uses CVPen).
     */
    LineSegment(  cv::Point2d dir
                , std::shared_ptr<int> const& l = nullptr
                , std::shared_ptr<CVPen> const& style = nullptr
    ) : mDirection{ dir }, mLength{ l }, mStyle{ style }{}

    /**
     * @brief Sets the style of the line segment using a reference to a CVPen.
     *
     * @param style The CVPen style to set.
     */
    void SetStyle(CVPen const& style)& { if(mStyle) *mStyle = style; }

    /**
     * @brief Sets the style of the line segment using a shared pointer to a CVPen.
     *
     * @param style The shared pointer to the CVPen style to set.
     */
    void SetStyle(std::shared_ptr<CVPen> const& style)& { mStyle = style; }

    /**
    * @brief Sets the length of the line segment.
    *
    * @param l The new length to set.
    */
    void SetLength(int l)& { if (mLength) *mLength = l; }
    void SetLength(std::shared_ptr<int> l)& { mLength = l; }

    /**
    * @brief Gets the length of the line segment.
    *
    * @return The length of the line segment, or 0 if not set.
    */
    auto GetLength()const& -> int { return (mLength != nullptr) ? *mLength : 0; }

    /**
     * @brief Gets the position of the line endpoint based on a starting point.
     *
     * @param pos The starting position.
     * @return The calculated endpoint of the line segment.
     */
    auto GetPos(cv::Point pos) const& { return cv::Point(GetLength() * mDirection) + pos; }

    /**
    * @brief Draws the line segment on the provided image.
    *
    * @param img The image to draw on.
    * @param pos The position to draw from.
    * @return The endpoint after drawing the line.
    */
    auto Draw(cv::Mat& img, cv::Point pos) const&->cv::Point;
public:
    bool mVisible = true; ///< Visibility of the line segment.
    cv::Point2d mDirection = cv::Point2d(1, 0); ///< The direction of the line segment.
    std::shared_ptr<CVPen> mStyle  = nullptr; ///< The style (pen) of the line segment.
    std::shared_ptr<int>   mLength = nullptr; ///< The length of the line segment.
};

/// Cell table data information
class XVT_EXPORTS Cell
{
public:

    /**
     * @brief Default constructor for the Cell class.
     *
     * @param str The text content of the cell.
     * @param fontPen The font style (pen) used in the cell.
     */
    Cell(std::string const& str = "", std::shared_ptr<CVPen> fontPen = nullptr, std::shared_ptr<CVPen> borderPen = nullptr)
        :mText{ str }, mFontStyle{ fontPen }
    {
        SetBorderStyle(borderPen);
    };

    /**
     * @brief Constructor for the Cell class with size and border customization.
     *
     * @param str The text content of the cell.
     * @param w The width of the cell (shared pointer).
     * @param h The height of the cell (shared pointer).
     * @param fontPen The font style (pen) used in the cell.
     * @param borderPen The border style (pen) used for the cell borders.
     */
    Cell(std::string const& str
         , std::shared_ptr<int> const& w
         , std::shared_ptr<int> const& h
         , std::shared_ptr<CVPen> fontPen = nullptr
         , std::shared_ptr<CVPen> borderPen = nullptr
    ) :mText{ str }, mFontStyle{ fontPen }
    {
        SetSize(w, h);
        SetBorderStyle(borderPen);
    }

    /**
     * @brief Gets the height of the cell.
     *
     * @return The height of the cell.
     */
    virtual auto GetHeight() const& -> int;

    /**
     * @brief Gets the width of the cell.
     *
     * @return The width of the cell.
     */
    virtual auto GetWidth() const& -> int;

    /**
     * @brief Draws the cell on the provided image at the specified offset.
     *
     * @param img The image to draw on.
     * @param offset The offset position.
     * @return The position after drawing the cell.
     */
    virtual auto Draw(cv::Mat& img, cv::Point offset = cv::Point())const& ->cv::Point;

    /**
     * @brief Draw the text
     * @param img The image to draw on.
     * @param offset The offset position.
     * @return The position bottom right after drawing the text.
    */
    virtual auto DrawText(cv::Mat& img, cv::Point offset = cv::Point())const& ->cv::Point;

    /**
     * @brief Draw all borders of the cell
     * @param img The image to draw on.
     * @param offset The offset position.
     * @return The bottom right position after drawing the cell.
    */
    virtual auto DrawBorder(cv::Mat& img, cv::Point offset = cv::Point())const& ->cv::Point;

    /**
     * @brief Sets the border style for all borders of the cell.
     *
     * @param borderStyle The border style (pen) to apply.
     */
    virtual void SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle)&;

    /**
    * @brief Sets the border style for a specific border of the cell.
    *
    * @param borderStyle The border style (pen) to apply.
    * @param idx The index of the border (0: top, 1: left, 2: bottom, 3: right).
    */
    virtual void SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle, int idx)&;

    /**
     * @brief Sets the width and height of the cell by updating the lengths of its borders.
     *
     * This function updates the lengths of the cell's borders to match the given width (`w`) and height (`h`).
     * The borders are updated as follows:
     * - Border 0 and 2: Updated with the provided width (`w`).
     * - Border 1 and 3: Updated with the provided height (`h`).
     *
     * @param w A shared pointer to the width to set for the cell.
     * @param h A shared pointer to the height to set for the cell.
     */
    void SetSize(std::shared_ptr<int>const& w, std::shared_ptr<int> const& h)&;

public:
    bool    mVisible  = true;
    bool    mIsMerged = false;

    std::string mText = "";
    TextAlign mTextAlign = TextAlign::MIDDLE_CENTER;
    std::shared_ptr<CVPen> mFontStyle = nullptr;

    std::vector<std::weak_ptr<Cell>> mRefCells;

public:
    std::shared_ptr<LineSegment> mBorderTop    = std::make_shared<LineSegment>(cv::Point2d(1, 0));
    std::shared_ptr<LineSegment> mBorderLeft   = std::make_shared<LineSegment>(cv::Point2d(0, 1));
    std::shared_ptr<LineSegment> mBorderBottom = std::make_shared<LineSegment>(cv::Point2d(1, 0));
    std::shared_ptr<LineSegment> mBorderRight  = std::make_shared<LineSegment>(cv::Point2d(0, 1));
};

/**
 * @brief Merged Cell
*/
class XVT_EXPORTS CellMerged : public Cell
{
public:
    CellMerged() = default;
    CellMerged(Cell const& other) : Cell(other) {}

    auto GetHeight() const& -> int override;
    auto GetWidth()  const& -> int override;

    void SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle) & override;
    void SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle, int idx) & override;

    auto DrawBorder(cv::Mat& img, cv::Point offset = cv::Point())const& ->cv::Point override;

    cv::Size mMergedSize;
};

/**
 * @brief Sets the text alignment for all cells in the provided vector.
 */
inline
void SetCellsAlign(std::vector<std::shared_ptr<Cell>> & cells, TextAlign align)
{
    for(auto &c: cells) c->mTextAlign = align;
}

/**
 * @brief Sets the font style for all cells in the provided vector.
 */
inline
void SetCellsFontStyle(std::vector<std::shared_ptr<Cell>> & cells, std::shared_ptr<CVPen> font)
{
    for(auto &c: cells) c->mFontStyle = font;
}

/**
 * @brief Sets the visibility for all cells in the provided vector.
 */
inline
void SetCellsVisible(std::vector<std::shared_ptr<Cell>> & cells, bool visible)
{
    for(auto &c: cells) c->mVisible = visible;
}

/**
 * @brief Sets the border style for all borders of the cells in the provided vector.
 */
inline
void SetCellsBorderStyle(std::vector<std::shared_ptr<Cell>> & cells, std::shared_ptr<CVPen> const& borderStyle)
{
    for(auto &c: cells) c->SetBorderStyle(borderStyle);
}

/**
 * @brief Sets the border style for a specific border (top, left, bottom, or right) for each cell in the vector.
 */
inline
void SetCellsBorderStyle(std::vector<std::shared_ptr<Cell>> & cells, std::shared_ptr<CVPen> const& borderStyle, int idx)
{
    for(auto &c: cells) c->SetBorderStyle(borderStyle, idx);
}

/**
 * @brief Table class that support to drawing table data on image
 * 
 * The table is structured as a collection of rows and columns, where each
 * cell contains data. It provides methods for adding columns, setting styles,
 * and customizing dimensions.
 * 
 * Table data struct as: tale(row(col)).
*/
class XVT_EXPORTS Table
{
public:
    using CellData   = std::shared_ptr<Cell>;
    using RowData    = std::vector<CellData>;
    using ColumnData = std::vector<CellData>;
    using TableData  = std::vector<RowData>;

    Table()=default;
	
    /**
     * @brief Constructs a Table with specified width and height.
     *
     * @param width Default width for each column.
     * @param height Default height for each row.
    */
    Table(int width, int height);
    
    /**
     * @brief Constructs a Table with font and border styles.
     *
     * @param fontStyle Font styling for text inside the table cells.
     * @param borderStyle Border styling for the table outline.
    */
    Table(std::shared_ptr<CVPen> fontStyle, std::shared_ptr<CVPen> borderStyle);
    
    /**
     * @brief Constructs a Table with specified dimensions and styles.
     *
     * @param width Width for each column.
     * @param h Height for each row.
     * @param fontStyle Font styling for text inside the table cells.
     * @param borderStyle Border styling for the table outline.
    */
    Table(int width, int height, std::shared_ptr<CVPen> fontStyle, std::shared_ptr<CVPen> borderStyle);
    
    /**
     * @brief Adds a new column to the table.
     *
     * @param colData The data for the new column.
     * @param width The width of the new column.
     */
    void AddColumn(ColumnData&& columnData, int width = -1)&;

    /**
     * @brief Adds a new column to the table (by copying the data).
     *
     * @param columnData The data for the new column.
     * @param width The width of the new column.
     */
    void AddColumn(ColumnData const& columnData, int width = -1)&;

    /**
     * @brief Adds a new column to the table using string data.
     *
     * @param columnData A vector of strings for the new column.
     * @param width The width of the new column.
     */
    void AddColumn(std::vector<std::string> const& columnData, int width = -1)&;

    /**
     * @brief Adds a new row to the table.
     *
     * @param rowData The data for the new row.
     * @param height The height of the new row.
     */
    void AddRow(RowData&& rowData, int height=-1)&;

    /**
     * @brief Adds a new row to the table (by copying the data).
     *
     * @param rowData The data for the new row.
     * @param height The height of the new row.
     */
    void AddRow(RowData const& rowData, int height = -1)&;

    /**
     * @brief Adds a new row to the table using string data.
     *
     * @param rowData A vector of strings for the new row.
     * @param height The height of the new row.
     */
    void AddRow(std::vector<std::string> const& rowData, int height = -1)&;

    /**
     * @brief Merges cells in a row between the specified column indices, if no previous merges exist.
     * @param row The row index of the cells to merge.
     * @param startCol The starting column index.
     * @param endCol The ending column index.
     */
    void MergeCellsHorizontal(int row, int startCol, int endCol)&;
    
    /**
     * @brief Merges cells in a column between the specified row indices, if no previous merges exist.
     *
     * @param col The column index of the cells to merge.
     * @param startRow The starting row index.
     * @param endRow The ending row index.
     */
    void MergeCellsVertical(int col, int startRow, int endRow)&;

    /**
     * @brief Merges cells in the specified range, if no previous merges exist.
     *
     * @param rowStart The starting row index.
     * @param colStart The starting column index.
     * @param rowEnd The ending row index.
     * @param colEnd The ending column index.
     */
    void MergeCells(int rowStart, int colStart, int rowEnd, int colEnd)&;

    /**
     * @brief Draws the table on an image at the specified position.
     *
     * @param img The image to draw the table on.
     * @param pos The position to start drawing the table at.
     */
    void Draw(cv::Mat& inputImage, cv::Point pos) const;

    /**
     * @brief Clears the table data.
     *
     * Clears all the data in the table and resets the maximum row and column counts.
     */
    void Clear()&;

    /**
     * @brief Get the number of columns in the table.
     *
     * @return The current maximum number of columns in the table.
     */
    auto GetColumns() const  -> int { return mMaxCol; }

    /**
     * @brief Get the number of rows in the table.
     *
     * @return The current maximum number of rows in the table.
     */
    auto GetRows() const -> int { return mMaxRow; }

    /**
     * @brief Get data of a specific column.
     *
     * @param idx The index of the column to retrieve.
     *
     * @return ColumnData The data associated with the specified column.
     *
     * @note Ensure that the `idx` is within the valid range
     * of existing columns.
     */
    auto GetColumn(int idx) const->ColumnData;

    /**
     * @brief Get data of a specific row.
     *
     * @param idx The index of the row to retrieve.
     *
     * @return A constant reference to the data associated with the specified row.
     *
     * @note Ensure that the `idx` is within the valid range
     * of existing rows.
     */
    auto GetRow(int idx) const->RowData  const&;

    /**
     * @brief Get data of a specific cell in the table.
     *
     * @param row The row index of the cell to retrieve.
     * @param col The column index of the cell to retrieve.
     *
     * @return A constant reference to the data in the specified cell.
     *
     * @note Ensure that both `row` and `col` are within valid ranges.
     */
    auto GetCell(int row, int col) const->CellData const&;

    /**
     * @brief Provides access to the entire table data.
     *
     * @return A constant reference to the table data (`TableData`) stored internally.
     */
    auto GetData() const& -> TableData const& { return mData; }

    /**
     * @brief Provides modifiable access to the table data.
     *
     * @return A reference to the table data (`TableData`) allowing modifications.
     */
    auto GetData() & -> TableData& { return mData; }

    /**
     * @brief Moves and returns the table data.
     *
     * This function transfers ownership of the table data, allowing
     * efficient moves without copying.
     *
     * @return The table data (`TableData`) by value, using move semantics.
     */
    auto GetData() && -> TableData { return std::move(mData); }

    /**
     * @brief Sets the color of all cells in a specific column.
     *
     * @param idx The index of the column to set the color for.
     * @param fontColor The font color to set for the cells.
     * @param borderColor The border color to set for the cells.
     */
    void SetColumnColor(int idx, std::shared_ptr<CVPen> const& fontColor, std::shared_ptr<CVPen> const& borderColor)&;
    
    /**
     * @brief Sets the color of all cells in a specific row.
     *
     * @param idx The index of the row to set the color for.
     * @param fontColor The font color to set for the cells.
     * @param borderColor The border color to set for the cells.
     */
    void SetRowColor(int idx, std::shared_ptr<CVPen> const& fontColor, std::shared_ptr<CVPen> const& borderColor)&;

    /**
     * @brief Sets the color of a specific cell.
     *
     * @param row The row index of the cell.
     * @param col The column index of the cell.
     * @param fontColor The font color to set for the cell.
     * @param borderColor The border color to set for the cell.
     */
    void SetCellColor(int row, int col, std::shared_ptr<CVPen> const& fontColor, std::shared_ptr<CVPen> const& borderColor)&;

    /**
     * @brief Sets the width of a specific column.
     *
     * @param idx The index of the column to set the width for.
     * @param width The width to set for the column.
     */
    void SetColumnWidth(int idx, int width)&;

    /**
     * @brief Sets the height of a specific row.
     *
     * @param idx The index of the row to set the height for.
     * @param height The height to set for the row.
     */
    void SetRowHeight(int idx, int height)&;

    /**
     * @brief Creates a default cell with the specified attributes. When create the cell should use this function to linking the cell with the table style.
     *
     * @param str The text content of the cell.
     * @param w A shared pointer to the width of the cell.
     * @param h A shared pointer to the height of the cell.
     * @return A shared pointer to the created cell.
     */
    auto CreateDefaultCell(std::string str = ""
                           , std::shared_ptr<int>const& w = nullptr
                           , std::shared_ptr<int>const& h = nullptr
    ) const& ->CellData;

    /**
     * @brief Sets the text alignment for a specific column.
     *
     * @param idx The index of the column to set the alignment for.
     * @param align The text alignment (TextAlign) to apply to the entire column.
     */
    void SetColumnAlign(int idx, TextAlign align)&;

    /**
     * @brief Sets the text alignment for a specific row.
     *
     * @param idx The index of the row to set the alignment for.
     * @param align The text alignment (TextAlign) to apply to the entire row.
     */
    void SetRowAlign(int idx, TextAlign align)&;

    /**
     * @brief Retrieves the current size of the table.
     * 
     * This function returns the dimensions of the table as a `cv::Size` object.
     * 
     * @return cv::Size The width and height of the table, based on current table settings.
     */
    auto GetSize()const& -> cv::Size;

private:
    auto RefineRowIdx(int rowIdx) const& -> int { return std::max(0, std::min(rowIdx, mMaxRow - 1)); }
    auto RefineColIdx(int colIdx) const& -> int { return std::max(0, std::min(colIdx, mMaxCol - 1)); }

    /**
     * @brief Gets the merge status of the specified cell range.
     *
     * @param startRow The starting row of the cell range.
     * @param startCol The starting column of the cell range.
     * @param endRow The ending row of the cell range.
     * @param endCol The ending column of the cell range.
     * @return True if any of the cells in the range are merged, false otherwise.
     */
    auto GetMergeStatus(int startRow, int startCol, int endRow, int endCol) const& -> bool;

    /**
     * @brief Sets the merge status of the specified cell range.
     *
     * @param startRow The starting row of the cell range.
     * @param startCol The starting column of the cell range.
     * @param endRow The ending row of the cell range.
     * @param endCol The ending column of the cell range.
     * @param value The merge status to set.
     */
    auto SetMergeStatus(int startRow, int startCol, int endRow, int endCol, bool value) & -> void;

public:
    /// Text style for whole table, if the cell did not set the local style it will use the that tabel style
    std::shared_ptr<CVPen> mFontStyle = std::make_shared<CVPen>();
    /// Border style for whole table, if the cell did not set the local style it will use the that tabel style
    std::shared_ptr<CVPen> mBorderStyle = std::make_shared<CVPen>();

private:
    /// The number of columns in the table.
    int mMaxCol = 0;

    /// The number of rows in the table.
    int mMaxRow = 0;

    /// The default width for each column.
    int mDefaultWidth = 100;

    /// The default height for each row.
    int mDefaultHeight = 30;

    /// A vector storing the width of each column as shared pointers.
    std::vector<std::shared_ptr<int>> mColumnWidth;

    /// A vector storing the height of each row as shared pointers.
    std::vector<std::shared_ptr<int>> mRowHeight;

    /// Table data struct as: tale(row(col)).
    TableData mData;
};

//! @} Endgroup Drawing

}//end namespace xvt
