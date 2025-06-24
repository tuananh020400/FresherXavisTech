#include "xvtCV/xvtTable.h"

namespace xvt {

Table::Table(int width, int height)
{
    if (width >= 0) mDefaultWidth = width;
    if (height >= 0) mDefaultHeight = height;
};
    

Table::Table(std::shared_ptr<CVPen> fontStyle, std::shared_ptr<CVPen> borderStyle)
    :mFontStyle{fontStyle}
    ,mBorderStyle{borderStyle}
{
};
    

Table::Table(int width, int height, std::shared_ptr<CVPen> fontStyle, std::shared_ptr<CVPen> borderStyle)
    :mFontStyle{fontStyle}
    ,mBorderStyle{borderStyle}
{
    if (width >= 0) mDefaultWidth = width;
    if (height >= 0) mDefaultHeight = height;
};

void Table::Clear()&
{
    mData.clear();
    mMaxCol = 0;
    mMaxRow = 0;
}

auto Table::GetColumn(int idx) const -> ColumnData
{
    if (idx < 0 || idx >= mMaxCol) return {};
    auto colData = ColumnData(mMaxRow, nullptr);
    for (int i = 0; i < mMaxRow; i++)
    {
        colData[i] = mData[i][idx];
    }
    return colData;
}

auto Table::GetRow(int idx) const -> RowData const&
{
    if (idx < 0 || idx >= mMaxRow) return {};
    return mData[idx];
}

auto Table::GetCell(int row, int col) const -> CellData const&
{
    if (row < 0 || row >= mMaxRow || col < 0 || col >= mMaxCol) return {};
    return mData[row][col];
}

void Table::SetColumnColor(int idx, std::shared_ptr<CVPen> const& fontColor, std::shared_ptr<CVPen> const& borderColor)&
{
    if (mData.empty() || idx < 0 || idx >= mMaxCol) return;

    for (int i = 0; i < mMaxRow; i++)
    {
        auto& cell = mData[i][idx];
        cell->mFontStyle = fontColor;
        cell->SetBorderStyle(borderColor);
    }
}

void Table::SetRowColor(int idx, std::shared_ptr<CVPen> const& fontColor, std::shared_ptr<CVPen> const& borderColor)&
{
    if (mData.empty() || idx < 0 || idx >= mMaxRow) return;

    auto& cells = mData[idx];
    for (auto& cell : cells)
    {
        cell->mFontStyle = fontColor;
        cell->SetBorderStyle(borderColor);
    }
}

void Table::SetCellColor(int row, int col, std::shared_ptr<CVPen> const& fontColor, std::shared_ptr<CVPen> const& borderColor)&
{
    if (mData.empty() || row < 0 || row >= mMaxRow || col < 0 || col >= mMaxCol) return;
    auto& cell = mData[row][col];
    cell->mFontStyle = fontColor;
    cell->SetBorderStyle(borderColor);
}

void Table::SetColumnWidth(int idx, int width)&
{
    if (mData.empty() || idx < 0 || idx >= mColumnWidth.size() || width < 0) return;
    *(mColumnWidth[idx]) = width;
}

void Table::SetRowHeight(int idx, int height)&
{
    if (mData.empty() || idx < 0 || idx >= mRowHeight.size() || height < 0) return;
    *(mRowHeight[idx]) = height;
}

void Table::SetColumnAlign(int idx, TextAlign align)&
{
    if (mData.empty() || idx < 0 || idx >= mMaxCol) return;

    for (int i = 0; i < mMaxRow; i++)
    {
        auto& cell = mData[i][idx];
        cell->mTextAlign = align;
    }
}

void Table::SetRowAlign(int idx, TextAlign align)&
{
    if (mData.empty() || idx < 0 || idx >= mMaxRow) return;

    auto& cells = mData[idx];
    SetCellsAlign(cells, align);
}

auto Table::CreateDefaultCell(  std::string str
                              , std::shared_ptr<int>const& w
                              , std::shared_ptr<int>const& h
) const& -> CellData
{
    return std::make_shared<Cell>(str, w, h, mFontStyle, mBorderStyle);
}

auto Table::GetSize()const& -> cv::Size
{
    cv::Size s(0,0);

    for(auto w : mColumnWidth) s.width += *w;
    for(auto h : mRowHeight) s.height += *h;

    return s;
}

auto Table::GetMergeStatus(int startRow, int startCol, int endRow, int endCol) const& -> bool
{
    for (int i = startRow; i <= endRow; ++i)
    {
        for (int j = startCol; j <= endCol; ++j)
        {
            auto& cell = mData[i][j];
            if (cell->mIsMerged)
                return true;
        }
    }

    return false;
}

auto Table::SetMergeStatus(int startRow, int startCol, int endRow, int endCol, bool value) & -> void
{
    for (int i = startRow; i <= endRow; ++i)
    {
        for (int j = startCol; j <= endCol; ++j)
        {
            auto& cell = mData[i][j];
            cell->mIsMerged = value;
        }
    }
}

void Table::AddColumn(ColumnData&& colData, int width)&
{
    if (colData.empty()) return;

    if (width < 0) width = mDefaultWidth;

    if (mRowHeight.empty())
    {
        for (int i = 0; i < colData.size(); i++)
            mRowHeight.emplace_back(std::make_shared<int>(mDefaultHeight));
    }

    if (mColumnWidth.empty()) mColumnWidth.emplace_back(std::make_shared<int>(width));

    int n = colData.size() - mMaxRow;
    // When addition data has bigger number of row than number of row in table
    // Add empty row data to the table
    if (n > 0)
    {
        for (int i = 0; i < n; i++)
        {
            auto rowData = Table::RowData();
            for (int c = 0; c < mMaxCol; c++)
            {
                auto w = mColumnWidth[c];
                auto h = std::make_shared<int>(mDefaultHeight);
                mRowHeight.push_back(h);
                rowData.push_back(CreateDefaultCell("", w, h));
            }
            mData.push_back(std::move(rowData));
        }
        mMaxRow = mData.size();
    }

    // When addition data has smaller number of col than number of col in table
    // Add empty data to the colData
    if (n < 0)
    {
       
        n = std::abs(n);
        for (int i = 0; i < n; i++)
        {
            colData.push_back(CreateDefaultCell());
        }
    }

    if (mColumnWidth.size() == mMaxCol)
    {
        mColumnWidth.emplace_back(std::make_shared<int>(width));
    }

    auto w = mColumnWidth.back();
    n = colData.size();
    for (int i = 0; i < n; i++)
    {
        auto&& cell = std::move(colData[i]);
        auto h = mRowHeight[i];
        cell->SetSize(w, h);
        auto& rowData = mData[i];
        if(!rowData.empty())
        {
            auto& preCell = rowData.back();
            cell->mBorderLeft = preCell->mBorderRight;
        }

        if (i > 0)
        {
            auto& preColCell = mData[i-1].back();
            cell->mBorderTop = preColCell->mBorderBottom;
        }

        rowData.push_back(std::move(cell));
    }
    mMaxCol = mData[0].size();
}

void Table::AddColumn(ColumnData const& columnData, int width)&
{
    AddColumn(ColumnData(columnData), width);
}

void Table::AddColumn(std::vector<std::string> const& columnData, int width)&
{
    if (columnData.empty()) return;

    ColumnData colAdd;
    colAdd.reserve(columnData.size());
    for (const auto& str : columnData)
    {
        colAdd.emplace_back(CreateDefaultCell(str));
    }
    AddColumn(std::move(colAdd), width);
}

void Table::AddRow(RowData&& rowData, int height)&
{
    if (rowData.empty()) return;
    if (height < 0) height = mDefaultHeight;
    if (mRowHeight.empty()) mRowHeight.emplace_back(std::make_shared<int>(height));

    if (mColumnWidth.empty())
    {
        for (int i = 0; i < rowData.size(); i++)
            mColumnWidth.emplace_back(std::make_shared<int>(mDefaultWidth));
    }

    int n = rowData.size() - mMaxCol;

    // When addition data has bigger number of col than number of col in table
    // Add empty col to the table
    if (n > 0)
    {
        for (int r = 0; r < mMaxRow; r++)
        {
            auto h = mRowHeight[r];
            auto& rows = mData[r];
            for (int i = 0; i < n; i++)
            {
                auto w = std::make_shared<int>(mDefaultWidth);
                mColumnWidth.push_back(w);
                rows.emplace_back(CreateDefaultCell("", w, h));
            }
        }

        mMaxCol = rowData.size();
    }

    // When addition data has smaller number of cols than number of cols in table
    // Add empty data to the colData
    if (n < 0)
    {
        n = std::abs(n);
        for (int i = 0; i < n; i++)
        {
            rowData.emplace_back(CreateDefaultCell());
        }
    }

    if (mRowHeight.size() == mMaxRow)
    {
        mRowHeight.emplace_back(std::make_shared<int>(height));
    }

    auto h = mRowHeight.back();
    for (int i = 0; i < rowData.size(); i++)
    {
        auto w = mColumnWidth[i];
        auto& cell = rowData[i];
        cell->SetSize(w, h);

        if (i > 0)
        {
            cell->mBorderLeft = rowData[i - 1]->mBorderRight;
        }

        if (mData.size() > 0)
        {
            auto&& lastRow = mData.back();
            cell->mBorderTop = lastRow[i]->mBorderBottom;
        }
    }
    mData.push_back(std::move(rowData));
    mMaxRow = mData.size();
}

void Table::AddRow(RowData const& rowData, int height)&
{
    AddRow(RowData(rowData), height);
}

void Table::AddRow(std::vector<std::string> const& rowData, int height)&
{
    if (rowData.empty()) return;

    RowData rowAdd;
    for (const auto& str : rowData)
    {
        rowAdd.emplace_back(CreateDefaultCell(str));
    }

    AddRow(std::move(rowAdd), height);
}

void Table::MergeCellsHorizontal(int row, int startCol, int endCol)&
{
    MergeCells(row, startCol, row, endCol);
}

void Table::MergeCellsVertical(int col, int startRow, int endRow)&
{
    MergeCells(startRow, col, endRow, col);
}

void Table::MergeCells(int rowStart, int colStart, int rowEnd, int colEnd)&
{
    if (mData.empty()) return;

    colStart = RefineColIdx(colStart);
    colEnd   = RefineColIdx(colEnd);
    rowStart = RefineRowIdx(rowStart);
    rowEnd   = RefineRowIdx(rowEnd);

    if (colStart > colEnd) std::swap(colStart, colEnd);
    if (rowStart > rowEnd) std::swap(rowStart, rowEnd);

    if (colStart == colEnd && rowStart == rowEnd) return;

    auto isMerged = GetMergeStatus(rowStart, colStart, rowEnd, colEnd);
    // if one cell is merged so skip process
    if (isMerged) return;

    auto mergedCell = std::make_shared<CellMerged>(*(mData[rowStart][colStart]));
    mergedCell->mIsMerged = true;
    mergedCell->mText.clear();
    mergedCell->mMergedSize = cv::Size(colEnd - colStart + 1, rowEnd - rowStart + 1);

    for (int i = rowStart; i <= rowEnd; ++i)
    {
        for (int j = colStart; j <= colEnd; ++j)
        {
            auto& cell = mData[i][j];
            if (mergedCell->mText.empty()) mergedCell->mText = cell->mText;
            else if (!cell->mText.empty())
                mergedCell->mText += ", " + cell->mText;
            else
                cell->mText = "";
            cell->mVisible = false;
            cell->mIsMerged = true;
            cell->mRefCells.push_back(mergedCell);
            mergedCell->mRefCells.push_back(cell);
        }
    }
    mergedCell->mRefCells[0] = mergedCell;
    mData[rowStart][colStart] = mergedCell;
}

void Table::Draw(cv::Mat& img, cv::Point pos) const
{
    auto rowPos = pos;
    for (size_t rowIndex = 0; rowIndex < mMaxRow; ++rowIndex)
    {
        rowPos.x = pos.x;
        auto cellPos = rowPos;
        const auto& row = mData[rowIndex];
        for (size_t colIndex = 0; colIndex < mMaxCol; ++colIndex)
        {
            const auto& cell = row[colIndex];
            cell->Draw(img, cellPos);
            const auto& w = mColumnWidth[colIndex];
            cellPos.x += *w;
        }

        const auto& h = mRowHeight[rowIndex];
        rowPos.y += *h;
    }
}

auto Cell::GetHeight() const& -> int
{
    if (!mBorderRight) return 0;
    return std::abs(mBorderRight->GetLength());
}

auto Cell::GetWidth() const& -> int
{
    if (!mBorderTop) return 0;
    return std::abs(mBorderTop->GetLength());
}

void Cell::SetSize(std::shared_ptr<int> const& w, std::shared_ptr<int> const& h)&
{
    mBorderTop->SetLength(w);
    mBorderLeft->SetLength(h);
    mBorderBottom->SetLength(w);
    mBorderRight->SetLength(h);
}

void Cell::SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle)&
{
    mBorderTop->SetStyle(borderStyle);
    mBorderLeft->SetStyle(borderStyle);
    mBorderBottom->SetStyle(borderStyle);
    mBorderRight->SetStyle(borderStyle);
}

void Cell::SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle, int idx)&
{
    std::shared_ptr<LineSegment> border = nullptr;
    switch (idx)
    {
        case 0:
            border = mBorderTop;
            break;
        case 1:
            border = mBorderLeft;
            break;
        case 2:
            border = mBorderBottom;
            break;
        case 3:
            border = mBorderRight;
            break;
        default:
            break;
    }

    if (border) border->SetStyle(borderStyle);
}

auto Cell::Draw(cv::Mat& img, cv::Point offset) const& ->cv::Point
{
    DrawText(img, offset);

    offset = DrawBorder(img, offset);

    return offset;
}

auto Cell::DrawText(cv::Mat& img, cv::Point offset) const& -> cv::Point
{
    if (img.empty()) return offset;

    cv::Size s = cv::Size(GetWidth(), GetHeight());
    cv::Rect cellRect(offset, s);

    auto rtnPos = cellRect.br();

    // Draw text inside the cell
    if (mVisible && mFontStyle && !s.empty())
    {
        cv::Size textSz = mFontStyle->GetTextSize(mText);
        cv::Point textPosition;

        int center_y = cellRect.y + (cellRect.height + textSz.height) / 2;

        if (IsAlign<TextAlign::RIGHT>(mTextAlign))
        {
            textPosition = cv::Point(cellRect.x + cellRect.width - textSz.width - 3, center_y);
        }
        else if (IsAlign<TextAlign::MIDDLE>(mTextAlign))
        {
            textPosition = cv::Point(cellRect.x + (cellRect.width - textSz.width) / 2, center_y);
        }
        else
        {
            textPosition = cv::Point(cellRect.x + 3, center_y);
        }

        cv::putText(img, mText, textPosition, mFontStyle->mFontFace, mFontStyle->mFontScale, mFontStyle->mColor, mFontStyle->mThickness);
    }

    return rtnPos;
}

auto Cell::DrawBorder(cv::Mat& img, cv::Point offset) const& -> cv::Point
{
    // Draw the border line
    if (mVisible)
    {
        // Draw the borders
        auto p0 = mBorderTop->Draw(img, offset);
        auto p1 = mBorderLeft->Draw(img, offset);

        mBorderBottom->Draw(img, p1);
        offset = mBorderRight->Draw(img, p0);
    }
    return offset;
}

auto LineSegment::Draw(cv::Mat& img, cv::Point pos) const& -> cv::Point
{
    if (img.empty() || img.type() != CV_8UC3 || !mVisible || mLength == 0) return pos;

    auto endpos = GetPos(pos);

    if(mStyle)
        cv::line(img, pos, endpos, mStyle->mColor, mStyle->mThickness, mStyle->mLineType);

    return endpos;
}

auto CellMerged::GetHeight() const& -> int
{
    int h = Cell::GetHeight();
    for (int i = 0; i < mRefCells.size(); i = i + mMergedSize.width)
    {
        auto const& cell = mRefCells[i].lock();
        assert(cell);
        if (cell.get() != this)
            h += cell->GetHeight();
    }
    return h;
}

auto CellMerged::GetWidth() const& -> int
{
    if (mMergedSize.area() > 0)
    {
        int w = Cell::GetWidth();
        for (int i = 0; i < mMergedSize.width; i++)
        {
            auto const& cell = mRefCells[i].lock();
            assert(cell);
            if(cell.get() != this)
                w += cell->GetWidth();
        }
        return w;
    }
}

void CellMerged::SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle)&
{
    Cell::SetBorderStyle(borderStyle);
    for (auto cell : mRefCells)
    {
        auto c = cell.lock();
        if (c && c.get() != this)
        {
            c->SetBorderStyle(borderStyle);
        }
    }
}

void CellMerged::SetBorderStyle(std::shared_ptr<CVPen> const& borderStyle, int idx)&
{
    Cell::SetBorderStyle(borderStyle, idx);
    for (auto cell : mRefCells)
    {
        auto c = cell.lock();
        if (c && c.get() != this)
        {
            c->SetBorderStyle(borderStyle, idx);
        }
    }
}

auto CellMerged::DrawBorder(cv::Mat& img, cv::Point offset) const& -> cv::Point
{
    // Draw the border line
    if (mVisible)
    {
        // Draw the top border
        cv::Point tr(offset);
        for (int i = 0; i < mMergedSize.width; i++)
        {
            auto const& cell = mRefCells[i].lock();
            tr = cell->mBorderTop->Draw(img, tr);
        }

        // Draw the left border
        cv::Point bl(offset);
        for (int i = 0; i < mRefCells.size(); i = i + mMergedSize.width)
        {
            auto const& cell = mRefCells[i].lock();
            bl = cell->mBorderLeft->Draw(img, bl);
        }

        // Draw the bottom border
        cv::Point br(bl);
        for (int i = mRefCells.size() - mMergedSize.width; i < mRefCells.size(); i++)
        {
            auto const& cell = mRefCells[i].lock();
            br = cell->mBorderBottom->Draw(img, br);
        }

        // Draw the right border
        br = tr;
        for (int i = mMergedSize.width - 1; i < mRefCells.size(); i = i + mMergedSize.width)
        {
            auto const& cell = mRefCells[i].lock();
            br = cell->mBorderRight->Draw(img, br);
        }
        offset = br;
    }

    return offset;
}

}//end namespace xvt