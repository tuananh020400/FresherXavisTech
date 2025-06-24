/**
 * @example{lineno} test/TableTest.cpp
 * This example demonstrates how to use xvt::Table class.
 */

#include "pch.h"
#include "xvtCV/ColorDefine.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTable.h"
#include "xvtCV/xvtConvert.h"
#include "xvtCV/IInspection.h"
#include <opencv2/highgui.hpp>

TEST(DrawingTableTest, DrawRow)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> row1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddRow(row1);
    drawing.Draw(image, cv::Point(20, 20));

    EXPECT_EQ(drawing.GetRows(), 1);
    EXPECT_EQ(drawing.GetColumns(), row1.size());
    EXPECT_TRUE(!drawing.GetData().empty());
}

TEST(DrawingTableTest, DrawRowA)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> row1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddRow(row1);
    drawing.AddRow({"0"});
    drawing.AddRow({"1"});

    drawing.MergeCells(2, 0, 2, 5);
    int cellWidth = 100;
    int cellHeight = 30;
    drawing.SetRowHeight(0, cellHeight);
    drawing.SetColumnWidth(0, cellWidth);
    drawing.SetRowColor(0, std::make_shared<xvt::CVPen>(COLOR_CV_DARKYELLOW, 1), std::make_shared<xvt::CVPen>(COLOR_CV_CYAN, 1));
    drawing.SetRowColor(1, std::make_shared<xvt::CVPen>(COLOR_CV_DARKYELLOW, 1), std::make_shared<xvt::CVPen>(COLOR_CV_BLUE, 1));

    int startPoint_x = 20;
    int startPoint_y = 20;

    drawing.Draw(image, cv::Point(startPoint_x, startPoint_y));

    for (int i = 0; i < row1.size(); i++) {
        cv::Vec3b pixelColor = image.at<cv::Vec3b>(startPoint_y + 10, startPoint_x + cellWidth * i);
        EXPECT_EQ(pixelColor[0], COLOR_CV_CYAN[0]);
        EXPECT_EQ(pixelColor[1], COLOR_CV_CYAN[1]);
        EXPECT_EQ(pixelColor[2], COLOR_CV_CYAN[2]);

        pixelColor = image.at<cv::Vec3b>(startPoint_y + 50, startPoint_x + cellWidth * i);
        EXPECT_EQ(pixelColor[0], COLOR_CV_BLUE[0]);
        EXPECT_EQ(pixelColor[1], COLOR_CV_BLUE[1]);
        EXPECT_EQ(pixelColor[2], COLOR_CV_BLUE[2]);
    }

    cv::Vec3b pixelColor = image.at<cv::Vec3b>(startPoint_y + 89, startPoint_x + cellWidth * 2);
    EXPECT_EQ(pixelColor[0], 255);
    EXPECT_EQ(pixelColor[1], 255);
    EXPECT_EQ(pixelColor[2], 250);
}

TEST(DrawingTableTest, DrawCol)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> col = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddColumn(col);
    drawing.Draw(image, cv::Point(20, 20));

    EXPECT_EQ(drawing.GetColumns(), 1);
    EXPECT_EQ(drawing.GetRows(), col.size());
    EXPECT_TRUE(!drawing.GetData().empty());
}

TEST(DrawingTableTest, DrawColA)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> col = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddColumn(col);
    drawing.AddColumn({"a"});
    drawing.AddColumn({"i"});

    int cellWidth = 60;
    int cellHeight = 30;
    drawing.SetRowHeight(0, cellHeight);
    drawing.SetColumnWidth(0, cellWidth);
    drawing.SetColumnColor(0, std::make_shared<xvt::CVPen>(COLOR_CV_DARKYELLOW, 1), std::make_shared<xvt::CVPen>(COLOR_CV_CYAN, 1));
    drawing.MergeCells(1, 2, 4, 2);
    int startPoint_x = 20;
    int startPoint_y = 20;
    drawing.Draw(image, cv::Point(startPoint_x, startPoint_y));

    for (int i = 0; i < col.size(); i++) {
        cv::Vec3b pixelColor = image.at<cv::Vec3b>(startPoint_y + cellHeight * i, startPoint_x + 10);
        EXPECT_EQ(pixelColor[0], COLOR_CV_CYAN[0]);
        EXPECT_EQ(pixelColor[1], COLOR_CV_CYAN[1]);
        EXPECT_EQ(pixelColor[2], COLOR_CV_CYAN[2]);

        pixelColor = image.at<cv::Vec3b>(startPoint_y + cellHeight * i, startPoint_x + 150);
        EXPECT_EQ(pixelColor[0], 0);
        EXPECT_EQ(pixelColor[1], 0);
        EXPECT_EQ(pixelColor[2], 0);
    }
    cv::Vec3b pixelColor = image.at<cv::Vec3b>(startPoint_y + cellHeight * 3, startPoint_x + 300);
    EXPECT_EQ(pixelColor[0], 255);
    EXPECT_EQ(pixelColor[1], 255);
    EXPECT_EQ(pixelColor[2], 250);
}

TEST(DrawingTableTest, DrawTableColor1)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> row1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    std::vector<std::string> row2 = { "a", "b", "c", "d", "e", "f" };
    std::vector<std::string> row3 = { "i", "ii", "iii", "iv", "v" , "vi" };
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddRow(row1);
    drawing.AddRow(row2);
    drawing.AddRow(row3);

    drawing.mFontStyle->mColor = COLOR_CV_BLUE;
    drawing.mBorderStyle->mColor = COLOR_CV_RED_BERRY;
    drawing.SetCellColor(1, 2, std::make_shared<xvt::CVPen>(COLOR_CV_RED_BERRY, 3), std::make_shared<xvt::CVPen>(COLOR_CV_CYAN, 3));
    drawing.SetColumnWidth(2, 150);
    drawing.Draw(image, cv::Point(20, 20));
    EXPECT_EQ(drawing.GetRows(), 3);
    EXPECT_EQ(drawing.GetColumns(), 8);
    EXPECT_TRUE(!drawing.GetData().empty());
}

TEST(DrawingTableTest, DrawTableColor2)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> row1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    std::vector<std::string> row2 = { "a", "b", "c", "d", "e", "f" };
    std::vector<std::string> row3 = { "i", "ii", "iii", "iv", "v" , "vi" };
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddColumn(row1);
    drawing.AddColumn(row2);
    drawing.AddColumn(row3);

    drawing.mFontStyle->mColor = COLOR_CV_BLUE;
    drawing.mBorderStyle->mColor = COLOR_CV_RED_BERRY;
    drawing.SetCellColor(2, 1, std::make_shared<xvt::CVPen>(COLOR_CV_RED_BERRY, 3), std::make_shared<xvt::CVPen>(COLOR_CV_CYAN, 3));
    drawing.Draw(image, cv::Point(20, 20));

    EXPECT_EQ(drawing.GetRows(), 8);
    EXPECT_EQ(drawing.GetColumns(), 3);
    EXPECT_TRUE(!drawing.GetData().empty());
}

TEST(DrawingTableTest, MergeCell)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));

    xvt::Table drawing;
    xvt::CVPen fontPen;
    std::vector<std::string> data(64, "");
    for (int i = 0; i < data.size(); i++)
    {
        data[i] = std::to_string(i);
        if (i >= 8 && i % 8 == 0)
        {
            drawing.AddRow(std::vector<std::string>{data.begin() + i - 8, data.begin() + i});
        }
    }

    drawing.MergeCells(0, 0, 1, 2);
    drawing.MergeCells(0, 5, 1, 7);
    drawing.MergeCells(5, 6, 6, 7);
    drawing.MergeCells(5, 0, 6, 1);
    drawing.MergeCells(3, 3, 4, 4);
    drawing.MergeCells(0, 3, 1, 3);
    drawing.MergeCells(3, 0, 3, 1);
    drawing.MergeCells(3, 5, 3, 7);
    drawing.MergeCells(5, 3, 6, 3);
    drawing.SetColumnWidth(0, 40);
    drawing.SetColumnWidth(1, 200);
    std::vector<std::string> vec = { "a","b" };
    drawing.AddRow(vec, 50);
    drawing.AddRow(vec, 50);
    drawing.AddRow(vec, 50);
    drawing.AddRow(vec, 50);
    drawing.AddColumn(vec, 50);
    drawing.Draw(image, cv::Point(50, 20));
    EXPECT_EQ(drawing.GetRows(), 11);
    EXPECT_EQ(drawing.GetColumns(), 9);
    EXPECT_TRUE(drawing.GetData().size() == 11);

}

TEST(DrawingTableTest, DrawTable0) {
    xvt::Table drawing;
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    cv::Mat outputImg1 = image.clone();
    cv::Mat outputImg2 = image.clone();
    cv::Mat outputImg3 = image.clone();
    cv::Mat outputImg4 = image.clone();

    xvt::CVPen fontPen;
    xvt::CVPen borderPen;
    std::vector<std::string> v1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    std::vector<std::string> v2 = { "a", "b", "c", "d", "e", "f" };
    std::vector<std::string> v3 = { "i", "ii", "iii", "iv", "v" , "vi" };
    std::vector<std::string> v4 = { "1", "2", "3", "4", "5","6","7","8" };
    std::vector<std::string> v42 = { "1", "2", "3", "4" };
    std::vector<std::string> v41 = { "1", "2" };

    drawing.AddColumn(v2);
    drawing.AddColumn(v2);
    drawing.AddColumn(v1);
    drawing.AddColumn(v42);
    drawing.AddRow(v42);
    drawing.AddRow(v41);
    drawing.AddRow(v4);
    drawing.AddColumn(v42);
    drawing.AddRow(v42);
    drawing.Draw(image, cv::Point(20, 20));

    std::vector<std::string> v5 = { "a", "b", "c", "d" };
    std::vector<std::string> v6 = { "i", "ii", "iii", "iv", "v" , "vi" };
    drawing.MergeCellsHorizontal(3, 3, 4);
    drawing.MergeCellsVertical(7, 5, 7);

    drawing.MergeCells(1, 1, 2, 3);
    drawing.Draw(outputImg2, cv::Point(20, 20));

    drawing.AddRow(v4);
    drawing.AddRow(v5);
    drawing.AddRow(v6);

    drawing.MergeCellsHorizontal(3, 3, 4);
    drawing.Draw(outputImg3, cv::Point(20, 20));

    drawing.AddColumn(v1);
    drawing.AddColumn(v2);
    drawing.AddColumn(v3);
    drawing.AddColumn(v4);
    drawing.AddColumn(v5);
    drawing.AddColumn(v6);

    drawing.MergeCellsHorizontal(3, 3, 4);
    drawing.MergeCellsVertical(1, 1, 2);

    drawing.Draw(outputImg4, cv::Point(20, 20));

    drawing.Clear();
    EXPECT_TRUE(drawing.GetData().empty());
    EXPECT_TRUE(drawing.GetColumns() == 0);
    EXPECT_TRUE(drawing.GetRows() == 0);
}

TEST(DrawingTableTest, Clear)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(0, 0, 0));

    std::vector<std::string> row1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    xvt::Table drawing;

    drawing.Clear();
    EXPECT_TRUE(drawing.GetData().empty());

    xvt::CVPen fontPen;
    drawing.AddRow(row1);
    drawing.Clear();
    EXPECT_TRUE(drawing.GetData().empty());

    drawing.Draw(image, cv::Point(20, 20));
    cv::Vec3b pixelColor = image.at<cv::Vec3b>(20, 20);
    EXPECT_EQ(pixelColor[0], 0);
    EXPECT_EQ(pixelColor[1], 0);
    EXPECT_EQ(pixelColor[2], 0);
}

TEST(DrawingTableTest, AddEmpty)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));
    std::vector<std::string> row1 = {};
    xvt::Table drawing;
    xvt::CVPen fontPen;

    drawing.AddRow(row1);
    EXPECT_EQ(drawing.GetRows(), 0);
    EXPECT_EQ(drawing.GetColumns(), 0);

    drawing.AddColumn(row1);
    EXPECT_EQ(drawing.GetColumns(), 0);
}

struct InspResult
{
    std::string mId = "";
    xvt::EResult mRes;
    int mInt = 0;
    float mFlt = 0.0f;
    double mDbl = 0.0;
};

TEST(DrawingTableTest, DrawTableExm)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));

    xvt::Table drawing;
    xvt::CVPen fontPen;
    std::vector<InspResult> vec = {
        InspResult{"1", xvt::EResult::OK, 1, 0.2f, 3.0 }
       ,InspResult{"2", xvt::EResult::NG, 2, 0.2f, 6.5 }
       ,InspResult{"3", xvt::EResult::OK, 2, 0.2f, 3.0 }
       ,InspResult{"4", xvt::EResult::NG, 2, 0.8f, 3.0 }
       ,InspResult{"5", xvt::EResult::OK, 2, 0.2f, 3.0 }
       ,InspResult{"6", xvt::EResult::NG, 7, 0.2f, 3.0 }
    };

    auto OKStyle = std::make_shared<xvt::CVPen>(COLOR_CV_GREEN);
    auto NGStyle = std::make_shared<xvt::CVPen>(COLOR_CV_RED_BERRY);

    for (auto&& res : vec)
    {
        xvt::Table::RowData row;
        row.push_back(std::make_shared<xvt::Cell>(res.mId, res.mRes == xvt::EResult::OK ? OKStyle : NGStyle));
        row.push_back(std::make_shared<xvt::Cell>(xvt::ToString(res.mRes), res.mRes == xvt::EResult::OK ? OKStyle : NGStyle));
        row.push_back(std::make_shared<xvt::Cell>(xvt::ToString(res.mInt), res.mInt < 5 ? OKStyle : NGStyle));
        row.push_back(std::make_shared<xvt::Cell>(xvt::ToString(res.mFlt, 2), res.mFlt < 0.3 ? OKStyle : NGStyle));
        row.push_back(std::make_shared<xvt::Cell>(xvt::ToString(res.mDbl, 2), res.mDbl < 4.0 ? OKStyle : NGStyle));
        drawing.AddRow(std::move(row));
    }
    drawing.SetColumnWidth(0, 40);
    drawing.SetColumnWidth(1, 40);
    drawing.SetColumnWidth(2, 40);
    drawing.Draw(image, cv::Point(20, 20));
}

TEST(DrawingTableTest, ChangeColor)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));

    xvt::Table drawing;
    //xvt::CVPen fontPen;
    std::vector<std::string> data(64, "");
    for (int i = 0; i < data.size(); i++)
    {
        data[i] = std::to_string(i);
        if (i >= 8 && i % 8 == 0)
        {
            drawing.AddRow(std::vector<std::string>{data.begin() + i - 8, data.begin() + i});
        }
    }

    auto fontPen = std::make_shared<xvt::CVPen>(cv::Scalar(255, 0, 0));
    auto borderPen = std::make_shared<xvt::CVPen>(cv::Scalar(0, 0, 255));

    drawing.SetCellColor(1, 1, std::make_shared<xvt::CVPen>(cv::Scalar(255, 0, 0)), std::make_shared<xvt::CVPen>(cv::Scalar(0, 0, 255)));
    drawing.SetRowColor(3, std::make_shared<xvt::CVPen>(cv::Scalar(255, 255, 0)), std::make_shared<xvt::CVPen>(cv::Scalar(0, 255, 255)));
    drawing.SetColumnColor(5, std::make_shared<xvt::CVPen>(cv::Scalar(100, 0, 255)), std::make_shared<xvt::CVPen>(cv::Scalar(255, 100, 255)));

    drawing.Draw(image, cv::Point(20, 20));

    EXPECT_EQ(drawing.GetRows(), 7);
    EXPECT_EQ(drawing.GetColumns(), 8);
    EXPECT_TRUE(drawing.GetData().size() == 7);
}

TEST(DrawingTableTest, ChangeColorMerged)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(255, 255, 250));

    xvt::Table drawing;
    //xvt::CVPen fontPen;
    std::vector<std::string> data(64, "");
    for (int i = 0; i < data.size(); i++)
    {
        data[i] = std::to_string(i);
        if (i >= 8 && i % 8 == 0)
        {
            drawing.AddRow(std::vector<std::string>{data.begin() + i - 8, data.begin() + i});
        }
    }

    auto fontPen = std::make_shared<xvt::CVPen>(cv::Scalar(255, 0, 0));
    auto borderPen = std::make_shared<xvt::CVPen>(cv::Scalar(0, 0, 255));

    drawing.MergeCellsHorizontal(1, 1, 3);
    drawing.MergeCellsVertical(2, 3, 4);
    drawing.MergeCells(3, 3, 5, 5);

    drawing.SetCellColor(1, 1, std::make_shared<xvt::CVPen>(cv::Scalar(255, 0, 0)), std::make_shared<xvt::CVPen>(cv::Scalar(0, 0, 255)));
    drawing.SetRowColor(3, std::make_shared<xvt::CVPen>(cv::Scalar(255, 255, 0)), std::make_shared<xvt::CVPen>(cv::Scalar(0, 255, 255)));
    drawing.SetColumnColor(5, std::make_shared<xvt::CVPen>(cv::Scalar(100, 0, 255)), std::make_shared<xvt::CVPen>(cv::Scalar(255, 100, 255)));

    drawing.SetCellColor(3, 2, std::make_shared<xvt::CVPen>(cv::Scalar(255, 0, 0)), std::make_shared<xvt::CVPen>(cv::Scalar(0, 250, 0)));

    drawing.Draw(image, cv::Point(20, 20));

    EXPECT_EQ(drawing.GetRows(), 7);
    EXPECT_EQ(drawing.GetColumns(), 8);
    EXPECT_TRUE(drawing.GetData().size() == 7);
}


TEST(DrawingTableTest, Size)
{
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(0, 0, 0));

    std::vector<std::string> row1 = { "1", "2", "3", "4" , "5", "6", "7", "8" };
    xvt::Table drawing(100, 50);
    drawing.AddRow(row1);
    drawing.AddRow(row1, 100);
    drawing.mFontStyle->mColor = COLOR_CV_BLUE;
    drawing.mBorderStyle->mColor = COLOR_CV_RED_BERRY;
    drawing.Draw(image, cv::Point(20, 20));
    auto size = drawing.GetSize();
    EXPECT_TRUE(size.height == 150);
    EXPECT_TRUE(size.width == row1.size()*100);
}