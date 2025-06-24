/**
 * @example{lineno} test/xvtCSVTest.cpp
 * This example demonstrates how to use xvt::CSV class.
 */

#include "pch.h"
#include "xvtCV/xvtCSV.h"
#include "xvtCV/IInspection.h"

auto header = xvt::VecString{ "header1","header2","header3","header4" };
auto headerExpectVec = xvt::VecString{ "header1","header2","header3","header4" };
auto headerExpect = std::string("header1,header2,header3,header4");

auto iVec = xvt::VecInt{ 1,2,3,4 };
auto iVecExpectVec = xvt::VecString{"1","2","3","4"};
auto iVecExpect = std::string("1,2,3,4");

auto fVec = xvt::VecFloat{ 1.1f,2.1f,3.1f,4.1f };
auto fVecExpectVec = xvt::VecString{ "1.100000","2.100000","3.100000","4.100000" };
auto fVecExpect = std::string("1.100000,2.100000,3.100000,4.100000");

auto dVec = xvt::VecFloat{ 1.1,2.1,3.1,4.1 };
auto dVecExpectVec = xvt::VecString{ "1.100000","2.100000","3.100000","4.100000" };
auto dVecExpect = std::string("1.100000,2.100000,3.100000,4.100000");

auto sVec = xvt::VecString{ "1.1", "2.1", "", "dsfasd" };
auto sVecExpectVec = xvt::VecString{ "1.1", "2.1", "", "dsfasd" };
auto sVecExpect = std::string("1.1,2.1,,dsfasd");

auto eVec = std::vector<xvt::EResult>{ xvt::EResult::UC,xvt::EResult::OK, xvt::EResult::NG, xvt::EResult::ER };
auto eVecExpectVec = xvt::VecString{ "UC","OK","NG","ER" };
auto eVecExpect = std::string("UC,OK,NG,ER");

template<typename T>
auto CreateData(xvt::VecString h, std::vector<T> d)->std::vector<std::pair<std::string, T>>
{
    assert(h.size(), d.size());
    std::vector<std::pair<std::string, T>> data;
    for (int i = 0, n = h.size(); i < n; i++)
    {
        data.emplace_back(h[i], d[i]);
    }
    return data;
}

auto CreateExpectData(xvt::VecString h, xvt::VecString d)->xvt::VecKeyValueStr
{
    assert(h.size(), d.size());
    xvt::VecKeyValueStr data;
    for (int i = 0, n = h.size(); i < n; i++)
    {
        data.emplace_back(h[i], d[i]);
    }
    return data;
}

auto idata = CreateData(header, iVec);
auto fdata = CreateData(header, fVec);
auto ddata = CreateData(header, dVec);
auto sdata = CreateData(header, sVec);
auto edata = CreateData(header, eVec);

TEST(TestCSV, GetStr)
{
    EXPECT_EQ(xvt::CSV::GetStr(iVec,false), iVecExpect);
    EXPECT_EQ(xvt::CSV::GetStr(fVec,false), fVecExpect);
    EXPECT_EQ(xvt::CSV::GetStr(dVec,false), dVecExpect);
    EXPECT_EQ(xvt::CSV::GetStr(sVec,false), sVecExpect);
    EXPECT_EQ(xvt::CSV::GetStr(eVec,false), eVecExpect);

    EXPECT_EQ(xvt::CSV::GetStr(iVec, true), iVecExpect+"\n");
    EXPECT_EQ(xvt::CSV::GetStr(fVec, true), fVecExpect+"\n");
    EXPECT_EQ(xvt::CSV::GetStr(dVec, true), dVecExpect+"\n");
    EXPECT_EQ(xvt::CSV::GetStr(sVec, true), sVecExpect+"\n");
    EXPECT_EQ(xvt::CSV::GetStr(eVec, true), eVecExpect+"\n");
}

TEST(TestCSV, GetHeaderDataStr)
{
    EXPECT_EQ(xvt::CSV::GetHeaderStr(idata, false), headerExpect);
    EXPECT_EQ(xvt::CSV::GetHeaderStr(fdata, false), headerExpect);
    EXPECT_EQ(xvt::CSV::GetHeaderStr(ddata, false), headerExpect);
    EXPECT_EQ(xvt::CSV::GetHeaderStr(sdata, false), headerExpect);
    EXPECT_EQ(xvt::CSV::GetHeaderStr(edata, false), headerExpect);

    EXPECT_EQ(xvt::CSV::GetHeaderStr(idata, true), headerExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetHeaderStr(fdata, true), headerExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetHeaderStr(ddata, true), headerExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetHeaderStr(sdata, true), headerExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetHeaderStr(edata, true), headerExpect + "\n");

    EXPECT_EQ(xvt::CSV::GetDataStr(idata, false), iVecExpect);
    EXPECT_EQ(xvt::CSV::GetDataStr(fdata, false), fVecExpect);
    EXPECT_EQ(xvt::CSV::GetDataStr(ddata, false), dVecExpect);
    EXPECT_EQ(xvt::CSV::GetDataStr(sdata, false), sVecExpect);
    EXPECT_EQ(xvt::CSV::GetDataStr(edata, false), eVecExpect);

    EXPECT_EQ(xvt::CSV::GetDataStr(idata, true), iVecExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetDataStr(fdata, true), fVecExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetDataStr(ddata, true), dVecExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetDataStr(sdata, true), sVecExpect + "\n");
    EXPECT_EQ(xvt::CSV::GetDataStr(edata, true), eVecExpect + "\n");
}

TEST(TestCSV, GetCSVData)
{
    EXPECT_EQ(xvt::CSV::GetData(idata), CreateExpectData(header, iVecExpectVec));
    EXPECT_EQ(xvt::CSV::GetData(fdata), CreateExpectData(header, fVecExpectVec));
    EXPECT_EQ(xvt::CSV::GetData(ddata), CreateExpectData(header, dVecExpectVec));
    EXPECT_EQ(xvt::CSV::GetData(sdata), CreateExpectData(header, sVecExpectVec));
    EXPECT_EQ(xvt::CSV::GetData(edata), CreateExpectData(header, eVecExpectVec));
}

TEST(TestCSV, Save)
{
    std::wstring name = XVT_PLATFORM_CONFIG_WPATH + std::wstring(L"/test_save_csv_");
    EXPECT_TRUE(xvt::CSV::Save(name + L"_veci.csv", header, iVec));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_vecf.csv", header, fVec));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_vecd.csv", header, dVec));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_vecs.csv", header, sVec));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_vece.csv", header, eVec));

    EXPECT_TRUE(xvt::CSV::Save(name + L"_datai.csv", idata));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_dataf.csv", fdata));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_datad.csv", ddata));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_datas.csv", sdata));
    EXPECT_TRUE(xvt::CSV::Save(name + L"_datae.csv", edata));
}

//! [Example CSV support]
//! 
//! [Define SomeResult Class]
// A result class that will support to save the result to csv file.
class SomeResult
    :public xvt::CSV
{
public:
    // CSV override supports funtion
    auto GetCSVData(xvt::CSVOutput& out, std::string prefix, bool isRecursive) const -> void override;
    auto GetTitleStr()const->std::string override;

    // Class member variables
    int         mInt = 10;
    float       mFloat = 1.32324f;
    double      mDouble = 1.23;
    std::string mString = "A String";
};

// Data format to save to csv file.
auto SomeResult::GetCSVData(xvt::CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
    out.emplace_back("Int Data", xvt::ToString(mInt));
    out.emplace_back("Float Data", xvt::ToString(mFloat, 6));
    out.emplace_back("Double Data", xvt::ToString(mDouble, 6));
    out.emplace_back("String Data", mString);
}
//! [Define SomeResult Class]

//! [Define SomeResult Class GetTitle]
// If you want to put the specific tile in the beginning of csv file.
auto SomeResult::GetTitleStr()const->std::string
{
    return "A Additional Title\n";
}
//! [Define SomeResult Class GetTitle]

TEST(TestCSV, SaveClass)
{
    // csv file name
    std::wstring name = XVT_PLATFORM_CONFIG_WPATH + std::wstring(L"/test_save_csv_class.csv");
    // Instance the class
    SomeResult test;
    // Save the data to csv file
    test.Save(name, L"", true);
}

//! [Example CSV support]
