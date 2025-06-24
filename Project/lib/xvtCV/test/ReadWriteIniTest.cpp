/**
 * @example{lineno} test/ReadWriteIniTest.cpp
 * This example demonstrates how to use xvt::PropertyList, xvt::ini,  classes.
 */

#include "pch.h"
#include "xvtCV/xvtProperty.h"
#include "xvtCV/xvtReadWriteINI.h"

#include <iostream>

class TestINI:public xvt::PropertyList
{
public:
    TestINI();

    bool TestBasicType(TestINI const& other)
    {
        bool rtn = true;
        rtn &= _i == other._i;
        rtn &= _f == other._f;
        rtn &= _d == other._d;
        rtn &= _str  == other._str;
        rtn &= _wstr == other._wstr;
        return rtn;
    }

    bool TestPointType(TestINI const& other)
    {
        bool rtn = true;
        rtn &= _pointi == other._pointi;
        rtn &= _pointf == other._pointf;
        rtn &= _pointd == other._pointd;
        return rtn;
    }

    bool TestRectType(TestINI const& other)
    {
        bool rtn = true;
        rtn &= _recti == other._recti;
        rtn &= _rectf == other._rectf;
        rtn &= _rectd == other._rectd;
        return rtn;
    }

    bool TestSizeType(TestINI const& other)
    {
        bool rtn = true;
        rtn &= _sizei == other._sizei;
        rtn &= _sizef == other._sizef;
        rtn &= _sized == other._sized;
        return rtn;
    }

    bool TestRangeType(TestINI const& other)
    {
        bool rtn = true;
        rtn &= _rangei == other._rangei;
        rtn &= _rangef == other._rangef;
        rtn &= _ranged == other._ranged;
        return rtn;
    }

    bool operator==(TestINI const& other)
    {
        bool rtn = true;
        rtn &= TestBasicType(other);
        rtn &= TestPointType(other);
        rtn &= TestRectType (other);
        rtn &= TestSizeType (other);
        rtn &= TestRangeType(other);
       return rtn;
    }

    int          _i      = 0;
    float        _f      = 0;
    double       _d      = 0;
    std::string  _str    = "";
    std::wstring _wstr   = L"";
    cv::Point2i  _pointi = cv::Point2i();
    cv::Point2f  _pointf = cv::Point2f();
    cv::Point2d  _pointd = cv::Point2d();
    cv::Rect2i   _recti  = cv::Rect2i();
    cv::Rect2f   _rectf  = cv::Rect2f();
    cv::Rect2d   _rectd  = cv::Rect2d();
    cv::Size2i   _sizei  = cv::Size2i();
    cv::Size2f   _sizef  = cv::Size2f();
    cv::Size2d   _sized  = cv::Size2d();
    xvt::Rangei  _rangei = xvt::Rangei();
    xvt::Rangef  _rangef = xvt::Rangef();
    xvt::Ranged  _ranged = xvt::Ranged();
};

TestINI::TestINI()
{
    static_cast<xvt::PropertyList&>(*this) = {
         xvt::CreateShareProperty(L"int", _i, 0  )
        ,xvt::CreateShareProperty(L"flt", _f, 0.f)
        ,xvt::CreateShareProperty(L"dbl", _d, 0.0)
        ,xvt::CreateShareProperty<std::string>(L"str", _str, "")
        ,xvt::CreateShareProperty<std::wstring>(L"wstr", _wstr, L"")

        ,xvt::CreateShareProperty(L"cv::Point2_int", _pointi, cv::Point2i())
        ,xvt::CreateShareProperty(L"cv::Point2_flt", _pointf, cv::Point2f())
        ,xvt::CreateShareProperty(L"cv::Point2_dbl", _pointd, cv::Point2d())

        ,xvt::CreateShareProperty(L"cv::Rect_int"  , _recti , cv::Rect2i())
        ,xvt::CreateShareProperty(L"cv::Rect_flt"  , _rectf , cv::Rect2f())
        ,xvt::CreateShareProperty(L"cv::Rect_dbl"  , _rectd , cv::Rect2d())

        ,xvt::CreateShareProperty(L"cv::Size_int"  , _sizei , cv::Size2i())
        ,xvt::CreateShareProperty(L"cv::Size_flt"  , _sizef , cv::Size2f())
        ,xvt::CreateShareProperty(L"cv::Size_dbl"  , _sized , cv::Size2d())

        ,xvt::CreateShareProperty(L"xvt::Range_int", _rangei, xvt::Rangei())
        ,xvt::CreateShareProperty(L"xvt::Range_flt", _rangef, xvt::Rangef())
        ,xvt::CreateShareProperty(L"xvt::Range_dbl", _ranged, xvt::Ranged())
    };
}

TEST(TestReadWriteIni, BasicFunction) {
    
    auto Section = L"BasicFunction";
    auto file = L".\\testReadWrite.ini";

    std::string   str_write = "string test"  ;
    std::wstring wstr_write = L"wstring test";

    xvt::ini::Write(file, Section, L"int", (int)1);
    xvt::ini::Write(file, Section, L"float", 1.123f);
    xvt::ini::Write(file, Section, L"double", 1.1211191119);
    xvt::ini::Write(file, Section, L"string",  "string test"    );
    xvt::ini::Write(file, Section, L"wstring", L"wstring test"  );
    xvt::ini::Write(file, Section, L"string2",  str_write );
    xvt::ini::Write(file, Section, L"wstring2", wstr_write );

    int i=0;
    float f=0;
    double d=0;
    std::string str;
    std::wstring wstr;
    std::wstring emptyString;

    xvt::ini::Read(file, Section, L"int", i);
    xvt::ini::Read(file, Section, L"float", f);
    xvt::ini::Read(file, Section, L"double", d);
    xvt::ini::Read(file, Section, L"string", str);
    xvt::ini::Read(file, Section, L"wstring", wstr);
    xvt::ini::Read(file, Section, L"emptyString", emptyString);

    EXPECT_EQ(i, 1);
    EXPECT_EQ(f, 1.123f);
    EXPECT_EQ(d, 1.1211191119);
    EXPECT_EQ(str,  "string test"   );
    EXPECT_EQ(wstr, L"wstring test" );
    EXPECT_EQ(emptyString, L"" );

    xvt::ini::Read(file, Section, L"string2", str);
    xvt::ini::Read(file, Section, L"wstring2", wstr);

    EXPECT_EQ(str,   str_write  );
    EXPECT_EQ(wstr,  wstr_write );
}

TEST(TestReadWriteIni, BasicType)
{
    TestINI writeTest;
    TestINI readTest;
    writeTest._i = 1;
    writeTest._f = 2.01f;
    writeTest._d = 3.01123;
    writeTest._str  =  "test123/sdf//sdf_df.asf.tif.jpg";
    writeTest._wstr = L"test123/sdf//sdf_df.asf.tif.jpg";
    writeTest._pointi = cv::Point2i(1, 30);
    writeTest._pointf = cv::Point2f(1.36f, 30.23f);
    writeTest._pointd = cv::Point2d(1.3126, 30.23123);
    writeTest._recti  = cv::Rect2i(1, 30, 5, 5);
    writeTest._rectf  = cv::Rect2f(1.36f, 30.23f, 5.23f, 5.12f);
    writeTest._rectd  = cv::Rect2d(1.3126, 30.23123, 5.789, 5.456);
    writeTest._sizei  = cv::Size2i(1, 30);
    writeTest._sizef  = cv::Size2f(1.36f, 30.23f);
    writeTest._sized  = cv::Size2d(1.3126, 30.23123);
    writeTest._rangei = xvt::Rangei(0, 10, true);
    writeTest._rangef = xvt::Rangef(0.1f, 10.6f, true);
    writeTest._ranged = xvt::Ranged(0.23, 10.456, true);

    auto Section = L"Basic_Types";
    auto file = L".\\testReadWrite.ini";

    writeTest.SaveINI(file, Section);
    readTest.LoadINI(file, Section);

    EXPECT_TRUE(writeTest.TestBasicType(readTest));
    EXPECT_TRUE(writeTest.TestPointType(readTest));
    EXPECT_TRUE(writeTest.TestRectType(readTest));
    EXPECT_TRUE(writeTest.TestSizeType(readTest));
    EXPECT_TRUE(writeTest.TestRangeType(readTest));
    EXPECT_TRUE(writeTest == readTest);
}
