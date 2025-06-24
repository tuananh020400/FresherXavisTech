/**
 * @example{lineno} test/xvtParseArgsTest.cpp
 * This example demonstrates how to use xvt::ParseArgs class.
 */

#include "pch.h"
#include "xvtCV/xvtParseArgs.h"

TEST(ParseArgsTest, ParsesTestOK) {

    int argc = 11;
    wchar_t* args[] = {L"test.exe", L"-s", L"-a=0", L"2", L"-b=test", L"-c=1", L"someText", L"-k", L"4", L"-8", L"-longOption=1"};

    std::map<std::wstring, std::wstring> result = xvt::ParseArgs::Parse(argc, args);
    ASSERT_EQ(result.size(), argc-2);
    ASSERT_EQ(result[L"0"], L"2");
    ASSERT_EQ(result[L"1"], L"someText");
    ASSERT_EQ(result[L"2"], L"4");
    ASSERT_EQ(result[L"s"], L"1");
    ASSERT_EQ(result[L"a"], L"0");
    ASSERT_EQ(result[L"b"], L"test");
    ASSERT_EQ(result[L"c"], L"1");
    ASSERT_EQ(result[L"k"], L"1");
    ASSERT_EQ(result[L"h"], L"");
    ASSERT_EQ(result[L"longOption"], L"1");
    ASSERT_EQ(result[std::to_wstring(5)], L"");

    xvt::ParseArgs parser(argc, args);
    ASSERT_EQ(parser.GetArg(L"0", 0), 2);
    ASSERT_EQ(parser.GetArg<std::wstring>(L"1", L""), L"someText");
    ASSERT_EQ(parser.GetArg(L"2", 0), 4);
    
    ASSERT_EQ(parser.GetArg(L"s", false), true);
    ASSERT_EQ(parser.GetArg(L"k", false), true);
    ASSERT_EQ(parser.GetArg(L"a", false), false);
    ASSERT_EQ(parser.GetArg(L"c", false), true);

    ASSERT_EQ(parser.GetArg(L"a", L""), std::wstring(L"0"));
    ASSERT_EQ(parser.GetArg(L"a", 1), 0);
    ASSERT_EQ(parser.GetArg(L"b", L""), std::wstring(L"test"));
    ASSERT_EQ(parser.GetArg(L"c", 0), 1);

    ASSERT_STREQ(parser.GetArg(L"1", L""), L"someText");
    //Not set args
    ASSERT_EQ(parser.GetArg(L"3", 0), 0);
    ASSERT_EQ(parser.GetArg(L"Invalid", false), false);
    ASSERT_EQ(parser.GetArg(L"Invalid", std::wstring(L"")), L"");
}

TEST(ParseArgsTest, ParsesTestNG) {
    int argc = 8; 
    wchar_t* argv[] = { L"test.exe", L"-", L"-a=123kk", L"dd2", L"-b=&^8932", L"-5", L"-=", L"-?" };

    xvt::ParseArgs arg(argc, argv);
    ASSERT_STREQ(arg.GetArg(L"0", L"0"), L"dd2");
    ASSERT_STREQ(arg.GetArg(L"a", L"123"), L"123kk");
    ASSERT_STREQ(arg.GetArg(L"b", L"test"), L"&^8932");

    ASSERT_EQ(arg.GetArg(L"0", 0), 0);
    ASSERT_EQ(arg.GetArg(L"a", 0), 123);
    ASSERT_EQ(arg.GetArg(L"b", 0), 0);

}