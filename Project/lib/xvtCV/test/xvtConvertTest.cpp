/**
 * @example{lineno} test/xvtConvertTest.cpp
 * This example demonstrates how to use the ` xvt::Split` function.
 */

#include "pch.h"
#include "xvtCV/xvtConvert.h"
#include "stringTestSet.h"

TEST(ConversionTest, StringToWString) {
	std::wstring result = xvt::ToWString(STRING_ASCII);
	EXPECT_EQ(result, WSTRING_ASCII);
}

TEST(ConversionTest, WStringToString) {
	std::string result = xvt::ToString(WSTRING_ASCII);
	EXPECT_EQ(result, STRING_ASCII);

}

TEST(TestXvtConvert, TestToLowerCaseWString) {
	std::wstring result = xvt::ToLowerCase(WSTRING_UPPERCASE);
	EXPECT_EQ(result, WSTRING_LOWERCASE);

	result = xvt::ToLowerCase(WSTRING_UNICODE);
	EXPECT_EQ(result, (WSTRING_LOWERCASE + WSTRING_LOWERCASE + WSTRING_NUMBER + WSTRING_SPECIAL_UNICODE + SYMBOL + OTHERS_LANG + WSTRING_ESCAPE_SEQUENCE));

	result = xvt::ToLowerCase(WSTRING_EMPTY);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::ToLowerCase(WSTRING_SPACE);
	EXPECT_EQ(result, WSTRING_SPACE);
}

TEST(TestXvtConvert, TestToLowerCaseString) {
	std::string result = xvt::ToLowerCase(STRING_UPPERCASE);
	EXPECT_EQ(result, STRING_LOWERCASE);

	result = xvt::ToLowerCase(STRING_ASCII);
	EXPECT_EQ(result, (STRING_LOWERCASE + STRING_LOWERCASE + STRING_NUMBER + STRING_SPECIAL + STRING_ESCAPE_SEQUENCE));

	result = xvt::ToLowerCase(STRING_EMPTY);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::ToLowerCase(STRING_SPACE);
	EXPECT_EQ(result, STRING_SPACE);
}

TEST(TestXvtConvert, TrimSpaceString) {
	std::string result = xvt::TrimSpace(STRING_SPACE);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::TrimSpace(STRING_EMPTY);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::TrimSpace(STRING_ASCII);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimSpace(STRING_ASCII_LEADING_SPACE);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimSpace(STRING_ASCII_TRAILING_SPACE);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimSpace(STRING_ASCII_SPACE);
	EXPECT_EQ(result, STRING_ASCII);
}

TEST(TestXvtConvert, TrimSpaceWString) {
	std::wstring result = xvt::TrimSpace(WSTRING_SPACE);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::TrimSpace(WSTRING_EMPTY);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::TrimSpace(WSTRING_UNICODE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimSpace(WSTRING_UNICODE_LEADING_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimSpace(WSTRING_UNICODE_TRAILING_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimSpace(WSTRING_UNICODE_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE);

}

TEST(TestXvtConvert, TrimLeadingSpaceString) {
	std::string result = xvt::TrimLeadingSpace(STRING_SPACE);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::TrimLeadingSpace(STRING_EMPTY);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::TrimLeadingSpace(STRING_ASCII);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimLeadingSpace(STRING_ASCII_LEADING_SPACE);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimLeadingSpace(STRING_ASCII_TRAILING_SPACE);
	EXPECT_EQ(result, STRING_ASCII_TRAILING_SPACE);
}

TEST(TestXvtConvert, TrimLeadingSpaceWString) {
	std::wstring result = xvt::TrimLeadingSpace(WSTRING_SPACE);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::TrimLeadingSpace(WSTRING_EMPTY);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::TrimLeadingSpace(WSTRING_UNICODE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimLeadingSpace(WSTRING_UNICODE_LEADING_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimLeadingSpace(WSTRING_UNICODE_TRAILING_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE_TRAILING_SPACE);

	result = xvt::TrimLeadingSpace(WSTRING_UNICODE_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE_TRAILING_SPACE);
}

TEST(TestXvtConvert, TrimTrailingSpaceString) {
	std::string result = xvt::TrimTrailingSpace(STRING_SPACE);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::TrimTrailingSpace(STRING_EMPTY);
	EXPECT_EQ(result, STRING_EMPTY);

	result = xvt::TrimTrailingSpace(STRING_ASCII);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimTrailingSpace(STRING_ASCII_LEADING_SPACE);
	EXPECT_EQ(result, STRING_ASCII_LEADING_SPACE);

	result = xvt::TrimTrailingSpace(STRING_ASCII_TRAILING_SPACE);
	EXPECT_EQ(result, STRING_ASCII);

	result = xvt::TrimTrailingSpace(STRING_ASCII_SPACE);
	EXPECT_EQ(result, STRING_ASCII_LEADING_SPACE);
}

TEST(TestXvtConvert, TrimTrailingSpaceWString) {
	std::wstring result = xvt::TrimTrailingSpace(WSTRING_SPACE);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::TrimTrailingSpace(WSTRING_EMPTY);
	EXPECT_EQ(result, WSTRING_EMPTY);

	result = xvt::TrimTrailingSpace(WSTRING_UNICODE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimTrailingSpace(WSTRING_UNICODE_TRAILING_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE);

	result = xvt::TrimTrailingSpace(WSTRING_UNICODE_LEADING_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE_LEADING_SPACE);

	result = xvt::TrimTrailingSpace(WSTRING_UNICODE_SPACE);
	EXPECT_EQ(result, WSTRING_UNICODE_LEADING_SPACE);
}

// split string
TEST(TestXvtConvert, SplitString_NoSeparator) {
	std::vector<std::string> result = xvt::Split(STRING_LOWERCASE + STRING_UPPERCASE, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 1);
	EXPECT_EQ(result[0], STRING_LOWERCASE + STRING_UPPERCASE);
}

TEST(TestXvtConvert, SplitString_WithSeparator) {
	std::string input = STRING_SPACE + STRING_LOWERCASE + STRING_SPACE + "," + STRING_SPACE + STRING_UPPERCASE + STRING_SPACE;
	std::vector<std::string> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 2);
	EXPECT_EQ(result[0], STRING_SPACE + STRING_LOWERCASE + STRING_SPACE);
	EXPECT_EQ(result[1], STRING_SPACE + STRING_UPPERCASE + STRING_SPACE);
}

TEST(TestXvtConvert, SplitString_OnlySeparator) {
	std::string input = ",";
	std::vector<std::string> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 2);
	EXPECT_EQ(result[0], STRING_EMPTY);
	EXPECT_EQ(result[1], STRING_EMPTY);
}

TEST(TestXvtConvert, SplitString_EmptyString) {
	std::vector<std::string> result = xvt::Split(STRING_EMPTY, COMMA_SEPARATOR);
	EXPECT_EQ(result.size(), 0);
}

TEST(TestXvtConvert, SplitASCIITest) {
	std::string input = STRING_LOWERCASE + COMMA_SEPARATOR + STRING_NUMBER + COMMA_SEPARATOR + STRING_UPPERCASE;
	std::vector<std::string> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 3);
	EXPECT_EQ(result[0], STRING_LOWERCASE);
	EXPECT_EQ(result[1], STRING_NUMBER);
	EXPECT_EQ(result[2], STRING_UPPERCASE);
}

TEST(TestXvtConvert, SplitASCIITest2) {
	std::string input = STRING_UPPERCASE + "s" + STRING_NUMBER;
	char SEPARATOR = 's';
	std::vector<std::string> result = xvt::Split(input, SEPARATOR);

	EXPECT_EQ(result.size(), 2);
	EXPECT_EQ(result[0], STRING_UPPERCASE);
	EXPECT_EQ(result[1], STRING_NUMBER);
}
// split wstring 
TEST(TestXvtConvert, SplitUnicodeTest) {
	std::wstring input = WSTRING_UPPERCASE + L"," + WSTRING_LOWERCASE;
	std::vector<std::wstring> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 2);
	EXPECT_EQ(result[0], WSTRING_UPPERCASE);
	EXPECT_EQ(result[1], WSTRING_LOWERCASE);
}

TEST(TestXvtConvert, SplitSpecialCharactersTest) {
	std::wstring input = SYMBOL + L"," + OTHERS_LANG;
	std::vector<std::wstring> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 2);
	EXPECT_EQ(result[0], SYMBOL);
	EXPECT_EQ(result[1], OTHERS_LANG);
}

TEST(TestXvtConvert, SplitOtherCharactersTest) {
	std::wstring input = WSTRING_SPACE + WSTRING_LOWERCASE + WSTRING_SPACE + L"," +	WSTRING_NUMBER + L"," +	SYMBOL + L"," + OTHERS_LANG;
	std::vector<std::wstring> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 4);
	EXPECT_EQ(result[0], WSTRING_SPACE + WSTRING_LOWERCASE + WSTRING_SPACE);
	EXPECT_EQ(result[1], WSTRING_NUMBER);
	EXPECT_EQ(result[2], SYMBOL);
	EXPECT_EQ(result[3], OTHERS_LANG);
}

TEST(TestXvtConvert, SplitString_EmptyWString) {
	std::vector<std::wstring> result = xvt::Split(WSTRING_EMPTY, COMMA_SEPARATOR);
	EXPECT_EQ(result.size(), 0);
}

TEST(TestXvtConvert, SplitWString_OnlySeparator) {
	std::wstring input = L",";
	std::vector<std::wstring> result = xvt::Split(input, COMMA_SEPARATOR);

	EXPECT_EQ(result.size(), 2);
	EXPECT_EQ(result[0], WSTRING_EMPTY);
	EXPECT_EQ(result[1], WSTRING_EMPTY);
}
