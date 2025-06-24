#pragma once
#include "xvtCV/xvtReadWriteINI.h"
#include "xvtCV/xvtConvert.h"
#include <tchar.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <assert.h>
#include <Windows.h>

namespace xvt {
namespace ini {

void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey,  bool& outValue, bool defaultValue)
{
    outValue = (bool)GetPrivateProfileInt(lpszSection, lpszKey, defaultValue, lpszIniFile);
}

void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, int& outValue, int defaultValue)
{
    outValue = GetPrivateProfileInt(lpszSection, lpszKey, defaultValue, lpszIniFile);
}

void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, float& outValue, float defaultValue)
{
    TCHAR tszDefault[255];
    TCHAR tszValue[255];
    _stprintf_s(tszDefault, 255, _T("%f"), defaultValue);

    GetPrivateProfileString(lpszSection, lpszKey, tszDefault, tszValue, 255, lpszIniFile);
    outValue = (float)_ttof(tszValue);
}

void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, double& outValue, double defaultValue)
{
    TCHAR tszDefault[255];
    TCHAR tszValue[255];
    _stprintf_s(tszDefault, 255, _T("%f"), defaultValue);
    GetPrivateProfileString(lpszSection, lpszKey, tszDefault, tszValue, 255, lpszIniFile);
    LPTSTR endPtr;
    outValue = _tcstod(tszValue, &endPtr);
}

void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::string& outValue, std::string defaultValue)
{
    TCHAR tszValue[255];

    auto tmpValue = ToWString(defaultValue);
    GetPrivateProfileString(lpszSection, lpszKey, tmpValue.c_str(), tszValue, 255, lpszIniFile);

    outValue = ToString(tszValue);
}

void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::wstring & outValue, std::wstring defaultValue)
{
    TCHAR tszValue[255];

    GetPrivateProfileString(lpszSection, lpszKey, defaultValue.c_str(), tszValue, 255, lpszIniFile);

    outValue = std::wstring(tszValue);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, bool value)
{
    TCHAR tszDefault[255];
    _stprintf_s(tszDefault, 255, _T("%d"), value);

    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tszDefault, lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, int value)
{
    TCHAR tszDefault[255];
    _stprintf_s(tszDefault, 255, _T("%d"), value);

    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tszDefault, lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, float value)
{
    TCHAR tszDefault[255];
    _stprintf_s(tszDefault, 255, _T("%.12f"), (double)value);

    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tszDefault, lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, double value)
{
    TCHAR tszDefault[255];
    _stprintf_s(tszDefault, 255, _T("%.12f"), value);
    //std::wostringstream woss;
    //woss << std::fixed << std::setprecision(12) << value; // Set desired precision
    
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tszDefault, lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, const char* value)
{
    std::wstring tmpValue = ToWString(value);
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tmpValue.c_str(), lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::string const& value)
{
    std::wstring tmpValue = ToWString(value);
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tmpValue.c_str(), lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::string && value)
{
    std::wstring tmpValue = ToWString(value);
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, tmpValue.c_str(), lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, const wchar_t* value)
{
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, value, lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::wstring const& value)
{
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, value.c_str(), lpszIniFile);
    assert(tmp);
}

void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::wstring && value)
{
    auto tmp = WritePrivateProfileString(lpszSection, lpszKey, value.c_str(), lpszIniFile);
    assert(tmp);
}

}
}