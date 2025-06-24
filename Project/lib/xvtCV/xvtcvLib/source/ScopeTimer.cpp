#include "xvtCV/ScopeTimer.h"
#include "xvtCV/Utils.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

namespace xvt
{

static std::wstring const scPerfResultFolder = XVT_PLATFORM_CONFIG_WPATH;
static std::wstring const scPerfNameAfterfix = L".txt";
static std::wstring const scPerfNamePrefix   = L"PERF_RESULT_";

std::size_t ReplaceAll(std::wstring& inout, std::wstring const& what, std::wstring const& with)
{
    std::size_t count{};
    for (std::string::size_type pos{};
         inout.npos != (pos = inout.find(what.data(), pos, what.length()));
         pos += with.length(), ++count)
    {
        inout.replace(pos, what.length(), with.data(), with.length());
    }
    return count;
}

auto GetCurrentTime(std::string format) -> std::string
{
    std::time_t t = std::time(nullptr);
    std::tm localTime;
    localtime_s(&localTime, &t);
    std::stringstream ss;
    ss << std::put_time(&localTime, format.c_str());
    return ss.str();
}

void WritePerformance(std::wstring const& srcFileName, int line, std::wstring const& funcName, float time)
{
    auto srcName = xvt::GetFileName(srcFileName);
    auto resultFileName = scPerfResultFolder + scPerfNamePrefix + funcName + scPerfNameAfterfix;
    ReplaceAll(resultFileName, L"::", L"_");
    std::wofstream resultStream(resultFileName, std::iostream::app | std::iostream::out);
    if (resultStream.is_open())
    {
        resultStream << srcName + L"," + std::to_wstring(line) + L"," + (std::to_wstring(time) + L"\n");
    }
    resultStream.close();
}

}