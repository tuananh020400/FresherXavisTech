#pragma once
#include "xvtCV/xvtConvert.h"
#include <map>
#include <string>

namespace xvt {
//! @addtogroup Utils
//! @{

/**
 * @brief Parsing arguments from command line
*/
class XVT_EXPORTS ParseArgs
{
public:
    ParseArgs() = default;

    /**
     * @brief Parsing arguments from command line
     * @param argc Number of argument
     * @param argv Value of each argument
    */
    ParseArgs(int argc, wchar_t* argv[]);

    /**
     * @brief Get the value from string
     * @tparam T Type of data represented by string
     * @param arg string value
     * @param defaultValue default value when can not parse
     * @return value of T data converted from string
    */
    template<typename T>
    auto GetArg(std::wstring const& arg, T defaultValue)->T;

    /**
     * @brief Parsing arguments from command line
     * @param argc Number of argument
     * @param argv Value of each argument
     * @return Map data of argmument and string value
    */
    static
    auto Parse(int argc, wchar_t* argv[])->std::map<std::wstring, std::wstring>;

private:
    /// Map data of argmument and string value
    std::map<std::wstring, std::wstring> mArgList;

};

inline
xvt::ParseArgs::ParseArgs(int argc, wchar_t* argv[])
{
    mArgList = xvt::ParseArgs::Parse(argc, argv);
}

template<>
inline
auto ParseArgs::GetArg<std::wstring>(std::wstring const& arg, std::wstring defaultValue)->std::wstring
{
    return mArgList[arg];
}

template<typename T>
inline
auto ParseArgs::GetArg(std::wstring const& arg, T defaultValue) -> T
{
    try {
        defaultValue = xvt::To_<T>(mArgList[arg]);
    }
    catch (std::exception const& ex) {

    }

    return defaultValue;
}
//! @} end of group Utils

}

