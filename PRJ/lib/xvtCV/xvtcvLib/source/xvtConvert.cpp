#include "xvtCV/xvtConvert.h"
#include <locale>
#include <codecvt>
#include <string>
#include <algorithm>

namespace xvt
{
std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

auto ToWString(std::string value)->std::wstring
{
    return converter.from_bytes(value);
}

auto ToString(std::wstring value)->std::string
{
    return converter.to_bytes(value);
}

auto ToLowerCase(std::string s)->std::string
{
    for (char& c : s)
        c = tolower(c);
    return s;
}

auto ToLowerCase(std::wstring s)->std::wstring
{
    for (wchar_t& c : s)
        c = tolower(c);
    return s;
}

auto TrimSpace(const std::string& s)->std::string
{
    auto wsfront = std::find_if_not(s.begin(), s.end(), [](int c) {return std::isspace(c); });
    auto wsend = std::find_if_not(s.rbegin(), std::string::const_reverse_iterator(wsfront), [](int c) {return std::isspace(c); });
    return std::string(wsfront, wsend.base());
}

auto TrimSpace(const std::wstring& s) -> std::wstring {
    std::wstring tmp;

    if (s.empty()) return tmp;

    auto pos = s.find_first_not_of(L" \t");
    if (pos == std::wstring::npos) {
        return tmp; 
    }

    auto pos2 = s.find_last_not_of(L" \t");
    return s.substr(pos, pos2 - pos + 1);
}

auto TrimLeadingSpace(std::string const& s) -> std::string
{
    auto  wsfront = std::find_if_not(s.begin(), s.end(), [](int c) {return std::isspace(c); });
    return std::string(wsfront, s.end());
}

auto TrimLeadingSpace(std::wstring const& s) -> std::wstring
{
    std::wstring tmp;
    if (s.empty()) return tmp;

    auto pos = s.find_first_not_of(L" \t");
    if (pos != std::wstring::npos)
    {
        tmp = s.substr(pos);
    }
    return tmp;
}

auto TrimTrailingSpace(std::string const& s) -> std::string
{
    auto  wsend = std::find_if_not(s.rbegin(), s.rend(), [](int c) {return std::isspace(c); });
    return std::string(s.begin(), wsend.base());
}

auto TrimTrailingSpace(std::wstring const& s) -> std::wstring
{
    std::wstring tmp;
    if (s.empty()) return tmp;

    auto pos = s.find_last_not_of(L" \t");
    if (pos != std::wstring::npos) {
        tmp = s.substr(0, pos + 1);
    }
    return tmp;
}

auto Split(std::string const& str, char separator) -> std::vector<std::string>
{
    std::vector<std::string> list;

    if (str.empty()) {
        return list;
    }

    size_t p0 = 0;
    size_t p1 = str.find(separator);

    while (p1 != std::string::npos) {
        list.emplace_back(str.substr(p0, p1 - p0));
        p0 = p1 + 1;
        p1 = str.find(separator, p0);
    }

    list.emplace_back(str.substr(p0));
    return list;
}

auto Split(std::wstring const& str, char separator) -> std::vector<std::wstring>
{
    std::vector<std::wstring> list;

    if (str.empty()) {
        return list;
    }

    size_t p0 = 0;
    size_t p1 = str.find(separator);

    while (p1 != std::wstring::npos) {
        list.emplace_back(str.substr(p0, p1 - p0));
        p0 = p1 + 1;
        p1 = str.find(separator, p0);
    }

    list.emplace_back(str.substr(p0));
    return list;
}

}
