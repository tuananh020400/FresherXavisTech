#include "xvtCV/xvtParseArgs.h"

auto xvt::ParseArgs::Parse(int argc, wchar_t* argv[])->std::map<std::wstring, std::wstring>
{
    int count = 0;
    std::map<std::wstring, std::wstring> mArgList;
    for (int i = 1; i < argc; i++)
    {
        std::wstring arg(argv[i]);
        if (arg[0] == L'-')
        {
            std::wstring nameStr;
            std::wstring valueStr=L"1";
            auto p0 = arg.find_first_not_of(L"-0123456789=");
            if (p0 != std::wstring::npos) {
                auto p1 = arg.find_first_of(L'=', p0);
                nameStr = arg.substr(p0, p1 - p0);
                if (p1 != std::wstring::npos)
                {
                    valueStr = arg.substr(p1 + 1);
                }
                mArgList.emplace(nameStr, valueStr);
            }
        }
        else {
            mArgList.emplace(std::to_wstring(count++), arg);
        }
    }

    return mArgList;
}

