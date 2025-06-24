#include "xvtCV/IInspection.h"

namespace xvt {

DECLARE_ENUM_MAP_DATA(EResult)
{
     { EResult::UC, "UC" }
    ,{ EResult::OK, "OK" }
    ,{ EResult::NG, "NG" }
    ,{ EResult::ER, "ER" }
};
DECLARE_CONVERT_ENUM_FUNCS(EResult);

auto IInspectionResult::GetResultStr() const->std::string
{
    return xvt::ToString(GetResult());
}

auto IInspectionResult::ToString() const->std::string
{
    auto msg = GetMsg();
    auto r = GetResultStr();
    auto str = msg.empty() ? r : r + " " + msg;

    return str;
}

bool IInspectionResult::Save(std::wstring const& path
                             , std::wstring const& imgName
                             , bool isNewFile
) const
{
    return false;
}

}