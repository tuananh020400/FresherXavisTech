#include "xvtCV/xvtCSV.h"
#include "xvtCV/ScopeTimer.h"

namespace xvt {

bool CSV::Save(std::wstring const& path, std::wstring const& imgName, bool isNewFile) const
{
    XVT_MESUARE_PERFORMANCE();
    CSVOutput data;
    GetCSVData(data, "", true);
    bool isOK = true;
    if (!data.empty())
    {
        int fileMode = isNewFile ? std::iostream::trunc : std::iostream::app;
        std::wfstream f(path, std::iostream::out | fileMode);
        auto isOK = f.is_open();
        if (isOK)
        {
            if (isNewFile)
            {
                f << GetTitleStr().c_str()<<"\n";

                if (!imgName.empty())
                    f << "Name,";

                f << CSV::GetHeaderStr(data, true).c_str();
            }

            if (!imgName.empty())
                f << imgName << ",";

            f << CSV::GetDataStr(data, true).c_str();
        }
        f.close();
    }

    return isOK;
}

}
