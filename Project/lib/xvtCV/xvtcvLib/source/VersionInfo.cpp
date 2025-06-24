#include "xvtCV/VersionInfo.h"

namespace xvt {

auto VersionInfo::GetVersionInfo(bool dependInfo) const -> std::string
{
    std::string tmp;
    if (mIsPrinted) return tmp;
    auto list = VersionInfoList();

    tmp = GetVersionInfo(list, dependInfo);

    for (auto&& s : list)
    {
        s->mIsPrinted = false;
    }

    return tmp;
}

auto VersionInfo::GetVersionInfo(xvt::VersionInfoList& list, bool dependInfo)const -> std::string
{
    std::string tmp;
    if (mIsPrinted) return tmp;

    list.push_back(this);

    mIsPrinted = true;

    if (!mName.empty()) tmp += mName + "-";
    tmp += mMajor + "." + mMinor + "." + mPatch;

    if (!mNumber.empty() || !mCommit.empty() || !mDirty.empty())
    {
        tmp += ("-" + mNumber) + ("-" + mCommit) + ("-" + mDirty);
    }

    if (!mBranch.empty()) tmp += ("-" + mBranch);

    if(dependInfo)
    {
        auto&& subInfo = GetDependenciesInfo(list, true);
        if (!subInfo.empty()) tmp += subInfo;
    }

    return tmp;
}

auto VersionInfo::GetDependenciesInfo(bool dependInfo) const -> std::string
{
    std::string tmp;
    if (mIsPrinted) return tmp;
    auto list = VersionInfoList();

    tmp = GetDependenciesInfo(list, dependInfo);

    for (auto&& s : list)
    {
        s->mIsPrinted = false;
    }
    return tmp;
}

auto VersionInfo::GetDependenciesInfo(xvt::VersionInfoList& list, bool dependInfo) const -> std::string
{
    std::string tmp;

    for (auto const& s : mDependencies)
    {
        if(s!=nullptr)
        {
            std::string subInfo = s->GetVersionInfo(list, dependInfo);
            if (!subInfo.empty())
            {
                tmp += " " + subInfo;
            }
        }
    }
    return tmp;
}

XVTCV_EXPORTS VersionInfo const opencv_VersionInfo{
     std::string("opencv")
    ,std::string("4")
    ,std::string("8")
    ,std::string("0")
};

XVTCV_EXPORTS VersionInfo const zlib_VersionInfo{
     std::string("zlib")
    ,std::string("1")
    ,std::string("2")
    ,std::string("11")
};

XVTCV_EXPORTS VersionInfo const mxml1_VersionInfo{
     std::string("mxml1")
    ,std::string("3")
    ,std::string("1")
    ,std::string("0")
};

XVTCV_EXPORTS VersionInfo const xvtCV_VersionInfo{
     std::string("xvtCV")
    ,std::string(XVT_CVT_STR(XVT_VERSION_MAJOR) )
    ,std::string(XVT_CVT_STR(XVT_VERSION_MINOR) )
    ,std::string(XVT_CVT_STR(XVT_VERSION_PATCH) )
    ,std::string(XVT_CVT_STR(XVT_VERSION_NUMBER))
    ,std::string(XVT_CVT_STR(XVT_VERSION_COMIT) )
    ,std::string(XVT_CVT_STR(XVT_VERSION_DIRTY) )
    ,std::string(XVT_CVT_STR(XVT_BRANCH)        )
    ,VersionInfoList{&opencv_VersionInfo, &mxml1_VersionInfo, &zlib_VersionInfo}
};

}