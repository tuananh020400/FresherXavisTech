#include "xvtBattery/VersionInfo.h"

namespace xvt {

const XVT_EXPORTS VersionInfo Battery_VersionInfo{
     std::string("xvtBattery")
    ,std::string( XVT_CVT_STR(XVT_BATTERY_VERSION_MAJOR)  )
    ,std::string( XVT_CVT_STR(XVT_BATTERY_VERSION_MINOR)  )
    ,std::string( XVT_CVT_STR(XVT_BATTERY_VERSION_PATCH)  )
    ,std::string( XVT_CVT_STR(XVT_BATTERY_VERSION_NUMBER) )
    ,std::string( XVT_CVT_STR(XVT_BATTERY_VERSION_COMIT)  )
    ,std::string( XVT_CVT_STR(XVT_BATTERY_VERSION_DIRTY)  )
    ,std::string( XVT_CVT_STR(XVT_BATTERY_BRANCH)         )
    ,VersionInfoList{&xvt::xvtCV_VersionInfo}
};
}