#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/VersionInfo.h"
#include <string>

#define XVT_BATTERY_VERSION_MAJOR  __REPLACE_VERSION_MAJOR__
#define XVT_BATTERY_VERSION_MINOR  __REPLACE_VERSION_MINOR__
#define XVT_BATTERY_VERSION_PATCH  __REPLACE_VERSION_PATCH__
#define XVT_BATTERY_VERSION_NUMBER __REPLACE_VERSION_NUMBER__
#define XVT_BATTERY_VERSION_COMIT  __REPLACE_VERSION_COMIT__
#define XVT_BATTERY_VERSION_DIRTY  __REPLACE_VERSION_DIRTY__
#define XVT_BATTERY_BRANCH         __REPLACE_BRANCH__

namespace xvt {
extern XVT_EXPORTS const VersionInfo Battery_VersionInfo;

}
