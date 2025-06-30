#include "xvtCV/xvtTypes.h"

namespace xvt {

/// Define the string map data and function support to convert Enum to string
DECLARE_ENUM_MAP_DATA(FittingMethod)
{
     { xvt::FittingMethod::LEAST_SQUARE, "LEAST_SQUARE" }
    ,{ xvt::FittingMethod::RANSAC,       "RANSAC"       }
};
DECLARE_CONVERT_ENUM_FUNCS(FittingMethod);

}