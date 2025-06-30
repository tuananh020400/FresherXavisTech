// Copyright (c) Xavis, All rights are reserved.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <xvtCV/xvtDefine.h>
#include <string>

namespace xvt {
/// @brief endcode and decode base64 functions
namespace base64
{

//! @addtogroup Converting
//! @{

// encode from binary to base64
XVT_EXPORTS std::wstring  EncodeBase64(std::wstring inputData);

// decode from base64 to binary
XVT_EXPORTS std::wstring  DecodeBase64(std::wstring inputData);

//! @} end of group Converting

}
}  // namespace xvt::base64

// for convenience
#define XVT_EncodeBase64		::xvt::base64::EncodeBase64
#define XVT_DecodeBase64		::xvt::base64::DecodeBase64



