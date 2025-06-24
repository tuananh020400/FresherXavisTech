/**
 * @example{lineno} test/Testxml.cpp
 * This example demonstrates how to use xvt::PixelInfo.h function.
 */

#include "pch.h"
#include "xvtCV/xvtProperty.h"
#include "xvtCV/xvtReadWritexml.h"
#include "xvtCV/xvtFile.h"

#include <iostream>

class Testxml :public xvt::PropertyList
{
public:
    Testxml();

    bool TestBasicType(Testxml const& other)
    {
        bool rtn = true;
        return rtn;
    }

    bool TestPointType(Testxml const& other)
    {
        bool rtn = true;
        return rtn;
    }
};

Testxml::Testxml()
{
    static_cast<PropertyList&>(*this) = {

    };
}
