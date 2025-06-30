#include "pch.h"
#include "xvtCV/PixelRef.h"

using namespace xvt;

template<typename T>
void MilimetTestFunction()
{
    double pSize = 2.0f;
    T p{ pSize };
    EXPECT_FLOAT_EQ(2, p.ToMilimet(1));
    EXPECT_FLOAT_EQ(-2, p.ToMilimet(-1));

    pSize = 2.5;
    double vl = 1;
    EXPECT_FLOAT_EQ(pSize * vl, p.ToMilimet(1));
    EXPECT_FLOAT_EQ(-pSize * vl, p.ToMilimet(-1));

    vl = 1.12;
    EXPECT_FLOAT_EQ(pSize * vl, p.ToMilimet(vl));
    EXPECT_FLOAT_EQ(-pSize * vl, p.ToMilimet(-vl));

    vl = 0;
    EXPECT_FLOAT_EQ(pSize * vl, p.ToMilimet(vl));
    EXPECT_FLOAT_EQ(-pSize * vl, p.ToMilimet(-vl));
}

template<typename T>
void MilimetTestFunctionRValue()
{
    T p{ 1.2 };

    auto pSize = 1.2f;
    double vl = 1;
    EXPECT_FLOAT_EQ(pSize * vl, p.ToMilimet(1));
    EXPECT_FLOAT_EQ(-pSize * vl, p.ToMilimet(-1));
}

TEST(TestCvt2mm, px2mm)
{
    double pxSize = 2.0f;
    PixelRef px{ pxSize };
    MilimetTestFunction<PixelRef>();
}
