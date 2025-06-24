/**
 * @example{lineno} test/ConvertMilimeterTest.cpp
 * This example demonstrates how to use xvt::PixelInfo.h function.
 */

#include "pch.h"
#include "xvtCV/PixelRef.h"

void PixelMemberTest(xvt::PixelInfo& p, double pSize, double scale)
{
    double exp_opticalsize = pSize / scale;
    EXPECT_DOUBLE_EQ(pSize, p.GetPhysicSize());
    EXPECT_DOUBLE_EQ(scale, p.GetScale());
    EXPECT_DOUBLE_EQ(exp_opticalsize, p.GetOpticalSize());
}

void PixelConvertTest(xvt::PixelInfo& p, double exp_px, double exp_mm)
{
    // test convert to pixel
    EXPECT_DOUBLE_EQ( exp_px, p.ToPixel( exp_mm));
    EXPECT_DOUBLE_EQ(-exp_px, p.ToPixel(-exp_mm));
    // test convert to millimeter
    EXPECT_DOUBLE_EQ( exp_mm, p.ToMillimeter( exp_px));
    EXPECT_DOUBLE_EQ(-exp_mm, p.ToMillimeter(-exp_px));
}

TEST(TestCvt2mm, TestPixelInfo)
{
    double pSize = 0.050;   // pixel size
    double pScale = 1.0;    // Optical scale
    xvt::PixelInfo p(pSize, pScale);

    double exp_mm = 1;
    double exp_px = exp_mm / pSize * pScale;

    PixelMemberTest(p, pSize, pScale);

    pScale = 1.5;
    exp_px = exp_mm / pSize * pScale;
    double exp_opticalsize = pSize / pScale;
    p.SetScale(pScale);
    // test the optical size calculation
    PixelMemberTest(p, pSize, pScale);
    PixelConvertTest(p, exp_px, exp_mm);

    // test change the pixel size
    p = xvt::PixelInfo(1.0, pScale);
    pSize = 0.01;
    exp_px = exp_mm / pSize * pScale;
    p.SetPhysicSize(pSize);
    PixelMemberTest(p, pSize, pScale);
    PixelConvertTest(p, exp_px, exp_mm);

    // test manual update the optical size
    double opticalSize2 = 0.01;
    p.SetOpticalSize(opticalSize2);
    pScale = pSize / opticalSize2;
    exp_px = exp_mm / pSize * pScale;

    EXPECT_DOUBLE_EQ(opticalSize2, p.GetOpticalSize());
    PixelMemberTest(p, pSize, pScale);
    PixelConvertTest(p, exp_px, exp_mm);
}

TEST(TestCvt2mm, px2mmPixelRef)
{
    double pSize = 2.0;
    xvt::PixelRef p{ pSize };
    EXPECT_DOUBLE_EQ(2, p.ToMilimet(1));
    EXPECT_DOUBLE_EQ(-2, p.ToMilimet(-1));

    pSize = 2.5;
    double vl = 1;
    EXPECT_DOUBLE_EQ(pSize * vl, p.ToMilimet(1));
    EXPECT_DOUBLE_EQ(-pSize * vl, p.ToMilimet(-1));

    vl = 1.12;
    EXPECT_DOUBLE_EQ(pSize * vl, p.ToMilimet(vl));
    EXPECT_DOUBLE_EQ(-pSize * vl, p.ToMilimet(-vl));

    vl = 0;
    EXPECT_DOUBLE_EQ(pSize * vl, p.ToMilimet(vl));
    EXPECT_DOUBLE_EQ(-pSize * vl, p.ToMilimet(-vl));
}

TEST(TestCvt2mm, mm2pxPixelRef)
{
    double pSize = 2.0;
    xvt::PixelRef p{ pSize };
    double vl = 2;
    EXPECT_DOUBLE_EQ(vl / pSize, p.ToPixel(vl));
    EXPECT_DOUBLE_EQ(-vl / pSize, p.ToPixel(-vl));

    pSize = 2.5;
    vl = 1;
    EXPECT_DOUBLE_EQ(vl / pSize, p.ToPixel(vl));
    EXPECT_DOUBLE_EQ(-vl / pSize, p.ToPixel(-vl));

    vl = 1.12;
    EXPECT_DOUBLE_EQ(vl / pSize, p.ToPixel(vl));
    EXPECT_DOUBLE_EQ(-vl / pSize, p.ToPixel(-vl));

    vl = 0;
    EXPECT_DOUBLE_EQ(vl / pSize, p.ToPixel(vl));
    EXPECT_DOUBLE_EQ(-vl / pSize, p.ToPixel(-vl));
}

TEST(TestCvt2mm, LargeAndSmallValues)
{
    double largeValue = 1.0e20;
    double pixelSizeLarge = 1.0e5;
    xvt::PixelRef pLarge{ pixelSizeLarge };

    EXPECT_DOUBLE_EQ(largeValue * pixelSizeLarge, pLarge.ToMilimet(largeValue));
    EXPECT_DOUBLE_EQ(largeValue / pixelSizeLarge, pLarge.ToPixel(largeValue));

    double smallValue = 1.0e-20;
    double pixelSizeSmall = 1.0e-5;
    xvt::PixelRef pSmall{ pixelSizeSmall };

    EXPECT_DOUBLE_EQ(smallValue * pixelSizeSmall, pSmall.ToMilimet(smallValue));
    EXPECT_DOUBLE_EQ(smallValue / pixelSizeSmall, pSmall.ToPixel(smallValue));
}
