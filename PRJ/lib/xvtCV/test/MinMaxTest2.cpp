#include "pch.h"
#include <iostream>
#define NOMINMAX
#include <Windows.h>
#include <algorithm>
//#include <gdiplus.h>
TEST(MinMax, DefineNOMINMAX2) {
    int a = 5;
    int b = 10;

    int mMin = std::min(a, b);
    int mMax = std::max(a, b);

    EXPECT_EQ(mMin, 5);
    EXPECT_EQ(mMax, 10);
}
