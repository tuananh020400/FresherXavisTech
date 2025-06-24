#include "pch.h"
#include <iostream>
#include <Windows.h>
#include <algorithm>
#include <gdiplus.h>
#include <xvtCV/xvtProperty.h>

TEST(MinMax, StandardCppUsage1) {
	using namespace std;
	int a = 5;
	int b = 110;

	int mMin = min(a, b);
	int mMax = max(a, b);

	EXPECT_EQ(mMin, 5); 
	EXPECT_EQ(mMax, 110); 
}

TEST(MinMax, RedefineInFiles3) {
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

    using namespace std;

    int a = 5;
    int b = 10;

    int mMin = min(a, b);
    int mMax = max(a, b);

    EXPECT_EQ(mMin, 5);
    EXPECT_EQ(mMax, 10);
}

TEST(MinMax, RedefineWithNonStandard4) {
#ifndef max
#define max __max
#endif
#ifndef min
#define min __min
#endif

    int a = 5;
    int b = 10;

    int mMin = __min(a, b);
    int mMax = __max(a, b);

    EXPECT_EQ(mMin, 5);
    EXPECT_EQ(mMax, 10);
}

TEST(MinMax, CompilerOptions5) {
    int a = 5;
    int b = 10;

    int mMin = __min(a, b);
    int mMax = __max(a, b);

    EXPECT_EQ(mMin, 5);
    EXPECT_EQ(mMax, 10);
}

TEST(MinMax, Test6) {
    int a = 15;
    int b = 10;

    int mMin = (std::min)(a, b);
    int mMax = (std::max)(a, b);
    EXPECT_EQ(mMin, 10);
    EXPECT_EQ(mMax, 15);
}

