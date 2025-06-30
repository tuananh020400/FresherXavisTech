/**
 * @example{lineno} test/xvtRangeTest.cpp
 * This example demonstrates how to use xvt::Range function.
 */
#include <iostream>
#include <stdexcept>
#include "pch.h"
#include "xvtCV/xvtRange.h"

class RangeIntTest : public ::testing::Test {
protected:
    xvt::Range<int> range;
};

TEST_F(RangeIntTest, SetValidRange_Int) {
    int lower = 50;
    int upper = 10;

    range.Set(lower, upper);

    EXPECT_TRUE(range.GetLower(), 10);
    EXPECT_TRUE(range.GetUpper(), 50);
    EXPECT_TRUE(range.mIsEnable);
}