#include "pch.h"
#include "xvtCV/IInspection.h"

using namespace xvt;

class TestResult : public ::testing::Test{
protected:
    void SetUp() override
    {

    }
    void TearDown() override
    {

    }
};

TEST_F(TestResult, TestCombineEResult)
{
    EResult resultList[] = {
            EResult::UC
           ,EResult::OK
           ,EResult::NG
           ,EResult::ER
    };

    EResult verifyMap[]{
         EResult::UC, EResult::OK,  EResult::NG, EResult::ER
        ,EResult::OK, EResult::OK,  EResult::NG, EResult::ER
        ,EResult::NG, EResult::NG,  EResult::NG, EResult::ER
        ,EResult::ER, EResult::ER,  EResult::ER, EResult::ER
    };

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            auto test = CombineResult(resultList[j], resultList[i]);
            auto test2 = resultList[j] & resultList[i];
            auto test3 = resultList[j];
            test3 &= resultList[i];
            EXPECT_TRUE(test  == verifyMap[i * 4 + j]);
            EXPECT_TRUE(test2 == verifyMap[i * 4 + j]);
            EXPECT_TRUE(test3 == verifyMap[i * 4 + j]);
        }
    }
}