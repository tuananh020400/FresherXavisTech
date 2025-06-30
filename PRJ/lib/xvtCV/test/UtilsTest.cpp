/**
 * @example{lineno} test/UtilsTest.cpp
 * Convert image and version information example  
 * This example demonstrates how to use xvt::Convert8Bits() function and xvt::VersionInfo class.
 */

#include "pch.h"
#include "xvtCV/Utils.h"
#include "xvtCV/VersionInfo.h"
#include "xvtCV/xvtTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

TEST(Convert8BitsTest, 16bits) {
    Mat src = Mat::zeros(Size(300,300), CV_16UC1);
    auto res = xvt::Convert8Bits(src, src);
    EXPECT_FALSE(res);
}

TEST(Convert8BitsTest, NullImage) {
    Mat src = Mat();
    auto res = xvt::Convert8Bits(src, src);
    EXPECT_TRUE(res);
}

TEST(Convert8BitsTest, ColorImage) {
    Mat src = Mat::zeros(Size(300, 300), CV_8UC3);
    auto res = xvt::Convert8Bits(src, src);
    EXPECT_FALSE(res);
}

TEST(VersionInfo, NestedVersionInfo)
{
    const xvt::VersionInfo xvtCV{
         "xvtCV"
        ,"0"
        ,"1"
        ,"0"
        ,"0"
        ,"commmit0"
        ,""
        ,"branch0"
    };

    const xvt::VersionInfo xvtBattery{
         std::string("xvtBattery")
        ,std::string("0")
        ,std::string("1")
        ,std::string("1")
        ,std::string("0")
        ,std::string("commmit1")
        ,std::string("")
        ,std::string("branch1")
        ,xvt::VersionInfoList{&xvtCV}
    };

    const xvt::VersionInfo xvtPCB{
         std::string("xvtPCB")
        ,std::string("0")
        ,std::string("1")
        ,std::string("2")
        ,std::string("0")
        ,std::string("commmit2")
        ,std::string("")
        ,std::string("branch2")
        ,xvt::VersionInfoList{&xvtCV}
    };

    const xvt::VersionInfo app{
         std::string("app")
        ,std::string("0")
        ,std::string("1")
        ,std::string("3")
        ,std::string("0")
        ,std::string("commmit3")
        ,std::string("")
        ,std::string("branch3")
        ,xvt::VersionInfoList{&xvtCV, &xvtBattery, &xvtPCB}
    };

    std::string app_version    = "app-0.1.3-0-commmit3--branch3";
    std::string xvtCV_version  = "xvtCV-0.1.0-0-commmit0--branch0";
    std::string xvtBat_version = "xvtBattery-0.1.1-0-commmit1--branch1";
    std::string xvtPCB_version = "xvtPCB-0.1.2-0-commmit2--branch2";

    // Get version information without the dependencies information
    auto tmp = app.GetVersionInfo(false);
    EXPECT_EQ(tmp, app_version);

    // Get version information without the dependencies information
    tmp = xvtBattery.GetVersionInfo(false);
    EXPECT_EQ(tmp, xvtBat_version);

    // Get version information without the dependencies information
    tmp = xvtPCB.GetVersionInfo(false);
    EXPECT_EQ(tmp, xvtPCB_version);

    // Get version information within the dependencies information
    auto tmp0 = app.GetVersionInfo();
    auto exp0 = app_version + " "
                + xvtCV_version + " "
                + xvtBat_version + " "
                + xvtPCB_version;
    EXPECT_EQ(tmp0, exp0);

    // Get version information within the dependencies information
    auto tmp2 = xvtBattery.GetVersionInfo();
    auto exp2 = xvtBat_version + " " + xvtCV_version;
    EXPECT_EQ(tmp2, exp2);

    // Get version information within the dependencies information
    auto tmp3 = xvtPCB.GetVersionInfo();
    auto exp3 = xvtPCB_version + " " + xvtCV_version;
    EXPECT_EQ(tmp3, exp3);

    //Get current xvtCV dll version info
    auto tmp4 = xvt::xvtCV_VersionInfo.GetVersionInfo();
    EXPECT_TRUE(tmp4.size() > 13);
    EXPECT_EQ(std::string(tmp4.begin(), tmp4.begin()+5),"xvtCV");
}

class base
{
public:
    base(int a = 0):mMem1 { a }{}
    virtual ~base() = 0 {};
    int mMem1=0;
};

class derive1 :public base
{
public:
    derive1(int a, int b) : base{ b }, mMem2{ a } {}
    int mMem2 = 0;
};

class derive2 :public base
{
public:
    derive2(int a, int b) : base{ b }, mMem3{ a } {}
    int mMem3 = 0;
};

TEST(DynamicCast, Success)
{
    auto d_ptr = unique_ptr<base>(new derive1(2, 1));

    auto ptr1 = xvt::DynamicCastUniquePtr<derive1>(std::move(d_ptr));
    EXPECT_TRUE(ptr1 != nullptr);
    EXPECT_TRUE(ptr1->mMem1 == 1);
    EXPECT_TRUE(ptr1->mMem2 == 2);

    d_ptr = unique_ptr<base>(new derive1(4,3));
    auto ptr2 = xvt::DynamicCastSharedPtr<derive1>(std::move(d_ptr));
    EXPECT_TRUE(ptr2 != nullptr);
    EXPECT_TRUE(ptr2->mMem1 == 3);
    EXPECT_TRUE(ptr2->mMem2 == 4);

    auto c_ptr = unique_ptr<derive1>(new derive1(5, 4));
    auto ptr3 = xvt::DynamicCastSharedPtr<base>(std::move(c_ptr));
    EXPECT_TRUE(ptr3 != nullptr);
    EXPECT_TRUE(ptr3->mMem1 == 4);
    auto ptr4 = dynamic_pointer_cast<derive1>(ptr3);
    EXPECT_TRUE(ptr4 != nullptr);
    EXPECT_TRUE(ptr4->mMem1 == 4);
    EXPECT_TRUE(ptr4->mMem2 == 5);
}

TEST(DynamicCast, Fail)
{
    auto d_ptr = unique_ptr<derive2>(new derive2(2,1));

    auto ptr1 = xvt::DynamicCastUniquePtr<derive1>(std::move(d_ptr));
    EXPECT_TRUE(d_ptr != nullptr);
    EXPECT_TRUE(ptr1 == nullptr);

    auto ptr2 = xvt::DynamicCastSharedPtr<derive1>(std::move(d_ptr));
    EXPECT_TRUE(d_ptr != nullptr);
    EXPECT_TRUE(ptr2 == nullptr);
}
