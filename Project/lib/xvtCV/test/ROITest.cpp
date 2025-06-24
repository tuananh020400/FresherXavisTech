/**
 * @example{lineno} test/ROITest.cpp
 * This example demonstrates how to use xvt::xvt::RefineROI function.
 */

#include "pch.h"
#include "xvtCV/Utils.h"

using namespace xvt;

//ROI in the size
TEST(RefineROITest, Case01)
{
    //case 1
    auto ROI = cv::Rect(400, 300, 200, 150);
    auto ROIOutput = cv::Rect(400, 300, 200, 150);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);
}
//x<0, y<0
TEST(RefineROITest, Case02)
{
    //case 2.1
    cv::Rect ROI = cv::Rect(50, -20, 200, 150);
    cv::Rect ROIOutput = cv::Rect(50, 0, 200, 130);
    bool rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);

    //case 2.2
    ROI = cv::Rect(-50, 100, 200, 150);
    ROIOutput = cv::Rect(0, 100, 150, 150);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);

    //case 2.3
    ROI = cv::Rect(-50, -100, 200, 150);
    ROIOutput = cv::Rect(0, 0, 150, 50);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);
}
//x+w>W, y+h>H
TEST(RefineROITest, Case03)
{
    //case 3.1
    auto ROI = cv::Rect(800, 900, 300, 200);
    auto ROIOutput = cv::Rect(800, 900, 200, 100);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);

    //case 3.2
    ROI = cv::Rect(500, -100, 1000, 600);
    ROIOutput = cv::Rect(500, 0, 500, 500);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);

    //case 3.3
    ROI = cv::Rect(-500, -100, 2000, 2000);
    ROIOutput = cv::Rect(0, 0, 1000, 1000);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);
}
//x>W,y>H
TEST(RefineROITest, Case04)
{
    //case 4
    auto ROI = cv::Rect(1100, 1100, 200, 150);
    auto ROIOutput = cv::Rect(999, 999, 0, 0);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);
}
//x+w<0, y+h<0
TEST(RefineROITest, Case05)
{
    //case 5.1
    auto ROI = cv::Rect(-250, 500, 200, 150);
    auto ROIOutput = cv::Rect(0, 500, 0, 150);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 5.2
    ROI = cv::Rect(10, -500, 200, 200);
    ROIOutput = cv::Rect(10, 0, 200, 0);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 5.3
    ROI = cv::Rect(-300, -200, 200, 150);
    ROIOutput = cv::Rect(0, 0, 0, 0);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);
}
//x+w=0, y+h=0
TEST(RefineROITest, Case07)
{
    //case 7.1
    auto ROI = cv::Rect(-200, 300, 200, 150);
    auto ROIOutput = cv::Rect(0, 300, 0, 150);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 7.2
    ROI = cv::Rect(200, -300, 200, 300);
    ROIOutput = cv::Rect(200, 0, 200, 0);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 7.3
    ROI = cv::Rect(-200, -300, 200, 300);
    ROIOutput = cv::Rect(0, 0, 0, 0);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);
}
//x=W, y=H
TEST(RefineROITest, Case08)
{
    //case 8.1
    auto ROI = cv::Rect(400, 1000, 200, 150);
    auto ROIOutput = cv::Rect(400, 999, 200, 0);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 8.2
    ROI = cv::Rect(1000, 300, 200, 300);
    ROIOutput = cv::Rect(999, 300, 0, 300);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 8.3
    ROI = cv::Rect(1000, 1000, 200, 300);
    ROIOutput = cv::Rect(999, 999, 0, 0);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);
}
//x+w=W, y+h=H
TEST(RefineROITest, Case09)
{
    //case 9.1
    auto ROI = cv::Rect(800, 0, 200, 200);
    auto ROIOutput = cv::Rect(800, 0, 200, 200);
    auto rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);

    //case 9.2
    ROI = cv::Rect(0, 800, 200, 200);
    ROIOutput = cv::Rect(0, 800, 200, 200);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);

    //case 9.3
    ROI = cv::Rect(800, 800, 200, 200);
    ROIOutput = cv::Rect(800, 800, 200, 200);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, true);
}
//W<=0, H<=0
TEST(RefineROITest, Case10)
{
    //case 10.1
    auto ROI = cv::Rect(800, 0, 200, 150);
    auto ROIOutput = cv::Rect(0, 0, 0, 0);
    auto rtn = xvt::RefineROI(ROI, cv::Size(0, 0));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 10.2
    rtn = xvt::RefineROI(ROI, cv::Size(0, 1000));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 10.3
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 0));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 10.4
    ROI = cv::Rect(800, 0, 200, 150);
    rtn = xvt::RefineROI(ROI, cv::Size(-10, -10));
    EXPECT_EQ(ROI, ROIOutput);
    EXPECT_EQ(rtn, false);

    //case 10.5
    ROI = cv::Rect(1000, 0, -200, 150);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, cv::Rect(999, 0, 0, 150));
    EXPECT_EQ(rtn, false);

    //case 10.6
    ROI = cv::Rect(1000, 0, -200, -150);
    rtn = xvt::RefineROI(ROI, cv::Size(1000, 1000));
    EXPECT_EQ(ROI, cv::Rect(999, 0, 0, 0));
    EXPECT_EQ(rtn, false);
}

// Test the RefineROI function
TEST(ROI_Test, RefineROI_InsideParent) {
    cv::Rect parentROI(0, 0, 100, 100);
    cv::Rect childROI(20, 20, 50, 50);

    // Expect the refinement to succeed (should not change since it's already inside)
    bool result = RefineROI(childROI, parentROI);
    EXPECT_TRUE(result);
    EXPECT_EQ(childROI, cv::Rect(20, 20, 50, 50));
}

TEST(ROI_Test, RefineROI_OutsideParent) {
    cv::Rect parentROI(0, 0, 100, 100);
    cv::Rect childROI(90, 90, 30, 30);

    // Expect the refinement to modify the child ROI and make it fit within the parent
    bool result = RefineROI(childROI, parentROI);
    EXPECT_TRUE(result);
    EXPECT_EQ(childROI, cv::Rect(90, 90, 10, 10)); // It should shrink to fit within the parent
}

TEST(ROI_Test, RefineROI_EmptyROI) {
    cv::Rect parentROI(0, 0, 100, 100);
    cv::Rect childROI(150, 150, 20, 20);

    // Expect the ROI to be invalid since it's completely outside
    bool result = RefineROI(childROI, parentROI);
    EXPECT_FALSE(result);
    EXPECT_TRUE(childROI.empty());
}

// Test the CreateROI function
TEST(ROI_Test, CreateROI_ValidInsideParent) {
    cv::Rect parentROI(0, 0, 100, 100);
    cv::Rect expectedROI(10, 10, 30, 30);

    // Create a ROI fully within the parent
    cv::Rect createdROI = CreateROI(10, 10, 30, 30, parentROI);
    EXPECT_EQ(createdROI, expectedROI);
}

TEST(ROI_Test, CreateROI_ValidOutsideParent) {
    cv::Rect parentROI(0, 0, 100, 100);
    cv::Rect expectedROI(50, 50, 50, 50);

    // Create a ROI partially outside, but it should be refined to fit inside the parent
    cv::Rect createdROI = CreateROI(50, 50, 100, 100, parentROI);
    EXPECT_EQ(createdROI, expectedROI);
}

TEST(ROI_Test, CreateROI_EmptyOutsideParent) {
    cv::Rect parentROI(0, 0, 100, 100);

    // Create a ROI completely outside the parent, resulting in an empty ROI
    cv::Rect createdROI = CreateROI(150, 150, 50, 50, parentROI);
    EXPECT_TRUE(createdROI.empty());
}
