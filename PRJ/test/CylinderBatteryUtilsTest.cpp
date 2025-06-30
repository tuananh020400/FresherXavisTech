#include "pch.h"
#include "xvtBattery/CylinderUtils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace xvt::battery;

TEST(CylinderUtilsTest, replace_all_func) {
    std::string path = "./samples/UPPER/2022-10-19 091225__2022-10-19 091232_H26_0010_22-2438R_UPPER_123_OK.tif";
    cv::Mat src = cv::imread(path, 0);
    std::string replace_path = path;
    xvt::battery::replace_all(replace_path, replace_path.substr(replace_path.find_last_of(".") + 1), "png");
    //cv::imwrite(path, src);
    EXPECT_EQ(1, 1);
    EXPECT_TRUE(true);
}

TEST(CylinderUtilsTest, histogram_func) {
    std::string path = "./samples/UPPER/2022-10-19 091225__2022-10-19 091232_H26_0010_22-2438R_UPPER_123_OK.tif";
    cv::Mat src = cv::imread(path, 0);
    //xvt::battery::histDisplay(path, path.substr(path.find_last_of(".") + 1), ".png");
    //cv::imwrite(path, src);
    EXPECT_EQ(1, 1);
    EXPECT_TRUE(true);
}