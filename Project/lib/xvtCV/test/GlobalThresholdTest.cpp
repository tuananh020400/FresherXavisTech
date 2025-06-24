/**
 * @example{lineno} test/GlobalThresholdTest.cpp
 * This example demonstrates how to use xvt::threshold classes.
 */

#include "pch.h"
#include "xvtCV/GlobalThresholding.h"
#include <opencv2/opencv.hpp>

class TestData
{
public:
    bool CheckImage(cv::Mat const& img)
    {
        bool rtn = !img.empty() && !_expImage.empty();
        if(rtn)
        {
            cv::Mat diff;
            cv::subtract(img, _expImage, diff, cv::noArray(), CV_16SC1);
            return cv::countNonZero(diff) == 0;
        }
        else
        {
            return img.empty() && _expImage.empty();
        }
    }

    cv::Mat _img;
    int     _lower;
    int     _upper;
    int     _type;
    cv::Mat _expImage;
};

class xvtThresholdTest : public testing::Test
{
protected:
    void SetUp() override
    {
        colorImg   = cv::Mat(3, 3, CV_8UC3, cv::Scalar(123, 124, 125));
        gray16Img  = (cv::Mat_<ushort>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
        gray8Img   = (cv::Mat_<uchar>(3, 3)  << 1, 2, 3, 4, 5, 6, 7, 8, 9);

        binImg5    = (cv::Mat_<uchar>(3, 3)  <<   0,   0,   0,   0,   0, 255, 255, 255, 255);
        binImg5Inv = (cv::Mat_<uchar>(3, 3)  << 255, 255, 255, 255, 255,   0,   0,   0,   0);
        binImg36   = (cv::Mat_<uchar>(3, 3)  <<   0,   0,   0, 255, 255, 255,   0,   0,   0);
        binImg36Inv= (cv::Mat_<uchar>(3, 3)  << 255, 255, 255,   0,   0,   0, 255, 255, 255);
        truncImg5  = (cv::Mat_<uchar>(3, 3) << 1, 2, 3, 4, 5, 5, 5, 5, 5);
        tozeroImg5 = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 0, 6, 7, 8, 9);
        tozeroImg5Inv = (cv::Mat_<uchar>(3, 3) << 1, 2, 3, 4, 5, 0, 0, 0, 0);
        zeroImg    = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);


        //imglist    = { emptyImg, gray8Img, gray16Img, colorImg };
        lowerList  = { -1,0,5,255,300 };
        upperList  = { -1,0,5,255,300 };
        typeList   = {
              cv::THRESH_BINARY
            , cv::THRESH_BINARY_INV
            , cv::THRESH_TRUNC
            , cv::THRESH_TOZERO
            , cv::THRESH_TOZERO_INV
            , cv::THRESH_MASK
            , cv::THRESH_OTSU
            , cv::THRESH_TRIANGLE
        };

        expectedList = {        
            {gray16Img, 0, 255, cv::THRESH_BINARY      , emptyImg}
            ,{colorImg, 0, 255, cv::THRESH_BINARY       , emptyImg}
            ,{emptyImg, 0, 255, cv::THRESH_BINARY       , emptyImg}
            ,{emptyImg, 5, 255, cv::THRESH_BINARY       , emptyImg}
            ,{gray8Img, 0, 255, cv::THRESH_BINARY       , binImg5}
            ,{gray8Img, 4,   0, cv::THRESH_BINARY       , binImg5}
            ,{gray8Img, 5, 255, cv::THRESH_BINARY       , binImg5}
            ,{gray8Img, 5, 255, cv::THRESH_BINARY_INV   , binImg5Inv}
            ,{gray8Img, 0, 255, cv::THRESH_OTSU         , binImg5}
            ,{gray8Img, 3, 6  , cv::THRESH_BINARY       , binImg36}
            ,{gray8Img, 3, 6  , cv::THRESH_BINARY_INV   , binImg36Inv}

            ,{gray8Img, 6, 3  , cv::THRESH_BINARY_INV   , binImg36Inv}
            ,{gray8Img,  5, 10, cv::THRESH_TRUNC        , truncImg5}
            ,{gray8Img, 10, 10, cv::THRESH_TRUNC        , gray8Img}
            //,{gray8Img,  0,255, cv::THRESH_TRUNC        , zeroImg}
            //,{gray8Img,  5,  0, cv::THRESH_TRUNC        , truncImg5}
            ,{gray8Img,  5, 10, cv::THRESH_TOZERO       , tozeroImg5}
            ,{gray8Img,  5, 10, cv::THRESH_TOZERO_INV   , tozeroImg5Inv}
        };

    }

    cv::Mat binImg5;
    cv::Mat binImg5Inv;
    cv::Mat binImg36;
    cv::Mat binImg36Inv;
    cv::Mat truncImg5;
    cv::Mat tozeroImg5;
    cv::Mat tozeroImg5Inv;
    cv::Mat zeroImg;

    cv::Mat emptyImg;
    cv::Mat gray8Img;
    cv::Mat gray16Img;
    cv::Mat colorImg;
    std::vector<int> lowerList;
    std::vector<int> upperList;
    std::vector<int> typeList;
    std::vector<cv::Mat>  imglist;
    std::vector<TestData> expectedList;
};

TEST(GlobalThreshold, OtsuMulti)
{
    cv::Mat a = cv::Mat(200, 200, CV_8U, cv::Scalar(255));
    xvt::threshold::ThresholdOtsuMulti(a);
    cv::Mat b = cv::Mat(200, 200, CV_8U, cv::Scalar(255));
    xvt::threshold::ThresholdOtsuMulti(b, 2);

    EXPECT_TRUE(!a.empty());
    EXPECT_TRUE(!b.empty());
}

void TestCrashed(std::vector<cv::Mat>& imglist
                     , std::vector<int>& lowerList
                     , std::vector<int>& upperList
                     , std::vector<int>& typeList
)
{
    cv::Mat dst_image;
    for (auto& img : imglist)
    {
        for (auto& l : lowerList)
        {
            for (auto& u : upperList)
            {
                for (auto& t : typeList)
                {
                    xvt::threshold::Threshold(img, dst_image, l, u, t);
                }
            }
        }
    }
}

TEST_F(xvtThresholdTest, CrashedTest)
{
    TestCrashed(imglist, lowerList, upperList, typeList);
}

TEST_F(xvtThresholdTest, TestOK)
{
    cv::Mat dst_image;
    for (auto& data : expectedList)
    {
        std::cout << data._lower << " " << data._upper << "\n";
        xvt::threshold::Threshold(data._img, dst_image, data._lower, data._upper, data._type);
        auto ok = data.CheckImage(dst_image);
        EXPECT_TRUE(ok);
    }
    xvt::threshold::Threshold(gray8Img, dst_image, 0, 0, cv::THRESH_BINARY);

}
