// InspectionCylBatt.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <omp.h>
#include <ctime>
#include <fstream>
#include <numeric>
#include <string>
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryCT.h"
#include "ImageProcessorCylinderBatUpperCT.h"
#include "ImageProcessorCylinderBatLowerCT.h"

int main()
{
	std::cout << "Inspection Cyl Battery Start" << std::endl;
	{ // Upper no beading inspection
		xf::ImageProcessor::CImageProcessorCylinderBatUpperCT cylUpperAlgo;
		struct xvt::CylinderCTInspection cylUpperParam;
		struct BatUpperResultStruct cylUpperResult;

		std::string fileName = "../samples/CT/16_20_42.565_3_78_16_512_512_beading_16u.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 

		cylUpperParam.UseBatteryInspection = 1;
		cylUpperParam.BatteryDirection = 0;

		cylUpperParam.StepNo = 1024;
		cylUpperParam.SliceNo = 1;
		cylUpperParam.Angle = 108;
		cylUpperParam.RoiX = 0;
		cylUpperParam.RoiY = 250;
		cylUpperParam.RoiWidth = 1000;
		cylUpperParam.RoiHeight = 240;

		cylUpperParam.batteryThreshold = 70;
		cylUpperParam.minBeadingHeight = 50;
		cylUpperParam.D1StartPosition = 0.5;
		cylUpperParam.poleRegionHeight = 100;
		cylUpperParam.PoleLeaningThreshold = 0;

		cylUpperParam.JR_ROIX = 20;
		cylUpperParam.JR_ROIY = 65;
		cylUpperParam.CenterNeglectionWidth = 140;
		cylUpperParam.tolerenceShift = 0;

		cylUpperParam.cathodeLineThredsholdInner = 0.0;
		cylUpperParam.cathodeLineThredsholdMiddle = 0.0;
		cylUpperParam.cathodeLineThredsholdOuter = 0.0;
		cylUpperParam.cathodeLineWindowSize = 5;
		cylUpperParam.polesHeight = 80;
		cylUpperParam.minProminence = 1;

		cylUpperParam.polesMinDistance = 3;
		cylUpperParam.polesMaxDistance = 30;

		cylUpperParam.anodeThresholdInner = 1.2;
		cylUpperParam.anodeThresholdMiddle = 2.2;
		cylUpperParam.anodeThresholdOuter = 2.2;
		cylUpperParam.OneSidePoleNumb = 70;
		cylUpperParam.isCheckPin = 0;

		cylUpperParam.skipPolesDistance = 0;
		cylUpperParam.MinCathode2Case = 3.5;
		cylUpperParam.MaxCathode2Case = 7;

		cylUpperParam.mAnode2CaseOffset = 0;
		cylUpperParam.mCathode2AnodeOffset = 0;
		cylUpperParam.mCathode2CaseOffset = 0;

		cylUpperParam.variationAnode2Case = 3;

		cylUpperParam.pixelSize = 0.050;
		cylUpperParam.gamma = 10;

		cylUpperParam.minLeaningDistance = 0;

		cylUpperParam.displayMode = 7;
		cylUpperParam.isSaveResults = 0;  // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylUpperParam.resultPath = "D:\\Result1";
		cylUpperParam.cellID = "0";

		if (GetFileAttributes((CString)cylUpperParam.resultPath.c_str()) == 0xFFFFFFFF) CreateDirectory((CString)cylUpperParam.resultPath.c_str(), NULL);
		cylUpperAlgo.ImageProcessing(imgSrc, cylUpperParam, cylUpperResult);
		//std::cout << "Upper csvTitle :" << (std::string)CT2CA(cylUpperResult.csvTitleStr) << std::endl;
		//std::cout << "Upper csvResult :" << (std::string)CT2CA(cylUpperResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Upper csvTitleXM :" << (std::string)CT2CA(cylUpperResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Upper csvResultXM :" << (std::string)CT2CA(cylUpperResult.csvResultStrXM4) << std::endl << std::endl;

		//cv::imshow("srcUpperImg", imgSrc);
		cv::imshow("resUpperImg1_" + fileName, cylUpperResult.resImg);
	}

	if(0)
	{ // Upper no beading inspection
		xf::ImageProcessor::CImageProcessorCylinderBatUpperCT cylUpperAlgo;
		struct xvt::CylinderCTInspection cylUpperParam;
		struct BatUpperResultStruct cylUpperResult;

		std::string fileName = "sample2_top_3_60_8_512_507_16u.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 

		cylUpperParam.UseBatteryInspection = 1;
		cylUpperParam.BatteryDirection = 0;

		cylUpperParam.StepNo = 1024;
		cylUpperParam.SliceNo = 1;
		cylUpperParam.Angle = 108;
		cylUpperParam.RoiX = 0;
		cylUpperParam.RoiY = 250;
		cylUpperParam.RoiWidth = 1000;
		cylUpperParam.RoiHeight = 240;

		cylUpperParam.batteryThreshold = 70;
		cylUpperParam.minBeadingHeight = 50;
		cylUpperParam.D1StartPosition = 0.5;
		cylUpperParam.poleRegionHeight = 100;
		cylUpperParam.PoleLeaningThreshold = 0;

		cylUpperParam.JR_ROIX = 20;
		cylUpperParam.JR_ROIY = 65;
		cylUpperParam.CenterNeglectionWidth = 140;
		cylUpperParam.tolerenceShift = 0;

		cylUpperParam.cathodeLineThredsholdInner = 0.0;
		cylUpperParam.cathodeLineThredsholdMiddle = 0.0;
		cylUpperParam.cathodeLineThredsholdOuter = 0.0;
		cylUpperParam.cathodeLineWindowSize = 5;
		cylUpperParam.polesHeight = 80;
		cylUpperParam.minProminence = 1;

		cylUpperParam.polesMinDistance = 3;
		cylUpperParam.polesMaxDistance = 30;

		cylUpperParam.anodeThresholdInner = 1.2;
		cylUpperParam.anodeThresholdMiddle = 2.2;
		cylUpperParam.anodeThresholdOuter = 2.2;
		cylUpperParam.OneSidePoleNumb = 70;
		cylUpperParam.isCheckPin = 0;

		cylUpperParam.skipPolesDistance = 0;
		cylUpperParam.MinCathode2Case = 3.5;
		cylUpperParam.MaxCathode2Case = 7;

		cylUpperParam.mAnode2CaseOffset = 0;
		cylUpperParam.mCathode2AnodeOffset = 0;
		cylUpperParam.mCathode2CaseOffset = 0;

		cylUpperParam.variationAnode2Case = 3;

		cylUpperParam.pixelSize = 0.050;
		cylUpperParam.gamma = 10;

		cylUpperParam.minLeaningDistance = 0;

		cylUpperParam.displayMode = 7;
		cylUpperParam.isSaveResults = 0;  // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylUpperParam.resultPath = "D:\\Result1";
		cylUpperParam.cellID = "0";

		if (GetFileAttributes((CString)cylUpperParam.resultPath.c_str()) == 0xFFFFFFFF) CreateDirectory((CString)cylUpperParam.resultPath.c_str(), NULL);
		cylUpperAlgo.ImageProcessing(imgSrc, cylUpperParam, cylUpperResult);
		//std::cout << "Upper csvTitle :" << (std::string)CT2CA(cylUpperResult.csvTitleStr) << std::endl;
		//std::cout << "Upper csvResult :" << (std::string)CT2CA(cylUpperResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Upper csvTitleXM :" << (std::string)CT2CA(cylUpperResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Upper csvResultXM :" << (std::string)CT2CA(cylUpperResult.csvResultStrXM4) << std::endl << std::endl;

		//cv::imshow("srcUpperImg", imgSrc);
		cv::imshow("resUpperImg2_" + fileName, cylUpperResult.resImg);
	}
	{ // Upper no beading inspection
		xf::ImageProcessor::CImageProcessorCylinderBatUpperCT cylUpperAlgo;
		struct xvt::CylinderCTInspection cylUpperParam;
		struct BatUpperResultStruct cylUpperResult;

		std::string fileName = "../samples/CT/sample3_top_3_60_8_511_509_16u.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 

		cylUpperParam.UseBatteryInspection = 1;
		cylUpperParam.BatteryDirection = 0;

		cylUpperParam.StepNo = 1024;
		cylUpperParam.SliceNo = 1;
		cylUpperParam.Angle = 108;
		cylUpperParam.RoiX = 0;
		cylUpperParam.RoiY = 300;
		cylUpperParam.RoiWidth = 1000;
		cylUpperParam.RoiHeight = 240;

		cylUpperParam.batteryThreshold = 70;
		cylUpperParam.minBeadingHeight = 50;
		cylUpperParam.D1StartPosition = 0.5;
		cylUpperParam.poleRegionHeight = 100;
		cylUpperParam.PoleLeaningThreshold = 0;

		cylUpperParam.JR_ROIX = 20;
		cylUpperParam.JR_ROIY = 100;
		cylUpperParam.CenterNeglectionWidth = 140;
		cylUpperParam.tolerenceShift = 0;

		cylUpperParam.cathodeLineThredsholdInner = 0.0;
		cylUpperParam.cathodeLineThredsholdMiddle = 0.0;
		cylUpperParam.cathodeLineThredsholdOuter = 0.0;
		cylUpperParam.cathodeLineWindowSize = 5;
		cylUpperParam.polesHeight = 80;
		cylUpperParam.minProminence = 1;

		cylUpperParam.polesMinDistance = 3;
		cylUpperParam.polesMaxDistance = 30;

		cylUpperParam.anodeThresholdInner = 1.2;
		cylUpperParam.anodeThresholdMiddle = 2.2;
		cylUpperParam.anodeThresholdOuter = 2.2;
		cylUpperParam.OneSidePoleNumb = 70;
		cylUpperParam.isCheckPin = 0;

		cylUpperParam.skipPolesDistance = 0;
		cylUpperParam.MinCathode2Case = 3.5;
		cylUpperParam.MaxCathode2Case = 10;

		cylUpperParam.mAnode2CaseOffset = 0;
		cylUpperParam.mCathode2AnodeOffset = 0;
		cylUpperParam.mCathode2CaseOffset = 0;

		cylUpperParam.variationAnode2Case = 3;

		cylUpperParam.pixelSize = 0.050;
		cylUpperParam.gamma = 10;

		cylUpperParam.minLeaningDistance = 0;

		cylUpperParam.displayMode = 7;
		cylUpperParam.isSaveResults = 0;  // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylUpperParam.resultPath = "D:\\Result1";
		cylUpperParam.cellID = "0";

		if (GetFileAttributes((CString)cylUpperParam.resultPath.c_str()) == 0xFFFFFFFF) CreateDirectory((CString)cylUpperParam.resultPath.c_str(), NULL);
		cylUpperAlgo.ImageProcessing(imgSrc, cylUpperParam, cylUpperResult);
		//std::cout << "Upper csvTitle :" << (std::string)CT2CA(cylUpperResult.csvTitleStr) << std::endl;
		//std::cout << "Upper csvResult :" << (std::string)CT2CA(cylUpperResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Upper csvTitleXM :" << (std::string)CT2CA(cylUpperResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Upper csvResultXM :" << (std::string)CT2CA(cylUpperResult.csvResultStrXM4) << std::endl << std::endl;

		//cv::imshow("srcUpperImg", imgSrc);
		cv::imshow("resUpperImg3_" + fileName, cylUpperResult.resImg);
	}

	if(0)
	{ // Upper no beading inspection
		xf::ImageProcessor::CImageProcessorCylinderBatUpperCT cylUpperAlgo;
		struct xvt::CylinderCTInspection cylUpperParam;
		struct BatUpperResultStruct cylUpperResult;

		std::string fileName = "sample4_top_3_60_8_510_522_16u.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 

		cylUpperParam.UseBatteryInspection = 1;
		cylUpperParam.BatteryDirection = 0;

		cylUpperParam.StepNo = 1024;
		cylUpperParam.SliceNo = 1;
		cylUpperParam.Angle = 108;
		cylUpperParam.RoiX = 0;
		cylUpperParam.RoiY = 250;
		cylUpperParam.RoiWidth = 1000;
		cylUpperParam.RoiHeight = 240;

		cylUpperParam.batteryThreshold = 50;
		cylUpperParam.minBeadingHeight = 50;
		cylUpperParam.D1StartPosition = 0.5;
		cylUpperParam.poleRegionHeight = 100;
		cylUpperParam.PoleLeaningThreshold = 0;

		cylUpperParam.JR_ROIX = 30;
		cylUpperParam.JR_ROIY = 100;
		cylUpperParam.CenterNeglectionWidth = 140;
		cylUpperParam.tolerenceShift = 0;

		cylUpperParam.cathodeLineThredsholdInner = 0.0;
		cylUpperParam.cathodeLineThredsholdMiddle = 0.0;
		cylUpperParam.cathodeLineThredsholdOuter = 0.0;
		cylUpperParam.cathodeLineWindowSize = 5;
		cylUpperParam.polesHeight = 80;
		cylUpperParam.minProminence = 1;

		cylUpperParam.polesMinDistance = 3;
		cylUpperParam.polesMaxDistance = 30;

		cylUpperParam.anodeThresholdInner = 1.2;
		cylUpperParam.anodeThresholdMiddle = 2.2;
		cylUpperParam.anodeThresholdOuter = 2.2;
		cylUpperParam.OneSidePoleNumb = 70;
		cylUpperParam.isCheckPin = 0;

		cylUpperParam.skipPolesDistance = 20;
		cylUpperParam.MinCathode2Case = 3.5;
		cylUpperParam.MaxCathode2Case = 10;

		cylUpperParam.mAnode2CaseOffset = 0;
		cylUpperParam.mCathode2AnodeOffset = 0;
		cylUpperParam.mCathode2CaseOffset = 0;

		cylUpperParam.variationAnode2Case = 7;

		cylUpperParam.pixelSize = 0.050;
		cylUpperParam.gamma = 10;

		cylUpperParam.minLeaningDistance = 0;

		cylUpperParam.displayMode = 7;
		cylUpperParam.isSaveResults = 0;  // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylUpperParam.resultPath = "D:\\Result1";
		cylUpperParam.cellID = "0";

		if (GetFileAttributes((CString)cylUpperParam.resultPath.c_str()) == 0xFFFFFFFF) CreateDirectory((CString)cylUpperParam.resultPath.c_str(), NULL);
		cylUpperAlgo.ImageProcessing(imgSrc, cylUpperParam, cylUpperResult);
		//std::cout << "Upper csvTitle :" << (std::string)CT2CA(cylUpperResult.csvTitleStr) << std::endl;
		//std::cout << "Upper csvResult :" << (std::string)CT2CA(cylUpperResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Upper csvTitleXM :" << (std::string)CT2CA(cylUpperResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Upper csvResultXM :" << (std::string)CT2CA(cylUpperResult.csvResultStrXM4) << std::endl << std::endl;

		//cv::imshow("srcUpperImg", imgSrc);
		cv::imshow("resUpperImg4_" + fileName, cylUpperResult.resImg);
	}

	if(0)
	{ // Upper inspection
		xf::ImageProcessor::CImageProcessorCylinderBatUpperCT cylUpperAlgo;
		struct xvt::CylinderCTInspection cylUpperParam;
		struct BatUpperResultStruct cylUpperResult;

		std::string fileName = "anode_side.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 

		cylUpperParam.UseBatteryInspection = 1;
		cylUpperParam.BatteryDirection = 0;

		cylUpperParam.StepNo = 1024;
		cylUpperParam.SliceNo = 1;
		cylUpperParam.Angle = 108;
		cylUpperParam.RoiX = 50;
		cylUpperParam.RoiY = 900;
		cylUpperParam.RoiWidth = 1900;
		cylUpperParam.RoiHeight = 350;

		cylUpperParam.batteryThreshold = 70;
		cylUpperParam.minBeadingHeight = 50;
		cylUpperParam.D1StartPosition = 0.5;
		cylUpperParam.poleRegionHeight = 200;
		cylUpperParam.PoleLeaningThreshold = 0;

		cylUpperParam.JR_ROIX = 40;
		cylUpperParam.JR_ROIY = 100;
		cylUpperParam.CenterNeglectionWidth = 200;
		cylUpperParam.tolerenceShift = 0;

		cylUpperParam.cathodeLineThredsholdInner = 0.0;
		cylUpperParam.cathodeLineThredsholdMiddle = 0.0;
		cylUpperParam.cathodeLineThredsholdOuter = 0.0;
		cylUpperParam.cathodeLineWindowSize = 5;
		cylUpperParam.polesHeight = 80;
		cylUpperParam.minProminence = 1;

		cylUpperParam.polesMinDistance = 8;
		cylUpperParam.polesMaxDistance = 30;

		cylUpperParam.anodeThresholdInner = 1.2;
		cylUpperParam.anodeThresholdMiddle = 2.2;
		cylUpperParam.anodeThresholdOuter = 2.2;
		cylUpperParam.OneSidePoleNumb = 70;
		cylUpperParam.isCheckPin = 0;

		cylUpperParam.skipPolesDistance = 60;
		cylUpperParam.MinCathode2Case = 3.0;
		cylUpperParam.MaxCathode2Case = 6.0;

		cylUpperParam.mAnode2CaseOffset = 0;
		cylUpperParam.mCathode2AnodeOffset = 0;
		cylUpperParam.mCathode2CaseOffset = 0;

		cylUpperParam.variationAnode2Case = 3;

		cylUpperParam.pixelSize = 0.025;
		cylUpperParam.gamma = 10;

		cylUpperParam.minLeaningDistance = 0;

		cylUpperParam.displayMode = 7;
		cylUpperParam.isSaveResults = 0;  // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylUpperParam.resultPath = "D:\\Result1";
		cylUpperParam.cellID = "0";

		if (GetFileAttributes((CString)cylUpperParam.resultPath.c_str()) == 0xFFFFFFFF) CreateDirectory((CString)cylUpperParam.resultPath.c_str(), NULL);
		cylUpperAlgo.ImageProcessing(imgSrc, cylUpperParam, cylUpperResult);
		//std::cout << "Upper csvTitle :" << (std::string)CT2CA(cylUpperResult.csvTitleStr) << std::endl;
		//std::cout << "Upper csvResult :" << (std::string)CT2CA(cylUpperResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Upper csvTitleXM :" << (std::string)CT2CA(cylUpperResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Upper csvResultXM :" << (std::string)CT2CA(cylUpperResult.csvResultStrXM4) << std::endl << std::endl;

		//cv::imshow("srcUpperImg", imgSrc);
		cv::imshow("resUpperImg9_" + fileName, cylUpperResult.resImg);
	}
	cv::waitKey(0);
}

