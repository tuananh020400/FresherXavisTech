#pragma once
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryCT.h"
//#include "InspectCylinderBattery.h"

namespace xf
{
	namespace ImageProcessor
	{
		class CImageProcessorCylinderBatLowerCT :
			public CImageProcessorBase
		{
		public:
			CImageProcessorCylinderBatLowerCT();
			~CImageProcessorCylinderBatLowerCT();
			
//			virtual void ImageProcessing(cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, xvt::battery::BatteryInspectionResult &BIresult);
			void ImageProcessing(const cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, BatLowerResultStruct& bInspResult); // xvt::battery::BatteryInspectionResult& BIresult);

			xvt::CylinderCTInspection cylUpperParam;
		protected:
			int Inspection(cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BIresult);

			void DrawTextResult(cv::Mat& img, xvt::battery::BatteryInspectionResult& BatInsp);

			void drawPoleTextResult(const cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BatInsp);

			cv::Mat draw_all_text(int cols, xvt::battery::BatteryInspectionResult& result);

		};
	}
}