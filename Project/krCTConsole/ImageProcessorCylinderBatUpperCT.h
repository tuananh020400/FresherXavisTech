#pragma once
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryCT.h"
//#include "InspectCylinderBattery.h"

namespace xf
{
	namespace ImageProcessor
	{
		class CImageProcessorCylinderBatUpperCT
			: public CImageProcessorBase
		{
		public:
			CImageProcessorCylinderBatUpperCT();
			~CImageProcessorCylinderBatUpperCT();
//			virtual void ImageProcessing(cv::Mat img, xvt::CylinderCTInspection batInsp, xvt::battery::BatteryInspectionResult& bIresult);
			void ImageProcessing(const cv::Mat img, xvt::CylinderCTInspection batInsp, BatUpperResultStruct& bInspResult); // xvt::battery::BatteryInspectionResult& bIresult);

		protected:
			int Inspection(cv::Mat img, xvt::CylinderCTInspection BatInsp, xvt::battery::BatteryInspectionResult& BIresult);
			int CheckUpperBaseline(cv::Mat img, xvt::CylinderCTInspection BatInsp);
			void DrawTextResult(cv::Mat& img, xvt::battery::BatteryInspectionResult BatInsp);
			void drawPoleTextResult(const cv::Mat img, xvt::CylinderCTInspection batInsp, xvt::battery::BatteryInspectionResult BatInsp);

			cv::Mat draw_all_text(int cols, xvt::battery::BatteryInspectionResult& result);
		};
	}
}