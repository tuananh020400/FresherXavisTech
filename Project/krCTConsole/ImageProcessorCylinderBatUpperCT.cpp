#include "ImageProcessorCylinderBatUpperCT.h"
//#include "InspectCylinderBattery.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <numeric>
#include <chrono>
#include <sys/timeb.h>
#include <xvtBattery/CTBatteryInspectorBase.h>
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryCT.h"


#pragma warning(disable : 4996) //_CRT_SECURE_NO_WARNINGS

using namespace std;
using namespace xvt;
using namespace xvt::battery;

int nUpperProcessingCount = 0;
static double gUpperPoleDiffQueue[10][MAX_POLE_NO];
extern CString gsFilePath;

CString gUpperRecentResultStr;

namespace xf
{
	namespace ImageProcessor
	{
		CImageProcessorCylinderBatUpperCT::CImageProcessorCylinderBatUpperCT()
		{
		}

		CImageProcessorCylinderBatUpperCT::~CImageProcessorCylinderBatUpperCT()
		{
		}

		void CImageProcessorCylinderBatUpperCT::ImageProcessing(const cv::Mat srcImg16bit, xvt::CylinderCTInspection batInsp, BatUpperResultStruct& bUpperResult) // xvt::battery::BatteryInspectionResult& bIresult);
		{
			xvt::battery::BatteryInspectionResult BIresult;

			{
				int t0 = clock();
				CString strTitle, cellStr = _T("");
				cellStr.Format(_T("Cell : %s"), "0");

				CString sCellID = batInsp.sCellID = cellStr;

				batInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH = false;
				batInsp.inspectingItems.ANODE_TO_CASE_GAP = false;
				batInsp.inspectingItems.ANODE_TO_CASE_VARIATION = false;
				batInsp.inspectingItems.CATHODE_TO_CASE_GAP = true;

				std::vector<cv::Point> vtPointDefect;
				DWORD dwStart = GetTickCount();

				CString  csvTitleStr = _T("");
				CString  csvTitleStrXM4 = _T("");

				csvTitleStr = "Opcode, Angle,CellID, NgCode, Final, Time, MinCath2Base, MaxCath2Base, AvgCath2Base, ";
				csvTitleStrXM4 = "NgCode, R0000337P, R0000338P, R0000339P, Opcode, Angle, CellID, Final, Time,  ";

				for (int i = 0; i < MAX_POLE_NO; i++) csvTitleStr.AppendFormat(_T("Cat2Base%d, "), i + 1);
				CString csvResultStr = _T("");
				CString csvResultStrXM4 = _T("");

				int inspectionErr = 0;

				{
					int nImageWidth = srcImg16bit.cols;
					int nImageHeight = srcImg16bit.rows;

					cv::Mat Img;
					Img = xvt::battery::AverageSliceImage(srcImg16bit, batInsp.SliceNo);

					cv::Mat resImg;
					xvt::ConvertRGB(Img, resImg);

					int ngCode = 0;

					CString timeStr = _T("");
					CTime cTime = CTime::GetCurrentTime();
					timeStr.AppendFormat(_T("%02d/%02d %02d:%02d:%02d."), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
					struct _timeb timebuffer;
					_ftime64_s(&timebuffer);
					CString strMiliSec = _T("");
					strMiliSec.Format(_T("%03d"), (int)(timebuffer.millitm));

					if (batInsp.UseBatteryInspection == NO_BAT_INSPECTION)
					{
						CString strA = timeStr + strMiliSec;
						strA.AppendFormat(_T(", %d x %d"), nImageWidth, nImageHeight);
						// cv::putText(resImg, (std::string)CT2CA(cellStr), cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
						cv::putText(resImg, "Upper - No Inspection", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, (std::string)CT2CA(strA), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 0.8);

						nUpperProcessingCount = 0;
						
						csvResultStrXM4.AppendFormat(_T("0, 0, 0, 0, ")); // NG code, MinCath2Base, MaxCath2Base, AvgCath2Base
						csvResultStrXM4.AppendFormat(_T("1, %d"), batInsp.Angle); // opcode, angle   
						csvResultStrXM4.AppendFormat(_T("%s, "), sCellID);
						csvResultStrXM4.AppendFormat(_T("OK, "));
						csvResultStrXM4.AppendFormat(_T("%s, "), timeStr + strMiliSec);

						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);

						gUpperRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
//						theApp.WriteNormalLog(_T("1st Upper no insp end"));
						
						DrawTextResult(bUpperResult.resImg, BIresult);
						drawPoleTextResult(bUpperResult.resImg, batInsp, BIresult);
						return;
					}

					BIresult.finalDecision = "NG";
					cv::Point errOutPosition = cv::Point(50, resImg.rows - 100);

					try {
						int resValue = Inspection(Img, batInsp, BIresult);
						nUpperProcessingCount++;
						if (resValue > 0) 
						{
							if(!BIresult.resImg.empty()) resImg = BIresult.resImg.clone();

							inspectionErr = resValue;
				
							cv::Rect settingROI(batInsp.RoiX, batInsp.RoiY, batInsp.RoiWidth, batInsp.RoiHeight);
							cv::rectangle(resImg, settingROI, cv::Scalar(0, 255, 255), 3);

							std::string description;
							if (resValue == errImageFormat) {description = "Error: ROI or Image format"; ngCode = NG_UpperImageFormat;}
							else if (resValue == errBatterySize) { description = "Error: ROI Selection or Input Image"; ngCode = NG_UpperEtc;}
							else if (resValue == errWidthBattery) { description = BIresult.Description; ngCode = NG_UpperCellWidth;} // "Err Battery Width";
							else if (resValue == errPixelSize) { description = "Error: Pixel Size cannot be 0"; ngCode = NG_UpperEtc;}
							else if (resValue == errBeadingArea) { description = "Error: ROI Selection"; ngCode = NG_UpperEtc;}
							else if (resValue == errBeadingInsp) { description = "Error: Beading Inspection"; ngCode = NG_UpperEtc;}
							else if (resValue == errJRFinding) { description = "Error: JR Finding "; ngCode = NG_UpperEtc;}
							else if (resValue == errReferenceCase) { description = "Error: Finding Reference Case Line"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection) { description = "Error: Pole X Detection "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection2) { description = "Error: Pole X Detection 2"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection3) { description = "Error: Pole X Detection 3"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection4) { description = "Error: Pole X Detection 4"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection5) { description = "Error: Pole X Detection 5"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine) { description = "Error: Pole Cathode Line "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine2) { description = "Error: Pole Cathode Line 2"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine3) { description = "Error: Pole Cathode Line 3"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine4) { description = "Error: Pole Cathode Line 4"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine5) { description = "Error: Pole Cathode Line 5"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine6) { description = "Error: Pole Cathode Line 6"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleAnodeLine) { description = "Error: Pole Anode Line "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleNotRefinable) { description = "Error: Pole Not Refinable "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleRefinement) { description = "Error: Pole Refinement  "; ngCode = NG_UpperEtc;	}
							else { description = "Error: " + BIresult.Description; ngCode = NG_UpperEtc;}
							cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
							cv::putText(resImg, description, errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
					
							bUpperResult.Description = BIresult.Description;

							CString timeStrA = _T("");
							CTime cTime = CTime::GetCurrentTime();
							timeStrA.AppendFormat(_T("%02d%02d_%02d%02d%02d."), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
							struct _timeb timebuffer;
							_ftime64_s(&timebuffer);
							CString strMiliSecA = _T("");
							strMiliSecA.Format(_T("%03d"), (int)(timebuffer.millitm));

							BIresult.cTime = CT2CA(timeStrA + strMiliSecA);
							BIresult.finalDecision = "NG";
							BIresult.minCathode2Anode = 0.0;
							BIresult.maxCathode2Anode = 0.0;
							BIresult.avgCathode2Anode = 0.0;
							BIresult.minAnode2Case = 0.0;
							BIresult.maxAnode2Case = 0.0;
							BIresult.avgAnode2Case = 0.0;
							BIresult.minCathode2Case = 0.0;
							BIresult.maxCathode2Case = 0.0;
							BIresult.avgCathode2Case = 0.0;

							bUpperResult.minCathode2Case = BIresult.minCathode2Case;
							bUpperResult.maxCathode2Case = BIresult.maxCathode2Case;
							bUpperResult.avgCathode2Case = BIresult.avgCathode2Case;

							//int baselineLoc = CheckUpperBaseline(Img, batInsp);

							//bUpperResult.baselineLoc = baselineLoc;

							//CString strB = timeStr; strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
							//cv::line(resImg, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

							//cv::putText(resImg, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
							//cv::putText(resImg, (std::string)CT2CA(cellStr), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
							if (batInsp.UseBatteryInspection == NORMAL_MULTI_INSPECTION)
							{
								cv::putText(resImg, "Angle: " + std::to_string(batInsp.Angle), cv::Point(nImageWidth - 200, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
							}

							gUpperRecentResultStr = "";

							Img.copyTo(bUpperResult.img8bit);
							resImg.copyTo(bUpperResult.resImg);
						}
					}
					catch (const std::exception& ex) {
						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);
						gUpperRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err1", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
//						theApp.WriteAlarmLog(_T("Upper algo : exception 1:"));
					}
					catch (const std::string& ex) {
						std::exception_ptr p = std::current_exception();

						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);
						gUpperRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err2", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
//						theApp.WriteAlarmLog(_T("Upper algo : exception 2:") + (CString)ex.c_str());
					}
					catch (...)
					{
						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);
						gUpperRecentResultStr = "";

						std::exception_ptr p = std::current_exception();
						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err3", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
//						theApp.WriteAlarmLog(_T("Upper algo : exception 3:"));
					}

					DWORD dTime = GetTickCount() - dwStart;
					if (dTime != 0)
					{
					}

					bUpperResult.finalDecision = BIresult.finalDecision;

					bUpperResult.cTime = BIresult.cTime;
					bUpperResult.minCathode2Case = BIresult.minCathode2Case;
					bUpperResult.maxCathode2Case = BIresult.maxCathode2Case;
					bUpperResult.avgCathode2Case = BIresult.avgCathode2Case;
					bUpperResult.sCathode2Case = BIresult.sCathode2Case;

					CString eFinalDecision = CString(BIresult.finalDecision.c_str());
					CString eTime          = CString(BIresult.cTime.c_str());
					std::vector<double> eCathode2BaseMeas = BIresult.sCathode2Case;
					double eCathode2BaseMin = BIresult.minCathode2Case;
					double eCathode2BaseMax = BIresult.maxCathode2Case;
					double eCathode2BaseAvg = BIresult.avgCathode2Case;

					std::vector<cv::Point> vtCathodeCPoints;
					for(const auto& cat : BIresult.vtCathodes) vtCathodeCPoints.emplace_back(cat.x, cat.y);

//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eCathodes, vtCathodeCPoints);

					std::vector<cv::Point> vtAnodeCPoints;
					for(const auto& ano : BIresult.vtAnodes) vtAnodeCPoints.emplace_back(ano.x, ano.y);
//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eAnodes, vtAnodeCPoints);


					int opCode = 0;
					int setJudge = 0;

					if (BIresult.finalDecision == "OK") opCode = 1;
					else if (BIresult.finalDecision == "NG") opCode = 2; //	opCode = BIresult.finalDecision ? 2 : 1;  // OK 1, NG 2 

					if (inspectionErr >= 100) opCode = inspectionErr;
	
					if (inspectionErr >= 100) csvResultStr.AppendFormat(_T("%d, "), inspectionErr);
					else
					{
						csvResultStr.AppendFormat(_T("%d, "), opCode);
						if      (BIresult.Anode2CathodeDecision == "NG") ngCode = NG_UpperAno2CathDistance;
						else if (BIresult.Cathode2CaseDecision   == "NG") ngCode = NG_UpperCat2CaseDistance;
						else if (BIresult.Anode2CaseVariationDecision   == "NG") ngCode = NG_UpperAno2CaseVariation;
					}

					csvResultStr.AppendFormat(_T("%d, "), batInsp.Angle);
					csvResultStr.AppendFormat(_T("%s, "), sCellID);
					if (BIresult.finalDecision == "OK") csvResultStr.AppendFormat(_T("%d, "), 0);
					else csvResultStr.AppendFormat(_T("%d, "), ngCode);

					csvResultStr.AppendFormat(_T("%s, "), (CString)BIresult.finalDecision.c_str());
					csvResultStr.AppendFormat(_T("%s, "), (CString)BIresult.cTime.c_str());

					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);

					//-----------------------------------------------------------------------------------------
					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 0);
					else csvResultStrXM4.AppendFormat(_T("%d, "), ngCode);

					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);
					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 1);  // opcode
					else csvResultStrXM4.AppendFormat(_T("%d, "), inspectionErr);

					csvResultStrXM4.AppendFormat(_T("%d, "), batInsp.Angle);
					csvResultStrXM4.AppendFormat(_T("%s, "), sCellID);

					csvResultStrXM4.AppendFormat(_T("%s, "), (CString)BIresult.finalDecision.c_str());
					csvResultStrXM4.AppendFormat(_T("%s, "), (CString)BIresult.cTime.c_str());
					// ------------------------------------------------------------------------------------------

					if (BIresult.sCathode2Case.size() > MAX_POLE_NO)
					{
						for (int i = 0; i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Case[i]);
						}
					}
					else
					{
						for (int i = 0; i < BIresult.sCathode2Case.size(); i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Case[i]);
						}
						for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T(" -1, "));
						}
					}

//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eCSVTitle, csvTitleStr);
//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eCSVResult, csvResultStr);
			
					bUpperResult.opCode = opCode;
					bUpperResult.ngCode = ngCode;
					bUpperResult.setJudge = setJudge;
					gUpperRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
					//if (inspectionErr == 0)
					{
						// cv::Mat text_img = draw_all_text(resImg.cols, BIresult);
						// cv::vconcat(BIresult.resImg, text_img, BIresult.resImg);
						bUpperResult.resImg = BIresult.resImg;
					}					
					//DrawTextResult(bUpperResult.resImg, BIresult);
					//drawPoleTextResult(bUpperResult.resImg, batInsp, BIresult);

					return;
				}
//				std::cout << " Algo time :" << clock() - t0 << "ms" << std::endl;
			}
			// end of pGI
		}

		int CImageProcessorCylinderBatUpperCT::Inspection(cv::Mat img16bit, xvt::CylinderCTInspection BatInsp, xvt::battery::BatteryInspectionResult& BIresult)
		{
			BIresult.sHeader = "";
			BIresult.sData = "";
			BIresult.iResult = 0;
			BIresult.Description = "";
			BIresult.finalDecision = "";

			CString FinalDecision;

			int nImageWidth = img16bit.cols;
			int nImageHeight = img16bit.rows;

			cv::Mat Img;
			img16bit.convertTo(Img, CV_16UC1);
			Img = xvt::battery::AverageSliceImage(Img, BatInsp.SliceNo);
			if (Img.empty()) return -1;

			nImageHeight = Img.rows;

			// check predefined setting 
			cv::Rect settingOuterROI = cv::Rect(BatInsp.RoiX, BatInsp.RoiY, BatInsp.RoiWidth, BatInsp.RoiHeight);
			settingOuterROI = RoiRefinement(settingOuterROI, cv::Size(nImageWidth, nImageHeight));
			BatInsp.RoiX = settingOuterROI.x;
			BatInsp.RoiY = settingOuterROI.y;
			BatInsp.RoiWidth = settingOuterROI.width;
			BatInsp.RoiHeight = settingOuterROI.height;

			if (BatInsp.RoiX >= nImageWidth || BatInsp.RoiY >= nImageHeight || BatInsp.RoiWidth <= RESIZE_SCALE || BatInsp.RoiHeight <= RESIZE_SCALE) {
				BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
				return errBatterySize;
			}

			//--- pre-preocessing --------------------------------------------------------------------------
			xvt::Convert8Bits(Img, Img, false);


//			cv::Mat Img, Img1;
//			if (BatInsp.additionalLine == 0)   // no pre-processing, bmp image ...
//			{
//				Img1 = cv::Mat_<int>(nImageHeight, nImageWidth, pImage);
//				Img1.convertTo(Img, CV_8UC1, 1.0 / 256.0);
//				Img1.release();
//			}
//			else if ((BatInsp.additionalLine == 1)) // && (fabs(BatInsp.depthScale) > 0)) // no pre-processing, bmp image ...
//			{
//				Img1 = cv::Mat_<int>(nImageHeight, nImageWidth, pImage);
//				Img1.convertTo(Img, CV_8UC1, 1.0 / BatInsp.depthScale);
//				Img1.release();
//			}
//			else if (BatInsp.additionalLine == 2) // no pre-processing, pre-processed image ...
//			{
//				Img1 = cv::Mat_<int>(nImageHeight, nImageWidth, pImage);
//				Img1.convertTo(Img, CV_8UC1, 1.0 / 256.0);
////				GammaCorrection(Img, BatInsp.gamma);
//				Img1.release();
//			}
//			else 
//			{
//				cv::Rect outputROI;
//				cv::Rect Roi = cv::Rect(BatInsp.RoiX, BatInsp.RoiY, BatInsp.RoiWidth, BatInsp.RoiHeight);
//				outputROI = FindRoughBatteryROI(pImage, nImageHeight, nImageWidth, Roi, BatInsp.depthScale, false);  // TIFF read true, display false
//
//				if (outputROI.x == 0 || outputROI.y == 0 || outputROI.width == 0)  // error
//				{
//					return xf::errImageFormat;
//				}
//				cv::Mat gammaImg8bit = 255 + cv::Mat::zeros(nImageHeight, nImageWidth, CV_8UC1);
//				int gamma = BatInsp.gamma;
//				GainCorrection(pImage, gammaImg8bit, gamma, nImageHeight, nImageWidth, outputROI, 10, 1, BatInsp.additionalLine);
//				Img = gammaImg8bit;
//			}

			//int baselineLoc = CheckUpperBaseline(Img, BatInsp);

			//-----end of pre-processing ------------------------------------------------------------------------
			xvt::battery::CTBatteryInspector ctIsp;
			auto& baseInspector = ctIsp.mIspBase;
			auto& beadingInspector = ctIsp.mIspBeading;
			auto& jrInspector = ctIsp.mIspJR;
			auto& poleInspector = ctIsp.mIspPole;
			auto& cathodeInspector = ctIsp.mIspCathode;
			auto& anodeInspector = ctIsp.mIspAnode;

			ctIsp.mEnable = true;

			ctIsp.mInspectingItems.ANODE_TO_CASE_GAP = BatInsp.inspectingItems.ANODE_TO_CASE_GAP;
			ctIsp.mInspectingItems.ANODE_TO_CASE_VARIATION = BatInsp.inspectingItems.ANODE_TO_CASE_VARIATION;
			ctIsp.mInspectingItems.ANODE_TO_CATHODE_LENGTH = BatInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH;
			ctIsp.mInspectingItems.CATHODE_TO_CASE_GAP = BatInsp.inspectingItems.CATHODE_TO_CASE_GAP;

			baseInspector.mRoi = {BatInsp.RoiX, BatInsp.RoiY, BatInsp.RoiWidth, BatInsp.RoiHeight};
			baseInspector.mThreshold = BatInsp.batteryThreshold;
			baseInspector.mDirection = BatInsp.BatteryDirection;
			baseInspector.mValidBatteryWidthRange = xvt::Ranged(BatInsp.cellSizeLowBound, BatInsp.cellSizeHighBound, BatInsp.EnableValidWidth);

			beadingInspector.mEnable = BatInsp.EnableBeading;

			jrInspector.mHeight = BatInsp.poleRegionHeight;
			jrInspector.mLeaningThreshold = BatInsp.PoleLeaningThreshold;
			jrInspector.mJROffsetX = BatInsp.JR_ROIX;
			jrInspector.mJROffsetY = BatInsp.JR_ROIY;
			jrInspector.mCenterNeglectionWidth = BatInsp.CenterNeglectionWidth;

			jrInspector.mBaseLineOffset = BatInsp.tolerenceShift;

			cathodeInspector.mCathodeLineThresholdInner = BatInsp.cathodeLineThredsholdInner;
			cathodeInspector.mCathodeLineThresholdMiddle = BatInsp.cathodeLineThredsholdMiddle;
			cathodeInspector.mCathodeLineThresholdOuter = BatInsp.cathodeLineThredsholdOuter;
			cathodeInspector.mProminence = BatInsp.cathodeLineProminence;
			cathodeInspector.mCathodeLineWindowSize = BatInsp.cathodeLineWindowSize;

			poleInspector.mPoleHeight = BatInsp.polesHeight;
			poleInspector.mPolesProminenceThreshold = BatInsp.minProminence;
			poleInspector.mPolesDistanceRange.Set(BatInsp.polesMinDistance, BatInsp.polesMaxDistance);

			anodeInspector.mAnodeThresholdInner = BatInsp.anodeThresholdInner;
			anodeInspector.mAnodeThresholdMiddle = BatInsp.anodeThresholdMiddle;
			anodeInspector.mAnodeThresholdOuter = BatInsp.anodeThresholdOuter;
			anodeInspector.mAnodeEnhanceScale = BatInsp.anodeEnhanceScale;

			poleInspector.mEnableValidCathode2Anode = BatInsp.inspectingItems.ANODE_TO_CASE_VARIATION;
			poleInspector.mValidCathode2AnodeRange=xvt::Ranged(BatInsp.MinCathode2Anode, BatInsp.MaxCathode2Anode,BatInsp.EnableCathode2Anode);
			poleInspector.mValidAnode2CaseRange = xvt::Ranged(BatInsp.MinAnode2Case, BatInsp.MaxAnode2Case, BatInsp.EnableAnode2Case);
			poleInspector.mValidCathode2CaseRange = xvt::Ranged(BatInsp.MinCathode2Case, BatInsp.MaxCathode2Case, BatInsp.EnableCathode2Case);

			poleInspector.mCathode2AnodeOffset = BatInsp.mCathode2AnodeOffset;
			poleInspector.mAnode2CaseOffset = BatInsp.mAnode2CaseOffset;
			poleInspector.mCathode2CaseOffset = BatInsp.mCathode2CaseOffset;
			poleInspector.mVariationAnode2Case = BatInsp.variationAnode2Case;
			jrInspector.mMinLeaningDistance = BatInsp.minLeaningDistance;

			ctIsp.mPixelSize = BatInsp.pixelSize;
			ctIsp.mDisplayMode = static_cast<xvt::battery::DisplayMode>(BatInsp.displayMode);
			ctIsp.mDisplayPen.mFontScale = 0.4;
			ctIsp.mDisplayPen.mSpace = 15;

			xvt::battery::CTBatteryInspectorResult ctResult;
			xvt::battery::ERR_CODE err = ctIsp.Inspect(Img, ctResult);
			BIresult = ctResult;
			BIresult.resImg = Img.clone();
			GammaCorrection(BIresult.resImg, BatInsp.gamma);
			xvt::ConvertRGB(BIresult.resImg, BIresult.resImg);
			ctResult.DrawResult(BIresult.resImg);

			if(err != xvt::battery::ERR_CODE::OK)
			{
				Img.release();
				//ctIsp.mIspResult.DrawResult(BIresult.resImg);
				return static_cast<int>(err);
			}

			cv::Mat res = BIresult.resImg;
			//cv::line(res, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

			if ((BatInsp.isSaveResults == 1) || (BatInsp.UseBatteryInspection == DATA_WRITE_BAT_INSPECTION))
			{
				std::string fileNameStr = "Result";
				std::string tStr = BIresult.cTime;

				std::string filepath;
				TCHAR szTmp[500];
				StrCpy(szTmp, gsFilePath); // pGI->GetImagePath()는 제대로 못받아옴, gsFilePath에는 path + file이 넘어옴
				PathRemoveFileSpec(szTmp);
				filepath = (std::string)CT2CA(szTmp);
				std::string outFileName = (std::string)CT2CA(gsFilePath);

				std::string fileName;
				if (filepath.size() == 0) fileName = "D:\\Result\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Upper_Data_1.csv";
				else                      fileName = filepath + "\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Upper_Data_1.csv";

				if (nUpperProcessingCount == 0)
				{
					for (int i = 0; i < 10; i++) for (int j = 0; j < MAX_POLE_NO; j++) gUpperPoleDiffQueue[i][j] = -1.0;

					std::string wStr = "";
					wStr += "No,Final, time, filename, Angle, OutRoiX, OutRoiY, OutRoiW, OutRoiH, BatThresh, BeadingH, D1_Start, PoleRegion, PoleLeaning, JRRoiX, JRRoiY,";
					wStr += "NeglectCenterArea, TolShift, CatInner, CatMiddle, CatOuter, CatWinSize, PoleHeight, PoleDetecThresh, MinDistance, ";
					wStr += "MaxDistance, AnodeInner, AnodeMiddle, AnodeOuter, No.oneSidePole, isCheckPoleNo, SkipPoleDist, PixSize,delthScale, additionalLine, SVminAn2CatMM, SVmaxAn2CatMM, SVminAn2BaseMM, SVmaxAn2BaseMM, SVminCat2BaseMM, SVmaxCat2BaseMM, ";
//					wStr += "MinAno2Cath, MaxAno2Cath, AvgAno2Cath, ,MinAno2Base, MaxAno2Base, AvgAno2Base,";
					wStr += ", PMinCath2Base, PMaxCath2Base, PAvgCath2Base, ,";
//					wStr += ", Cat2An, ";
//					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
					wStr += "Cat2Base,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
//					wStr += "An2Case,";
//					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';

					std::ofstream fout;
					fout.open(fileName, std::ios_base::out | std::ios_base::app);
					fout << wStr << std::endl;
					fout.close();
				}

//				double poleDiffLength[MAX_POLE_NO];
//				if (BIresult.sCathode2Anode.size() > MAX_POLE_NO) {
//					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength[i] = BIresult.sCathode2Anode[i];
//				}
//				else
//				{
//					for (int i = 0; i < BIresult.sCathode2Anode.size(); i++) poleDiffLength[i] = BIresult.sCathode2Anode[i];
//					for (int i = BIresult.sCathode2Anode.size(); i < MAX_POLE_NO; i++) poleDiffLength[i] = -1;
//				}

				double poleDiffLength2[MAX_POLE_NO];

				if (BIresult.sCathode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Case.size(); i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
					for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength2[i] = -1;
				}

//				double poleDiffLength3[MAX_POLE_NO];
//				if (BIresult.sAnode2Case.size() > MAX_POLE_NO) {
//					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
//				}
//				else
//				{
//					for (int i = 0; i < BIresult.sAnode2Case.size(); i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
//					for (int i = BIresult.sAnode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength3[i] = -1;
//				}

				std::string wStr = std::to_string(nUpperProcessingCount + 1) + ", ";
//				if (pGI->IsLeft()) wStr += std::to_string(0) + ",";
//				else wStr += std::to_string(90) + ",";
				wStr += BIresult.finalDecision + ",";
//				wStr += BIresult.Anode2CathodeDecision + ",";
//				wStr += BIresult.Anodes2CaseDecision + ",";
				wStr += BIresult.cTime + ",";

				wStr += outFileName + ",";
				wStr += std::to_string(BatInsp.Angle) + ",";

				wStr += std::to_string(BatInsp.RoiX) + ",";
				wStr += std::to_string(BatInsp.RoiY) + ",";
				wStr += std::to_string(BatInsp.RoiWidth) + ",";
				wStr += std::to_string(BatInsp.RoiHeight) + ",";
				wStr += std::to_string(BatInsp.batteryThreshold) + ",";
				wStr += std::to_string(BatInsp.minBeadingHeight) + ",";
				wStr += std::to_string(BatInsp.D1StartPosition) + ",";
				wStr += std::to_string(BatInsp.poleRegionHeight) + ",";
				wStr += std::to_string(BatInsp.PoleLeaningThreshold) + ",";
				wStr += std::to_string(BatInsp.JR_ROIX) + ",";
				wStr += std::to_string(BatInsp.JR_ROIY) + ",";
				wStr += std::to_string(BatInsp.CenterNeglectionWidth) + ",";
				wStr += std::to_string(BatInsp.tolerenceShift) + ",";
				wStr += std::to_string(BatInsp.cathodeLineThredsholdInner) + ",";
				wStr += std::to_string(BatInsp.cathodeLineThredsholdMiddle) + ",";
				wStr += std::to_string(BatInsp.cathodeLineThredsholdOuter) + ",";
				wStr += std::to_string(BatInsp.cathodeLineWindowSize) + ",";
				wStr += std::to_string(BatInsp.polesHeight) + ",";
				wStr += std::to_string(BatInsp.minProminence) + ",";
				wStr += std::to_string(BatInsp.polesMinDistance) + ",";
				wStr += std::to_string(BatInsp.polesMaxDistance) + ",";

				wStr += std::to_string(BatInsp.anodeThresholdInner) + ",";
				wStr += std::to_string(BatInsp.anodeThresholdMiddle) + ",";
				wStr += std::to_string(BatInsp.anodeThresholdOuter) + ",";
				wStr += std::to_string(BatInsp.OneSidePoleNumb) + ",";
				wStr += std::to_string(BatInsp.skipPolesDistance) + ",";
				wStr += std::to_string(BatInsp.pixelSize) + ",";
				wStr += std::to_string(BatInsp.depthScale) + ",";
				wStr += std::to_string(BatInsp.additionalLine) + ",";
				wStr += std::to_string(BatInsp.MinCathode2Anode) + ",";
				wStr += std::to_string(BatInsp.MaxCathode2Anode) + ",";
				wStr += std::to_string(BatInsp.MinAnode2Case) + ",";
				wStr += std::to_string(BatInsp.MaxAnode2Case) + ",";
				wStr += std::to_string(BatInsp.MinCathode2Case) + ",";
				wStr += std::to_string(BatInsp.MaxCathode2Case) + ", ,";

//				wStr += std::to_string(BIresult.minCathode2Anode) + ",";
//				wStr += std::to_string(BIresult.maxCathode2Anode) + ",";
//				wStr += std::to_string(BIresult.avgCathode2Anode) + ", ,";
//				wStr += std::to_string(BIresult.minAnode2Case) + ",";
//				wStr += std::to_string(BIresult.maxAnode2Case) + ",";
//				wStr += std::to_string(BIresult.avgAnode2Case) + ", ,";
				wStr += std::to_string(BIresult.minCathode2Case) + ",";
				wStr += std::to_string(BIresult.maxCathode2Case) + ",";
				wStr += std::to_string(BIresult.avgCathode2Case) + ", , ,";

				//wStr += std::to_string(BIresult.corverHeight) + ", ";
				//wStr += std::to_string(BIresult.corverDiameter) + ",";
				//wStr += std::to_string(BIresult.outerDiameter) + ",";
				//wStr += std::to_string(BIresult.grooveDepth) + ",";
				//wStr += std::to_string(BIresult.grooveHeight) + ", ,";

				if (BIresult.sCathode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Case.size(); i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
					for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength2[i] = -1;
				}

				for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(poleDiffLength2[i]) + ",";

//				CImageProcessorBase::WrProcessingData(wStr, nUpperProcessingCount, 2*BatInsp.OneSidePoleNumb, gUpperPoleDiffQueue, poleDiffLength2, poleDiffLength, poleDiffLength3, 45);

				std::ofstream fout;

				fout.open(fileName, std::ios_base::out | std::ios_base::app);

				fout << wStr << std::endl;
				fout.close();

//				cv::imwrite(filepath + "\\" + fileNameStr + "_" + tStr + "jpg", res);
				//cv::imwrite(outFileName + "_" + tStr + "jpg", res);
			}

			//change image from cv::Mat to *int 

			CString strB = _T("");
			CTime cTime = CTime::GetCurrentTime();
			strB.AppendFormat(_T("%02d/%02d %02d:%02d:%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
			//strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
			cv::putText(res, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2.0);

			if (BatInsp.UseBatteryInspection == NORMAL_MULTI_INSPECTION)
			{
				cv::putText(res, "Angle: " + std::to_string(BatInsp.Angle), cv::Point(nImageWidth - 200, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
			}

			CString strTitle, cellStr = _T("");
			cellStr.Format(_T("Cell : %s"), "0");
			//cv::putText(res, (std::string)CT2CA(cellStr), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

			return 0;
		}

		int CImageProcessorCylinderBatUpperCT::CheckUpperBaseline(cv::Mat img16bit, xvt::CylinderCTInspection BatInsp)
		{
			{
				{
					int nImageWidth = img16bit.cols;
					int nImageHeight = img16bit.rows;

					cv::Mat Img;
					img16bit.convertTo(Img, CV_8UC1, 1.0 / 256.0);

					cv::Rect roi  = cv::Rect(BatInsp.RoiX, BatInsp.RoiY, BatInsp.RoiWidth/2, BatInsp.RoiHeight);
					cv::Rect roi2 = cv::Rect(BatInsp.RoiX + BatInsp.RoiWidth / 2, BatInsp.RoiY, BatInsp.RoiWidth/2, BatInsp.RoiHeight);

					roi  = CImageProcessorBase::RoiRefinement(roi, Img.size());
					roi2 = CImageProcessorBase::RoiRefinement(roi2, Img.size());

					cv::Mat grayImg, grayImg2;
					cv::Mat dst  = cv::Mat::zeros(Img.size(), CV_8U);
					cv::Mat dst2 = cv::Mat::zeros(Img.size(), CV_8U);

					Img(roi).copyTo(dst(roi));
					Img(roi2).copyTo(dst2(roi2));
					cv::medianBlur(dst(roi), dst(roi), 15);
					cv::medianBlur(dst2(roi2), dst2(roi2), 15);

					double otsuThresh  = cv::threshold(dst, grayImg, 0, 255, cv::THRESH_OTSU);
					double otsuThresh2 = cv::threshold(dst2, grayImg2, 0, 255, cv::THRESH_OTSU);

					vector<int> vtLeftX, vtRightX, vtLeftY, vtRightY;

					for (int y = BatInsp.RoiY; y < Img.rows/2; y++)
					{
						for (int x = BatInsp.RoiX + 1; x < Img.cols/2; x++)
						{
							if ((grayImg.at<BYTE>(y, x - 1) == 0) && (grayImg.at<BYTE>(y, x) == 255))
							{
								vtLeftX.push_back(x);
								vtLeftY.push_back(y);
								break;
							}
						}
						for (int x = BatInsp.RoiX + BatInsp.RoiWidth - 2; x >= Img.cols /2 + 1; x--)
						{
							if ((grayImg2.at<BYTE>(y, x + 1) == 0) && (grayImg2.at<BYTE>(y, x) == 255))
							{
								vtRightX.push_back(x);
								vtRightY.push_back(y); 
								break;
							}
						}
					}
					//if (vtLeftX.size() == 0 || vtLeftY.size() == 0) {
					//	std::string msg = std::format("CheckUpperBaseline, vtLeftX.size()==0 or vtLeftY.size()==0 ");
					//	throw std::runtime_error(msg);
					//}

					auto leftX      = std::max_element(vtLeftX.begin(), vtLeftX.end());
					auto leftXindex = std::max_element(vtLeftX.begin(), vtLeftX.end()) - vtLeftX.begin();
					auto rightX     = std::min_element(vtRightX.begin(), vtRightX.end());
					auto rightXindex = std::min_element(vtRightX.begin(), vtRightX.end()) - vtRightX.begin();

					if ((vtLeftX.size() < 10) || (vtRightX.size() < 10))
					{
						std::cout << "Cell baseline finding error0 ..." << std::endl;
						return (300);
					}
					int locYLeft  = vtLeftY[leftXindex];
					int locYRight = vtRightY[rightXindex];

					if (abs(locYLeft - locYRight) > 3) std::cout << "Cell baseline finding error ..." << locYLeft << ", " << locYRight << std::endl;
					return((locYLeft + locYRight)/2);
				}
			}
		}
		
		void CImageProcessorCylinderBatUpperCT::DrawTextResult(cv::Mat& img, xvt::battery::BatteryInspectionResult BatInsp)
		{

			if (BatInsp.finalDecision == "OK") {
				cv::putText(BatInsp.resImg, "Result: OK", cv::Point(BatInsp.resImg.cols / 3.1, 85 ), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);
				
				auto matTest = BatInsp.resImg;
				double textSize = 0.50;
				int rowHeight = 30;
				int thickness = 1;

				std::vector<std::string> rangeRow;
				std::stringstream ss;
				ss << "Base->Cathode [" << std::fixed << std::setprecision(3) << 0.2 << " ~ " << std::fixed << std::setprecision(3) << 0.8 << "]"; //change min/max base to cathode
				rangeRow.push_back(ss.str()); ss.str("");

				std::vector<std::string> Row1 = { "Min", "Max", "Average" };
				std::vector<std::string> Row2;
				ss << std::fixed << std::setprecision(3) << BatInsp.minCathode2Case; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(3) << BatInsp.maxCathode2Case; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(3) << BatInsp.avgCathode2Case; Row2.push_back(ss.str()); ss.str("");
				int cellWidth = 270;
				int startX = (matTest.cols - rangeRow.size() * cellWidth) / 2;
				int startY = matTest.rows - 200;

				// Draw rows range
				int startRowFirstX = startX;
				int startRowFirstY = startY;
				for (size_t i = 0; i < rangeRow.size(); ++i) {
					int baseline = 0;
					cv::Size textSizeObj = cv::getTextSize(rangeRow[i], cv::FONT_HERSHEY_SIMPLEX, textSize, thickness, &baseline);
					int textX = startRowFirstX + (cellWidth - textSizeObj.width) / 2;
					int textY = startRowFirstY + 20;

					cv::Scalar textColor = cv::Scalar(0, 255, 0);
					putText(matTest, rangeRow[i], cv::Point(textX, textY), cv::FONT_HERSHEY_SIMPLEX, textSize, textColor, thickness);

					rectangle(matTest, cv::Point(startRowFirstX, startRowFirstY), cv::Point(startRowFirstX + cellWidth, startRowFirstY + rowHeight), cv::Scalar(0, 255, 0), thickness);
					startRowFirstX += cellWidth;

				}
				// Draw Row1
				cellWidth = 90;
				startRowFirstX = startX;
				startRowFirstY += rowHeight;
				for (size_t i = 0; i < Row1.size(); ++i) {
					int baseline = 0;
					cv::Size textSizeObj = cv::getTextSize(Row1[i], cv::FONT_HERSHEY_SIMPLEX, textSize, thickness, &baseline);
					int textX = startRowFirstX + (cellWidth - textSizeObj.width) / 2;
					int textY = startRowFirstY + 20;

					cv::Scalar textColor = cv::Scalar(0, 255, 0);
					putText(matTest, Row1[i], cv::Point(textX, textY), cv::FONT_HERSHEY_SIMPLEX, textSize, textColor, thickness);

					rectangle(matTest, cv::Point(startRowFirstX, startRowFirstY), cv::Point(startRowFirstX + cellWidth, startRowFirstY + rowHeight), cv::Scalar(0, 255, 0), thickness);
					startRowFirstX += cellWidth;
				}

				// Draw Row2
				startRowFirstX = startX;
				startRowFirstY += rowHeight;
				for (size_t i = 0; i < Row2.size(); ++i) {
					int baseline = 0;
					cv::Size textSizeObj = cv::getTextSize(Row2[i], cv::FONT_HERSHEY_SIMPLEX, textSize, thickness, &baseline);
					int textX = startRowFirstX + (cellWidth - textSizeObj.width) / 2;
					int textY = startRowFirstY + 20;
					cv::Scalar textColor = cv::Scalar(2, 165, 249);
					putText(matTest, Row2[i], cv::Point(textX, textY), cv::FONT_HERSHEY_SIMPLEX, textSize, textColor, thickness);
					rectangle(matTest, cv::Point(startRowFirstX, startRowFirstY), cv::Point(startRowFirstX + cellWidth, startRowFirstY + rowHeight), cv::Scalar(0, 255, 0), thickness);
					startRowFirstX += cellWidth;
				}
				img = matTest;
			}
		}

		void CImageProcessorCylinderBatUpperCT::drawPoleTextResult(const cv::Mat img, xvt::CylinderCTInspection batInsp, xvt::battery::BatteryInspectionResult BatInsp)
		{
			int nLeftPole = 50;
			cv::Mat resImg = BatInsp.resImg;
			const xvt::VecDouble & listAnode2CathodeDistance = BatInsp.sCathode2Anode;
			const xvt::VecDouble & listAnode2CaseDistance = BatInsp.sAnode2Case;
			const xvt::VecDouble & listCathode2CaseDistance = BatInsp.sCathode2Case;
			const xvt::VecDouble & listPolePos = BatInsp.sXPos;
			const std::vector<bool>& listAno2CathodDecision = BatInsp.vtAno2CathodDecision;
			const std::vector<bool>& listAno2CaseDecision = BatInsp.vtAno2CaseDecision;
			const std::vector<bool>& listCathode2CaseDecision = BatInsp.vtCathode2CaseDecision;
			//int nLeftPole = bInspResult.nLeftPole;
			double cathode2AnodeOffset = batInsp.mCathode2AnodeOffset;
			double anode2CaseOffset = batInsp.mAnode2CaseOffset;
			double cathode2CaseOffset = batInsp.mCathode2CaseOffset;
			bool isCheckPoleNo = batInsp.OneSidePoleNumb;
			int OneSidePoleNumb = batInsp.OneSidePoleNumb;
			float fontScale = 0.5; // change if needed
			int textLineSpace = 30; // change if needed
			cv::Point position = cv::Point(0, 150);

			bool result = true;
			const int detectedPoleNumb = listAnode2CathodeDistance.size();
			//constexpr float fontScale = 0.7;
			constexpr int fontWeight = 2;
			// constexpr int textLineSpace = 25;
			constexpr int floatingPrecesion = 2;
			constexpr int lineThickness = 3;
			const int txtSpace = 2;
			constexpr int txtFloatWidth = 5;
			constexpr int txtNumWidth = 5;
			constexpr int txtPosWidth = txtFloatWidth + txtSpace;
			constexpr int txtAnode2CathodeWidth = txtFloatWidth + txtSpace;
			constexpr int txtAnode2CathodeLimitWidth = 4 * 2 + 3 + txtSpace;
			constexpr int txtAnode2CaseWidth = txtFloatWidth + txtSpace + 1;
			constexpr int txtAnode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;
			constexpr int txtCathode2CaseWidth = txtFloatWidth + txtSpace + 1;
			constexpr int txtCathode2CaseLimitWidth = 4 * 2 + 3 + txtSpace;

			std::ostringstream anode2CathodeLimitStr;
			anode2CathodeLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << batInsp.MinCathode2Anode << "~"
				<< batInsp.MaxCathode2Anode << "]";

			std::ostringstream anode2CaseLimitStr;
			anode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << batInsp.MinAnode2Case << "~"
				<< batInsp.MaxAnode2Case << "]";

			std::ostringstream anode2CaseOffsetStr;
			anode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << anode2CaseOffset;

			std::ostringstream cathode2anodeOffsetStr;
			cathode2anodeOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2AnodeOffset;

			std::ostringstream cathode2CaseLimitStr;
			cathode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << batInsp.MinCathode2Case << "~"
				<< batInsp.MaxCathode2Case << "]";

			std::ostringstream cathode2CaseOffsetStr;
			cathode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2CaseOffset;

			cv::String poleNumStr;

			int paddingX = 40;
			//int putTxtOffsetY = (resImg.rows <= 1800) ? resImg.rows * 0.64 : resImg.rows * 0.68;

			//cv::Scalar inforColor(180, 10, 10);
			cv::Scalar inforColor(245, 164, 66);
			cv::Scalar OKColor(60, 193, 5);
			cv::Scalar NGColor(0, 0, 255);
			cv::Scalar anode2CathodeColor(inforColor);
			cv::Scalar anode2CaseColor(inforColor);
			cv::Scalar cathode2CaseColor(inforColor);
			cv::Scalar poleColor(inforColor);
			cv::Scalar cathodeColor(180, 10, 10);
			cv::Point startPos;

			std::ostringstream tmpStream;
			std::vector<std::pair<std::string, cv::Scalar>> resultHeader(8, std::pair<std::string, cv::Scalar>("", inforColor));
			tmpStream << std::left << std::setw(txtNumWidth) << std::setfill(' ') << "| No";
			resultHeader[0].first = tmpStream.str();
			tmpStream.str("");
			tmpStream << std::left << std::setw(txtPosWidth) << std::setfill(' ') << " | Pos";
			resultHeader[1].first = tmpStream.str();
			tmpStream.str("");
			std::vector<std::pair<std::string, cv::Scalar>> resultInfo(8, std::pair<std::string, cv::Scalar>("", inforColor));
			if (1)
			{
				tmpStream << std::left << std::setw(txtAnode2CathodeWidth + txtAnode2CathodeLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
					<< " | Length[Range]" << cathode2anodeOffsetStr.str();
				resultInfo[3].first = anode2CathodeLimitStr.str();
			}
			resultHeader[2].first = tmpStream.str();
			tmpStream.str("");

			if (1)
			{
				tmpStream << std::left << std::setw(txtAnode2CaseWidth + txtAnode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
					<< " | A2Case[Range]" << anode2CaseOffsetStr.str();
				resultInfo[5].first = anode2CaseLimitStr.str();
			}
			resultHeader[4].first = tmpStream.str();
			tmpStream.str("");
			if (1)
			{
				tmpStream << std::left << std::setw(txtCathode2CaseWidth + txtCathode2CaseLimitWidth - 1 - 2 - floatingPrecesion) << std::setfill(' ')
					<< " | Cat2Case[Range]" << cathode2CaseOffsetStr.str();
				resultInfo[7].first = cathode2CaseLimitStr.str();
			}
			resultHeader[6].first = tmpStream.str();

			tmpStream.str("");
			resultHeader[7].first = " |";

			auto msg = resultHeader[0].first + resultHeader[1].first + resultHeader[2].first + resultHeader[3].first + resultHeader[4].first +
				resultHeader[5].first + resultHeader[6].first + resultHeader[7].first;
			cv::Size txtSize = cv::getTextSize(msg, cv::FONT_HERSHEY_SIMPLEX, fontScale, fontWeight, 0);

			assert(listAnode2CathodeDistance.size() == listAnode2CaseDistance.size());

			for (int poleIdx = 0; poleIdx < detectedPoleNumb; poleIdx++)
			{
				bool isAnode2CathodeError = !listAno2CathodDecision[poleIdx];
				bool isAnode2CaseError = !listAno2CaseDecision[poleIdx];
				bool isCathode2CaseError = !listCathode2CaseDecision[poleIdx];
				//result &= isAnode2CathodeError && isAnode2CaseError && isCathode2CaseError;

				anode2CathodeColor = isAnode2CathodeError ? NGColor : OKColor;
				anode2CaseColor = isAnode2CaseError ? NGColor : OKColor;
				cathode2CaseColor = isCathode2CaseError ? NGColor : OKColor;
				poleColor = (isAnode2CathodeError || isAnode2CaseError || isCathode2CaseError) ? NGColor : OKColor;

				//Assign the pole number
				std::ostringstream poleNumber;
				poleNumber << std::fixed << std::right << std::setw(txtNumWidth - 3) << std::setfill(' ') << "P" << std::setw(2) << std::setfill('0')
					<< (poleIdx + 1);
				resultInfo[0].first = poleNumber.str();
				resultInfo[0].second = poleColor;

				//Assign the X pole position
				std::ostringstream strAnode2Boundery;
				strAnode2Boundery << std::right << std::setw(txtPosWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
					<< listPolePos[poleIdx];
				resultInfo[1].first = strAnode2Boundery.str();

				if (1)
				{
					std::ostringstream strCath2Ano;
					strCath2Ano << std::setw(txtAnode2CathodeWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
						<< listAnode2CathodeDistance[poleIdx];
					//Assign the anode to cathode infor
					resultInfo[2].first = strCath2Ano.str();
					resultInfo[2].second = anode2CathodeColor;
				}

				if (1)
				{
					std::ostringstream strAno2Case;
					strAno2Case << std::setw(txtAnode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
						<< listAnode2CaseDistance[poleIdx];
					//Assign the anode to case infor
					resultInfo[4].first = strAno2Case.str();
					resultInfo[4].second = anode2CaseColor;
				}

				if (1)
				{
					std::ostringstream strCat2Case;
					strCat2Case << std::setw(txtCathode2CaseWidth) << std::setfill(' ') << std::fixed << std::setprecision(floatingPrecesion)
						<< listCathode2CaseDistance[poleIdx];
					//Assign the anode to case infor
					resultInfo[6].first = strCat2Case.str();
					resultInfo[6].second = cathode2CaseColor;
				}

				//Cathode 2 Anode Measurement
				if (poleIdx < nLeftPole)
				{
					startPos.x = position.x + paddingX;
					startPos.y = position.y + (poleIdx % nLeftPole) * textLineSpace;
				}
				else
				{
					startPos.x = resImg.cols - txtSize.width - paddingX;
					startPos.y = position.y + (poleIdx - nLeftPole) * textLineSpace;
				}

				bool isFirstLeft = poleIdx == 0;
				bool isFirstRight = (poleIdx - nLeftPole) == 0;
				if (isFirstLeft || isFirstRight)
				{
					drawText(resImg, cv::Point(startPos.x, startPos.y - textLineSpace), resultHeader, fontScale, fontWeight);
				}

				drawText(resImg, startPos, resultInfo, fontScale, fontWeight);
			}

			///Plot missing pole to results
			if (isCheckPoleNo)
			{
				int nRightPoles = detectedPoleNumb - nLeftPole;
				int nLeftMiss = OneSidePoleNumb - nLeftPole;
				int nRightMiss = OneSidePoleNumb - nRightPoles;
				poleNumStr = " P##   Missing";

				for (int pIdx = 0; pIdx < nLeftMiss; pIdx++)
				{
					int missPoleIdx = nLeftPole + pIdx;
					startPos = cv::Point(position.x + paddingX, position.y + missPoleIdx * textLineSpace);
					drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
				}

				startPos.x = resImg.cols - txtSize.width - paddingX;
				for (int pIdx = 0; pIdx < nRightMiss; pIdx++)
				{
					int missPoleIdx = nRightPoles + pIdx;
					startPos.y = position.y + missPoleIdx * textLineSpace;
					drawText(resImg, startPos, poleNumStr, NGColor, fontScale, fontWeight);
				}
			}
		}

		cv::Mat CImageProcessorCylinderBatUpperCT::draw_all_text(int cols, xvt::battery::BatteryInspectionResult& result) {
			cv::HersheyFonts font = cv::FONT_HERSHEY_SIMPLEX;
			float font_size = 0.7;
			float font_weight = 1.5;
			int offset = 12;

			std::string title_txt = " No  |  pos  | A-CL | A-CR | B-A | B-C ";
			cv::Size txtsize = cv::getTextSize(title_txt, font, font_size, font_weight, 0);
			int title_x = int(cols / 4 - txtsize.width / 2);
			int title_y = txtsize.height + 1;

			int cnt = result.sCathode2Case.size();
			int rows = (cnt / 2 + 4) * (txtsize.height + offset);

			cv::Mat txtImg = cv::Mat::zeros(rows, cols, CV_8UC3);
			cv::putText(txtImg, title_txt, cv::Point(title_x, title_y), font, font_size, cv::Scalar(255, 255, 255), font_weight);
			cv::putText(txtImg, title_txt, cv::Point(title_x + cols / 2, title_y), font, font_size, cv::Scalar(255, 255, 255), font_weight);

			for (int i = 0; i < cnt; i++) {
				auto pos = result.sXPos[i];
				auto b2c = result.sCathode2Case[i];
				auto decision = result.vtCathode2CaseDecision[i];
				// std::string txt = std::format("A{:03}  {:05.2f}   {:.2f}   {:.2f}   {:.2f}  {:.2f}", i + 1, pos, a2cL, a2cR, b2a, b2c);
				std::string txt = std::format("C{:03}  {:05.2f}    -      -      -   {:.2f}", i + 1, pos, b2c);
				if (i % 2 == 0) {
					title_x = int(cols / 4 - txtsize.width / 2);
				}
				else {
					title_x = int(cols / 4 * 3 - txtsize.width / 2);
				}
				cv::Scalar color = cv::Scalar(255, 255, 255);
				if (decision == false) {
					color = cv::Scalar(0, 0, 255);
				}
				cv::putText(txtImg, txt, cv::Point(title_x, ((i / 2) + 2) * (txtsize.height + offset)), font, font_size, color, font_weight);
			}
			return txtImg;
		}
	}
}