#include "ImageProcessorCylinderBatLowerCT.h"
#include "ImageProcessorBase.h"
#include <sys/timeb.h>
#include <fstream>
#include <xvtBattery/CTBatteryInspectorBase.h>
#include "GFuncCylinderBaterryCT.h"

using namespace xvt;
using namespace xvt::battery;

int nLowerProcessingCount = 0;
int gnLowerAngle = 0;
static double gLowerPoleDiffQueue[10][MAX_POLE_NO];
CString gsFilePath;
CString gLowerRecentResultStr;

namespace xf
{
	namespace ImageProcessor
	{
		CImageProcessorCylinderBatLowerCT::CImageProcessorCylinderBatLowerCT()
		{
		}

		CImageProcessorCylinderBatLowerCT::~CImageProcessorCylinderBatLowerCT()
		{
		}

		void CImageProcessorCylinderBatLowerCT::ImageProcessing(const cv::Mat srcImg16bit, xvt::CylinderCTInspection BatLowerInsp, BatLowerResultStruct& bLowerResult) //xvt::battery::BatteryInspectionResult &BIresult)
		{
			xvt::battery::BatteryInspectionResult BIresult;

			CString sCellID = _T("0");

			{
				BatLowerInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH = true;
				BatLowerInsp.inspectingItems.ANODE_TO_CASE_GAP = true;
				BatLowerInsp.inspectingItems.CATHODE_TO_CASE_GAP = true;

				CString sCellID = BatLowerInsp.sCellID = _T("0"); 

//				cIU.GetSettingValue((UINT)CFactoryAlgorithm::CCylinderBatLowerCT::S::eBatteryDirection, BatLowerInsp.BatteryDirection);
				//BatLowerInsp.BatteryDirection = 1;

				BIresult.sData = "";
				std::vector<cv::Point> vtPointDefect;

				DWORD dwStart = GetTickCount();

				CString  csvTitleStr = _T("");
				CString  csvTitleStrXM4 = _T("");

				csvTitleStr = "Opcode,Angle,CellID, NGCode,Final, Ano2Cath, Ano2Base, Time, MinAno2Cath, MaxAno2Cath, AvgAno2Cath, MinAno2Base, MaxAno2Base, AvgAno2Base, MinCath2Base, MaxCath2Base, AvgCath2Base, ";
				csvTitleStrXM4 = "NGCode, R0000343P, R0000344P, R0000345P, R0000346P, R0000347P, R0000348P, Opcode, Angle, CellID, Final, Ano2Cath, Ano2Base, Time, MinCath2Base, MaxCath2Base, AvgCath2Base, ";

				for (int i = 0; i < MAX_POLE_NO; i++) csvTitleStr.AppendFormat(_T("Ano2Cat%d, "), i + 1);
				CString csvResultStr = _T("");
				CString csvResultStrXM4 = _T("");

				int inspectionErr = 0;

				{
					int nImageWidth = srcImg16bit.cols;
					int nImageHeight = srcImg16bit.rows;

					cv::Mat Img;

					Img = xvt::battery::AverageSliceImage(srcImg16bit, BatLowerInsp.SliceNo);

					cv::Mat resImg;
					xvt::Convert8Bits(Img, resImg);
					GammaCorrection(resImg, BatLowerInsp.gamma);
					xvt::ConvertRGB(resImg, resImg);

					CString timeStr = _T("");
					CTime cTime = CTime::GetCurrentTime();
					timeStr.AppendFormat(_T("%02d/%02d %02d:%02d:%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
					struct _timeb timebuffer;
					_ftime64_s(&timebuffer);
					CString strMiliSec = _T("");
					strMiliSec.Format(_T("%03d"), (int)(timebuffer.millitm));

					if (BatLowerInsp.UseBatteryInspection == NO_BAT_INSPECTION)
					{
						CString strA = timeStr;
						strA.AppendFormat(_T(", %d x %d"), nImageWidth, nImageHeight);

						cv::putText(resImg, "Lower - No Inspection", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, (std::string)CT2CA(strA), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 0.8);

						csvResultStrXM4.AppendFormat(_T("0, 0, 0, 0, 0, 0, 0,"));  // NGCode, MinAno2Cath, MaxAno2Cath, AvgAno2Cath, MinAno2Base, MaxAno2Base, AvgAno2Base
						csvResultStrXM4.AppendFormat(_T("1, %d"), BatLowerInsp.Angle); // opcode, angle   
						csvResultStrXM4.AppendFormat(_T("OK, OK, OK, ")); // Final, Ano2Cath, Ano2Base, MinCath2Base, MaxCath2Base, AvgCath2Base
						csvResultStrXM4.AppendFormat(_T("%s,"), timeStr + strMiliSec); // MinCath2Base, MaxCath2Base, AvgCath2Base
						csvResultStrXM4.AppendFormat(_T("0, 0, 0, "));

						gLowerRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
	
						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);

						DrawTextResult(bLowerResult.resImg, BIresult);

						return;
					}

					BIresult.finalDecision = "NG";
					BIresult.Anodes2CaseDecision = "NG";
					BIresult.Anode2CathodeDecision = "NG";

					cv::Point errOutPosition = cv::Point(50, resImg.rows - 100);

					try
					{
						int resValue = Inspection(Img, BatLowerInsp, BIresult);
						nLowerProcessingCount++;

						if (resValue > 0) {
							inspectionErr = resValue;

							cv::Rect settingROI(BatLowerInsp.RoiX, BatLowerInsp.RoiY, BatLowerInsp.RoiWidth, BatLowerInsp.RoiHeight);
							cv::rectangle(resImg, settingROI, cv::Scalar(0, 255, 255), 3);

							std::string description;
							if (resValue == errImageFormat) { description = "Error: ROI or Image format";}
							else if (resValue == errBatterySize) { description = "Error: ROI Selection or Input Image";}
							else if (resValue == errWidthBattery) { description = BIresult.Description;}
							else if (resValue == errPixelSize) { description = "Error: Pixel Size cannot be 0";}
							else if (resValue == errBeadingArea) { description = "Error: ROI Selection";}
							else if (resValue == errBeadingInsp) { description = "Error: Beading Insp ";}
							else if (resValue == errJRFinding) { description = "Error: JR Finding ";}
							else if (resValue == errCenterPin) { description = "Error: Cannot find CENTER PIN ";}
							else if (resValue == errPoleXDetection) { description = "Error: Pole X Detection ";}
							else if (resValue == errPoleXDetection2) { description = "Error: Pole X Detection 2";}
							else if (resValue == errPoleXDetection3) { description = "Error: Pole X Detection 3";}
							else if (resValue == errPoleXDetection4) { description = "Error: Pole X Detection 4";}
							else if (resValue == errPoleXDetection5) { description = "Error: Pole X Detection 5";}
							else if (resValue == errPoleCathodeLine) { description = "Error: Pole Cathode Line ";}
							else if (resValue == errPoleCathodeLine1) { description = "Error: Pole Cathode Line 1";}
							else if (resValue == errPoleCathodeLine2) { description = "Error: Pole Cathode Line 2";}
							else if (resValue == errPoleCathodeLine3) { description = "Error: Pole Cathode Line 3";}
							else if (resValue == errPoleCathodeLine4) { description = "Error: Pole Cathode Line 4";}
							else if (resValue == errPoleCathodeLine5) { description = "Error: Pole Cathode Line 5";}
							else if (resValue == errPoleCathodeLine6) { description = "Error: Pole Cathode Line 6";}
							else if (resValue == errPoleAnodeLine) { description = "Error: Pole Anode Line ";}
							else if (resValue == errPoleNotRefinable) { description = "Error: Pole Not Refinable ";}
							else if (resValue == errPoleRefinement) { description = "Error: Pole Refinement  ";}
							else { description = "Error: " + BIresult.Description;}

							bLowerResult.Description = BIresult.Description;

							cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
							cv::putText(resImg, description, errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);

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

							bLowerResult.minCathode2Case = 0.0;
							bLowerResult.maxCathode2Case = 0.0;
							bLowerResult.avgCathode2Case = 0.0;

//							int baselineLoc = CheckLowerBaseline(pGI, BatLowerInsp);
							int baselineLoc = BIresult.yA2; 
							bLowerResult.baselineLoc = BIresult.yA2;

							cv::line(resImg, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

							CString strB = timeStr; strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
							cv::putText(resImg, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);

							if (BatLowerInsp.UseBatteryInspection == NORMAL_MULTI_INSPECTION)
							{
								cv::putText(resImg, "Angle: " + std::to_string(BatLowerInsp.Angle), cv::Point(nImageWidth - 200, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
							}

							gLowerRecentResultStr = "";

							Img.copyTo(bLowerResult.img8bit);
							resImg.copyTo(bLowerResult.resImg);
							//throw BIresult.Description;
						}
					}
					catch (const std::exception& ex) {
						// ...
						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);
						gLowerRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err1", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

						//						std::cout << ex.what() << std::endl;
						// theApp.WriteAlarmLog(_T("Lower algo : exception 1 :"));
					}
					catch (const std::string& ex)
					{
						std::exception_ptr p = std::current_exception();
						std::cout << ex.c_str() << std::endl;

						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);
						gLowerRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err2", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

						// theApp.WriteAlarmLog(_T("Lower algo : exception 2:") + (CString)ex.c_str());
					}
					catch (...)
					{
						std::exception_ptr p = std::current_exception();

						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);
						gLowerRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err3", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

						// theApp.WriteAlarmLog(_T("Lower algo : exception 3:"));
					}
				
					DWORD dTime = GetTickCount() - dwStart;

					if (dTime != 0)
					{
					}

					bLowerResult.finalDecision = BIresult.finalDecision;
					bLowerResult.Anodes2CaseDecision = BIresult.Anodes2CaseDecision;
					bLowerResult.Anode2CathodeDecision = BIresult.Anode2CathodeDecision;
					bLowerResult.Cathode2CaseDecision = BIresult.Cathode2CaseDecision;

					bLowerResult.cTime = BIresult.cTime;
					bLowerResult.minCathode2Anode = BIresult.minCathode2Anode;
					bLowerResult.maxCathode2Anode = BIresult.maxCathode2Anode;
					bLowerResult.avgCathode2Anode = BIresult.avgCathode2Anode;
					bLowerResult.minAnode2Case    =  BIresult.minAnode2Case;
					bLowerResult.maxAnode2Case = BIresult.maxAnode2Case;
					bLowerResult.avgAnode2Case = BIresult.avgAnode2Case;
					bLowerResult.minCathode2Case = BIresult.minCathode2Case;
					bLowerResult.maxCathode2Case = BIresult.maxCathode2Case;
					bLowerResult.avgCathode2Case =  BIresult.avgCathode2Case;
					bLowerResult.sCathode2Anode = BIresult.sCathode2Anode;

					std::vector<cv::Point> vtCathodeCPoints;
					for(const auto& cat : BIresult.vtCathodes) vtCathodeCPoints.emplace_back(cat.x, cat.y);

					std::vector<cv::Point> vtAnodeCPoints;
					for(const auto& ano : BIresult.vtAnodes) vtAnodeCPoints.emplace_back(ano.x, ano.y);

					int opCode = 0;
					int ngCode = 0;
					int setJudge = 0;

					if (BIresult.finalDecision == "OK") opCode = 1;
					else if (BIresult.finalDecision == "NG") opCode = 2; //	opCode = bLowerResult.finalDecision ? 2 : 1;  // OK 1, NG 2 

					if (inspectionErr >= 100) csvResultStr.AppendFormat(_T("%d, "), inspectionErr);
					else
					{
						csvResultStr.AppendFormat(_T("%d, "), opCode);
						if (BIresult.Anode2CathodeDecision == "NG")            ngCode = NG_LowerAno2CathDistance;
						else if (BIresult.Anodes2CaseDecision == "NG")         ngCode = NG_LowerAno2CaseDistance;
                        else if (BIresult.Anode2CaseVariationDecision == "NG") ngCode = NG_LowerAno2CaseVariation;
					}

					csvResultStr.AppendFormat(_T("%d, "), BatLowerInsp.Angle);
					csvResultStr.AppendFormat(_T("%s, "), sCellID);
					if (BIresult.finalDecision == "OK") csvResultStr.AppendFormat(_T("%d, "), 0);
					else csvResultStr.AppendFormat(_T("%d, "), ngCode);

					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.finalDecision.c_str()));
					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.Anode2CathodeDecision.c_str()));
					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.Anodes2CaseDecision.c_str()));
					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.cTime.c_str()));
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minCathode2Anode);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Anode);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Anode);

					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minAnode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxAnode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgAnode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);

			//============================================================================================
					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 0);
					else csvResultStrXM4.AppendFormat(_T("%d, "), ngCode);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minCathode2Anode);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Anode);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Anode);

					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minAnode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxAnode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgAnode2Case);

					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 1);  // opcode
					else csvResultStrXM4.AppendFormat(_T("%d, "), inspectionErr);

					csvResultStrXM4.AppendFormat(_T("%d, "), BatLowerInsp.Angle);
					csvResultStrXM4.AppendFormat(_T("%s, "), sCellID);
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.finalDecision.c_str()));
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.Anode2CathodeDecision.c_str()));
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.Anodes2CaseDecision.c_str()));
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.cTime.c_str()));
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);

			//============================================================================================

					if (BIresult.sCathode2Anode.size() > MAX_POLE_NO)
					{
						for (int i = 0; i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Anode[i]);
						}
					}
					else
					{
						for (int i = 0; i < BIresult.sCathode2Anode.size(); i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Anode[i]);
						}
						for (int i = BIresult.sCathode2Anode.size(); i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T(" -1, "));
						}
					}

//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatLowerCT::R::eCSVTitle, csvTitleStr);
//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatLowerCT::R::eCSVResult, csvResultStr);

					bLowerResult.opCode = opCode;
					bLowerResult.ngCode = ngCode;
					bLowerResult.setJudge = setJudge;

					if (inspectionErr >= 100) setJudge = inspectionErr;
					else  setJudge = opCode;

					gLowerRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
					
					//if (inspectionErr == 0)
					{
						// cv::Mat text_img = draw_all_text(resImg.cols, BIresult);
						// cv::vconcat(BIresult.resImg, text_img, BIresult.resImg);
						bLowerResult.resImg = BIresult.resImg;
					}
					// Img.copyTo(bLowerResult.img8bit);
					// BIresult.resImg.copyTo(resImg);
					
					// DrawTextResult(bLowerResult.resImg, BIresult);
					// drawPoleTextResult(bLowerResult.resImg, BatLowerInsp, BIresult);
					// BIresult.resImg.copyTo(bLowerResult.resImg);

					return;
				}
			}
		}

		int CImageProcessorCylinderBatLowerCT::Inspection(cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BIresult)
		{
			BIresult = {};

			int nImageWidth = img.cols;
			int nImageHeight = img.rows;

			cv::Mat Img;
	
			Img = xvt::battery::AverageSliceImage(img, BatLowerInsp.SliceNo);
			if (Img.empty()) return -1;

			nImageHeight = Img.rows;

			// check predefined setting 
			cv::Rect settingOuterROI = cv::Rect(BatLowerInsp.RoiX, BatLowerInsp.RoiY, BatLowerInsp.RoiWidth, BatLowerInsp.RoiHeight);
			settingOuterROI = RoiRefinement(settingOuterROI, cv::Size(nImageWidth, nImageHeight));
			BatLowerInsp.RoiX = settingOuterROI.x;
			BatLowerInsp.RoiY = settingOuterROI.y;
			BatLowerInsp.RoiWidth = settingOuterROI.width;
			BatLowerInsp.RoiHeight = settingOuterROI.height;

			if (BatLowerInsp.RoiX >= nImageWidth || BatLowerInsp.RoiY >= nImageHeight || BatLowerInsp.RoiWidth <= RESIZE_SCALE || BatLowerInsp.RoiHeight <= RESIZE_SCALE) {
				BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
				return errBatterySize;
			}

	  		//--- pre-preocessing --------------------------------------------------------------------------
			xvt::Convert8Bits(Img, Img, false);

			//-----end of pre-processing ------------------------------------------------------------------------
			xvt::battery::CTBatteryInspector ctIsp;
			auto& baseInspector = ctIsp.mIspBase;
			auto& beadingInspector = ctIsp.mIspBeading;
			auto& jrInspector = ctIsp.mIspJR;
			auto& poleInspector = ctIsp.mIspPole;
			auto& cathodeInspector = ctIsp.mIspCathode;
			auto& anodeInspector = ctIsp.mIspAnode;
			auto& displayPen = ctIsp.mDisplayPen;

			ctIsp.mEnable = true;
			baseInspector.mRoi = {BatLowerInsp.RoiX, BatLowerInsp.RoiY, BatLowerInsp.RoiWidth, BatLowerInsp.RoiHeight};
			baseInspector.mThreshold = BatLowerInsp.batteryThreshold;
			baseInspector.mDirection = BatLowerInsp.BatteryDirection;
			ctIsp.mPixelSize = BatLowerInsp.pixelSize;
			ctIsp.mDisplayMode = static_cast<xvt::battery::DisplayMode>(BatLowerInsp.displayMode);
			baseInspector.mValidBatteryWidthRange = xvt::Ranged(BatLowerInsp.cellSizeLowBound, BatLowerInsp.cellSizeHighBound, BatLowerInsp.EnableValidWidth);
			beadingInspector.mEnable = BatLowerInsp.EnableBeading;

			jrInspector.mHeight = BatLowerInsp.poleRegionHeight;
			jrInspector.mLeaningThreshold = BatLowerInsp.PoleLeaningThreshold;
			jrInspector.mJROffsetX = BatLowerInsp.JR_ROIX;
			jrInspector.mJROffsetY = BatLowerInsp.JR_ROIY;
			jrInspector.mCenterNeglectionWidth = BatLowerInsp.CenterNeglectionWidth;
			jrInspector.mOneSidePoleNumber = BatLowerInsp.OneSidePoleNumb;
			jrInspector.mMinLeaningDistance = BatLowerInsp.minLeaningDistance;
			jrInspector.mBaseLineOffset = BatLowerInsp.tolerenceShift;

			cathodeInspector.mCathodeLineThresholdInner = BatLowerInsp.cathodeLineThredsholdInner;
			cathodeInspector.mCathodeLineThresholdMiddle = BatLowerInsp.cathodeLineThredsholdMiddle;
			cathodeInspector.mCathodeLineThresholdOuter = BatLowerInsp.cathodeLineThredsholdOuter;
			cathodeInspector.mProminence = BatLowerInsp.cathodeLineProminence;
			cathodeInspector.mCathodeLineWindowSize = BatLowerInsp.cathodeLineWindowSize;

			anodeInspector.mAnodeThresholdInner = BatLowerInsp.anodeThresholdInner;
			anodeInspector.mAnodeThresholdMiddle = BatLowerInsp.anodeThresholdMiddle;
			anodeInspector.mAnodeThresholdOuter = BatLowerInsp.anodeThresholdOuter;
			anodeInspector.mAnodeEnhanceScale = BatLowerInsp.anodeEnhanceScale;

			poleInspector.mSkipPolesDistance = BatLowerInsp.skipPolesDistance;

			poleInspector.mPoleHeight = BatLowerInsp.polesHeight;
			poleInspector.mPolesProminenceThreshold = BatLowerInsp.minProminence;
			poleInspector.mPolesDistanceRange.Set(BatLowerInsp.polesMinDistance, BatLowerInsp.polesMaxDistance);

			poleInspector.mValidCathode2AnodeRange = xvt::Ranged(BatLowerInsp.MinCathode2Anode, BatLowerInsp.MaxCathode2Anode, BatLowerInsp.EnableCathode2Anode);
			poleInspector.mValidAnode2CaseRange = xvt::Ranged(BatLowerInsp.MinAnode2Case, BatLowerInsp.MaxAnode2Case, BatLowerInsp.EnableAnode2Case);
			poleInspector.mValidCathode2CaseRange = xvt::Ranged(BatLowerInsp.MinCathode2Case, BatLowerInsp.MaxCathode2Case, BatLowerInsp.EnableCathode2Case);

			poleInspector.mValidCathode2AnodeRange.Set(BatLowerInsp.MinCathode2Anode, BatLowerInsp.MaxCathode2Anode);
			poleInspector.mValidAnode2CaseRange.Set(BatLowerInsp.MinAnode2Case, BatLowerInsp.MaxAnode2Case);
			poleInspector.mValidCathode2CaseRange.Set(BatLowerInsp.MinCathode2Case, BatLowerInsp.MaxCathode2Case);

			poleInspector.mVariationAnode2Case = BatLowerInsp.variationAnode2Case;
			poleInspector.mCathode2AnodeOffset = BatLowerInsp.mCathode2AnodeOffset;
			poleInspector.mAnode2CaseOffset = BatLowerInsp.mAnode2CaseOffset;
			poleInspector.mCathode2CaseOffset = BatLowerInsp.mCathode2CaseOffset;

			//ctIsp.mInspectingItems.ANODE_TO_CASE_GAP = BatLowerInsp.inspectingItems.ANODE_TO_CASE_GAP;
			//ctIsp.mInspectingItems.ANODE_TO_CASE_VARIATION = (BatLowerInsp.variationAnode2Case != 0);
			//ctIsp.mInspectingItems.ANODE_TO_CATHODE_LENGTH = BatLowerInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH;
			//ctIsp.mInspectingItems.CATHODE_TO_CASE_GAP = BatLowerInsp.inspectingItems.CATHODE_TO_CASE_GAP;
			
			displayPen.mFontScale = 0.4;
			displayPen.mSpace = 15;

			xvt::battery::CTBatteryInspectorResult ctResult;
			xvt::battery::ERR_CODE err = ctIsp.Inspect(Img, ctResult);
			BIresult = ctResult;
			BIresult.resImg = Img.clone();
			GammaCorrection(BIresult.resImg, BatLowerInsp.gamma);
			xvt::ConvertRGB(BIresult.resImg, BIresult.resImg);
			ctResult.DrawResult(BIresult.resImg);
			//int baselineLoc = BIresult.yA2;

			if(err != xvt::battery::ERR_CODE::OK)
			{
				Img.release();
				//ctIsp.mIspResult.DrawResult(BIresult.resImg);
				//BIresult.resImg.release();
				return static_cast<int>(err);
			}

			cv::Mat res = BIresult.resImg;
		
			if ((BatLowerInsp.isSaveResults == 1) || (BatLowerInsp.UseBatteryInspection == DATA_WRITE_BAT_INSPECTION))
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
				if (filepath.size() == 0) fileName = "D:\\Result\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Lower_Data_1.csv";
				else                      fileName = filepath + "\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Lower_Data_1.csv";

				if (nLowerProcessingCount == 0)
				{
					for (int i = 0; i < 10; i++) for (int j = 0; j < MAX_POLE_NO; j++) gLowerPoleDiffQueue[i][j] = -1.0;

					std::string wStr = "";

					wStr += "No,Final, An2Cat, An2Case,time,filename,Angle,OutRoiX,OutRoiY,OutRoiW, OutRoiH, BatThresh, PoleRegion, PoleLeaning, JRRoiX, JRRoiY,";
					wStr += "NeglectCenterArea, TolShift, CatInner, CatMiddle, CatOuter, CatWinSize, PoleHeight, PoleDetecThresh, MinDistance, ";
					wStr += "MaxDistance, AnodeInner, AnodeMiddle, AnodeOuter, No.oneSidePole, isCheckPoleNo, CheckingPin, SkipPoleDist, PixSize, depthScale, additionalLine, SVminAn2CatMM, SVmaxAn2CatMM, SVminAn2CaseMM, SVmaxAn2CaseMM, SVminCat2BaseMM, SVmaxCat2BaseMM,,";
					wStr += "PMinAno2Cath, PMaxAno2Cath, PAvgAno2Cath, ,PMinAno2Base, PMaxAno2Base, PAvgAno2Base, ,PMinCath2Base, PMaxCath2Base, PAvgCath2Base,, PCath2Ano,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
					wStr += "Cat2Base,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
					wStr += "An2Base,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';

					std::ofstream fout;
					fout.open(fileName, std::ios_base::out | std::ios_base::app);
					fout << wStr << std::endl;
					fout.close();
				}

				double poleDiffLength[MAX_POLE_NO];

				if (BIresult.sCathode2Anode.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++)	poleDiffLength[i] = BIresult.sCathode2Anode[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Anode.size(); i++) poleDiffLength[i] = BIresult.sCathode2Anode[i];
					for (int i = BIresult.sCathode2Anode.size(); i < MAX_POLE_NO; i++) poleDiffLength[i] = -1;;
				}

				double poleDiffLength2[MAX_POLE_NO];

				if (BIresult.sCathode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Case.size(); i++)	poleDiffLength2[i] = BIresult.sCathode2Case[i];
					for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength2[i] = -1;;
				}

				double poleDiffLength3[MAX_POLE_NO];

				if (BIresult.sAnode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sAnode2Case.size(); i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
					for (int i = BIresult.sAnode2Case.size(); i < MAX_POLE_NO; i++)	poleDiffLength3[i] = -1;;
				}

				std::string wStr = std::to_string(nLowerProcessingCount + 1) + ",";
//				if (pGI->IsLeft() ) wStr += std::to_string(0) + ",";
//				else wStr += std::to_string(90) + ",";
				wStr += BIresult.finalDecision + ",";
				wStr += BIresult.Anode2CathodeDecision + ",";
				wStr += BIresult.Anodes2CaseDecision + ",";
				wStr += BIresult.cTime + ",";

				wStr += outFileName + ",";
				wStr += std::to_string(BatLowerInsp.Angle) + ",";
				wStr += std::to_string(BatLowerInsp.RoiX) + ",";
				wStr += std::to_string(BatLowerInsp.RoiY) + ",";
				wStr += std::to_string(BatLowerInsp.RoiWidth) + ",";
				wStr += std::to_string(BatLowerInsp.RoiHeight) + ",";
				wStr += std::to_string(BatLowerInsp.batteryThreshold) + ",";
				wStr += std::to_string(BatLowerInsp.poleRegionHeight) + ",";
				wStr += std::to_string(BatLowerInsp.PoleLeaningThreshold) + ",";
				wStr += std::to_string(BatLowerInsp.JR_ROIX) + ",";
				wStr += std::to_string(BatLowerInsp.JR_ROIY) + ",";
				wStr += std::to_string(BatLowerInsp.CenterNeglectionWidth) + ",";
				wStr += std::to_string(BatLowerInsp.tolerenceShift) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineThredsholdInner) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineThredsholdMiddle) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineThredsholdOuter) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineWindowSize) + ",";
				wStr += std::to_string(BatLowerInsp.polesHeight) + ",";
				wStr += std::to_string(BatLowerInsp.minProminence) + ",";
				wStr += std::to_string(BatLowerInsp.polesMinDistance) + ",";
				wStr += std::to_string(BatLowerInsp.polesMaxDistance) + ",";

				wStr += std::to_string(BatLowerInsp.anodeThresholdInner) + ",";
				wStr += std::to_string(BatLowerInsp.anodeThresholdMiddle) + ",";
				wStr += std::to_string(BatLowerInsp.anodeThresholdOuter) + ",";
				wStr += std::to_string(BatLowerInsp.OneSidePoleNumb) + ",";
				wStr += std::to_string(BatLowerInsp.isCheckPin) + ",";
				wStr += std::to_string(BatLowerInsp.skipPolesDistance) + ",";
				wStr += std::to_string(BatLowerInsp.pixelSize) + ",";
				wStr += std::to_string(BatLowerInsp.depthScale) + ",";
				wStr += std::to_string(BatLowerInsp.additionalLine) + ",";
				wStr += std::to_string(BatLowerInsp.MinCathode2Anode) + ",";
				wStr += std::to_string(BatLowerInsp.MaxCathode2Anode) + ",";
				wStr += std::to_string(BatLowerInsp.MinAnode2Case) + ",";
				wStr += std::to_string(BatLowerInsp.MaxAnode2Case) + ",";
				wStr += std::to_string(BatLowerInsp.MinCathode2Case) + ",";
				wStr += std::to_string(BatLowerInsp.MaxCathode2Case) + ", ,";

				wStr += std::to_string(BIresult.minCathode2Anode) + ",";
				wStr += std::to_string(BIresult.maxCathode2Anode) + ",";
				wStr += std::to_string(BIresult.avgCathode2Anode) + ", ,";
				wStr += std::to_string(BIresult.minAnode2Case) + ",";
				wStr += std::to_string(BIresult.maxAnode2Case) + ",";
				wStr += std::to_string(BIresult.avgAnode2Case) + ", ,";
				wStr += std::to_string(BIresult.minCathode2Case) + ",";
				wStr += std::to_string(BIresult.maxCathode2Case) + ",";
				wStr += std::to_string(BIresult.avgCathode2Case) + ", , ,";

				CImageProcessorBase::WrProcessingData(wStr, nLowerProcessingCount, 2*BatLowerInsp.OneSidePoleNumb, gLowerPoleDiffQueue, poleDiffLength, poleDiffLength2, poleDiffLength3, 54);

				std::ofstream fout;

				fout.open(fileName, std::ios_base::out | std::ios_base::app);
				fout << wStr << std::endl;
				fout.close();

//				cv::imwrite(filepath + "\\" + fileNameStr + "_" + tStr + "jpg", res);
			}

			CString strB = _T("");
			CTime cTime = CTime::GetCurrentTime();
			strB.AppendFormat(_T("%02d/%02d %02d:%02d:%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
			//strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
			cv::putText(res, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2.0);
			//cv::line(res, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

			return 0;
		}

		void CImageProcessorCylinderBatLowerCT::DrawTextResult(cv::Mat& img, xvt::battery::BatteryInspectionResult& BatInsp)
		{
			if (BatInsp.finalDecision == "OK") {

				cv::putText(BatInsp.resImg, "Result: OK", cv::Point(BatInsp.resImg.cols / 3.1, 85), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);

				auto matTest = BatInsp.resImg;
				double textSize = 0.48;
				int rowHeight = 30;
				int thickness = 1;
				constexpr int noPrecision = 3;
				std::vector<std::string> rangeRow;
				std::stringstream ss;
				ss << "Base->Cathode [" << std::fixed << std::setprecision(noPrecision) << 0 << " ~ " << std::fixed << std::setprecision(noPrecision) << 0 << "]";
				rangeRow.push_back(ss.str()); ss.str("");

				ss << "Base->Anode [" << std::fixed << std::setprecision(noPrecision) << 0 << " ~ " << std::fixed << std::setprecision(noPrecision) << 0 << "]";
				rangeRow.push_back(ss.str()); ss.str("");

				ss << "Anode->Cathode [" << std::fixed << std::setprecision(noPrecision) << 0 << " ~ " << std::fixed << std::setprecision(noPrecision) << 0 << "]";
				rangeRow.push_back(ss.str()); ss.str("");

				std::vector<std::string> Row1 = { "Min", "Max", "Average", "Min", "Max", "Average", "Min", "Max", "Average" };

				std::vector<std::string> Row2;
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.minCathode2Case; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.maxCathode2Case; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.avgCathode2Case; Row2.push_back(ss.str()); ss.str("");

				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.minAnode2Case; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.maxAnode2Case; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.avgAnode2Case; Row2.push_back(ss.str()); ss.str("");

				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.minCathode2Anode; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.maxCathode2Anode; Row2.push_back(ss.str()); ss.str("");
				ss << std::fixed << std::setprecision(noPrecision) << BatInsp.avgCathode2Anode; Row2.push_back(ss.str()); ss.str("");

				int cellWidth = 270;
				int startX = (matTest.cols - rangeRow.size() * cellWidth) / 2;
				int startY = 160;

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
		void CImageProcessorCylinderBatLowerCT::drawPoleTextResult(const cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BatInsp)
		{
			int nLeftPole = 50;
			cv::Mat resImg = BatInsp.resImg;
			const VecDouble &listAnode2CathodeDistance = BatInsp.sCathode2Anode;
			const VecDouble &listAnode2CaseDistance = BatInsp.sAnode2Case;
			const VecDouble &listCathode2CaseDistance = BatInsp.sCathode2Case;
			const VecDouble &listPolePos = BatInsp.sXPos;
			const std::vector<bool> &listAno2CathodDecision = BatInsp.vtAno2CathodDecision;
			const std::vector<bool> &listAno2CaseDecision = BatInsp.vtAno2CaseDecision;
			const std::vector<bool> &listCathode2CaseDecision = BatInsp.vtCathode2CaseDecision;
			//int nLeftPole = bInspResult.nLeftPole;
			double cathode2AnodeOffset = BatLowerInsp.mCathode2AnodeOffset;
			double anode2CaseOffset = BatLowerInsp.mAnode2CaseOffset;
			double cathode2CaseOffset = BatLowerInsp.mCathode2CaseOffset;
			bool isCheckPoleNo = BatLowerInsp.OneSidePoleNumb;
			int OneSidePoleNumb = BatLowerInsp.OneSidePoleNumb;
			float fontScale = 1; // change if needed
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
			anode2CathodeLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << BatLowerInsp.MinCathode2Anode << "~"
								  << BatLowerInsp.MaxCathode2Anode << "]";

			std::ostringstream anode2CaseLimitStr;
			anode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << BatLowerInsp.MinAnode2Case << "~"
							   << BatLowerInsp.MaxAnode2Case << "]";

			std::ostringstream anode2CaseOffsetStr;
			anode2CaseOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << anode2CaseOffset;

			std::ostringstream cathode2anodeOffsetStr;
			cathode2anodeOffsetStr << std::fixed << std::setprecision(floatingPrecesion) << cathode2AnodeOffset;

			std::ostringstream cathode2CaseLimitStr;
			cathode2CaseLimitStr << "[" << std::fixed << std::setprecision(floatingPrecesion) << BatLowerInsp.MinCathode2Case << "~"
								 << BatLowerInsp.MaxCathode2Case << "]";

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

		cv::Mat CImageProcessorCylinderBatLowerCT::draw_all_text(int cols, xvt::battery::BatteryInspectionResult& result) {
			cv::HersheyFonts font = cv::FONT_HERSHEY_SIMPLEX;
			float font_size = 0.7;
			float font_weight = 1.5;
			int offset = 12;

			std::string title_txt = " No  |  pos  | A-CL | A-CR | B-A | B-C ";
			cv::Size txtsize = cv::getTextSize(title_txt, font, font_size, font_weight, 0);
			int title_x = int(cols / 4 - txtsize.width / 2);
			int title_y = txtsize.height + 1;

			int cnt = result.sCathode2Anode.size();
			int rows = (cnt / 2 + 4) * (txtsize.height + offset);

			cv::Mat txtImg = cv::Mat::zeros(rows, cols, CV_8UC3);
			cv::putText(txtImg, title_txt, cv::Point(title_x, title_y), font, font_size, cv::Scalar(255, 255, 255), font_weight);
			cv::putText(txtImg, title_txt, cv::Point(title_x + cols / 2, title_y), font, font_size, cv::Scalar(255, 255, 255), font_weight);

			for (int i = 0; i < cnt; i++) {
				auto pos = result.sXPos[i];
				// auto a2c = result.sCathode2Anode[i];
				auto a2cL = result.sCathode2AnodeLR[i].first;
				auto a2cR = result.sCathode2AnodeLR[i].second;
				auto b2a = result.sAnode2Case[i];
				auto b2c = result.sCathode2Case[i];
				auto decision = result.vtAno2CathodDecision[i];
				std::string txt = std::format("A{:03}  {:05.2f}   {:.2f}   {:.2f}   {:.2f}  {:.2f}", i + 1, pos, a2cL, a2cR, b2a, b2c);
				// std::string txt = std::format("P{:02} {:05.2f}  {:.2f}  {:.2f}  {:.2f}", i + 1, pos, a2c, b2a, b2c);
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