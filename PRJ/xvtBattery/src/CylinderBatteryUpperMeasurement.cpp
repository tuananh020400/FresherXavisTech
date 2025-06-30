#include "xvtBattery/CylinderBatteryUpperMeasurement.h"
#include <xvtCV/Utils.h>
#include <xvtCV/Drawing.h>
#include <xvtCV/Peak.h>
#include <CvPlot/cvplot.h>

namespace xvt {
namespace battery{

CylinderBatteryUpperMeasurement::CylinderBatteryUpperMeasurement()
{
	poleRegionROI = cv::Rect();
	CenterROI = cv::Rect();
	leftCenterRefX = 0;
	rightCenterRefX = 0;
	path = "";
	outImg = cv::Mat();
	autoMeasure = false;
}

ERR_CODE CylinderBatteryUpperMeasurement::Inspection2(const cv::Mat& inImg, BatteryInspectionResult& BIresult)
{
	//-----end of pre-processing ------------------------------------------------------------------------
	cv::Mat src;
	auto res = xvt::Convert8Bits(inImg, src);
	if (res)
	{
		BIresult.Description = "Image empty or not support!";
		return res == 1 ? ERR_CODE::errImageEmpty : ERR_CODE::errImageFormat;
	}

	mGamma.Apply(src, src);

	if (mPixelSize == 0) {
		BIresult.Description = "Pixel size cannot set to 0";
		return ERR_CODE::errPixelSize;
	}

	// check predefined setting 
    if (false == RefineROI(mRoi, cv::Size(src.cols, src.rows)))
    {
        BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
        return ERR_CODE::errROIRefinement;
    }

	inputImg = src.clone();
	cv::cvtColor(src, BIresult.resImg, cv::COLOR_GRAY2BGR);
	outImg = BIresult.resImg.clone();

	const cv::Rect settingROI(mRoi);
	std::string FinalDecision;

	std::vector<std::string> warnMsg;

	cv::Rect batteryROI = FindBatteryROI(src);

	if (batteryROI.width > 0 && batteryROI.height > 0)
	{
		//Check the width of battery is in the range of masterJigCellSize
		if (!mValidCellWidthRange.IsInRange(round_f(batteryROI.width * mPixelSize,2)))
		{
			BIresult.Description = "Error: Width of battery is out of range 2 : " + std::to_string(batteryROI.width * mPixelSize);
			return ERR_CODE::errWidthBattery;
		}
	}
	else
	{
		BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
		return ERR_CODE::errBatterySize;
	}
	//======================================Beading Inspection====================================//
	if (batteryROI.height < mBeadingHeightMin) {
		BIresult.Description = "Cannot inspect the Beading, please check again Min Beading Height, Outer Roi or input image";
		return ERR_CODE::errBeadingArea;
	}

	VecDouble beadingInsResult = { -1,-1,-1,-1,-1 };
	//inspect beading and get case reference line (yA2)
	int yA2 = InspectBeading(src, BIresult.resImg, batteryROI, beadingInsResult);
	//For calculate the anode to case distance
	const int caseLine = yA2 + mCaseLineOffset;

	//The ROI from setting should contain enough pole information (at least 200 rows from yA2);
	if (yA2 - batteryROI.y < mBeadingHeightMin) {
		BIresult.Description = "Cannot inspect the Beading, please check again Min Beading Height, Outer Roi or input image";
		return ERR_CODE::errBeadingInsp;
	}
	if (yA2 >= (batteryROI.y + batteryROI.height)) {
		BIresult.Description = "The pole region height is out of Image ROI, please check Pole Region Height parameter in setting";
		return ERR_CODE::errReferenceCase;
	}

	//2022-05-30 Nhan change checkpole learning before check JRROI to get correct PoleRegion avoid detect pole at boundary fail
	bool poleRegionAuto = false;
	if (mEnableAutoMode) {
		JR_ROIX = 20;
	}
	if (mEnableAutoMode || JR_ROIY == 0) {
		JR_ROIY = 0;
	}
	if (mEnableAutoMode || mPoleRegionHeight == 0) {
		if ((batteryROI.height + batteryROI.y) > yA2) {
			mPoleRegionHeight = ((batteryROI.height + batteryROI.y) - yA2) * 0.75;
			poleRegionAuto = true;
		}
	}

	// 4: Find the shift of inner Poles (Pole leaning)
	cv::Rect poleLeaningROI = batteryROI;
	poleLeaningROI.y = yA2 + JR_ROIY;
	poleLeaningROI.height = mPoleRegionHeight;

	double LeftBorder = 0;
	double RightBorder = poleLeaningROI.width;
	int leaningThreshold = mEnableAutoMode ? 0 : mPoleLeaningThreshold;

	if (poleLeaningROI.empty() || !CheckPoleLeaning(src(poleLeaningROI), mPoleRegionHeight, leaningThreshold, LeftBorder, RightBorder))
	{
		warnMsg.push_back("! Detection Pole Leaning is disable");
	}

	// JRROI
	int maxLeaningDistance = std::max(LeftBorder, poleLeaningROI.width - RightBorder);
	cv::Rect JRROI = cv::Rect();
	JRROI.x = maxLeaningDistance + poleLeaningROI.x;
	JRROI.y = poleLeaningROI.y;
	JRROI.width = poleLeaningROI.width - (maxLeaningDistance * 2);
	JRROI.height = mPoleRegionHeight;

	// 2: Finding the Jelly Roll (J/R) area
	std::pair<int, int> jrPos = FindTopReferenceLine(src(JRROI));
	if (jrPos.first < 0)
	{
		BIresult.Description = "Cannot identify the top blank area of pole in battery (for reference line), please check again JR Setting";
		return ERR_CODE::errJRFinding;
	}

	//Roi for get clahe in full width image, otherwise, the most outer pole will be affected by the bounder.
	poleRegionROI.x = poleLeaningROI.x + LeftBorder;// +JR_ROIX;
	poleRegionROI.y = JRROI.y + jrPos.first;
	poleRegionROI.width = RightBorder - LeftBorder;// -2 * JR_ROIX;
	poleRegionROI.height = mPoleRegionHeight;
	int poleHeight = mValidCathode2AnodeRange.GetUpper() / mPixelSize;
	poleHeight = std::max(30, std::min(poleHeight, 80));
	if ((mEnableAutoMode || poleRegionAuto) && jrPos.second > 0 && jrPos.second - jrPos.first > 10) {
		if (jrPos.second + JRROI.y > poleRegionROI.y) {
			int region = ((jrPos.second + JRROI.y - poleRegionROI.y) + poleHeight) * 2;
			int region2 = batteryROI.y + batteryROI.height - poleRegionROI.y;
			region = std::min(region, region2);
			poleRegionROI.height = region;
			mPoleRegionHeight = poleRegionROI.height;
		}

	}

	cv::Mat poleImg = src(poleRegionROI).clone();

	cv::Size imgSize = poleImg.size();
	VecPoint vecAllCath;
	std::string descript = "";
	ERR_CODE errFindCathodeLine = FindCathodeLine(poleImg, vecAllCath, descript);
	if (errFindCathodeLine != ERR_CODE::OK)
	{
		return errFindCathodeLine;
	}

	cv::Rect rect = cv::boundingRect(vecAllCath);
	int fixCenterWidth = round(poleRegionROI.width / 3.0);
	const int centerLeft = (poleRegionROI.width - fixCenterWidth) / 2;
	//const int centerRight = (poleRegionROI.width + mCenterNeglectionWidth) / 2;
	CenterROI.x = poleRegionROI.x + centerLeft;
	CenterROI.y = poleRegionROI.y;
	CenterROI.width = fixCenterWidth;
	CenterROI.height = rect.br().y;
	RefineROI(CenterROI, outImg.size());
	//Draw pole leaning position
	cv::line(outImg, poleRegionROI.tl(), cv::Point(poleRegionROI.x, poleRegionROI.br().y), cv::Scalar(255, 0, 255), 2);
	cv::line(outImg, cv::Point(poleRegionROI.br().x, poleRegionROI.y), poleRegionROI.br(), cv::Scalar(255, 0, 255), 2);

	BIresult.resImg = outImg;
	return ERR_CODE::OK;
}

cv::Mat CylinderBatteryUpperMeasurement::GetCumulativeHorizontalImage(cv::Size size, std::string title, std::string lable)
{
	int scale = 2;
	cv::Mat mat(size.height * scale, size.width * scale, CV_8UC3);
	mat.setTo(cv::Scalar(255, 255, 255));
	if (!inputImg.empty() && !CenterROI.empty())
	{
		cv::Mat srcTemp, signal;
		inputImg(CenterROI).convertTo(srcTemp, CV_32FC1);

		//Cumulative function vertically
		cv::reduce(srcTemp, signal, 0, cv::REDUCE_AVG, CV_32FC1);
		int signalSize = signal.cols;
		VecDouble list;
		for (int i = 0; i < signalSize; i++)
		{
			list.push_back(signal.at<float>(i));
		}

		CvPlot::Axes axes = CvPlot::makePlotAxes();
		axes.enableHorizontalGrid();
		axes.enableVerticalGrid();
		//axes.title(title);
		//axes.yLabel(lable);
		cv::blur(list, list, cv::Size(10, 1));
		axes.create<CvPlot::Series>(list, "-r");
		int borderLeft = 70, borderRight = 10, borderTop = 30, borderBottom = 30;
		axes.setMargins(borderLeft, borderRight, borderTop, borderBottom);
		axes.setXLim(std::pair<int, int>(0, list.size()));
		axes.setXTight(true);
		double maxList = *max_element(list.begin(), list.end());
		double minList = *min_element(list.begin(), list.end());
		//axes.setYLim(std::pair<int, int>((int)round(minList - 50), (int)round(maxList + 50)));

		axes.render(mat);
		int leftRef = 0, rightRef = 0;
		float axesWidth = mat.cols - (borderLeft + borderRight);
		float scaleRatio = axesWidth / CenterROI.width;
		leftCenterRefX = 0;
		rightCenterRefX = CenterROI.width;
		if (autoMeasure)
		{
			FindPeaks findPeaks = FindPeaks(PeakType::Valley, PeakFindingMethod::Prominence);
			std::vector<float> floatVec(list.begin(), list.end());

			findPeaks.Process(floatVec);
			std::vector<Peak> peakList = findPeaks.GetPeakResult(0.1);

			float leftMinY = INT32_MAX, rightMinY = INT32_MAX;
			for (int i = 0; i < peakList.size(); i++)
			{
				if (peakList[i].index < CenterROI.width / 2)
				{
					if (leftMinY > peakList[i].value)
					{
						leftCenterRefX = peakList[i].index;
						leftMinY = peakList[i].value;
					}
				}
				else
				{
					if (rightMinY > peakList[i].value)
					{
						rightCenterRefX = peakList[i].index;
						rightMinY = peakList[i].value;
					}
				}
			}

			if (leftCenterRefX > 0 && rightCenterRefX > 0)
			{
				leftRef = leftCenterRefX * scaleRatio, rightRef = rightCenterRefX * scaleRatio;
				int newCenter = leftRef + (rightCenterRefX - leftCenterRefX) * scaleRatio / 2;
				cv::line(mat, cv::Point(borderLeft + leftRef, borderTop), cv::Point(borderLeft + leftRef, mat.rows - borderBottom), cv::Scalar(0, 255, 0), 2);
				cv::line(mat, cv::Point(borderLeft + rightRef, borderTop), cv::Point(borderLeft + rightRef, mat.rows - borderBottom), cv::Scalar(0, 255, 0), 2);
				//cv::line(mat, cv::Point(borderLeft + newCenter, borderTop), cv::Point(borderLeft + newCenter, mat.rows - borderBottom), cv::Scalar(0, 255, 0), 2);
				Drawing::DrawDashedLine(mat, cv::Point(borderLeft + newCenter, borderTop), cv::Point(borderLeft + newCenter, mat.rows - borderBottom), cv::Scalar(255, 0, 0), 2, "", 10);
			}
		}
		else
		{
			int wCenterW = mCenterNeglectionWidth * scaleRatio;
			leftCenterRefX = (CenterROI.width - mCenterNeglectionWidth) / 2;
			rightCenterRefX = (CenterROI.width + mCenterNeglectionWidth) / 2;
			leftRef = (axesWidth - wCenterW) / 2, rightRef = (axesWidth + wCenterW) / 2;
			cv::line(mat, cv::Point(borderLeft + leftRef, borderTop), cv::Point(borderLeft + leftRef, mat.rows - borderBottom), cv::Scalar(0, 255, 0), 2);
			cv::line(mat, cv::Point(borderLeft + rightRef, borderTop), cv::Point(borderLeft + rightRef, mat.rows - borderBottom), cv::Scalar(0, 255, 0), 2);
			//cv::line(mat, cv::Point(borderLeft + axesWidth / 2, borderTop), cv::Point(borderLeft + axesWidth / 2, mat.rows - borderBottom), cv::Scalar(255, 0, 0), 2);
			Drawing::DrawDashedLine(mat, cv::Point(borderLeft + axesWidth / 2, borderTop), cv::Point(borderLeft + axesWidth / 2, mat.rows - borderBottom), cv::Scalar(255, 0, 0), 2, "", 10);
		}


		cv::line(outImg, cv::Point(CenterROI.x + leftCenterRefX, poleRegionROI.y), cv::Point(CenterROI.x + leftCenterRefX, poleRegionROI.br().y), cv::Scalar(0, 255, 0), 2);
		cv::line(outImg, cv::Point(CenterROI.x + rightCenterRefX, poleRegionROI.y), cv::Point(CenterROI.x + rightCenterRefX, poleRegionROI.br().y), cv::Scalar(0, 255, 0), 2);

	}
	
	return mat;
}

void CylinderBatteryUpperMeasurement::SetCenterROI(cv::Rect subROI) { if (RefineROI(subROI, inputImg.size()))CenterROI = subROI; };
}
}