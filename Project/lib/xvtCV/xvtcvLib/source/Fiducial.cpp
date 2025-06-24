#include "xvtCV/Fiducial.h"
#include "xvtCV/ContrastEnhancement.h"
#include "xvtCV/LocalThresholding.h"
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/KuwaharaFilter.h"
#include "xvtCV/CurveCSS.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <ppl.h>
//#include "parralel.h"

#ifdef _DEBUG
//#define _DEBUG_LOG
//#define _PARALLEL
#endif // _DEBUG

namespace xvtThd = xvt::threshold;
namespace xvtEnhance = xvt::enhance;

namespace xvt {
namespace feature {

Fiducial::Fiducial()
{
	warpType = WarpFiducialType::NONE;
	vtPointREF_warp = {};
	vtTemplateImagePath = {};
}


Fiducial* Fiducial::Clone()
{
	return new Fiducial(*this);
}

bool Fiducial::WarpImage(std::string path, std::string imgname, cv::Mat img, cv::Mat& outImage)
{
	bool bret = false;
	switch (warpType)
	{
	case WarpFiducialType::NONE:
		outImage = img.clone();
		bret = true;
		break;
	case WarpFiducialType::FIDUCIAL_AUTO:
		bret = Warp_FiducialObject(path, imgname, img, outImage, vtPointREF_warp);
		break;
	case WarpFiducialType::FIDUCIAL_MANUAL:
		bret = Warp_Manual(path, imgname, img, outImage, vtPointREF_warp, vtTemplateImagePath);
		break;
	case WarpFiducialType::FIDUCIAL_GRID:
		break;
	default:
		break;
	}

	return bret;

}

void Fiducial::FindSetofPoint(Pair_Fifucial* PairF)
{
	PairF->bMatchTemplate = false;
	double score;
	cv::Point2f centerMatchingPoint = cv::Point2f(-1, -1);
	std::vector<cv::Rect> segList2;
	if (Matching_template(PairF->img, PairF->templateImage, PairF->imgname, score, 0, 0, centerMatchingPoint))
	{
		PairF->bMatchTemplate = true;
		PairF->PointCentermatch = centerMatchingPoint;
	}

}

bool Fiducial::Warp_Manual(std::string path, std::string imgname, cv::Mat img, cv::Mat& outImage, std::vector<cv::Point2f> vtPointREF_warp, std::vector<std::string> vtTemplateImagePath)
{
	int FiducialQty = (int)vtPointREF_warp.size();
	std::vector<cv::Point2f> vtPointREF_new; //List the point in vtPointREF_warp matching with template


	//double score;
	cv::Point2f centerMatchingPoint;
	std::vector<cv::Point2f> vtPoint_origin;
	std::vector<bool> vtResult;
#ifdef _DEBUG_LOG
	auto t1_extractroi = high_resolution_clock::now();
#endif		
	std::vector<Pair_Fifucial*> vtPairF{};

	for (int i = 0; i < FiducialQty; i++)
	{
		cv::Mat templateimage = cv::imread(vtTemplateImagePath[i], 0);
		//setting 
		Pair_Fifucial* pairF = new Pair_Fifucial();
		pairF->imgname = imgname;
		pairF->img = img;
		pairF->bMatchTemplate = false;
		pairF->PointREF_warp = vtPointREF_warp[i];
		pairF->PointCentermatch = cv::Point2f();
		pairF->templateImage = templateimage;

		vtPairF.push_back(pairF);
	}

#ifdef _PARALLEL

	concurrency::parallel_for_each(vtPairF.begin(), vtPairF.end(), [&](Pair_Fifucial* pF)
								   {

									   FindSetofPoint(pF);
								   }
	);
	for (int i = 0; i < FiducialQty; i++)
	{
		if (vtPairF[i]->bMatchTemplate)
		{
			vtPointREF_new.push_back(vtPairF[i]->PointREF_warp);
			vtPoint_origin.push_back(vtPairF[i]->PointCentermatch);
		}
	}

#else
	for (int i = 0; i < FiducialQty; i++)
	{
		FindSetofPoint(vtPairF[i]);
		if (vtPairF[i]->bMatchTemplate)
		{
			vtPointREF_new.push_back(vtPairF[i]->PointREF_warp);
			vtPoint_origin.push_back(vtPairF[i]->PointCentermatch);
		}
	}
#endif // _PARALLEL

	vtPairF.clear();


#ifdef _DEBUG_LOG
	auto t2_extractroi = high_resolution_clock::now();
	auto ms_message = std::chrono::duration_cast<std::chrono::milliseconds>(t2_extractroi - t1_extractroi).count();
	std::cout << "Matching_template: " << ms_message << "ms" << std::endl;
#endif
	if (vtPoint_origin.size() < 4) { std::cout << "Not found enough 4 fiducial points to warpping  " << std::endl; return false; }
	else
	{
#ifdef _DEBUG_LOG
		auto t1_extractroi = high_resolution_clock::now();
#endif
		cv::Mat H = cv::findHomography(vtPoint_origin, vtPointREF_new);
		cv::warpPerspective(img, outImage, H, img.size());
		H.release();

#ifdef _DEBUG_LOG
		auto t2_extractroi = high_resolution_clock::now();
		auto ms_message = std::chrono::duration_cast<std::chrono::milliseconds>(t2_extractroi - t1_extractroi).count();
		std::cout << "Warp Fiducial: " << ms_message << "ms" << std::endl;
#endif // _DEBUG_LOG

	}
	img.release();


	return true;
}

bool Fiducial::Warp_FiducialObject(std::string path, std::string filename, cv::Mat img, cv::Mat& outImage, std::vector<cv::Point2f> warpCorner)
{
	cv::Mat outimg, imgclone;
	xvt::enhance::AGCWD(img, outimg);
	cv::Mat binaryImg(img.rows, img.cols, CV_8U);
	xvtThd::ThresholdWolf(outimg, binaryImg, cv::Size(51, 51), 0.5, 128);
	cv::Rect rect = FindLargestContour(~binaryImg, outimg, cv::Scalar(0, 0, 255));
	/* std::cout << rect << endl;
	 std::cout << "p1=" << rect.x << "," << rect.y << " p2=" << rect.x << "," << rect.y + rect.height << endl;
	 std::cout << "p3=" << rect.x + rect.width << "," << rect.y + rect.height << " p4=" << rect.x + rect.width << "," << rect.y << endl;*/

	imgclone = img(rect).clone();
	cv::Mat newoutImg;
	Pre_process_Fulloverlap(imgclone, newoutImg);
	cv::Rect newrect = FindLargestContour(~newoutImg, newoutImg, cv::Scalar(0, 0, 255));
	//std::cout << "new p1=" << rect.x + newrect.x << "," << rect.y + newrect.y << " p2=" << rect.x + newrect.x << "," << rect.y + newrect.y + newrect.height << endl;
	//std::cout << "new p3=" << rect.x + newrect.x + newrect.width << "," << rect.y + newrect.y + newrect.height << " p4=" << rect.x + newrect.x + newrect.width << "," << rect.y + newrect.y << endl;

	double dx = abs(warpCorner[3].x - warpCorner[0].x);
	double dy = abs(warpCorner[1].y - warpCorner[0].y);

	std::vector<cv::Point2f> vtcorners;
	vtcorners.push_back(cv::Point2f(float(rect.x + newrect.x), float(rect.y + newrect.y)));
	vtcorners.push_back(cv::Point2f(float(rect.x + newrect.x), float(rect.y + newrect.y + newrect.height)));
	vtcorners.push_back(cv::Point2f(float(rect.x + newrect.x + newrect.width), float(rect.y + newrect.y + newrect.height)));
	vtcorners.push_back(cv::Point2f(float(rect.x + newrect.x + newrect.width), float(rect.y + newrect.y)));

	cv::Mat H = cv::findHomography(vtcorners, warpCorner);
	//std::cout << "H:\n" << H << std::endl;
	cv::warpPerspective(img, outImage, H, img.size());
	H.release();

	//std::cout << "Can not warp fiducial. " << std::endl;  return false;
#ifdef DEBUGWARP
	cvtColor(outImage, tempimage, cv::COLOR_GRAY2BGR);
	rectangle(tempimage, cv::Rect(warpCorner[0].x, warpCorner[0].y, dx, dy), cv::Scalar(0, 0, 255), 2);
	show_window("tempF " + imgname, tempimage);
	xf::ini::WriteImg(path, filename + "_warp.bmp", outImage);
	waitKey();
#endif
	outimg.release();
	binaryImg.release();
	imgclone.release();
	newoutImg.release();
	return true;
}

void Fiducial::Pre_process_Fulloverlap(cv::Mat srcImg, cv::Mat& dst)
{
	//std::cout << "Full overlap" << endl;
	cv::Mat srcImgA = srcImg.clone();

	xvt::filter::Kuwahara kuwaFitler1(srcImg, 3);
	srcImgA = kuwaFitler1.Apply();

#pragma warning (suppress:26812)
	cv::normalize(srcImgA, srcImgA, 0, 255, cv::NORM_MINMAX);
	xvtEnhance::AGCIE(srcImgA, srcImgA);

	cv::Mat binaryimg(srcImgA.rows, srcImgA.cols, CV_8U);
	std::vector<int>  vtOtsu = xvtThd::ThresholdOtsuMulti(srcImgA, 3);
	cv::threshold(srcImgA, binaryimg, vtOtsu[2], 255, cv::THRESH_BINARY_INV);

	cv::Mat1b newImg = srcImgA.clone();
	newImg.setTo(cv::Scalar(255));
	srcImgA.copyTo(newImg, binaryimg);

	xvt::filter::Kuwahara kuwaFitler(newImg, 4);
	newImg = kuwaFitler.Apply();
	xvtEnhance::AGCIE(newImg, newImg);
	cv::Mat1b th;
	cv::Mat1b thmask = binaryimg;
	double th_value = xvtThd::ThresholdWithMask(newImg, th, 10, 100, cv::THRESH_OTSU, thmask);

	cv::threshold(newImg, dst, 0, 255, cv::THRESH_OTSU);
	int secondThresh = xvtThd::ThresholdIntermodes(newImg);
	float cdfHist = xvt::shape::getAccHistAtThreshold(newImg, secondThresh);
	if (cdfHist < 0.4)
	{
		cv::threshold(newImg, dst, secondThresh, 255, cv::THRESH_BINARY);
	}
	else
	{
		xvtThd::ThresholdSauvola(newImg, dst, cv::Size(21, 21), 0.5, 128);
	}
	bitwise_not(dst, dst);
	srcImgA.release();
	binaryimg.release();
	newImg.release();
	thmask.release();
	th.release();
}

cv::Rect Fiducial::FindLargestContour(cv::Mat BinaryImg, cv::Mat& output, cv::Scalar color)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(BinaryImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	int indexlargestcontour = -1;
	double largestarea = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area >= largestarea)
		{
			largestarea = area; indexlargestcontour = i;
		}
	}
	cv::cvtColor(BinaryImg, BinaryImg, cv::COLOR_GRAY2BGR);
	drawContours(BinaryImg, contours, indexlargestcontour, cv::Scalar(0, 255, 0));
	output = BinaryImg;
	cv::Rect minRectcontour = boundingRect(contours[indexlargestcontour]);
	rectangle(BinaryImg, minRectcontour, color, 2);
	return minRectcontour;
}

int ii = 0;
bool Fiducial::Matching_template(cv::Mat imgsrc, cv::Mat templateimage, std::string imagename, double& score, int x, int y, cv::Point2f& centerpoint)
{
	auto t1_extractroi = std::chrono::high_resolution_clock::now();
	cv::Mat scoreImg;
	double minVal;  cv::Point maxLoc;
	cv::Point matchLoc;
	if (templateimage.rows > imgsrc.rows || templateimage.cols > imgsrc.cols)
	{
		return false;
	}
	////scale image
	//cv::Mat imgsrc_Scale = imgsrc.clone();
	//cv::Mat templateimage_Scale = templateimage.clone();
	//double dscale = 0.1;
	//double ratio = 1 / dscale;
	//cv::resize(imgsrc_Scale, imgsrc_Scale, cv::Size(), dscale, dscale);
	//cv::resize(templateimage_Scale, templateimage_Scale, cv::Size(), dscale, dscale);
	//ii++;
	matchTemplate(imgsrc, templateimage, scoreImg, cv::TM_CCOEFF_NORMED);
	//matchTemplate(imgsrc_Scale, templateimage_Scale, scoreImg, cv::TM_CCOEFF_NORMED);
	minMaxLoc(scoreImg, &minVal, &score, 0, &maxLoc, cv::Mat());
	matchLoc = maxLoc;

	cv::Rect2d bound = cv::Rect2d(cv::Point2d(matchLoc.x, matchLoc.y), cv::Point2d(static_cast<double>(matchLoc.x) + templateimage.cols, static_cast<double>(matchLoc.y) + templateimage.rows));
	bound.x = bound.x + x;
	bound.y = bound.y + y;
	maxLoc = cv::Point(0, 0);
	centerpoint = cv::Point2f(float(bound.x + bound.width / 2), float(bound.y + bound.height / 2));



	//if (score > 0.8)
	//{
	//	int border = 10;
	//	cv::Rect2d bound = cv::Rect2d(cv::Point2d(matchLoc.x * ratio - border, matchLoc.y * ratio - border),
	//		cv::Point2d((matchLoc.x + templateimage_Scale.cols) * ratio + border, (matchLoc.y + templateimage_Scale.rows) * ratio + border));

	//	//cv::Rect2d bound = cv::Rect2d(cv::Point2d(matchLoc.x , matchLoc.y ), cv::Point2d(matchLoc.x  + templateimage.cols, matchLoc.y + templateimage.rows));
	//	bound.x = bound.x + x;
	//	bound.y = bound.y + y;
	//	maxLoc = cv::Point(0, 0);
	//	centerpoint = cv::Point2f(float(boundnew.x + boundnew.width / 2), float(boundnew.y + boundnew.height / 2)) + cv::Point2f(bound.tl());
	//	//match secondtime with enough ROI
	//	matchTemplate(imgsrc(bound), templateimage, scoreImg, cv::TM_CCOEFF_NORMED);
	//	minMaxLoc(scoreImg, &minVal, &score, 0, &maxLoc, cv::Mat());
	//	matchLoc = maxLoc;
	//	cv::Rect2d boundnew = cv::Rect2d(cv::Point2d(matchLoc.x, matchLoc.y), cv::Point2d(matchLoc.x + templateimage.cols, matchLoc.y + templateimage.rows));

	//	centerpoint = cv::Point2f(float(boundnew.x + boundnew.width / 2), float(boundnew.y + boundnew.height / 2)) + cv::Point2f(bound.tl());

	std::cout << "matchLoc:" << matchLoc << " center:" << centerpoint << " score:" << score << std::endl;

	//	cv::Mat tmat = imgsrc.clone();
	//	cv::cvtColor(tmat, tmat, cv::COLOR_GRAY2BGR);
	//	cv::rectangle(tmat, bound, cv::Scalar(0, 255, 0), 2);
	//	cv::namedWindow("r" + std::to_string(ii), cv::WINDOW_NORMAL);
	//	cv::imshow("r" + std::to_string(ii), tmat);

	//	cv::namedWindow("srr" + std::to_string(ii), cv::WINDOW_NORMAL);
	//	cv::imshow("srr" + std::to_string(ii), imgsrc(bound));

	//	cv::namedWindow("temp" + std::to_string(ii), cv::WINDOW_NORMAL);
	//	cv::imshow("temp" + std::to_string(ii), templateimage);

	//	cv::waitKey();
	//	tmat.release();

	//imgsrc_Scale.release();
	//templateimage_Scale.release();
	//}
#ifdef _DEBUG_LOG
			//std::string printtoconsole = "Image" + imagename + " Score: " + std::to_string(score) + "\n";
			//std::cout << printtoconsole;
#endif // _DEBUG_LOG
	scoreImg.release();
	templateimage.release();
	imgsrc.release();


	auto t2_extractroi = std::chrono::high_resolution_clock::now();
	auto ms_message = std::chrono::duration_cast<std::chrono::milliseconds>(t2_extractroi - t1_extractroi).count();
	std::cout << "temp: " << ms_message << "ms" << std::endl;

	if (score <= 0.8) { return false; }
	return true;
}

}
}