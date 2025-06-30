#include "pch.h"
#include <xvtCV/xvtTypes.h>
#include <xvtCV/xvtDefine.h>
#include <xvtBattery/CylinderSetting.h>
#include <xvtBattery/CTBatteryInspectorLower.h>

#define INI_SECTION L"INI_SECTION_"

using namespace xvt;
using namespace xvt::battery;

std::string folder_name = "./samples/CT/";
std::string folder_extra = ""; // "E:/Ky/XavisTech/Images/CT/CD/";
//std::string folder_name = "D:/GiangND/Battery/Cylinder/Images/CT/8255T OCT SK/Test/";
enum class BatteryType
{
	e21700Bead,
	e21700JR,
	e4680JR,
	e4680Bead,
	e4680Can,
};

std::map<BatteryType, std::string> listSetting = {
    {BatteryType::e21700JR   , "./samples/CT/2170_JR.ini"      },
    {BatteryType::e21700Bead , "./samples/CT/2170_BEADING.ini" },
    {BatteryType::e4680JR    , "./samples/CT/4680_JR.ini"      },
    {BatteryType::e4680Bead  , "./samples/CT/4680_BEADING.ini" },
    {BatteryType::e4680Can   , "./samples/CT/4680_CAN.ini"     },
};

std::vector<std::pair<BatteryType, std::string>> listFolder = {
    std::make_pair(BatteryType::e21700JR		, folder_name + "2170 JR"             ),
    std::make_pair(BatteryType::e21700Bead		, folder_name + "2170 Beading"        ),
    std::make_pair(BatteryType::e4680Bead		, folder_name + "4680 Beading"        ),
    std::make_pair(BatteryType::e4680JR			, folder_name + "4680 JR"             ),
	std::make_pair(BatteryType::e4680Can		, folder_name + "4680 Can"            ),
	std::make_pair(BatteryType::e4680Can		, folder_extra            ),
};

auto TestInspect(std::wstring folder, std::wstring setting, std::wstring part) -> xvt::InspectionResult
{
	xvt::InspectionResult IResult;
    if (folder.empty() || setting.empty()) return IResult;
	

	CTBatteryInspector mIsp = CTBatteryInspector();
	CTPropeties settingLoader(mIsp);
	auto section = INI_SECTION + part;
	auto lowerCase = ToLowerCase(part);
	settingLoader.LoadINI(setting, section);
	//settingLoader.SaveINI(setting + L"_backup.ini", section);

	VecString imgPaths;
	auto filter = folder + L"/*" + lowerCase + L"*16u*.tif";
	cv::glob(ToString(filter), imgPaths, true);

	for (auto& path : imgPaths)
	{
		cv::Mat img = xvt::ReadImage(ToWString(path), cv::ImreadModes::IMREAD_ANYDEPTH);

		xvt::battery::ERR_CODE result = xvt::battery::ERR_CODE::NA;
		auto tmp = std::make_shared<xvt::battery::CTBatteryInspectorResult>();
		mIsp.mDisplayMode = DisplayMode::POLE;
		result = mIsp.Inspect(img, dynamic_cast<xvt::battery::CTBatteryInspectorResult&>(*tmp));

		auto resImg = img;
		tmp->DrawResult(resImg);
		
		auto imgName = xvt::GetFileNameWithoutExtension(path);
		auto resName = ToWString(xvt::GetParentDirectory(path)) + L"/" + ToWString(imgName);

		if (1)
		{
			xvt::WriteImage(resName + +L"_res.png", resImg);
		}
		else
		{
			//xvt::WriteImage(resName + L"_org.png", img(mIsp.mIspBase.mRoi));
			xvt::WriteImage(resName + L"_res.png", resImg(mIsp.mIspBase.mRoi));
		}

		//tmp->Save(resName + L"_sum.csv", L"", true);
		//tmp->SaveDetailsCSV(resName + L"_detail.csv");

		IResult.Combine(tmp->GetResult(), tmp->GetMsg());
	}
	return IResult;
}

auto Test2(BatteryType type, std::wstring part)
{
	for (auto f : listFolder)
	{
		if (f.first == type)
		{
			auto resultLower = TestInspect(ToWString(f.second), ToWString(listSetting[type]), part);
			EXPECT_TRUE(!resultLower.IsER());
		}
	}
}

TEST(CTBatteryInspectTest, RemoveCathodes)
{
	CTBatteryInspectorResult res;
	auto& cathodes = res.mCathodePos;
	res.mAnodePoles.mPoles = {
		PoleInfo(cv::Point(), cv::Point{1,2}),
		PoleInfo(cv::Point(), cv::Point{5,2}),
		PoleInfo(cv::Point(), cv::Point{9,2}),
		PoleInfo(cv::Point(), cv::Point{13,2}),
		PoleInfo(cv::Point(), cv::Point{17,2}),
		PoleInfo(cv::Point(), cv::Point{21,2}),
		PoleInfo(cv::Point(), cv::Point{25,2}),
		PoleInfo(cv::Point(), cv::Point{29,2}),
	};
	cathodes = {
		cv::Point{1,2},//remove
		cv::Point{7,2},
		cv::Point{11,2},
		cv::Point{14,2},
		cv::Point{17,2},
		cv::Point{23,2},
		cv::Point{27,2},
		cv::Point{29,2},//remove
	};

	VecPoint expect = {
		cv::Point{7,2},
		cv::Point{11,2},
		cv::Point{14,2},
		cv::Point{17,2},
		cv::Point{23,2},
		cv::Point{27,2},
	};

	CTBatteryInspector isp;
	isp.RemoveCathodeByAnode(res);

	EXPECT_TRUE(cathodes.size() == expect.size());
	for (int i = 0; i < cathodes.size(); i++)
	{
		EXPECT_TRUE(cathodes[i].x = expect[i].x);
	}
}

TEST(CTBatteryInspectTest, Case_2170_BEADING_lower)
{
	Test2(BatteryType::e21700Bead, L"LOWER");
}

TEST(CTBatteryInspectTest, Case_2170_BEADING_upper)
{
	Test2(BatteryType::e21700Bead, L"UPPER");
}

TEST(CTBatteryInspectTest, Case_2170_JR_lower)
{
	Test2(BatteryType::e21700JR, L"LOWER");
}

TEST(CTBatteryInspectTest, Case_2170_JR_upper)
{
	Test2(BatteryType::e21700JR, L"UPPER");
}

TEST(CTBatteryInspectTest, Case_4680_Beading_lower)
{
	Test2(BatteryType::e4680Bead, L"LOWER");
}

TEST(CTBatteryInspectTest, Case_4680_Beading_upper)
{
	Test2(BatteryType::e4680Bead, L"UPPER");
}

TEST(CTBatteryInspectTest, Case_4680_JR_lower)
{
	Test2(BatteryType::e4680JR, L"LOWER");
}

TEST(CTBatteryInspectTest, Case_4680_JR_upper)
{
	Test2(BatteryType::e4680JR, L"UPPER");
}

TEST(CTBatteryInspectTest, Case_4680_CAN_lower)
{
	Test2(BatteryType::e4680Can, L"LOWER");
}

TEST(CTBatteryInspectTest, Case_4680_CAN_upper)
{
	Test2(BatteryType::e4680Can, L"UPPER");
}

#ifdef DEBUG_JSON
#include <nlohmann/json.hpp>
struct range_s
{
	bool enable;
	double lower;
	double upper;
};

using json = nlohmann::json;
void to_json(json& j, CTBatteryInspector& ctIsp, std::wstring pre) {
	auto& baseIsp = ctIsp.mIspBase;
	auto& beadingIsp = ctIsp.mIspBeading;
	auto& jrIsp = ctIsp.mIspJR;
	auto& poleIsp = ctIsp.mIspPole;
	auto& cathodeIsp = ctIsp.mIspCathode;
	auto& anodeIsp = ctIsp.mIspAnode;
	auto& displayPen = ctIsp.mDisplayPen;

	range_s validRange;
	/*j = json{
		{pre + L""			, ctIsp.mEnable},
		{pre + L"address"	, baseIsp.mRoi.x},
		{pre + L"age"		, baseIsp.mRoi.y},
		{pre + L"age"		, baseIsp.mRoi.width},
		{pre + L"age"		, baseIsp.mRoi.height},
		{pre + L"age"		, baseIsp.mThreshold},
		{pre + L"age"		, baseIsp.mDirection},
		{pre + L"age"		, baseIsp.mThreshold},
		{pre + L"age"		, baseIsp.mValidBatteryWidthRange.mIsEnable},
		{pre + L"age"		, validRange.lower},
		{pre + L"age"		, validRange.upper},
	};*/

	baseIsp.mValidBatteryWidthRange.mIsEnable = validRange.enable;
	baseIsp.mValidBatteryWidthRange.Set(validRange.lower, validRange.upper);
	ctIsp.mDisplayMode = xvt::battery::DisplayMode::POLE;
	ctIsp.mAvgSliceNo = 3;
	ctIsp.mSliceNo = 7;
	ctIsp.mPixelSize = 0.050;

	beadingIsp.mEnable = false;
	beadingIsp.mBeadingHeightMin = 50;
	beadingIsp.mD1StartPosition = 0.5;

	jrIsp.mJROffsetX = 20;
	jrIsp.mJROffsetY = 65;
	jrIsp.mHeight = 80;
	jrIsp.mCenterNeglectionWidth = 130;
	jrIsp.mBaseLineOffset = 0;
	jrIsp.mOneSidePoleNumber = 70;
	jrIsp.mEnableCheckLeaning = false;
	jrIsp.mLeaningThreshold = 0;
	jrIsp.mMinLeaningDistance = 0;

	cathodeIsp.mCathodeLineThresholdInner = 0;
	cathodeIsp.mCathodeLineThresholdMiddle = 0;
	cathodeIsp.mCathodeLineThresholdOuter = 0;
	cathodeIsp.mCathodeLineWindowSize = 25;

	anodeIsp.mAnodeThresholdInner = 1.2;
	anodeIsp.mAnodeThresholdMiddle = 2.2;
	anodeIsp.mAnodeThresholdOuter = 2.2;

	poleIsp.mPoleHeight = 80;
	poleIsp.mPolesProminenceThreshold = 1;
	poleIsp.mPolesDistanceRange.Set(3, 30);
	poleIsp.mSkipPolesDistance = 0;
	poleIsp.mAnode2CaseOffset = 0;
	poleIsp.mCathode2AnodeOffset = 0;
	poleIsp.mCathode2CaseOffset = 0;
	poleIsp.mValidCathode2AnodeRange.mIsEnable = true;
	poleIsp.mValidCathode2AnodeRange.Set(0.45, 4.0);
	poleIsp.mValidAnode2CaseRange.mIsEnable = true;
	poleIsp.mValidAnode2CaseRange.Set(0.8, 3.0);
	poleIsp.mValidCathode2CaseRange.mIsEnable = true;
	poleIsp.mValidCathode2CaseRange.Set(3.5, 7);
	poleIsp.mVariationAnode2Case = 3;

	displayPen.mFontScale = 0.4;
	displayPen.mSpace = 15;
	ctIsp.mGamma = 1.5;
}
#endif // DEBUG_JSON
