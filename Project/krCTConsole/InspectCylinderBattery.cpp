#include "xvtBattery/CylinderSettingDefine.h"

std::pair<cv::Mat, libjson::json> CylinderBattery::inspectLower(const cv::Mat& img0) {
	spdlog::info("inspectLower");
	libjson::json insp_result;
	cv::Mat img16u_padded, vis_img;
	float padding_minV = std::get<float>(xri::getProperty("minV", m_props));
	float padding_maxV = std::get<float>(xri::getProperty("maxV", m_props));
	int padding_size = std::get<int>(xri::getProperty("padding size", m_props));

	// mprops => xvt inspection struct
	CopyPropToLowerInspectionStructureVar(img0); 
	xf::ImageProcessor::CImageProcessorCylinderBatLowerCT lowerCTalgo;

	img16u_padded = make_16u_padded(img0, padding_minV, padding_maxV, padding_size);

	lowerCTalgo.ImageProcessing(img16u_padded, BatLowerInsp, BatLowerResult);
	BatLowerResult.resImg.copyTo(vis_img);
	insp_result["summary"] = BatLowerResult;
	return std::make_pair(vis_img, insp_result);
}

std::pair<cv::Mat, libjson::json> CylinderBattery::inspectUpper(const cv::Mat& img0) {
	spdlog::info("inspectUpper");
	libjson::json insp_result;
	cv::Mat img16u_padded, vis_img;
	float padding_minV = std::get<float>(xri::getProperty("minV", m_props));
	float padding_maxV = std::get<float>(xri::getProperty("maxV", m_props));
	int padding_size = std::get<int>(xri::getProperty("padding size", m_props));

	CopyPropToUpperInspectionStructureVar(img0);
	xf::ImageProcessor::CImageProcessorCylinderBatUpperCT upperCTalgo;

	img16u_padded = make_16u_padded(img0, padding_minV, padding_maxV, padding_size);
	/*
	if (BatUpperInsp.flip == true) {
		cv::flip(img16u_padded, img16u_padded, 0);
	}
	*/

	// img16u_padded = img0.clone();
	upperCTalgo.ImageProcessing(img16u_padded, BatUpperInsp, BatUpperResult);
	BatUpperResult.resImg.copyTo(vis_img);
	insp_result["summary"] = BatUpperResult;
	return std::make_pair(vis_img, insp_result);
}

cv::Mat CylinderBattery::make_16u_padded(const cv::Mat& img, float minV, float maxV, int padding_size = 256) {

	cv::Mat img_16u;
	auto alpha = 65535.0f / (maxV - minV);
	auto beta = -minV * alpha;
	img.convertTo(img_16u, CV_16UC1, alpha, beta);
	cv::Mat padded_16u;
	cv::copyMakeBorder(img_16u, padded_16u, padding_size, padding_size, 0, 0, cv::BORDER_CONSTANT, 0);
	return padded_16u;
}

// please change the dic as your dictionary and remove "typename Dic"
template<typename T, typename Dic>
void GetValue(T& pro, std::wstring name,Dic const& m_props)
{
	pro = std::get<T>(xri::getProperty(name, m_props));
}

//change the json type as your type
//load from variable to JSON props
void LoadProperties(json & m_props, CylinderCTInspection const& setting, std::wstring preFix = L"")
{
	GetValue(setting.UseBatteryInspection			, preFix + BATTERY_ENABLE			, m_props);
	GetValue(setting.RoiX							, preFix + BATTERY_ROI + L" X"		, m_props);
	GetValue(setting.RoiY							, preFix + BATTERY_ROI + L" Y"		, m_props);
	GetValue(setting.RoiWidth						, preFix + BATTERY_ROI + L" W"		, m_props);
	GetValue(setting.RoiHeight						, preFix + BATTERY_ROI + L" H"		, m_props);
	GetValue(setting.batteryThreshold				, preFix + BATTERY_THRESHOLD		, m_props);
	GetValue(setting.BatteryDirection				, preFix + BATTERY_DIRECTION		, m_props);

	GetValue(setting.EnableValidWidth				, preFix + BATTERY_VALID_WIDTH + L" Enable"	, m_props);
	GetValue(setting.cellSizeLowBound				, preFix + BATTERY_VALID_WIDTH + L" Min"	, m_props);
	GetValue(setting.cellSizeHighBound				, preFix + BATTERY_VALID_WIDTH + L" Max"	, m_props);
	GetValue(setting.displayMode					, preFix + BATTERY_DISPLAY_MODE			, m_props);
	GetValue(setting.pixelSize						, preFix + BATTERY_PIXEL_SIZE			, m_props);

	GetValue(setting.EnableBeading					, preFix + BEADING_ENABLE				, m_props);
	GetValue(setting.minBeadingHeight				, preFix + BEADING_MIN_HEIGHT			, m_props);
	GetValue(setting.D1StartPosition				, preFix + BEADING_D1_START_POS			, m_props);

	GetValue(setting.JR_ROIX						, preFix + JR_OFFSET_X                  , m_props);
	GetValue(setting.JR_ROIY						, preFix + JR_OFFSET_Y                  , m_props);
	GetValue(setting.poleRegionHeight				, preFix + JR_POLE_REGION_HEIGHT        , m_props);
	GetValue(setting.CenterNeglectionWidth			, preFix + JR_CENTER_NEGLECTION_WIDTH   , m_props);
	GetValue(setting.tolerenceShift					, preFix + JR_BASE_LINE_OFFSET          , m_props);
	GetValue(setting.OneSidePoleNumb				, preFix + JR_CHECK_ONE_SIDE_POLE_NO    , m_props);

	GetValue(setting.EnableLeaning					, preFix + JR_LEANING_ENABLE            , m_props);
	GetValue(setting.PoleLeaningThreshold			, preFix + JR_LEANING_THRESHOLD         , m_props);
	GetValue(setting.minLeaningDistance				, preFix + JR_LEANING_DISTANCE          , m_props);

	GetValue(setting.cathodeLineThredsholdInner		, preFix + CATHODE_THRESHOLD_INNER		, m_props);
	GetValue(setting.cathodeLineThredsholdMiddle	, preFix + CATHODE_THRESHOLD_MIDDLE		, m_props);
	GetValue(setting.cathodeLineThredsholdOuter		, preFix + CATHODE_THRESHOLD_OUTER		, m_props);
	GetValue(setting.cathodeProminence				, preFix + CATHODE_PROMINENCE			, m_props);
	GetValue(setting.cathodeLineWindowSize			, preFix + CATHODE_WINDOW_SIZE			, m_props);

	GetValue(setting.anodeThresholdInner			, preFix + ANODE_THRESHOLD_INNER		, m_props);
	GetValue(setting.anodeThresholdMiddle			, preFix + ANODE_THRESHOLD_MIDDLE		, m_props);
	GetValue(setting.anodeThresholdOuter			, preFix + ANODE_THRESHOLD_OUTER		, m_props);
	GetValue(setting.enableAnodeBaseLineRef			, preFix + ANODE_BASE_LINE_REF_ENABLE	, m_props);

	GetValue(setting.polesHeight					, preFix + POLE_HEIGHT                      , m_props);
	GetValue(setting.minProminence					, preFix + POLE_PROMINENCE                  , m_props);
	GetValue(setting.polesMinDistance				, preFix + POLE_DISTANCE      + L" Min"     , m_props);
	GetValue(setting.polesMaxDistance				, preFix + POLE_DISTANCE      + L" Max"     , m_props);
	GetValue(setting.skipPolesDistance				, preFix + POLE_SKIP_DISTANCE               , m_props);

	GetValue(setting.mCathode2AnodeOffset			, preFix + POLE_OFFSET_CATHODE_2_ANODE		, m_props);
	GetValue(setting.mAnode2CaseOffset				, preFix + POLE_OFFSET_ANODE_2_BASE			, m_props);
	GetValue(setting.mCathode2CaseOffset			, preFix + POLE_OFFSET_CATHODE_2_BASE		, m_props);

	GetValue(setting.EnableCathode2Case				, preFix + POLE_VALID_CATHODE_2_BASE + L" Enable"   , m_props);
	GetValue(setting.MinCathode2Case				, preFix + POLE_VALID_CATHODE_2_BASE + L" Min"      , m_props);
	GetValue(setting.MaxCathode2Case				, preFix + POLE_VALID_CATHODE_2_BASE + L" Max"      , m_props);

	GetValue(setting.EnableCathode2Anode			, preFix + POLE_VALID_CATHODE_2_ANODE + L" Enable"  , m_props);
	GetValue(setting.MinCathode2Anode				, preFix + POLE_VALID_CATHODE_2_ANODE + L" Min"     , m_props);
	GetValue(setting.MaxCathode2Anode				, preFix + POLE_VALID_CATHODE_2_ANODE + L" Max"     , m_props);

	GetValue(setting.EnableAnode2Case				, preFix + POLE_VALID_ANODE_2_BASE + L" Enable"     , m_props);
	GetValue(setting.MinAnode2Case					, preFix + POLE_VALID_ANODE_2_BASE + L" Min"        , m_props);
	GetValue(setting.MaxAnode2Case					, preFix + POLE_VALID_ANODE_2_BASE + L" Max"        , m_props);

	GetValue(setting.variationAnode2Case			, preFix + POLE_VALID_VARIATION_ANODE_2_BASE		, m_props);
	GetValue(setting.gamma							, preFix + RESULT_GAMMA								, m_props);
	
}

//change the json type as your type
//save from JSON props to variable
void SaveProperties(json const& m_props, CylinderCTInspection& setting, std::wstring preFix = L"")
{
	// lower Cylinder inspection parameter
	m_props[preFix + BATTERY_ENABLE]       = setting.UseBatteryInspection;
	m_props[preFix + BATTERY_ROI + L" X" ] = setting.RoiX;
	m_props[preFix + BATTERY_ROI + L" Y" ] = setting.RoiY;
	m_props[preFix + BATTERY_ROI + L" W" ] = setting.RoiWidth;
	m_props[preFix + BATTERY_ROI + L" H" ] = setting.RoiHeight;
	m_props[preFix + BATTERY_THRESHOLD	 ] = setting.batteryThreshold;
	m_props[preFix + BATTERY_DIRECTION	 ] = setting.BatteryDirection;

	m_props[preFix + BATTERY_VALID_WIDTH + L" Enable"] = setting.EnableValidWidth;
	m_props[preFix + BATTERY_VALID_WIDTH + L" Min"] = setting.cellSizeLowBound;
	m_props[preFix + BATTERY_VALID_WIDTH + L" Max"] = setting.cellSizeHighBound;
	m_props[preFix + BATTERY_DISPLAY_MODE] = setting.displayMode;
	m_props[preFix + BATTERY_PIXEL_SIZE]   = setting.pixelSize;

	m_props[preFix + BEADING_ENABLE      ] = setting.EnableBeading;
	m_props[preFix + BEADING_MIN_HEIGHT  ] = setting.minBeadingHeight;
	m_props[preFix + BEADING_D1_START_POS] = setting.D1StartPosition;

	m_props[preFix + JR_OFFSET_X                  ] = setting.JR_ROIX;
	m_props[preFix + JR_OFFSET_Y                  ] = setting.JR_ROIY;
	m_props[preFix + JR_POLE_REGION_HEIGHT        ] = setting.poleRegionHeight;
	m_props[preFix + JR_CENTER_NEGLECTION_WIDTH   ] = setting.CenterNeglectionWidth;
	m_props[preFix + JR_BASE_LINE_OFFSET          ] = setting.tolerenceShift;
	m_props[preFix + JR_CHECK_ONE_SIDE_POLE_NO    ] = setting.OneSidePoleNumb;

	m_props[preFix + JR_LEANING_ENABLE            ] = setting.EnableLeaning;
	m_props[preFix + JR_LEANING_THRESHOLD         ] = setting.PoleLeaningThreshold;
	m_props[preFix + JR_LEANING_DISTANCE          ] = setting.minLeaningDistance;

	m_props[preFix + CATHODE_THRESHOLD_INNER	  ] = setting.cathodeLineThredsholdInner;
	m_props[preFix + CATHODE_THRESHOLD_MIDDLE	  ] = setting.cathodeLineThredsholdMiddle;
	m_props[preFix + CATHODE_THRESHOLD_OUTER	  ] = setting.cathodeLineThredsholdOuter;
	m_props[preFix + CATHODE_PROMINENCE			  ] = setting.cathodeProminence;
	m_props[preFix + CATHODE_WINDOW_SIZE		  ] = setting.cathodeLineWindowSize;

	m_props[preFix + ANODE_THRESHOLD_INNER		  ] = setting.anodeThresholdInner;
	m_props[preFix + ANODE_THRESHOLD_MIDDLE		  ] = setting.anodeThresholdMiddle;
	m_props[preFix + ANODE_THRESHOLD_OUTER		  ] = setting.anodeThresholdOuter;
	m_props[preFix + ANODE_BASE_LINE_REF_ENABLE   ] = setting.enableAnodeBaseLineRef;

	m_props[preFix + POLE_HEIGHT                  ] = setting.polesHeight;
	m_props[preFix + POLE_PROMINENCE              ] = setting.minProminence;
	m_props[preFix + POLE_DISTANCE      + L" Min" ] = setting.polesMinDistance;
	m_props[preFix + POLE_DISTANCE      + L" Max" ] = setting.polesMaxDistance;
	m_props[preFix + POLE_SKIP_DISTANCE           ] = setting.skipPolesDistance;

	m_props[preFix + POLE_OFFSET_CATHODE_2_ANODE]		= setting.mCathode2AnodeOffset;
	m_props[preFix + POLE_OFFSET_ANODE_2_BASE] = setting.mAnode2CaseOffset;
	m_props[preFix + POLE_OFFSET_CATHODE_2_BASE] = setting.mCathode2CaseOffset;

	m_props[preFix + POLE_VALID_CATHODE_2_BASE + L" Enable"   ] = setting.EnableCathode2Case;
	m_props[preFix + POLE_VALID_CATHODE_2_BASE + L" Min"      ] = setting.MinCathode2Case;
	m_props[preFix + POLE_VALID_CATHODE_2_BASE + L" Max"      ] = setting.MaxCathode2Case;

	m_props[preFix + POLE_VALID_CATHODE_2_ANODE + L" Enable"  ] = setting.EnableCathode2Anode;
	m_props[preFix + POLE_VALID_CATHODE_2_ANODE + L" Min"     ] = setting.MinCathode2Anode;
	m_props[preFix + POLE_VALID_CATHODE_2_ANODE + L" Max"     ] = setting.MaxCathode2Anode;

	m_props[preFix + POLE_VALID_ANODE_2_BASE + L" Enable"     ] = setting.EnableAnode2Case;
	m_props[preFix + POLE_VALID_ANODE_2_BASE + L" Min"        ] = setting.MinAnode2Case;
	m_props[preFix + POLE_VALID_ANODE_2_BASE + L" Max"        ] = setting.MaxAnode2Case;

	m_props[preFix + POLE_VALID_VARIATION_ANODE_2_BASE ] = setting.variationAnode2Case;

	m_props[preFix + RESULT_GAMMA]         = setting.gamma;
}

void CylinderBattery::CopyPropToUpperInspectionStructureVar(const cv::Mat& img)
{
	SaveProperties(m_props, BatUpperInsp, L"Upper ");

#if 0
	//------------------------------------------------------------------------------------------
	BatUpperInsp.UseBatteryInspection = std::get<int>(xri::getProperty("UUseBatteryInspection", m_props));
	BatUpperInsp.StepNo = std::get<int>(xri::getProperty("UStep", m_props));
	BatUpperInsp.SliceNo = std::get<int>(xri::getProperty("USlice No", m_props));
	BatUpperInsp.Angle = std::get<int>(xri::getProperty("UAngle[deg]", m_props));
	BatUpperInsp.RoiX = std::get<int>(xri::getProperty("UROI X[pixel]", m_props));
	BatUpperInsp.RoiY = std::get<int>(xri::getProperty("UROI Y[pixel]", m_props));
	//BatUpperInsp.RoiY = img.cols / 2 - img.rows / 2 ;
	BatUpperInsp.RoiWidth = std::get<int>(xri::getProperty("UROI Width[pixel]", m_props));
	//BatUpperInsp.RoiWidth = img.cols;
	BatUpperInsp.RoiHeight = std::get<int>(xri::getProperty("UROI Height[pixel]", m_props));
//	BatUpperInsp.RoiHeight = img.rows;
	BatUpperInsp.batteryThreshold = std::get<int>(xri::getProperty("UBattery Threshold", m_props)); 

	BatUpperInsp.minBeadingHeight = std::get<int>(xri::getProperty("UMin Beading Height[pixel]", m_props));
	BatUpperInsp.D1StartPosition = (double)std::get<float>(xri::getProperty("UD1 Starting Index[pixel]", m_props));
	BatUpperInsp.poleRegionHeight = std::get<int>(xri::getProperty("UPole Region Height[pixel]", m_props));
	BatUpperInsp.PoleLeaningThreshold = std::get<int>(xri::getProperty("UThresh for Pole Leaning", m_props));
	BatUpperInsp.JR_ROIX = std::get<int>(xri::getProperty("UJR ROI X[pixel]", m_props));
	BatUpperInsp.JR_ROIY = std::get<int>(xri::getProperty("UJR ROI Y[pixel]", m_props));
	BatUpperInsp.CenterNeglectionWidth = std::get<int>(xri::getProperty("UNeglected Center Width[pixel]", m_props));
	BatUpperInsp.tolerenceShift = std::get<int>(xri::getProperty("UTolerance Shift[pixel]", m_props));

	BatUpperInsp.cathodeLineThredsholdInner = (double)std::get<float>(xri::getProperty("UCathode Line Inner Thresh", m_props));
	BatUpperInsp.cathodeLineThredsholdMiddle = (double)std::get<float>(xri::getProperty("UCathode Line Middle Thresh", m_props));
	BatUpperInsp.cathodeLineThredsholdOuter = (double)std::get<float>(xri::getProperty("UCathode Line Outer Thresh", m_props));
	BatUpperInsp.cathodeLineWindowSize = std::get<int>(xri::getProperty("UCathode Line Window Size", m_props));

	BatUpperInsp.polesHeight = std::get<int>(xri::getProperty("UPoles Height[pixel]", m_props));
	BatUpperInsp.minProminence = (double)std::get<float>(xri::getProperty("UMinProminence", m_props));

	BatUpperInsp.polesMinDistance = std::get<int>(xri::getProperty("UMin Poles Distance[pixel]", m_props));
	BatUpperInsp.polesMaxDistance = std::get<int>(xri::getProperty("UMax Poles Distance[pixel]", m_props));
	BatUpperInsp.anodeThresholdInner = (double)std::get<float>(xri::getProperty("UAnode Thresh for Inner Poles", m_props));
	BatUpperInsp.anodeThresholdMiddle = (double)std::get<float>(xri::getProperty("UAnode Thresh for Middle Poles", m_props));
	BatUpperInsp.anodeThresholdOuter = (double)std::get<float>(xri::getProperty("UAnode Thresh for Outer Poles", m_props));

	BatUpperInsp.OneSidePoleNumb = std::get<int>(xri::getProperty("UNum of Poles in One Side", m_props));
	BatUpperInsp.isCheckPoleNo = std::get<int>(xri::getProperty("UCheck Num of Poles", m_props));
	BatUpperInsp.isCheckPin = std::get<int>(xri::getProperty("UChecking Pin[0: No, 1: Yes]", m_props));
	BatUpperInsp.skipPolesDistance = std::get<int>(xri::getProperty("USkip Poles Distance[pixel]", m_props));

	BatUpperInsp.MinCathode2Case = std::get<float>(xri::getProperty("UMin Base->Cathode[mm]", m_props));
	BatUpperInsp.MaxCathode2Case = std::get<float>(xri::getProperty("UMax Base->Cathode[mm]", m_props));

	BatUpperInsp.mAnode2CaseOffset = std::get<int>(xri::getProperty("UBase->Anode offset[pixel]", m_props));
	BatUpperInsp.mCathode2AnodeOffset = std::get<int>(xri::getProperty("UAnode->Cathode offset[pixel]", m_props));
	BatUpperInsp.mCathode2CaseOffset = std::get<int>(xri::getProperty("UBase->Cathode offset[pixel]", m_props));
	BatUpperInsp.variationAnode2Case = std::get<int>(xri::getProperty("UvariationAnode2Case", m_props));

	BatUpperInsp.minLeaningDistance = std::get<int>(xri::getProperty("UMin Leaning Distance[pixel]", m_props));
	BatUpperInsp.pixelSize = (double)std::get<float>(xri::getProperty("UPixel Size", m_props));
	BatUpperInsp.gamma = std::get<int>(xri::getProperty("UGamma(10~99)", m_props));
	BatUpperInsp.displayMode = std::get<int>(xri::getProperty("UDisplaymode", m_props));
	// BatUpperInsp.flip = std::get<bool>(xri::getProperty("UFlip", m_props));

	BatUpperInsp.EnableCathode2Case = getProperty<bool>("UEnable Cathode to Base");
	BatUpperInsp.EnableAnode2Case = getProperty<bool>("UEnable Anode to Base");
	BatUpperInsp.EnableCathode2Anode = getProperty<bool>("UEnable Anode to Cathode");
	BatUpperInsp.EnableBeading = getProperty<bool>("UEnable Beading");
#endif
}

void CylinderBattery::CopyPropToLowerInspectionStructureVar(const cv::Mat& img)
{
	SaveProperties(m_props, BatLowerInsp, L"Lower ");

#if 0
	BatLowerInsp.UseBatteryInspection = std::get<int>(xri::getProperty("LUseBatteryInspection", m_props));
	BatLowerInsp.BatteryDirection = 1;
	BatLowerInsp.StepNo = std::get<int>(xri::getProperty("LStep", m_props));
	BatLowerInsp.SliceNo = std::get<int>(xri::getProperty("LSlice No", m_props));
	BatLowerInsp.Angle = std::get<int>(xri::getProperty("LAngle[deg]", m_props));
	BatLowerInsp.InspItem = std::get<int>(xri::getProperty("LInspecting Item", m_props));
	BatLowerInsp.RoiX = std::get<int>(xri::getProperty("LROI X[pixel]", m_props));
	BatLowerInsp.RoiY = std::get<int>(xri::getProperty("LROI Y[pixel]", m_props));
	BatLowerInsp.D1StartPosition = std::get<float>(xri::getProperty("LD1 Starting Index[pixel]", m_props));

	//BatLowerInsp.RoiY = img.cols / 2 - img.rows / 2;
	BatLowerInsp.RoiWidth = std::get<int>(xri::getProperty("LROI Width[pixel]", m_props));
	BatLowerInsp.RoiHeight = std::get<int>(xri::getProperty("LROI Height[pixel]", m_props));
	BatLowerInsp.batteryThreshold = std::get<int>(xri::getProperty("LBattery Threshold", m_props));
	BatLowerInsp.minBeadingHeight = std::get<int>(xri::getProperty("LMin Beading Height[pixel]", m_props));
	BatLowerInsp.poleRegionHeight = std::get<int>(xri::getProperty("LPole Region Height[pixel]", m_props));
	BatLowerInsp.PoleLeaningThreshold = std::get<int>(xri::getProperty("LThresh for Pole Leaning", m_props));
	BatLowerInsp.JR_ROIX = std::get<int>(xri::getProperty("LJR ROI X[pixel]", m_props));
	BatLowerInsp.JR_ROIY = std::get<int>(xri::getProperty("LJR ROI Y[pixel]", m_props));
	
	BatLowerInsp.CenterNeglectionWidth = std::get<int>(xri::getProperty("LNeglected Center Width[pixel]", m_props));
	BatLowerInsp.tolerenceShift = std::get<int>(xri::getProperty("LTolerance Shift[pixel]", m_props));

	BatLowerInsp.cathodeLineThredsholdInner = std::get<float>(xri::getProperty("LCathode Line Inner Thresh", m_props));
	BatLowerInsp.cathodeLineThredsholdMiddle = std::get<float>(xri::getProperty("LCathode Line Middle Thresh", m_props));
	BatLowerInsp.cathodeLineThredsholdOuter = std::get<float>(xri::getProperty("LCathode Line Outer Thresh", m_props));
	BatLowerInsp.cathodeLineWindowSize = std::get<int>(xri::getProperty("LCathode Line Window Size", m_props));
	BatLowerInsp.polesHeight = std::get<int>(xri::getProperty("LPoles Height[pixel]", m_props));
	BatLowerInsp.minProminence = std::get<float>(xri::getProperty("LMinProminence", m_props));

	BatLowerInsp.polesMinDistance = std::get<int>(xri::getProperty("LMin Poles Distance[pixel]", m_props));
	BatLowerInsp.polesMaxDistance = std::get<int>(xri::getProperty("LMax Poles Distance[pixel]", m_props));
	BatLowerInsp.anodeThresholdInner = (double)std::get<float>(xri::getProperty("LAnode Thresh for Inner Poles", m_props));
	BatLowerInsp.anodeThresholdMiddle = (double)std::get<float>(xri::getProperty("LAnode Thresh for Middle Poles", m_props));
	BatLowerInsp.anodeThresholdOuter = (double)std::get<float>(xri::getProperty("LAnode Thresh for Outer Poles", m_props));

	BatLowerInsp.OneSidePoleNumb = std::get<int>(xri::getProperty("LNum of Poles in One Side", m_props));
	BatLowerInsp.isCheckPoleNo = std::get<int>(xri::getProperty("LCheck Num of Poles", m_props));
	BatLowerInsp.isCheckPin = std::get<int>(xri::getProperty("LChecking Pin[0: No, 1: Yes]", m_props));
	BatLowerInsp.skipPolesDistance = std::get<int>(xri::getProperty("LSkip Poles Distance[pixel]", m_props));

	BatLowerInsp.MinCathode2Anode = (double)std::get<float>(xri::getProperty("LMin Anode->Cathode[mm]", m_props));
	BatLowerInsp.MaxCathode2Anode = (double)std::get<float>(xri::getProperty("LMax Anode->Cathode[mm]", m_props));
	BatLowerInsp.MinAnode2Case = (double)std::get<float>(xri::getProperty("LMin Base->Anode[mm]", m_props));
	BatLowerInsp.MaxAnode2Case = (double)std::get<float>(xri::getProperty("LMax Base->Anode[mm]", m_props));

	BatLowerInsp.MinCathode2Case = (double)std::get<float>(xri::getProperty("LMin Base->Cathode[mm]", m_props));
	BatLowerInsp.MaxCathode2Case = (double)std::get<float>(xri::getProperty("LMax Base->Cathode[mm]", m_props));

	BatLowerInsp.variationAnode2Case = (double)std::get<float>(xri::getProperty("LVariation Base->Anode[mm]", m_props));
	BatLowerInsp.mAnode2CaseOffset = std::get<int>(xri::getProperty("LBase->Anode offset[pixel]", m_props));
	BatLowerInsp.mCathode2AnodeOffset = std::get<int>(xri::getProperty("LAnode->Cathode offset[pixel]", m_props));
	BatLowerInsp.mCathode2CaseOffset = std::get<int>(xri::getProperty("LBase->Cathode offset[pixel]", m_props));
	BatLowerInsp.minLeaningDistance = std::get<int>(xri::getProperty("LMin Leaning Distance[pixel]", m_props));
	BatLowerInsp.pixelSize = std::get<float>(xri::getProperty("LPixel Size", m_props));
	// BatLowerInsp.additionalLine = std::get<int>(xri::getProperty("LadditionalLine[pixel]", m_props));
	BatLowerInsp.gamma = std::get<int>(xri::getProperty("LGamma(10~99)", m_props));
	BatLowerInsp.displayMode = std::get<int>(xri::getProperty("LDisplaymode", m_props));
	// BatLowerInsp.flip = std::get<bool>(xri::getProperty("LFlip", m_props));

	/*_props["LEnable Cathode to Base"] = false;
	m_props["LEnable Anode to Base"] = false;
	m_props["LEnable Anode to Cathode"] = false;
	m_props["LEnable Beading"] = false;
	m_props["LEnable Valid Width"] = false;*/
	BatLowerInsp.EnableCathode2Case = getProperty<bool>("LEnable Cathode to Base");
	BatLowerInsp.EnableAnode2Case = getProperty<bool>("LEnable Anode to Base");
	BatLowerInsp.EnableCathode2Anode = getProperty<bool>("LEnable Anode to Cathode");
	BatLowerInsp.EnableBeading = getProperty<bool>("LEnable Beading");
#endif
}

void CylinderBattery::BuildDefaultProperties() {
	BuildLowerProperties();
	BuildUpperProperties();
	BuildZProperties();
	BuildBeadingProperties();
}

void CylinderBattery::BuildBeadingProperties() {
	m_props["flip Y"] = false;
	m_props["is Cap"] = true;
	m_props["ref pos threshold ratio"] = 2.5f;
	m_props["find localmax threshold"] = 0.00065f;
	m_props["mean gap search offset(mm)"] = 2.5f;
	m_props["top distance offset(mm)"] = 2.0f;
	m_props["top 1 roi ratio"] = 0.75f;
	m_props["top 2 threshold"] = 0.f;
	m_props["top 2 distance offset(mm)"] = 1.5f;
	m_props["top 3 roi offset(mm)"] = 2.5f;
	m_props["mid distance offset(mm)"] = 1.0f;
	m_props["mid 3 roi ratio"] = 0.5f;
	m_props["mid 3 threshold ratio"] = 1.125f;
	m_props["mid 4 roi ratio"] = 1.2f;
	m_props["mid 4 thickness criteria[mm]"] = 3.5f;
	m_props["mid 5 roi ratio"] = 1.3f;
	m_props["mid 5 threshold ratio"] = 1.0f;
	m_props["mid 5 minimum area"] = 1000;
	m_props["btm 6 offset[pixel]"] = 3;
	m_props["save_16u"] = true;
	m_props["beading1_min[mm]"] = 0.0f;
	m_props["beading1_max[mm]"] = 5.0f;
	m_props["beading2_min[mm]"] = 0.0f;
	m_props["beading2_max[mm]"] = 5.0f;
	m_props["beading3_min[mm]"] = 0.0f;
	m_props["beading3_max[mm]"] = 5.0f;
	m_props["beading4_min[mm]"] = 0.0f;
	m_props["beading4_max[mm]"] = 5.0f;
	m_props["beading5_min[mm]"] = 0.0f;
	m_props["beading5_max[mm]"] = 5.0f;
	m_props["beading6_min[mm]"] = 0.0f;
	m_props["beading6_max[mm]"] = 5.0f;
}

void CylinderBattery::BuildZProperties() {
	m_props["intensity ratio for can detection"] = 0.5f;
	m_props["can radius(mm)"] = 23.0f;
	m_props["JR inner R(mm)"] = 2.0f;
	m_props["rot. step(deg)"] = 1.0f;
	m_props["no of pilot profile"] = 8;
	m_props["intensity ratio for can detection"] = 0.5f;
	m_props["intensity ratio for turn detection"] = 0.75f;
	// m_props["Z score for outlier"] = 1.65f; //90% z-score
	// m_props["Z score for outlier"] = 1.96f; //95%
	m_props["Z score for outlier"] = 2.58f; //99% 
	m_props["SEG_AVG_LINES"] = 5;
	m_props["SEG_TH_SCALE"] = 0.001f;

	// nhj test /////////////////////////////
	m_props["nhj_thresh_ratio_cathode_inner"] = 0.8f;
	m_props["nhj_thresh_ratio_anode_inner"] = 0.6f;
	m_props["nhj_thresh_ratio_cathode_outer"] = 0.8f;
	m_props["nhj_thresh_ratio_anode_outer"] = 0.5f;
	m_props["nhj_sigma_anode"] = 0.444f;
	m_props["nhj_sigma_cathode"] = 0.888f;
	m_props["save_16u"] = true;
}

void CylinderBattery::BuildUpperProperties()
{
	CylinderCTInspection tmp;
	tmp.BatteryDirection = 1;
	tmp.RoiX = 0;
	tmp.RoiY = 280;
	tmp.RoiWidth = 1000;
	tmp.RoiHeight = 270;
	tmp.batteryThreshold = 65;
	tmp.EnableValidWidth = true;
	//beading
	tmp.EnableBeading = false;
	tmp.isCheckBeading = false;
	tmp.minBeadingHeight = 0;
	tmp.D1StartPosition = 0.5f;
	//JR
	tmp.JR_ROIX = 15;
	tmp.JR_ROIY = 20;
	tmp.poleRegionHeight = 150;
	tmp.CenterNeglectionWidth = 120;
	tmp.tolerenceShift = 0;
	tmp.OneSidePoleNumb = 0;
	tmp.isCheckPoleNo = 0;
	//pole leaning
	tmp.EnableLeaning = 0;
	tmp.PoleLeaningThreshold = 255;
	tmp.minLeaningDistance = 0;
	//Pole region
	tmp.polesHeight = 40;
	tmp.minProminence = 1.0;
	tmp.polesMinDistance = 3;
	tmp.polesMaxDistance = 15;
	tmp.skipPolesDistance = 0;
	//cathode2case
	tmp.EnableCathode2Case = true;
	tmp.MinCathode2Case = 1.5f;
	tmp.MaxCathode2Case = 6.5f;
	//cathode2anode
	tmp.EnableCathode2Anode = false;
	tmp.MinCathode2Anode = 0.5f;
	tmp.MaxCathode2Anode = 1.8f;
	//Anode 2 case
	tmp.EnableAnode2Case = false;
	tmp.MinAnode2Case = 1.0f;
	tmp.MaxAnode2Case = 5.0f;
	//pole length variant
	tmp.variationAnode2Case = 3.0f;
	//offset
	tmp.mCathode2AnodeOffset = 0;
	tmp.mAnode2CaseOffset = 0;
	tmp.mCathode2CaseOffset = 0;
	//Cathode threshold
	tmp.cathodeLineThredsholdInner = 0.0f;
	tmp.cathodeLineThredsholdMiddle = 0.0f;
	tmp.cathodeLineThredsholdOuter = 0.0f;
	tmp.cathodeLineWindowSize = 5;
	//Anode threshold
	tmp.anodeThresholdInner = 3.0f;
	tmp.anodeThresholdMiddle = 3.0f;
	tmp.anodeThresholdOuter = 4.0f;

	//center pin
	tmp.isCheckPin = 0;
	//display
	tmp.displayMode = 7;
	tmp.pixelSize = 0.05f;
	tmp.gamma = 1.0;
	tmp.SliceNo = 0;
	tmp.StepNo = 0;
	LoadProperties(m_props, tmp, L"Upper");

#if 0
	// Upper Cylinder inspection parameter - by SH_Park
	m_props["UUseBatteryInspection"] = 1;
	m_props["UStep"]                 = 512;
	m_props["USlice No"]             = 1;
	m_props["UAngle[deg]"]           = 108;
	m_props["UROI X[pixel]"]         = 0;
	m_props["UROI Y[pixel]"]         = 280;
	m_props["UROI Width[pixel]"]     = 1000;
	m_props["UROI Height[pixel]"]    = 220;

	m_props["UBattery Threshold"]    = 80;
	m_props["UMin Beading Height[pixel]"] = 50;
	m_props["UD1 Starting Index[pixel]"] = 0.5f;
	m_props["UPole Region Height[pixel]"] = 100;
	m_props["UThresh for Pole Leaning"] = 0;

	m_props["UJR ROI X[pixel]"]         = 10;
	m_props["UJR ROI Y[pixel]"]         = 20;
	m_props["UNeglected Center Width[pixel]"] = 120;
	m_props["UTolerance Shift[pixel]"]     = 0;

	m_props["UCathode Line Inner Thresh"]  = 0.0f;
	m_props["UCathode Line Middle Thresh"] = 0.0f;
	m_props["UCathode Line Outer Thresh"]  = 0.0f;
	m_props["UCathode Line Window Size"]   = 5;
	m_props["UPoles Height[pixel]"]        = 40;
	m_props["UMinProminence"]			   = 0.1f;
	//m_props["UPole Detection Thresh"]      = 1.0f;

	m_props["UMin Poles Distance[pixel]"]  = 3;
	m_props["UMax Poles Distance[pixel]"]  = 15;

	m_props["UAnode Thresh for Inner Poles"]  = 3.0f;
	m_props["UAnode Thresh for Middle Poles"] = 3.0f;
	m_props["UAnode Thresh for Outer Poles"]  = 3.0f;
	m_props["UNum of Poles in One Side"]  = 70;
	m_props["UCheck Num of Poles"]        = 55;
	m_props["UChecking Pin[0: No, 1: Yes]"] = 1;

	m_props["USkip Poles Distance[pixel]"] = 0;
	m_props["UMin Base->Cathode[mm]"]      = 3.5f;
	m_props["UMax Base->Cathode[mm]"]      = 5.0f;

	m_props["UBase->Anode offset[pixel]"]    = 0;
	m_props["UAnode->Cathode offset[pixel]"] = 0;
	m_props["UBase->Cathode offset[pixel]"]  = 0;
	m_props["UvariationAnode2Case"] = 3;

	m_props["UPixel Size"] = 0.050f;
	m_props["UGamma(10~99)"] = 10;

	m_props["UMin Leaning Distance[pixel]"] = 0;
	
	m_props["UDisplaymode"]					= 7;
	m_props["UFlip"] = false;
	m_props["padding size"] = 256;
	m_props["save_16u"] = true;

	//2024-11-15
	m_props["UEnable Cathode to Base"] = true;
	m_props["UEnable Anode to Base"] = false;
	m_props["UEnable Anode to Cathode"] = false;
	m_props["UEnable Beading"] = true;
	m_props["UEnable Valid Width"] = false;
	//m_props["LEnable Valid width"] = false;
#endif
}

void CylinderBattery::BuildLowerProperties()
{
	CylinderCTInspection tmp;
    tmp.BatteryDirection = 1;
    tmp.RoiX = 0;
    tmp.RoiY = 280;
    tmp.RoiWidth = 1000;
    tmp.RoiHeight = 270;
    tmp.batteryThreshold = 65;
    tmp.EnableValidWidth = true;
    //beading
    tmp.EnableBeading = false;
    tmp.isCheckBeading = false;
    tmp.minBeadingHeight = 0;
    tmp.D1StartPosition = 0.5f;
    //JR
    tmp.JR_ROIX = 15;
    tmp.JR_ROIY = 20;
    tmp.poleRegionHeight = 150;
    tmp.CenterNeglectionWidth = 120;
    tmp.tolerenceShift = 0;
    tmp.OneSidePoleNumb = 0;
    tmp.isCheckPoleNo = 0;
    //pole leaning
    tmp.EnableLeaning = 0;
    tmp.PoleLeaningThreshold = 255;
    tmp.minLeaningDistance = 0;
    //Pole region
    tmp.polesHeight = 40;
    tmp.minProminence = 1.0;
    tmp.polesMinDistance = 3;
    tmp.polesMaxDistance = 15;
    tmp.skipPolesDistance = 0;
    //cathode2case
    tmp.EnableCathode2Case = true;
    tmp.MinCathode2Case = 1.5f;
    tmp.MaxCathode2Case = 6.5f;
    //cathode2anode
    tmp.EnableCathode2Anode = true;
    tmp.MinCathode2Anode = 0.5f;
    tmp.MaxCathode2Anode = 1.8f;
    //Anode 2 case
    tmp.EnableAnode2Case = true;
    tmp.MinAnode2Case = 1.0f;
    tmp.MaxAnode2Case = 5.0f;
    //pole length variant
    tmp.variationAnode2Case = 3.0f;
    //offset
    tmp.mCathode2AnodeOffset = 0;
    tmp.mAnode2CaseOffset = 0;
    tmp.mCathode2CaseOffset = 0;
    //Cathode threshold
    tmp.cathodeLineThredsholdInner = 0.0f;
    tmp.cathodeLineThredsholdMiddle = 0.0f;
    tmp.cathodeLineThredsholdOuter = 0.0f;
    tmp.cathodeLineWindowSize = 5;
    //Anode threshold
    tmp.anodeThresholdInner = 3.0f;
    tmp.anodeThresholdMiddle = 3.0f;
    tmp.anodeThresholdOuter = 4.0f;

    //center pin
    tmp.isCheckPin = 0;
    //display
    tmp.displayMode = 7;
    tmp.pixelSize = 0.05f;
    tmp.gamma = 1.0;
    tmp.SliceNo = 0;
    tmp.StepNo = 0;

	LoadProperties(m_props, tmp, L"Lower");

#if 0
	// lower Cylinder inspection parameter
	m_props["LUseBatteryInspection"] = 1;									 
	m_props["LStep"]                 = 512;
	m_props["LSlice No"]             = 1;										 
	m_props["LAngle[deg]"]           = 74;										 
	m_props["LInspecting Item"]      = 0;											 
	//	m_props["Battery Direction"];											 
	m_props["LROI X[pixel]"]         = 0;
	m_props["LROI Y[pixel]"]		 = 280;
	m_props["LROI Width[pixel]"]     = 1000;									 
	m_props["LROI Height[pixel]"]    = 270;									 

	m_props["LBattery Threshold"]    = 65;											 
	m_props["LMin Beading Height[pixel]"] = 50;
	m_props["LD1 Starting Index[pixel]"] = 0.5f;
	m_props["LPole Region Height[pixel]"] = 100;
	m_props["LThresh for Pole Leaning"] = 0;

	m_props["LJR ROI X[pixel]"]         = 15;											 
	m_props["LJR ROI Y[pixel]"]         = 20;											 
	m_props["LNeglected Center Width[pixel]"] = 120;								 
	m_props["LTolerance Shift[pixel]"] = 0;

	m_props["LCathode Line Inner Thresh"] = 0.0f;
	m_props["LCathode Line Middle Thresh"] = 0.0f;
	m_props["LCathode Line Outer Thresh"] = 0.0f;
	m_props["LCathode Line Window Size"] = 5;
	m_props["LPoles Height[pixel]"] = 40;
	m_props["LMinProminence"] = 0.1f;

	m_props["LMin Poles Distance[pixel]"] = 3;
	m_props["LMax Poles Distance[pixel]"] = 15;

	m_props["LAnode Thresh for Inner Poles"] = 3.0f;
	m_props["LAnode Thresh for Middle Poles"] = 3.5f;
	m_props["LAnode Thresh for Outer Poles"] = 4.0f;
	m_props["LNum of Poles in One Side"] = 70;
	m_props["LCheck Num of Poles"] = 55;
	m_props["LChecking Pin[0: No, 1: Yes]"] = 1;

	m_props["LSkip Poles Distance[pixel]"] = 0;
	m_props["LMin Base->Cathode[mm]"] = 1.5f;
	m_props["LMax Base->Cathode[mm]"] = 6.5f;
	m_props["LMin Anode->Cathode[mm]"] = 0.5f;
	// m_props["LMax Anode->Cathode[mm]"] = 3.0f;
	m_props["LMax Anode->Cathode[mm]"] = 1.8f;
	m_props["LMin Base->Anode[mm]"] = 1.0f;
	m_props["LMax Base->Anode[mm]"] = 5.0f;
	m_props["LBase->Anode offset[pixel]"] = 0;
	m_props["LAnode->Cathode offset[pixel]"] = 0;
	m_props["LBase->Cathode offset[pixel]"] = 0;

	m_props["LVariation Base->Anode[mm]"] = 3.0f;

	m_props["LPixel Size"] = 0.05f;
	m_props["LGamma(10~99)"] = 10;

	m_props["LMin Leaning Distance[pixel]"] = 0;

	m_props["LDisplaymode"] = 7;

	//2024-11-15
	m_props["LEnable Cathode to Base"] = true;
	m_props["LEnable Anode to Base"] = true;
	m_props["LEnable Anode to Cathode"] = true;
	m_props["LEnable Beading"] = false;
	m_props["LEnable Valid Width"] = false;
	//m_props["LEnable Valid width"] = false;
#endif

	m_props["LFlip"] = false;
	m_props["padding size"] = 256;
	m_props["save_16u"] = true;

	// 검사팀 극검사 알고리즘 파라메터
	m_props["SEG_AVG_LINES"] = 5;
	m_props["SEG_TH_SCALE"] = 0.001f;
	m_props["SEG_GAP"] = 7.0f;

	m_props["CATHODE_SEARCH_OFFSET"] = 0;
	m_props["CATHODE_AVG_LINES"] = 100;
	m_props["CATHODE_CUTOFF_SIGMA"] = 2.58f;
	m_props["EIGEN_MASK_SIZE"] = 3;
	m_props["EIGEN_MASK_TH"] = 7;
	m_props["EIGEN_MIN_GAP_RATIO"] = 0.53f;
	m_props["CATHODE_INTENSITY_CUTOFF_RATIO"] = 0.75f;
	m_props["ANODE_MIN_LENGTH_PX"] = 5;
	m_props["ANODE_LENGTH_RATIO"] = 0.5f;
}

libjson::json CylinderBattery::getPropertiesAsJson() {
	libjson::json json_props;
	for (const auto& p : m_props) {
		size_t type_id = p.second.index();
		// spdlog::trace("getPropertiesAsJson, {}, {}, ", p.first, type_id);
		switch (type_id) {
			case 0: {
				bool v = std::get<bool>(p.second);
				json_props.push_back({ {"name",p.first}, {"type",type_id}, {"value", v} });
				// spdlog::info("val = {} ", v);
				break;
			}
			case 1: {
				int v = std::get<int>(p.second);
				json_props.push_back({ {"name",p.first}, {"type",type_id}, {"value", v} });
				break;
			}
			case 2: {
				float v = std::get<float>(p.second);
				json_props.push_back({ {"name",p.first}, {"type",type_id}, {"value", v} });
				break;
			}
			case 3: {
				std::string v = std::get<std::string>(p.second);
				json_props.push_back({ {"name",p.first}, {"type",type_id}, {"value", v} });
				break;
			}
			case 4: {
				cv::Scalar v = std::get<cv::Scalar>(p.second);
				libjson::json arr_v = { static_cast<uint8_t>(v.val[0]), static_cast<uint8_t>(v.val[1]), static_cast<uint8_t>(v.val[2]) };
				json_props.push_back({ {"name",p.first}, {"type",type_id}, {"value", arr_v} });
				break;
			}
		}
	}
	return json_props;
}

void CylinderBattery::setPropertiesFromJson(const libjson::json& json_arr) {
	if (json_arr.is_array()) {
		for (const auto& e : json_arr) {
			std::string name = e["name"];
			int type_id = e["type"];
			spdlog::trace("setPropertiesFromJson, {}, {}, {}", name, type_id, e.dump());
			switch (type_id) {
				case 0: {
					if (e["value"].is_boolean()) {
						m_props[name] = e["value"].get<bool>();
					}
					else {
						std::string value = e["value"].get<std::string>();
						if (xri::tolower(value).compare("true") == 0) {
							m_props[name] = true;
						}
						else {
							m_props[name] = false;
						}
					}
					break;
				}
				case 1: {
					if (e["value"].is_number_integer()) {
						m_props[name] = e["value"].get<int>();
					}
					else if (e["value"].is_number_float()) {
						m_props[name] = static_cast<int>(e["value"].get<float>());
					}
					else {
						std::string value = e["value"].get<std::string>();
						m_props[name] = std::atoi(value.c_str());
					}
					break;
				}
				case 2: {
					if (e["value"].is_number_float()) {
						m_props[name] = e["value"].get<float>();
					}
					else {
						std::string value = e["value"].get<std::string>();
						m_props[name] = static_cast<float>(std::atof(value.c_str()));
					}
					break;
				}
				case 3: {
					std::string v = e["value"].get<std::string>();
					m_props[name] = v;
					break;
				}
				case 4: {
					uint8_t v0 = e["value"][0];
					uint8_t v1 = e["value"][1];
					uint8_t v2 = e["value"][2];
					m_props[name] = cv::Scalar(v0, v1, v2);
					break;
				}
			}
		}
	}
	else {
		throw std::runtime_error("Json is not array");
	}
}

// 현재시간을 포함하여 파일 이름을 만들고 미리 지정된 폴더에 저장한다.
void CylinderBattery::saveImage(const std::string& name, const cv::Mat& img){
	std::string fn = std::format("{}/{}", getDir(), name);
	if (img.rows > 0 && img.cols > 0) {
		bool ret = cv::imwrite(fn, img);
		if (ret == false) {
			spdlog::error("Failed to save image. fn={}, name");
		}
	}
	else {
		spdlog::error("There is an error with the size of the image. rows={}, cols={}", img.rows, img.cols);
	}
}

// 같은 기능을 하지만 비동기로 처리한다. throughtput이 빨라짐. 다만 나중에 future.get() 실행해줘야 함.
// 클래스가 삭제 될때, async task 가 있으면 기다림.
void CylinderBattery::saveImageAsync(const std::string& name, const cv::Mat& img) {
	std::future<void> fut = std::async(std::launch::async, &CylinderBattery::saveImage, this, name, img);
	m_async_save_tasks.push_back(std::move(fut));
	return;
}

void CylinderBattery::savePropsAsJson(const std::string& name) {
	std::string fn = std::format("{}/{}", getDir(), name);
	std::future<void> fut = std::async(std::launch::async, [this](const std::string fn) {
		std::ofstream file(fn);
		file << getPropertiesAsJson().dump(4);
		file.close();
		}, fn);
	m_async_save_tasks.push_back(std::move(fut));
}

CylinderBattery::~CylinderBattery() {
	for (auto& task : m_async_save_tasks) {
		task.get();
	}
}