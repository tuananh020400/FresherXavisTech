#include "xvtBattery/CylinderSetting.h"
#include "xvtBattery/CylinderSettingDefine.h"

DECLARE_ENUM_MAP_DATA(CylinderType) {
    { CylinderType::PROJECT_2D   , "Cylinder 2D X-Ray image"},
    { CylinderType::PROJECT_2D_JR, "Cylinder 2D JR X-Ray image" },
    { CylinderType::SLICE_3D     , "Cylinder CT" }
};
DECLARE_CONVERT_ENUM_FUNCS(CylinderType);
namespace xvt {

TypedProperty<xvt::battery::BatteryInspectorBase>::TypedProperty(Type_Trait& data, double& pixelSize)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait(pixelSize) }
{
    auto& ptr = mData.get();

    static_cast<PropertyList&>(*this) = {
        //CreateShareProperty(USE_INSPECT          , upperPtr., mDefaultValue.mEnable)
        //CreateShareProperty(INSPECT_ITEMS          , basePtr.mEnable                               , mDefaultValue.mEnable)
        CreateShareProperty(BATTERY_ROI                    , ptr.mRoi                                , cv::Rect()),
        CreateShareProperty(BATTERY_THRESHOLD              , ptr.mThreshold                          , 0),
        CreateShareProperty(BATTERY_DIRECTION              , ptr.mDirection                          , 0),
        CreateShareProperty(BATTERY_VALID_WIDTH            , ptr.mValidBatteryWidthRange             , xvt::Ranged(0,0)),
    };
}

TypedProperty<xvt::battery::BatteryInspectorBeading>::TypedProperty(Type_Trait& data, double& pixelSize)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait(pixelSize) }
{
    auto& beadPtr = mData.get();

        //Beading
    static_cast<PropertyList&>(*this) = {
        CreateShareProperty(BEADING_ENABLE                 , beadPtr.mEnable                          , false),
        CreateShareProperty(BEADING_MIN_HEIGHT             , beadPtr.mBeadingHeightMin                , 0),
        CreateShareProperty(BEADING_D1_START_POS           , beadPtr.mD1StartPosition                 , 0.f),
    };
}

TypedProperty<xvt::battery::BatteryInspectorJR>::TypedProperty(Type_Trait& data, double& pixelSize)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait(pixelSize) }
{
    auto& jrPtr = mData.get();
    int& type = reinterpret_cast<int&>(jrPtr.mBaseLineType);

    //JR
    static_cast<PropertyList&>(*this) = {
        CreateShareProperty(JR_OFFSET_X                    , jrPtr.mJROffsetX                 , 0),
        CreateShareProperty(JR_OFFSET_Y                    , jrPtr.mJROffsetY                 , 0),
        CreateShareProperty(JR_POLE_REGION_HEIGHT          , jrPtr.mHeight                    , 0),
        CreateShareProperty(JR_CENTER_NEGLECTION_WIDTH     , jrPtr.mCenterNeglectionWidth     , 0),
        CreateShareProperty(JR_BASE_LINE_OFFSET            , jrPtr.mBaseLineOffset            , 0),
        CreateShareProperty(JR_BASE_LINE_TYPE              , type                             , 0),
        CreateShareProperty(JR_CHECK_ONE_SIDE_POLE_NO      , jrPtr.mOneSidePoleNumber         , 0),
        CreateShareProperty(JR_LEANING_ENABLE              , jrPtr.mEnableCheckLeaning        , false),
        CreateShareProperty(JR_LEANING_THRESHOLD           , jrPtr.mLeaningThreshold          , 0),
        CreateShareProperty(JR_LEANING_DISTANCE            , jrPtr.mMinLeaningDistance        , 0.0),
        CreateShareProperty(JR_TAB_REMOVE_ENABLE           , jrPtr.mEnableRemoveTab           , false),
        CreateShareProperty(JR_TAB_THRESHOLD               , jrPtr.mTabThreshold              , 0),
    };
}

TypedProperty<xvt::battery::BatteryInspectorPole>::TypedProperty(Type_Trait& data, double& pixelSize)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait(pixelSize) }
{
    auto& polePtr = mData.get();

    //POLE
    static_cast<PropertyList&>(*this) = {
        CreateShareProperty(POLE_HEIGHT                       , polePtr.mPoleHeight                   , 0),
        CreateShareProperty(POLE_PROMINENCE                   , polePtr.mPolesProminenceThreshold     , 0.0),
        CreateShareProperty(POLE_THRESHOLD                    , polePtr.mPoleThreshold                , 0.0),
        CreateShareProperty(POLE_DISTANCE                     , polePtr.mPolesDistanceRange           , xvt::Rangei(0,0)),
        CreateShareProperty(POLE_SKIP_DISTANCE                , polePtr.mSkipPolesDistance            , 0),

        CreateShareProperty(POLE_OFFSET_CATHODE_2_BASE        , polePtr.mCathode2CaseOffset           , 0),
        CreateShareProperty(POLE_OFFSET_CATHODE_2_ANODE       , polePtr.mCathode2AnodeOffset          , 0),
        CreateShareProperty(POLE_OFFSET_ANODE_2_BASE          , polePtr.mAnode2CaseOffset             , 0),

        CreateShareProperty(POLE_VALID_CATHODE_2_ANODE        , polePtr.mValidCathode2AnodeRange      , xvt::Ranged(0,0)),
        CreateShareProperty(POLE_VALID_ANODE_2_BASE           , polePtr.mValidAnode2CaseRange         , xvt::Ranged(0,0)),
        CreateShareProperty(POLE_VALID_CATHODE_2_BASE         , polePtr.mValidCathode2CaseRange       , xvt::Ranged(0,0)),
        CreateShareProperty(POLE_VALID_VARIATION_ANODE_2_BASE , polePtr.mVariationAnode2Case          , 0.0),
    };
}

TypedProperty<xvt::battery::BatteryInspectorCathode>::TypedProperty(Type_Trait& data)
    : mData{ std::ref(data) }
{
    auto& cathodePtr = mData.get();

    //cathode
    static_cast<PropertyList&>(*this) = {
        CreateShareProperty(CATHODE_THRESHOLD_INNER           , cathodePtr.mCathodeLineThresholdInner  , 0.0),
        CreateShareProperty(CATHODE_THRESHOLD_MIDDLE          , cathodePtr.mCathodeLineThresholdMiddle , 0.0),
        CreateShareProperty(CATHODE_THRESHOLD_OUTER           , cathodePtr.mCathodeLineThresholdOuter  , 0.0),
        CreateShareProperty(CATHODE_PROMINENCE                , cathodePtr.mProminence                 , 0.0),
        CreateShareProperty(CATHODE_WINDOW_SIZE               , cathodePtr.mCathodeLineWindowSize      ,   0),
    };
}

TypedProperty<xvt::battery::BatteryInspectorAnode>::TypedProperty(Type_Trait& data)
    : mData{ std::ref(data) }
{
    auto& anodePtr = mData.get();

    //anode
    static_cast<PropertyList&>(*this) = {

        CreateShareProperty(ANODE_THRESHOLD_INNER             , anodePtr.mAnodeThresholdInner  , 0.0),
        CreateShareProperty(ANODE_THRESHOLD_MIDDLE            , anodePtr.mAnodeThresholdMiddle , 0.0),
        CreateShareProperty(ANODE_THRESHOLD_OUTER             , anodePtr.mAnodeThresholdOuter  , 0.0),
        CreateShareProperty(ANODE_ENHANCE_SCALE               , anodePtr.mAnodeEnhanceScale    , 0.0),
    };
}

TypedProperty<xvt::battery::CTBatteryInspector>::TypedProperty(Type_Trait& data)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait() }
{
    auto& ptr        = mData.get();
    auto& basePtr    = ptr.mIspBase;
    auto& beadPtr    = ptr.mIspBeading;
    auto& jrPtr      = ptr.mIspJR;
    auto& polePtr    = ptr.mIspPole;
    auto& anodePtr   = ptr.mIspAnode;
    auto& cathodePtr = ptr.mIspCathode;
    auto& displayPtr = ptr.mDisplayPen;

    int& display = reinterpret_cast<int&>(ptr.mDisplayMode);

    static_cast<PropertyList&>(*this) = {
        CreateShareProperty(BATTERY_ENABLE                      , ptr.mEnable              , false),

        std::make_shared<TypedProperty <battery::BatteryInspectorBase>>(basePtr, ptr.mPixelSize),

        CreateShareProperty(BATTERY_DISPLAY_MODE                , display                  ,   7),
        CreateShareProperty(BATTERY_PIXEL_SIZE                  , ptr.mPixelSize           , 0.0),
        CreateShareProperty(BATTERY_SLICE_AVG_NO                , ptr.mAvgSliceNo          ,   0),
        CreateShareProperty(BATTERY_SLICE_NO                    , ptr.mSliceNo             ,   0),

        std::make_shared<TypedProperty <battery::BatteryInspectorBeading>>(beadPtr, ptr.mPixelSize),
        std::make_shared<TypedProperty <battery::BatteryInspectorJR>>(jrPtr, ptr.mPixelSize),
        std::make_shared<TypedProperty <battery::BatteryInspectorCathode>>(cathodePtr),
        std::make_shared<TypedProperty <battery::BatteryInspectorAnode>>(anodePtr),
        std::make_shared<TypedProperty <battery::BatteryInspectorPole>>(polePtr, ptr.mPixelSize),

        CreateShareProperty(RESULT_FONT_SCALE                  , displayPtr.mFontScale       , 0.0),
        CreateShareProperty(RESULT_FONT_LINE_SPACE             , displayPtr.mSpace       ,   0),
        CreateShareProperty(RESULT_GAMMA                       , ptr.mGamma               , 0.0),
    };
}
#if 0
TypedProperty<xvt::battery::CTBatteryInspectorUpper>::TypedProperty(Type_Trait& data)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait() }
{
    auto& upperPtr   = mData.get();
    auto& beadPtr    = upperPtr.mIspBeading;
    battery::CTBatteryInspector& basePtr   = upperPtr;

    static_cast<PropertyList&>(*this) = {
        std::make_shared<TypedProperty <battery::CTBatteryInspector>>(basePtr)
    };
}

TypedProperty<xvt::battery::CTBatteryInspectorLower>::TypedProperty(Type_Trait& data)
    : mData{ std::ref(data) }, mDefaultValue{ Type_Trait() }
{
    auto& ptr = mData.get();
    battery::CTBatteryInspector& basePtr = ptr;
    auto& beadPtr = ptr.mIspBeading;

    static_cast<PropertyList&>(*this) = {
        std::make_shared<TypedProperty <battery::CTBatteryInspector> >(basePtr)
    };
}
#endif
}//namesapce xvt