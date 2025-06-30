#pragma once

#define USE_INSPECT                         L"use_inspection"
#define INSPECT_ITEMS                       L"insp_items"

// ROI detection
#define BATTERY_ENABLE                      L"Battery Enable"
#define BATTERY_ROI                         L"Battery ROI(px)"
#define BATTERY_DIRECTION                   L"Battery Direction"
#define BATTERY_THRESHOLD                   L"Battery Threshold(0~255)"
#define BATTERY_PIXEL_SIZE                  L"Battery Pixel Size(mm)"
#define BATTERY_DISPLAY_MODE                L"Battery Display Mode(1:Pole, 7:All)"

#define BATTERY_SLICE_NO                    L"Battery Slice No"
#define BATTERY_SLICE_AVG_NO                L"Battery Slice Avg No"

#define BATTERY_VALID_WIDTH                 L"Battery Valid Width(mm)"

// Beading
#define BEADING_ENABLE                      L"Beading Enable(0:No, 1:Yes)"
#define BEADING_MIN_HEIGHT                  L"Beading Min Height(px)"
#define BEADING_D1_START_POS                L"Beading D1 Start position"

// J/R offset
#define JR_OFFSET_X                         L"JR Offset X(px)"
#define JR_OFFSET_Y                         L"JR Offset Y(px)"
#define JR_POLE_REGION_HEIGHT               L"JR Pole Region Height(px)"
#define JR_CENTER_NEGLECTION_WIDTH          L"JR Center Neglection Width(px)"
#define JR_BASE_LINE_OFFSET                 L"JR Base Line Offset(px)"
#define JR_BASE_LINE_TYPE					L"JR Base Line Type(0:None, 1:Center, 2:Anode, 3:Conner)"
#define JR_CHECK_ONE_SIDE_POLE_NO           L"JR One Side Pole No."
#define JR_LEANING_ENABLE                   L"JR Leaning Enable"
#define JR_LEANING_THRESHOLD                L"JR Leaning Threshold"
#define JR_LEANING_DISTANCE                 L"JR Leaning Distance(px)"
#define JR_TAB_REMOVE_ENABLE                L"JR Tab Remove Enable"
#define JR_TAB_THRESHOLD					L"JR Tab Threshold"

// Cathode
#define CATHODE_THRESHOLD_INNER             L"Cathode Thredshold Inner"
#define CATHODE_THRESHOLD_MIDDLE            L"Cathode Thredshold Middle"
#define CATHODE_THRESHOLD_OUTER             L"Cathode Thredshold Outer"
#define CATHODE_PROMINENCE                  L"Cathode Prominence"
#define CATHODE_WINDOW_SIZE                 L"Cathode Window Size"

// Anode
#define ANODE_THRESHOLD_INNER               L"Anode Thredshold Inner"
#define ANODE_THRESHOLD_MIDDLE              L"Anode Thredshold Middle"
#define ANODE_THRESHOLD_OUTER               L"Anode Thredshold Outer"
#define ANODE_ENHANCE_SCALE					L"Anode Enhance Scale"

//Pole ROI setting
#define POLE_HEIGHT                         L"Pole Height(px)"
#define POLE_PROMINENCE                     L"Pole Prominence"
#define POLE_THRESHOLD                      L"Pole Threshold"
#define POLE_DISTANCE                       L"Pole Distance(px)"
#define POLE_SKIP_DISTANCE                  L"Pole Skip Distance(px)"
#define POLE_VALID_CATHODE_2_BASE           L"Pole Cathode-Base(mm)"
#define POLE_VALID_CATHODE_2_ANODE          L"Pole Cathode-Anode(mm)"
#define POLE_VALID_ANODE_2_BASE             L"Pole Anode-Base(mm)"
#define POLE_VALID_VARIATION_ANODE_2_BASE   L"Pole Variation Anode-Base(mm)"

#define POLE_OFFSET_CATHODE_2_BASE          L"Pole Offset Cathode-Base"
#define POLE_OFFSET_CATHODE_2_ANODE         L"Pole Offset Cathode-Anode"
#define POLE_OFFSET_ANODE_2_BASE            L"Pole Offset Anode-Base"

// Setting
#define RESULT_SAVE                         L"Result Save"
#define RESULT_PATH                         L"Result Path"
#define RESULT_GAMMA                        L"Result Gamma"
#define RESULT_WHITE_TOP                    L"Result White Top"
#define RESULT_WHITE_BOTTOM                 L"Result White Bottom"
#define RESULT_ADDITIONAL_LINE              L"Result Additional Line"
#define RESULT_LINE_TYPE                    L"Result Line Type"
#define RESULT_FONT_SCALE                   L"Result Font Scale"
#define RESULT_FONT_LINE_SPACE              L"Result Line Space"
#define RESULT_RECEIPE_PATH                 L"Result Receipe Path"
