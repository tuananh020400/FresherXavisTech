#include "xvtBattery/CylinderBatteryResult.h"
#include "xvtCV/Drawing.h"
#include "xvtCV/ColorDefine.h"

namespace xvt {
namespace battery {
    BatteryInspectionResult::BatteryInspectionResult()
    {
		yA2 = 0;
        sHeader = "";
        sData = "";						// Data
        iResult = 0;
        Description = "";				// Final Decision
        corverHeight = 0;				// Cover Height
        corverDiameter = 0;				// Cover Diameter
        outerDiameter = 0;				// Max Outer Diameter
        grooveDepth = 0;				// Groove Depth
        grooveHeight = 0;				// Groove Height
        minCathode2Anode = 0;			// Min Positive to Negative
        maxCathode2Anode = 0;			// Max Positive to Negative
        avgCathode2Anode = 0;			// Avg Positive to Negative
        minAnode2Case = 0;				// Min Negative to Case
        maxAnode2Case = 0;				// Max Negative to Case
        avgAnode2Case = 0;				// Avg Negative to Case
        minCathode2Case = 0;			// Min Positive to Case
        maxCathode2Case = 0;			// Max Positive to Case
        avgCathode2Case = 0;			// Avg Positive to Case
        Anode2CathodeDecision = "";		// True if no error, otherwise false
        Anodes2CaseDecision = "";		// True if no error, otherwise false
        Cathode2CaseDecision = "";		// True if no error, otherwise false
        Anode2CaseVariationDecision = ""; // True if no error, otherwise false
        BlackCloudDecision = "";		// True if no error, otherwise false
        BeadingCoverDiameterDecision = ""; // True if no error, otherwise false
        BeadingGrooveDepthDecision = ""; // True if no error, otherwise false
        BeadingGrooveHeightDecision = ""; // True if no error, otherwise false
        BeadingInnerDiameterDecision = ""; // True if no error, otherwise false
        BlackCloudDecision = "";		// True if no error, otherwise false
        BlackCloudDecision = "";		// True if no error, otherwise false
        finalDecision = "";				// Final Decision: true if no error, otherwise false
        centerPinWidth = 0;				// Center PIN Width
        centerPinDepth = 0;				// Center PIN Depth
        isPinExist = false;				// true if there is pin, otherwise false
        nLeftPole = 0;
    }

	std::string textWrap(std::string str, int location) {
		// your other code
		int n = str.rfind(' ', location);
		if (n != std::string::npos) {
			str.at(n) = '\n';
		}
		// your other code
		return str;
	}

	void saveErroImage(const cv::Mat& resImg, const std::string& outFileName, std::string errDesc, ERR_CODE errCode)
	{
		float fontScale = 1;
		float fontWeight = 2;
		std::string strWrapepdDesc = textWrap(errDesc, 60);
		cv::Scalar cath2AnoColor(10, 10, 250);
		cv::putText(resImg, "NG - Inspection exited with error code: " + std::to_string(static_cast<int>(errCode)),
			cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, fontScale, cath2AnoColor, fontWeight);

		std::string delimiter = "\n";
		size_t pos = 0;
		std::string subStr;
		int lineIdx = 0;
		while ((pos = strWrapepdDesc.find(delimiter)) != std::string::npos) {
			subStr = strWrapepdDesc.substr(0, pos);
			std::cout << subStr << std::endl;
			strWrapepdDesc.erase(0, pos + delimiter.length());
			cv::putText(resImg, "Error Message: " + subStr, cv::Point(100, 150 + lineIdx * 50), cv::FONT_HERSHEY_SIMPLEX, fontScale, cath2AnoColor, fontWeight);
			lineIdx++;
			cv::putText(resImg, strWrapepdDesc, cv::Point(100, 150 + lineIdx * 50), cv::FONT_HERSHEY_SIMPLEX, fontScale, cath2AnoColor, fontWeight);
		}

		cv::imwrite(outFileName + "_Poles_Measurement.jpg", resImg);
	}
}
}