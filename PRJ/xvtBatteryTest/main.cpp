#include "MyForm.h"
#include <map>
#include <iostream>
#include <string>
#include <fstream>
#include <xvtBattery/VersionInfo.h>

int main() {
	std::cout << "Battery Lib Version: " + xvt::Battery_VersionInfo.GetVersionInfo() + "\n";
	xvtBatteryTest::MyForm Form;
	Form.ShowDialog();
	return 0;
}