#include "header.h"
#include <iostream>
#include "xvtBattery/VersionInfo.h"

enum test_t {CT, JR, CYLINDER};

int wmain(int argc, wchar_t* argv[])
{
	std::cout << "Battery Lib Version: " + xvt::Battery_VersionInfo.GetVersionInfo() + "\n";
	test_t t = CYLINDER;

	switch (t)
	{
	case CT:
		ct_main();
		break;
	case JR:
		jr_main();
		break;
	case CYLINDER:
		cylinder_main();
		break;
	default:
		break;
	}

	return 0;
}