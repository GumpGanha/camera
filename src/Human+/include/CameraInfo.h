#pragma once

#include <string>
#include "CameraSetting.h"

using namespace std;

namespace movesense {

struct CameraInfo
{
	string name;
	string protocol;
	string parseProtocol;
	string vid;
	string pid;

	string devName;
	string serialNum;
	string usbVer;
	
	CameraSetting setting;
};

}
