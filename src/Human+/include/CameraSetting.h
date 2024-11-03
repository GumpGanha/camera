#pragma once

#include <vector>
using namespace std;

namespace movesense {

enum CAMERA_MODE
{
	CAMERA_LR,  //0
	CAMERA_LD,  //1
	CAMERA_LRD, //2
	CAMERA_LR_HD = 4,
	CAMERA_LD_HD = 5,
	CAMERA_LRD_HD = 6,
};

struct Resolution
{
	int width;
	int height;
};

struct Disparity
{
	int type;
	int bpp;
};

struct CameraSetting
{
    //Current setting
    Resolution resolution;
    int framerate;
	int mode;

    //Support list
    vector<Resolution> resolutionList;
    vector<int> framerateList;
    vector<int> modeList;

	//Disparity
	Disparity disparity;
};

}
