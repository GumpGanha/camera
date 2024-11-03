#pragma once

namespace movesense {


#ifdef _WIN32
#define MOVESENSE_EXPORT __declspec(dllexport)
#else 
#define MOVESENSE_EXPORT
#endif
	

class MOVESENSE_EXPORT Algorithm
{
public:
    Algorithm(float *stereo_parameter, float *rgb_parameter);
    ~Algorithm();

public:
	void SpeckleCheck( unsigned short int* src, unsigned short int * dst, const int &width, const int &height, int speckleRange = 200, int cntSize = 200  );
	void GetPcd(const unsigned short int *src, const int &width, const int &height,   float *pcd);
	bool  isDepthMap;
	bool  isRegistration;
private:
	float tP2_Stereo[3][4];
	float M2_Rgb[3][3];
	float R_Rgb[3][3]; 
	float T_Rgb[3][1]; 

	float rRGB_00, rRGB_01, rRGB_02, rRGB_10, rRGB_11, rRGB_12, rRGB_20, rRGB_21, rRGB_22;
	float tRGB_0, tRGB_1, tRGB_2;
	float cuStereo, cvStereo;
	float fStereo, fStereo1;
	float fxRGB, fyRGB, fxRGB1, fyRGB1;
	float cuRGB, cvRGB;
};

}
