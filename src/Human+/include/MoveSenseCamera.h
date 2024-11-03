#pragma once

#include "CameraBase.h"
#include "CameraPara.h"
#include "Algorithm.h"

namespace movesense {


#ifdef _WIN32
#define MOVESENSE_EXPORT __declspec(dllexport)
#else 
#define MOVESENSE_EXPORT
#endif
	

class MOVESENSE_EXPORT MoveSenseCamera
{
public:
    MoveSenseCamera(int index=0);
    ~MoveSenseCamera();

	//Available
	bool IsAvailable();

    //Open and Close
    int Open();
    void Close();

	// Reset
    int Reset();

    //Control image stream
    int StartStream(uint8_t channels=MS_CHANNEL_ALL);
    int StopStream();

	//Image data control
	int GetImageData(unsigned char *data, int &len, int ms=2000);
	int GetImageData(unsigned char *left, unsigned char *right, unsigned char *disparity, int &len, int ms=2000);

	//Depth: mm
	int DisparityToDepth(int disparity);

	//Stereo rectify
	int SetStereoRectify(bool onoff, uint8_t channels=MS_CHANNEL_ALL);

    //Calib parameter
	int SetCalibParameter(float *para, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetCalibParameter(float *para, uint8_t channel=MS_CHANNEL_FORWARD);

	//Auto: Exposure and Gain
	int SetAutoExposure(bool onoff, uint8_t channels=MS_CHANNEL_ALL);
	int SetAutoGain(bool onoff, uint8_t channels=MS_CHANNEL_ALL);

	int SetMaxExposure(uint16_t value, uint8_t channel=MS_CHANNEL_FORWARD);
	int SetMinExposure(uint16_t value, uint8_t channel=MS_CHANNEL_FORWARD);
	int SetMaxGain(uint16_t value, uint8_t channel=MS_CHANNEL_FORWARD);
	int SetMinGain(uint16_t value, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetMaxExposure(uint16_t *value, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetMinExposure(uint16_t *value, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetMaxGain(uint16_t *value, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetMinGain(uint16_t *value, uint8_t channel=MS_CHANNEL_FORWARD);

	int SetDesiredBringhtness(uint8_t value, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetDesiredBringhtness(uint8_t *value, uint8_t channel=MS_CHANNEL_FORWARD);

    //Manual: Exposure and Gain
	int SetExposure(uint16_t value, uint8_t channel=MS_CHANNEL_FORWARD);
    int SetGain(uint16_t value, uint8_t channel=MS_CHANNEL_FORWARD);

	//Auto&Manual: Exposure and Gain
	int GetExposure(uint16_t *value, uint8_t channel=MS_CHANNEL_FORWARD);
	int GetGain(uint16_t *value, uint8_t channel=MS_CHANNEL_FORWARD);

	int GetSN(uint32_t *sn);

public:
	/* for register use */
	// Register (8 bits)
	int SetRegister(uint16_t addr, uint8_t value);
	int GetRegister(uint16_t addr, uint8_t *value);

	// Registers (N*8 bits)
	int SetRegisters(uint16_t addr, uint8_t *value, int n);
	int GetRegisters(uint16_t addr, uint8_t *value, int n);
public:
	void SpeckleCheck( unsigned short int* src, unsigned short int * dst, int speckleRange = 200, int cntSize = 200  );
	void GetPcd(const unsigned short int *src,   float *pcd);

public:
	string GetName();
	int GetImageDataLength();
	CameraSetting GetSetting();
	CameraPara GetPara();

	Resolution GetResolution();
	int GetFramerate();
	int GetMode();

	int SetResolution(int width, int height);
	int SetFramerate(int framerate);
	int SetMode(int mode);

	int GetBitDepth();

private:
	int imageDataLength;
	CameraSetting *setting;
	CameraPara para;
	Algorithm *algorithm;

private:
    CameraBase *camera;
	unsigned char imageBuffer[MAX_BUFFER_BYTES_LENGTH];

public:
    // Return the number of camera connected to PC
    static int ScanCameras();
};

}
