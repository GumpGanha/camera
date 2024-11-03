#pragma once

#ifndef POINTER_64
#define POINTER_64 __ptr64
#endif

#include <stdint.h>
#include <string>
#include "MS_MACRO.h"
#include "CameraInfo.h"
#include <mutex>
using namespace std;

namespace movesense {

uint8_t ChannelOffset(uint8_t channel);

class CameraBase
{
public:
	CameraBase(CameraInfo info);
	virtual ~CameraBase(void);

public:
	CameraInfo info;

protected:
	virtual int SetSaturation(long value) = 0;
	virtual int GetSaturation(long *value) = 0;
	virtual int SetWhiteBalance(long value) = 0;
	virtual int GetWhiteBalance(long *value) = 0;

	int WriteRegister(uint16_t addr, uint8_t value);
	int ReadRegister(uint16_t addr, uint8_t *value);

public:
	virtual int Open(int width, int height) = 0;
	virtual void Close() = 0;

	virtual int StartStream() = 0;
	virtual int StopStream() = 0;

	virtual int GetImageData(unsigned char *data, int &len, int ms) = 0;

public:
	// Set channels on / off
	int SetChannelSwitch(uint16_t addr, bool onoff, uint8_t channels);
	// Set register value of the channel
	int SetChannelRegister(uint16_t addr, uint8_t value, uint8_t channel=MS_CHANNEL_FORWARD);

	/**** Set/Get register with check ****/
	// Register (8 bits)
	int SetRegister(uint16_t addr, uint8_t value);
	int GetRegister(uint16_t addr, uint8_t *value);

	// Registers (N*8 bits)
	int SetRegisters(uint16_t addr, uint8_t *value, int n);
	int GetRegisters(uint16_t addr, uint8_t *value, int n);

	// Wait when busy 
	int WaitForAvailable(int ms=3000);
	
private:
	std::mutex regMutex;
};

}

