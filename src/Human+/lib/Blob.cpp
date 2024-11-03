#include "Blob.h"
#include <iostream>
#include <Windows.h>
#include <string.h>

namespace stereoai
{
    std::string float2string(float val, int precision)
    {
        std::stringstream buf;
        buf.precision(precision);  //覆盖默认精度
        buf.setf(std::ios::fixed); //保留小数位
        buf << val;
        std::string str;
        str = buf.str();
        return std::string(str);
    }
    void Blob::getBlobCenters(cv::Mat& rawRGB, cv::Mat& depth, std::vector<cv::Point2f>& blobLeft, std::vector<cv::Point2f>& blobRight)
    {
        blobLeft.clear();
        blobRight.clear();
        unsigned char* blobDataL = rawRGB.data + IMAGE_BUFFER_OFFSET_LEFT;
        unsigned char* blobDataR = rawRGB.data + IMAGE_BUFFER_OFFSET_RIGHT;

        //解析左图
        int leftCnt = blobDataL[2] * 256 + blobDataL[3];
        int rightCnt = blobDataR[2] * 256 + blobDataR[3];
       
        leftCnt = leftCnt < BLOB_MAX_COUNT ? leftCnt : BLOB_MAX_COUNT;
        rightCnt = rightCnt < BLOB_MAX_COUNT ? rightCnt : BLOB_MAX_COUNT;
        int frameIndex = *((unsigned*)(blobDataL + 4));
        float* coorLeft = (float*)(blobDataL + 8);
        float* coorRight = (float*)(blobDataR + 8);
        for (int i = 0; i < leftCnt; i++)
        {
            blobLeft.push_back(cv::Point2f(coorLeft[2 * i + 0], coorLeft[2 * i + 1]));
        }
        for (int i = 0; i < rightCnt; i++)
        {
            blobRight.push_back(cv::Point2f(coorRight[2 * i + 0], coorRight[2 * i + 1]));
        }
//        memset(blobDataL, 0, 6 * 1280);

      /*  cv::Mat tmp;
        rawRGB.copyTo(tmp);
        if (!mRawRGBLast.empty())
            mRawRGBLast.copyTo(rawRGB);
        tmp.copyTo(mRawRGBLast);

        depth.copyTo(tmp);
        if (!mDepthLast.empty())
            mDepthLast.copyTo(depth);
        tmp.copyTo(mDepthLast);*/
        
    }
	void Blob::getBlobCenters2(cv::Mat & rawRGB, cv::Mat & depth, std::vector<cv::Point2f>& blobLeft, std::vector<cv::Point2f>& blobRight)
	{
		blobLeft.clear();
		blobRight.clear();
		unsigned char* blobDataL = rawRGB.data + IMAGE_BUFFER_OFFSET_LEFT;
		unsigned char* blobDataR = rawRGB.data + IMAGE_BUFFER_OFFSET_RIGHT;

		//解析左图
		int leftCnt = blobDataL[2] * 256 + blobDataL[3];
		int rightCnt = blobDataR[2] * 256 + blobDataR[3];

		leftCnt = leftCnt < BLOB_MAX_COUNT ? leftCnt : BLOB_MAX_COUNT;
		rightCnt = rightCnt < BLOB_MAX_COUNT ? rightCnt : BLOB_MAX_COUNT;
		int frameIndex = *((unsigned*)(blobDataL + 4));
		float* coorLeft = (float*)(blobDataL + 8);
		float* coorRight = (float*)(blobDataR + 8);
		for (int i = 0; i < leftCnt; i++)
		{
			blobLeft.push_back(cv::Point2f(coorLeft[2 * i + 0], coorLeft[2 * i + 1]));
		}
		for (int i = 0; i < rightCnt; i++)
		{
			blobRight.push_back(cv::Point2f(coorRight[2 * i + 0], coorRight[2 * i + 1]));
		}
	//	memset(blobDataL, 0, 6 * 1280);

		cv::Mat tmp;
		rawRGB.copyTo(tmp);
		if (!mRawRGBLast.empty())
			mRawRGBLast.copyTo(rawRGB);
		tmp.copyTo(mRawRGBLast);

	/*	depth.copyTo(tmp);
		if (!mDepthLast.empty())
			mDepthLast.copyTo(depth);
		tmp.copyTo(mDepthLast);*/
	}
    void setBlobThreshold(movesense::MoveSenseCamera* camera, unsigned char value)
    {
        camera->SetRegister(SETTINGS_BLOB_THRESHVALUE, value);
    }

    void setBlobAreaMin(movesense::MoveSenseCamera* camera, int value)
    {
        camera->SetRegister(SETTINGS_BLOB_AREA_MIN, value & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_AREA_MIN + 1, (value >> 8 ) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_AREA_MIN + 2, (value >> 16) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_AREA_MIN + 3, (value >> 24) & 0xFF);
    }
    void setBlobAreaMax(movesense::MoveSenseCamera* camera, int value)
    {
        camera->SetRegister(SETTINGS_BLOB_AREA_MAX, value & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_AREA_MAX + 1, (value >> 8) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_AREA_MAX + 2, (value >> 16) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_AREA_MAX + 3, (value >> 24) & 0xFF);
    }

    void setBlobWitdhMin(movesense::MoveSenseCamera* camera, int value)
    {
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MIN, value & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MIN + 1, (value >> 8  )& 0xFF);
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MIN + 2, (value >> 16 )& 0xFF);
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MIN + 3, (value >> 24 )& 0xFF);
    }

    void setBlobWitdhMax(movesense::MoveSenseCamera* camera, int value)
    {
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MAX, value & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MAX + 1, (value >> 8 ) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MAX + 2, (value >> 16) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_WIDTH_MAX + 3, (value >> 24) & 0xFF);
    }


    void setBlobHeightMin(movesense::MoveSenseCamera* camera, int value)
    {
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MIN, value & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MIN + 1, (value >> 8 ) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MIN + 2, (value >> 16) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MIN + 3, (value >> 24) & 0xFF);
    }

    void setBlobHeightMax(movesense::MoveSenseCamera* camera, int value)
    {
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MAX, value & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MAX + 1, (value >> 8 )& 0xFF);
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MAX + 2, (value >> 16) & 0xFF);
        camera->SetRegister(SETTINGS_BLOB_HEIGHT_MAX + 3, (value >> 24) & 0xFF);
    }


    void startBlobDetectoion(movesense::MoveSenseCamera* camera)
    {
        camera->SetRegister(SETTINGS_BLOB_START_STOP,1);
    }
    void stopBlobDetectoion(movesense::MoveSenseCamera* camera)
    {
        camera->SetRegister(SETTINGS_BLOB_START_STOP,0);
    }

};

 