#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "MoveSenseCamera.h"
namespace stereoai
{
#define IMAGE_BUFFER_OFFSET_LEFT 1280*794
#define IMAGE_BUFFER_OFFSET_RIGHT 1280*797
#define BLOB_MAX_COUNT 400
#define SETTINGS_BLOB_THRESHVALUE			    0x0664
#define SETTINGS_BLOB_AREA_MIN			        0x0672
#define SETTINGS_BLOB_AREA_MAX			        0x0676
#define SETTINGS_BLOB_WIDTH_MIN			        0x0680
#define SETTINGS_BLOB_WIDTH_MAX				    0x0684
#define SETTINGS_BLOB_HEIGHT_MIN			    0x0688
#define SETTINGS_BLOB_HEIGHT_MAX				0x0692
#define SETTINGS_BLOB_START_STOP      			0x0696

    std::string float2string(float val, int precision = 2);
    class Blob
    {
    public:
        Blob() {}
        ~Blob() {}

    public:
        void getBlobCenters(cv::Mat& rawRGB, cv::Mat& depth, std::vector<cv::Point2f> & blobLeft, std::vector<cv::Point2f> & blobRight);
		void getBlobCenters2(cv::Mat& rawRGB, cv::Mat& depth, std::vector<cv::Point2f> & blobLeft, std::vector<cv::Point2f> & blobRight);
        cv::Mat mRawRGBLast, mDepthLast;
    };



    void setBlobThreshold(movesense::MoveSenseCamera* camera, unsigned char value);

    void setBlobAreaMin(movesense::MoveSenseCamera* camera, int value);

    void setBlobAreaMax(movesense::MoveSenseCamera* camera, int value);

    void setBlobWitdhMin(movesense::MoveSenseCamera* camera, int value);

    void setBlobWitdhMax(movesense::MoveSenseCamera* camera, int value);

    void setBlobHeightMin(movesense::MoveSenseCamera* camera, int value);

    void setBlobHeightMax(movesense::MoveSenseCamera* camera, int value);

    void startBlobDetectoion(movesense::MoveSenseCamera* camera);
    void stopBlobDetectoion(movesense::MoveSenseCamera* camera);
};
