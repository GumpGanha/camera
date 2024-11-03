#pragma once
#include "MoveSenseCamera.h"
#include "StereoBlob.h"
#include "Blob.h"
#include <opencv2/opencv.hpp>
#include "SendMat.h"

#ifdef _WIN32
#undef ACCESS_MASK
#endif

//#undef ACCESS_MASK
#include <boost/bind.hpp>
using namespace boost::placeholders;
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include<boost/thread.hpp>

#ifdef _WIN32

#include <direct.h>
#include <io.h>
#else
#include <stdio.h>
#include <unistd.h> 
#include <sys/stat.h>

#endif 

#ifdef _WIN32

#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#else 
#define ACCESS access
#define MKDIR(a) mkdir((a),0755)

#endif

using namespace movesense;
//using namespace cv;
using namespace stereoai;




class StereoVisionSystem {


public:
    //构造函数
    StereoVisionSystem();
    
    

    //析构函数
    ~StereoVisionSystem();


    void close();
    bool initializeCamera();
    static  void onTrackbarExposure(int value, void*);
    static  void onTrackbarGain(int value, void*);
    void SetCameraNo(MoveSenseCamera* camera, unsigned char index);
    void GetCameraNo(MoveSenseCamera* camera, unsigned char& index);
    void SetSNOnce(movesense::MoveSenseCamera* camera, unsigned int SN);
    unsigned int GetSN(movesense::MoveSenseCamera* camera);
    SendMat pointcapture();


private:

  
    int mode = CAMERA_LR_HD;

    int minExposure = 0;
    int maxExposure = 870;
    int minGain = 0;
    int maxGain = 64;
    int currentStereoExposure = 200;
    int currentStereoGain = 16;
    int currentRgbExposure = 300;
    int currentRgbGain = 16;
    int currenDoeDuty = 200;

    MoveSenseCamera* camera;

    cv::Mat left, right, disparity, depth, depthFilter, depthShow, depthShowColor;
    int w, h, len, bitDepth;
    int frameCnt;
    double time1;
    CameraPara para;
    SendMat  m_SendMat;
    //string constraintsFile = "json_object.json";

   // string constraintsFile2 = "json_glasses.json";



};

typedef boost::shared_ptr<StereoVisionSystem> StereoVisionSystem_spointer;
