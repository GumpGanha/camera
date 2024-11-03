#include <opencv2/opencv.hpp>
#include "MoveSenseCamera.h"
#include "StereoBlob.h"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc.hpp>
#include "ImageCapture.h"
#include "SendMat.h"

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
using namespace cv;
using namespace stereoai;
//MoveSenseCamera* camera;

StereoVisionSystem::StereoVisionSystem() : camera(new MoveSenseCamera) {
	if (!camera->IsAvailable()) {
		cerr << "No camera connected..." << endl;
		exit(1);
	}
}

StereoVisionSystem::~StereoVisionSystem() {
	camera->StopStream();
	camera->Close();
	delete camera;
}

 void  StereoVisionSystem::onTrackbarExposure(int value, void* userdata)
{
	static_cast<StereoVisionSystem*>(userdata)->camera->SetExposure(value);
}

 void StereoVisionSystem::onTrackbarGain(int value, void* userdata)
{
	static_cast<StereoVisionSystem*>(userdata)->camera->SetGain(value);
}

void StereoVisionSystem::SetCameraNo(MoveSenseCamera*camera, unsigned char index)
{
	camera->SetRegister(0x0700, index);
}

void StereoVisionSystem::GetCameraNo(MoveSenseCamera*camera, unsigned char &index)
{
	camera->GetRegister(0x0028, &index);
}

void StereoVisionSystem::SetSNOnce(movesense::MoveSenseCamera* camera, unsigned int SN)
{
	unsigned char sns[4];
	sns[0] = SN;
	sns[1] = SN >> 8;
	sns[2] = SN >> 16;
	sns[3] = SN >> 24;
	camera->SetRegister(0x001A, sns[0]);
	camera->SetRegister(0x001B, sns[1]);
	camera->SetRegister(0x001C, sns[2]);
	camera->SetRegister(0x001D, sns[3]);
	camera->SetRegister(0x00F0, 0x01);                //存储至相机非易失单元
}


unsigned int StereoVisionSystem::GetSN(movesense::MoveSenseCamera* camera)
{
	unsigned int SN = 0;
	unsigned char* ptr = (unsigned char*)&SN;
	camera->GetRegister(0x001A, ptr + 0);
	camera->GetRegister(0x001B, ptr + 1);
	camera->GetRegister(0x001C, ptr + 2);
	camera->GetRegister(0x001D, ptr + 3);
	return SN;
}

bool StereoVisionSystem::initializeCamera() {

	bool rectify_sw = true;

	if (camera->SetMode(mode) != MS_SUCCESS) {
		cerr << "The mode is not supported..." << endl;
		return false;
	}

	if (camera->Open() != MS_SUCCESS) {
		cerr << "Open camera failed!" << endl;
		return false;
	}
	cout << "Open camera ok: " << camera->GetName() << endl;


	len = camera->GetImageDataLength();
	bitDepth = camera->GetBitDepth();
	if (bitDepth != 8 && bitDepth != 16) {
		cerr << "bitDepth illegal (should be 8 or 16)!" << endl;
		return false;
	}
	CameraPara para = camera->GetPara();
	cout << "Camera para:" << endl << 
		"  f = " <<  para.f*2.0 << endl <<
		"  cu = " << para.cu * 2.0 << endl <<
		"  cv = " << para.cv * 2.0 << endl <<
		"  b = " << para.b << endl;
	
	camera->StartStream();
	camera->SetStereoRectify(rectify_sw);
	camera->SetAutoExposure(false);
	camera->SetAutoGain(false);
	//Get camera setting		
	Resolution resolution = camera->GetResolution();
	int w = resolution.width;
	int h = resolution.height;
	std::cout << "Image Width:" << w << std::endl;
	std::cout << "Image Height:" << h << std::endl;
	int len = camera->GetImageDataLength();
	int bitDepth = camera->GetBitDepth();
	if( bitDepth != 8 && bitDepth != 16 )
	{
		cout << "bitDepth illegal (should be 8 or 16)!" << endl;
		camera->StopStream();
		camera->Close();
		delete camera;
		return MS_FAIL;
	}
	return true;
}


void StereoVisionSystem::close() {
	camera->StopStream();
	camera->Close();
};


SendMat StereoVisionSystem::pointcapture()
{
	//camera= new MoveSenseCamera(0);
	StereoVisionSystem::initializeCamera();
	//int mode = CAMERA_LR_HD;
	int res = camera->SetMode(mode);

	cv::Mat left(h,w,CV_8UC1),right(h,w,CV_8UC1),disparity(h,w,CV_8UC1);
	cv::Mat depth(h,w,CV_16UC1), depthFilter(h, w, CV_16UC1),depthShow(h,w,CV_8U),depthShowColor(h,w,CV_8UC3);

	int imgIndex = 0;

	if (-1 == ACCESS("image", 0))  MKDIR("image");

	int frameCnt = 0;
	double time1=cv::getTickCount(), time2=0, time=0;

	cv::namedWindow("left");
	cv::createTrackbar("exposure", "left", &currentStereoExposure, maxExposure, onTrackbarExposure);
	cv::createTrackbar("gain", "left", &currentStereoGain, maxGain, onTrackbarGain);

	/***************************************主要调用代码**************************************/
	std::string constraintsFile = "json_object.json";
	std::string constraintsFile2 = "json_glasses.json";
	//注册一个stereoBlob类，将两个几何约束文件分别解析到stereo.mGeometryConstraint和stereo.mGeometryConstraint2这两个变量中。
	//后续使用updateWithmGeometryConstraint()接口时会调用这两个变量
	//声明接口有两个重写方式。另外一个不用管，是用于彩色画面相机的声明。如果只找一个或者需要再加一个几何约束。就仿照这个接口再写一个重写接口
	stereoai::StereoBlob stereo(constraintsFile, constraintsFile2, para.f * 2.0, para.cu * 2.0, para.cv * 2.0, para.b);
	/***************************************主要调用代码**************************************/

	//正常参数设置，不用管
	camera->SetExposure(200);
	stereoai::setBlobThreshold(camera, 200);
	stereoai::setBlobAreaMin(camera, 6);
	stereoai::setBlobAreaMax(camera, 10000);
	stereoai::setBlobWitdhMin(camera, 4);
	stereoai::setBlobWitdhMax(camera, 150);
	stereoai::setBlobHeightMin(camera, 4);
	stereoai::setBlobHeightMax(camera, 150);
	stereoai::startBlobDetectoion(camera);


	cv::Mat lastImage;
	cv::Mat  record_blob_glasses = cv::Mat::zeros(3, 3, CV_32F);
	cv::Mat  record_blob_object = cv::Mat::zeros(3, 3, CV_32F);


	Mat dot_model_glasses = (Mat_<float>(3, 3) << -61, -13, 63, -88, -88, -90, -29.5, 12, -0.5);   //每一列表示坐标值，输入的时候按照行输入
	Mat dot_model_object = (Mat_<float>(3, 3) << 24.06,-25.93,-41.26,60.75,61.08,97.32,87.03,87.51,139.25);

	Mat glasses2camera;  //所求坐标系对相机坐标系的其次描述
	Mat object2camera;

	//死循环，采取图像。可以改成线程处理。
	while (1)
	{
		//采取图像
		/***************************************主要调用代码**************************************/
		res = camera->GetImageData(left.data, right.data, depth.data, len);
	
		//这段注释的代码
		//是分别解析获取两个几何约束的小球刚体结构。取数据时，可以参考这段代码的for循环
		stereoai::Blob3DPoints blobsRGBAll;
		//cout << blobsRGBAll.points2D.size();
		stereo.update(left, right, blobsRGBAll);
		//stereo.drawBlobRGB(left, blobsRGBAll);
		//第一个约束的小球刚体结构
		stereoai::Blob3DPointsRes blobsRGB;
		blobsRGB = stereo.updateWithGeometryConstraint(left, right);
		//stereo.drawBlobRGB(left, blobsRGB);
		//第二个约束的小球刚体结构
		stereoai::Blob3DPointsRes blobsRGB2;
		blobsRGB2 = stereo.updateWithGeometryConstraint2(left, right);
		//stereo.drawBlobRGB(left, blobsRGB2);
		cv::Mat rgb;

		cv::cvtColor(left, rgb, cv::COLOR_BayerGR2RGB);

		//cout << blobsRGB.points2D.size();


		for (int i = 0; i < blobsRGB.points2D.size(); i++)
		{
			cout << "for xunhuan ing" << endl;

			int u = (int)blobsRGB.points2D[i].x;
			int v = (int)blobsRGB.points2D[i].y;
			int x = blobsRGB.points3D[i].x;
			int y = blobsRGB.points3D[i].y;
			int z = blobsRGB.points3D[i].z;
			if (i == 0)
				cv::circle(rgb, cv::Point(u, v), 10, cv::Scalar(0, 255, 0), 3);//green
			else if (i == 1)
				cv::circle(rgb, cv::Point(u, v), 10, cv::Scalar(0, 255, 0), 3);
			else if (i == 2)
				cv::circle(rgb, cv::Point(u, v), 10, cv::Scalar(0, 255, 0), 3);
			
			cv::putText(rgb, std::to_string(i) + ":(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")", cv::Point(u, v - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.0);
			
		
			record_blob_object.at<float>(0, i) = x;
			record_blob_object.at<float>(1, i) = y;
			record_blob_object.at<float>(2, i) = z;

		}
		//std::cout << "Recorded Blob Object Matrix:\n" << record_blob_object << std::endl;
		
		object2camera = kabsch(record_blob_object, dot_model_object);
		//std::cout << "object2camera\n" << object2camera << endl;

		for (int i = 0; i < blobsRGB2.points2D.size(); i++)
		{
			cout << "for xunhuan ing" << endl;
			int u = (int)blobsRGB2.points2D[i].x;
			int v = (int)blobsRGB2.points2D[i].y;
			int x = blobsRGB2.points3D[i].x;
			int y = blobsRGB2.points3D[i].y;
			int z = blobsRGB2.points3D[i].z;
			if (i == 0)
				cv::circle(rgb, cv::Point(u, v), 10, cv::Scalar(255, 0, 0), 3);
			else if (i == 1)
				cv::circle(rgb, cv::Point(u, v), 10, cv::Scalar(255, 0, 0), 3);                  //blue
			else if (i == 2)
				cv::circle(rgb, cv::Point(u, v), 10, cv::Scalar(255, 0, 0), 3);
			
		cv::putText(rgb, std::to_string(i) + ":(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")", cv::Point(u, v - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.0);
		
		record_blob_glasses.at<float>(0, i) = x;
		record_blob_glasses.at<float>(1, i) = y;
		record_blob_glasses.at<float>(2, i) = z;
		}
		//std::cout << "Recorded Blob Object Matrix:\n" << record_blob_object << std::endl;
		glasses2camera = kabsch(record_blob_glasses, dot_model_glasses);
		//std::cout << "glasses2camera\n" << glasses2camera << endl;

		for (int i = 0; i < blobsRGBAll.points2D.size(); i++)
		{
			for (int j = 0; j < blobsRGBAll.points2D[i].size(); j++)
			{
				cout << "for xunhuan ing" << endl;

				int u = (int)blobsRGBAll.points2D[i][j].x;
				int v = (int)blobsRGBAll.points2D[i][j].y;
				int x = blobsRGBAll.points3D[i][j].x;
				int y = blobsRGBAll.points3D[i][j].y;
				int z = blobsRGBAll.points3D[i][j].z;
				cv::circle(rgb, cv::Point(u, v), 3, cv::Scalar(0, 0, 255), 3);
				//cv::putText(rgb, std::to_string(i) + "_" + std::to_string(j) + ":(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")", cv::Point(u, v - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.0);
				
			}
		}
		/********************************************************************************/
		cv::imshow("blobResDebug", rgb);
		/********************************************************************************/
		
		if(res == MS_SUCCESS)
		{
			//计算帧数
			frameCnt++;
			//开始计算帧率
			if (frameCnt == 50)
			{
				time2 = cv::getTickCount();
				time = (time2-time1)/cv::getTickFrequency();
				cout << "fps: " << (frameCnt)/time << endl;

				frameCnt = 0;
				time1 = time2;
			}
			/*左边和右边的图像显示*/
			/*cv::imshow("left", left);
			if( mode == CAMERA_LR_HD)
			{
				cv::imshow("right", right);  
			}*/
			if( mode == CAMERA_LD || mode == CAMERA_LRD || mode == CAMERA_LD_HD || mode == CAMERA_LRD_HD)
			{
				if( bitDepth == 16 )
				{
					for(int j = 0 ; j < w*h; j++)							   
					{
						if(depth.data[j * 2 + 1] >1024/256 && depth.data[j * 2 + 1] < 30*1024 / 256)
							depthShow.data[j]	= depthFilter.data[j*2 + 1];
						else
							depthShow.data[j] = 0;
					}
					cv::imshow("depth", depthShow);
					colorDisparity(depthShow, depthShowColor);
					cv::imshow("depthShowColor", depthShowColor);
				}
				else
				{
					cv::imshow("disp", disparity);
				}
			}


			int key = cv::waitKey(5);

			////关闭所有窗口，退出当前循环，结束程序，这些 k ==‘ ’都可以删。
			//if (key == 'q' || key == 'Q')
			//{
			//	cv::destroyAllWindows();
			//	break;
			//}

			////空格保存图片
			//else if (key == ' ')
			//{
			//	cout << "img_cnt: " << imgIndex << endl;
			//
			//	char leftname[100], rightname[100], dispname[100];
			//	sprintf(leftname,  "image/left_%d.png",  imgIndex);
			//	cv::imwrite(leftname,  left);
			//		sprintf(rightname, "image/right_%d.png", imgIndex);
			//		cv::imwrite(rightname, right);
			//
			//	if( mode == CAMERA_LD || mode == CAMERA_LRD )
			//	{
			//		if( bitDepth == 16 )
			//		{
			//			sprintf(dispname,  "image/depth_%d.png",  imgIndex);
			//			cv::imwrite(dispname,  depth);
			//		}
			//		else
			//		{
			//			sprintf(dispname,  "image/disp_%d.png",  imgIndex);
			//			cv::imwrite(dispname,  disparity);
			//		}
			//	}
			//	imgIndex++;
			//}

		}
		else if (res == MS_TIMEOUT)
		{
			cout << "GetImageData: timeout" << endl;
		}
		else
		{
			cout << "GetImageData: failed" << endl;
		}


	}
	res = MS_SUCCESS; 
	camera->StopStream();
	camera->Close();
	delete camera;

	return  { object2camera ,glasses2camera };
}
