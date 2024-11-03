#pragma once

#include <vector>
#include "Blob.h"


//using namespace cv;
namespace stereoai
{
    void colorDisparity(const cv::Mat msrc, cv::Mat& mdst, int nmax = 120);
    void SpeckleCheck(unsigned short int* src, unsigned short int* dst, const int& width, const int& height, int speckleRange = 200, int cntSize = 200);

     cv::Mat kabsch(cv::Mat dot2camera, cv::Mat dot_model);

    struct Point3D {
        float x, y, z;
    };
    
    struct NodeRelation
    {
        int startIndex;
        int endIndex;
        float dist;
    };

    class GeometryConstraint
    {
    public:
        GeometryConstraint(std::string fileName);
        int loadConstraint();
        void print();

    public:
        std::string mCfgName;
        int mNodeNum;
        std::vector<NodeRelation> mRelations;
        float mJudgerThresh;
        float mEpipolarThresh;
        float mMinDepth;
        float mMaxDepth;

    //only surport 3 blobs
        float dist01;
        float dist02;
        float dist12;
    };

    struct Blob3DPoints 
    {
        std::vector<std::vector<cv::Point2f>> points2D;
        std::vector<std::vector<cv::Point3f>> points3D;
    };


    struct Blob3DPointsRes
    {
        std::vector<cv::Point2f > points2D;
        std::vector<cv::Point3f> points3D;
    };

    struct StereoParams
    {
        float fx;
        float fy;
        float cx;
        float cy;
        float b;
        float minDepth;
        float maxDepth;

        float fx_rgb;
        float fy_rgb;
        float cx_rgb;
        float cy_rgb;

        cv::Mat R;  //从左图坐标系变换至RGB图像坐标系的旋转矩阵
        cv::Mat T;  //从左图坐标系变换至RGB图像坐标系的平移矩阵
        StereoParams(float *setereoData,float *rgbData);
        StereoParams(float f, float cx, float cy,float b);
    };

    //存在1对多问题
    struct StereoBlobCentersPair
    {
        std::vector<cv::Point2f> mBlobCentersLeftPaired;
        std::vector<std::vector<cv::Point2f>> mBlobCentersRightPaired;
        std::vector<std::vector<cv::Point3f>> mBlobCenters3D;
    };

    class StereoBlob
    {
    public:
        StereoBlob(std::string constraintFile, std::string constraintFile2,float *stereoData,float *rgbData):mGeometryConstraint(constraintFile), mGeometryConstraint2(constraintFile2), mStereoParams(stereoData, rgbData)
        {

        }
        StereoBlob(std::string constraintFile, std::string constraintFile2,float f, float cx, float cy,float b) :mGeometryConstraint(constraintFile), mGeometryConstraint2(constraintFile2), mStereoParams(f,cx,cy,b)
        {

        }
        ~StereoBlob() {}

    public:
        void update(cv::Mat& rawRGB, cv::Mat& depth, Blob3DPoints &blobs);
        Blob3DPointsRes updateWithGeometryConstraint(cv::Mat& rawRGB, cv::Mat& depth);
		Blob3DPointsRes updateWithGeometryConstraint2(cv::Mat& rawRGB, cv::Mat& depth);
        StereoBlobCentersPair getStereoBlobsPairs(std::vector<cv::Point2f> &blobCentersLeft, std::vector<cv::Point2f> &blobCentersRight);
		StereoBlobCentersPair getStereoBlobsPairs2(std::vector<cv::Point2f> &blobCentersLeft, std::vector<cv::Point2f> &blobCentersRight);

    public:
        GeometryConstraint mGeometryConstraint;
		GeometryConstraint mGeometryConstraint2;
        StereoParams mStereoParams;
        Blob mBlob;
		Blob mBlob2;
    public: 
        void drawBlobRGB(cv::Mat &rawRGB, Blob3DPoints& blobsReg);
        void drawBlobRGB(cv::Mat& rawRGB, Blob3DPointsRes& Blob3DPointsRes);
    };
};