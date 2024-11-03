#include "StereoBlob.h"
#include "json.hpp"
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>  
//#include <opencv2/highgui/highgui.hpp>  
//#include <opencv2/imgproc.hpp>

using json = nlohmann::json;


/*restrict  约束      filter 过滤      stereo 立体*/

using namespace cv;

namespace stereoai
{
    void colorDisparity(const cv::Mat msrc, cv::Mat& mdst, int nmax)
    {
        double ninterval = 300. / nmax;
        int alpha = nmax * 0.8;
        mdst.create(msrc.size(), CV_8UC3);
        int ar[] = { 0, 119, 119, 110, 110, 102, 102, 93, 93, 85, 85, 76, 76, 68,
            68, 59, 59, 51, 51, 42, 42, 34, 34, 25, 25, 17, 17, 8, 8, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 17, 17, 25, 25, 34, 34, 42, 42,
            51, 51, 59, 59, 68, 68, 76, 76, 85, 85, 93, 93, 102, 102, 110,
            110, 119, 119, 127, 127, 135, 135, 144, 144, 152, 152, 161, 161,
            169, 169, 178, 178, 186, 186, 195, 195, 203, 203, 212, 212, 220,
            220, 229, 229, 237, 237, 246, 246, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254 };
        int ag[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 17, 17, 25, 25, 34, 34, 42,
            42, 51, 51, 59, 59, 68, 68, 76, 76, 85, 85, 93, 93, 102, 102, 110,
            110, 119, 119, 127, 127, 135, 135, 144, 144, 152, 152, 161, 161,
            169, 169, 178, 178, 186, 186, 195, 195, 203, 203, 212, 212, 220,
            220, 229, 229, 237, 237, 246, 246, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 246,
            246, 237, 237, 229, 229, 220, 220, 212, 212, 203, 203, 195, 195,
            186, 186, 178, 178, 169, 169, 161, 161, 152, 152, 144, 144, 135,
            135, 127, 127, 119, 119, 110, 110, 102, 102, 93, 93, 85, 85, 76,
            76, 68, 68, 59, 59, 51, 51, 42, 42, 34, 34, 25, 25, 17, 17, 8, 8,
            0, 0, 0, 8, 8, 17, 17, 25, 25, 34, 34, 42, 42, 51, 51, 59, 59, 68,
            68, 76, 76, 85, 85, 93, 93, 102, 102, 110, 110, 119, 119, 127 };
        int ab[] = { 0, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
            246, 246, 237, 237, 229, 229, 220, 220, 212, 212, 203, 203, 195,
            195, 186, 186, 178, 178, 169, 169, 161, 161, 152, 152, 144, 144,
            135, 135, 127, 127, 119, 119, 110, 110, 102, 102, 93, 93, 85, 85,
            76, 76, 68, 68, 59, 59, 51, 51, 42, 42, 34, 34, 25, 25, 17, 17, 8,
            8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        for (int _row = 0; _row < msrc.rows; _row++)
        {
            for (int _col = 0; _col < msrc.cols; _col++)
            {
                int r, g, b;
                int disp = msrc.at<uchar>(_row, _col);
                if (disp > nmax)
                {
                    r = ar[300];
                    g = ag[300];
                    b = ab[300];
                }
                else
                {
                    int idx = (int)disp * ninterval;
                    r = ar[idx];
                    g = ag[idx];
                    b = ab[idx];
                }
                mdst.at<cv::Vec3b>(_row, _col) = cv::Vec3b(b, g, r);
            }
        }

    }
    void SpeckleCheck(unsigned short int* src, unsigned short int* dst, const int& width, const int& height, int speckleRange/* = 200*/, int cntSize/* = 200 */)
    {
        memcpy(dst, src, width * height * sizeof(unsigned short int));

        short newVal = 0;
        int _row, _col;

        int npixels = width * height;
        size_t bufSize = npixels * (int)(sizeof(int) * 2 + sizeof(int) + sizeof(uchar));
        uchar* _buf;
        _buf = new uchar[(int)bufSize];

        uchar* buf = _buf;
        int* labels = (int*)buf;//the label of points, noise or not

        int intSize = npixels * sizeof(int);
        buf += intSize;
        int* rcdPoints_x = (int*)buf;
        int* rcdPoints_y = (int*)buf + 1;

        buf += 2 * intSize;
        uchar* delePoints = (uchar*)buf; //label the noise pixel which should be delete

        int currLable = 0;
        memset(labels, 0, intSize);

        for (_row = 0; _row < height; ++_row)
        {
            int rowTmp = _row * width;
            unsigned short* dispPix = dst + rowTmp;
            int* lable_1 = labels + rowTmp; //label the group index of pixels

            for (_col = 0; _col < width; _col++)
            {
                if ((unsigned short)(dispPix[_col]) != newVal)   // not a bad disparity
                {
                    if (lable_1[_col])     //each pixel has a label, which illustrates whether the pixel is a noise or not
                    {
                        if (delePoints[lable_1[_col]]) // small region, zero out disparity
                            dispPix[_col] = (short)newVal; //zero the pixel
                    }
                    else
                    {
                        int* rp_x = rcdPoints_x;
                        int* rp_y = rcdPoints_y;

                        int currp_x = _col;
                        int currp_y = _row;

                        currLable++; // group label
                        int count = 0;  // current region size
                        lable_1[_col] = currLable;

                        while (rp_x >= rcdPoints_x)
                        {
                            count++;
                            int currRowTmp = currp_y * width;
                            unsigned short int* dpp = &dst[currRowTmp + currp_x];

                            unsigned short dp = *dpp; //the depth of the current pixel
                            int* lable_2 = labels + currRowTmp + currp_x;

                            //the pixel below the current location
                            if (currp_y < height - 1 && !lable_2[+width] && (unsigned short)(dpp[+width]) != newVal && abs(dp - (unsigned short)(dpp[+width])) <= speckleRange)
                            {
                                lable_2[+width] = currLable;
                                *rp_x = currp_x;
                                *rp_y = currp_y + 1;
                                rp_x += 2;
                                rp_y += 2;
                            }

                            //the pixel above the current location
                            if (currp_y > 0 && !lable_2[-width] && (unsigned short)(dpp[-width]) != newVal && abs(dp - (unsigned short)(dpp[-width])) <= speckleRange)
                            {
                                lable_2[-width] = currLable;
                                *rp_x = currp_x;
                                *rp_y = currp_y - 1;
                                rp_x += 2;
                                rp_y += 2;
                            }

                            //the pixel behind the current location
                            if (currp_x < width - 1 && !lable_2[+1] && (unsigned short)(dpp[+1]) != newVal && abs(dp - (unsigned short)(dpp[+1])) <= speckleRange)
                            {
                                lable_2[+1] = currLable;
                                *rp_x = currp_x + 1;
                                *rp_y = currp_y;
                                rp_x += 2;
                                rp_y += 2;
                            }

                            //the pixel before the current location
                            if (currp_x > 0 && !lable_2[-1] && (unsigned short)(dpp[-1]) != newVal && abs(dp - (unsigned short)(dpp[-1])) <= speckleRange)
                            {
                                lable_2[-1] = currLable;
                                *rp_x = currp_x - 1;
                                *rp_y = currp_y;
                                rp_x += 2;
                                rp_y += 2;
                            }
                            rp_x -= 2;
                            rp_y -= 2;
                            currp_x = *rp_x;
                            currp_y = *rp_y;
                        }

                        if (count <= cntSize)   // speckle region, the noises
                        {
                            delePoints[lable_1[_col]] = 1;   // small region label
                            dispPix[_col] = (short)newVal;
                        }
                        else
                        {
                            delePoints[lable_1[_col]] = 0;   // large region label
                        }
                    }
                }
            }
        }
        delete _buf;
    }
    cv::Mat kabsch(Mat dot2camera, Mat dot_model)
    {
        //Mat P = (Mat_<float>(3, 3) << 39.5157, -25.9335, 24.0632, 99.9103, 61.0845, 60.7504, 142.8585, 87.5094, 87.0324);
        Mat H = (Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
        Mat U, W, VT, R, T;
        Mat  sigma = (Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
        Mat  s = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        Mat temp = (Mat_<float>(3, 3));
        Vec<float, 3> pp(0, 0, 0);
        Vec<float, 3> pq(0, 0, 0);
        Vec<float, 3> q[3], q1[3];

        //计算质心
        for (int i = 0; i < 9; i++)
        {
            pp(i / 3) += dot_model.at<float>(i) / 3;
            pq(i / 3) += dot2camera.at<float>(i) / 3;
        }
        //求各点相对于质心的位移向量
        for (int i = 0; i < 9; i++)
        {
            q[i % 3](i / 3) = dot_model.at<float>(i) - pp(i / 3);
            q1[i % 3](i / 3) = dot2camera.at<float>(i) - pq(i / 3);
        }
        //利用质心位移向量，计算H矩阵（质点的协方差矩阵）
        for (int i = 0; i < 3; i++)
        {
            Mat q_mat = (Mat_<float>(3, 1) << q[i][0], q[i][1], q[i][2]);
            Mat q1_mat = (Mat_<float>(3, 1) << q1[i][0], q1[i][1], q1[i][2]);
            Mat temp = q_mat * q1_mat.t();  // 将Vec转换为Mat进行矩阵乘法
            H += temp;
        }
        //对H进行奇异值分解
        SVD::compute(H, W, U, VT);
        for (int i = 0; i < 3; i++)
        {
            sigma.at<float>(i, i) = W.at<float>(i);
        }

         //基于矩阵U和V，计算旋转矩阵R
        s.at<float>(2, 2) = (float)determinant(U * VT);
        //cout << s << endl;
        R = U * s * VT;
        R = R.inv();//我也不知道为什么要求转置，但是算出来的矩阵就是反的
        //计算原点的位移
        Mat pp_mat = Mat(pp).reshape(1, 3);
        Mat pq_mat = Mat(pq).reshape(1, 3);

        T = R * pp_mat;  // 使用Mat对象进行矩阵乘法
        T = pq_mat - T;  // 更新T的计算

        cv::Mat C, D, E = (Mat_<float>(1, 4) << 0, 0, 0, 1);
        cv::hconcat(R, T, C);
        cv::vconcat(C, E, D);
        return D;
    }







    GeometryConstraint::GeometryConstraint(std::string fileName)
    {
        mCfgName = fileName;
        loadConstraint();
    }
    int GeometryConstraint::loadConstraint()
    {
        mRelations.clear();
        std::ifstream inf_cfg(mCfgName);
        if (!inf_cfg.good())
        {
            return -1;
        }
        json cfg_j;
        inf_cfg >> cfg_j;
        inf_cfg.close();

        mNodeNum = cfg_j["nodeNum"];
        for (int i = 0; i < cfg_j["constrains"].size(); i++)
        {
            NodeRelation nodeRealtion;
            nodeRealtion.startIndex = cfg_j["constrains"][i]["startIndex"];
            nodeRealtion.endIndex = cfg_j["constrains"][i]["endIndex"];
            
            nodeRealtion.dist = cfg_j["constrains"][i]["distance"];
            if (nodeRealtion.startIndex == 0 && nodeRealtion.endIndex == 1)
                dist01 = nodeRealtion.dist;
            if (nodeRealtion.startIndex == 0 && nodeRealtion.endIndex == 2)
                dist02 = nodeRealtion.dist;
            if (nodeRealtion.startIndex == 1 && nodeRealtion.endIndex == 2)
                dist12 = nodeRealtion.dist;
            mRelations.push_back(nodeRealtion);
        }
        mJudgerThresh = cfg_j["judgerThresh"];
        mEpipolarThresh = cfg_j["epipolarThresh"];
        mMinDepth = cfg_j["minDepth"];
        mMaxDepth = cfg_j["maxDepth"];
        return 0;
    }

    void GeometryConstraint::print()
    {
        std::cout << "JudgerThresh: " << mJudgerThresh << std::endl;
        std::cout << "epipolarThresh: " << mEpipolarThresh << std::endl;
        std::cout << "minDepth: " << mMinDepth << std::endl;
        std::cout << "maxDepth: " << mMaxDepth << std::endl;
    }



    StereoBlobCentersPair StereoBlob::getStereoBlobsPairs(std::vector<cv::Point2f> &blobCentersLeft, std::vector<cv::Point2f> &blobCentersRight)
    {
        StereoBlobCentersPair stereoPaired;
        //step 0 : basic check
        if (blobCentersLeft.size() == 0 || blobCentersRight.size() == 0)
        {
            return stereoPaired;
        }

        //step 1 : 匹配矩阵  左相机M个点和右相机N个点进行极线约束配准
        int numLeft = blobCentersLeft.size();
        int numRight = blobCentersRight.size();
        std::vector<std::vector<int>> relations(numLeft); //第一轮: 以左相机的点作为参考，遍历所有满足条件的

        float maxDisparity = mStereoParams.fx * mStereoParams.b / mGeometryConstraint.mMinDepth;
        float minDisparity = mStereoParams.fy * mStereoParams.b / mGeometryConstraint.mMaxDepth;
        for (int i = 0; i < numLeft; i++)
        {
            cv::Point2f &PLeft = blobCentersLeft[i];
            std::vector<int> matchedIndexes;
            for (int j = 0; j < numRight; j++)
            {
                cv::Point2f &PRight = blobCentersRight[j];
                float epipolar = PLeft.y - PRight.y;
                float disparity = PLeft.x - PRight.x;
               
                if ((disparity >= minDisparity) && (disparity <= maxDisparity) && (abs(epipolar) < mGeometryConstraint.mEpipolarThresh))
                {
                    matchedIndexes.push_back(j);

                }
            }
            relations[i] = matchedIndexes;
        }
        //step 2 :计算出所有可能的三维点坐标以及齐依赖的左图索引和右图索引
        for (int i = 0; i < numLeft; i++)
        {
            cv::Point2f &PLeft = blobCentersLeft[i];
            std::vector<cv::Point3f> points3D;
            std::vector<cv::Point2f> right2D;
            for (int j = 0; j < relations[i].size(); j++)
            {
               
                cv::Point2f &PRight = blobCentersRight[relations[i][j]];
                float disparity = PLeft.x - PRight.x;
                float z = mStereoParams.fx * mStereoParams.b / disparity;
                float x = (PLeft.x - mStereoParams.cx) / mStereoParams.fx * z;
                float y = (PLeft.y - mStereoParams.cy) / mStereoParams.fy * z;
                right2D.push_back(PRight);
                points3D.push_back(cv::Point3f(x, y, z));
            }
            if (right2D.size() > 0)
            {
                stereoPaired.mBlobCenters3D.push_back(points3D);
                stereoPaired.mBlobCentersRightPaired.push_back(right2D);
                stereoPaired.mBlobCentersLeftPaired.push_back(PLeft);
            }
        }
       
        return stereoPaired;
    }

	StereoBlobCentersPair StereoBlob::getStereoBlobsPairs2(std::vector<cv::Point2f>& blobCentersLeft, std::vector<cv::Point2f>& blobCentersRight)
	{
		StereoBlobCentersPair stereoPaired;
		//step 0 : basic check
		if (blobCentersLeft.size() == 0 || blobCentersRight.size() == 0)
		{
			return stereoPaired;
		}

		//step 1 : 匹配矩阵  左相机M个点和右相机N个点进行极线约束配准
		int numLeft = blobCentersLeft.size();
		int numRight = blobCentersRight.size();
		std::vector<std::vector<int>> relations(numLeft); //第一轮: 以左相机的点作为参考，遍历所有满足条件的

		float maxDisparity = mStereoParams.fx * mStereoParams.b / mGeometryConstraint2.mMinDepth;
		float minDisparity = mStereoParams.fy * mStereoParams.b / mGeometryConstraint2.mMaxDepth;
		for (int i = 0; i < numLeft; i++)
		{
			cv::Point2f &PLeft = blobCentersLeft[i];
			std::vector<int> matchedIndexes;
			for (int j = 0; j < numRight; j++)
			{
				cv::Point2f &PRight = blobCentersRight[j];
				float epipolar = PLeft.y - PRight.y;
				float disparity = PLeft.x - PRight.x;

				if ((disparity >= minDisparity) && (disparity <= maxDisparity) && (abs(epipolar) < mGeometryConstraint2.mEpipolarThresh))
				{
					matchedIndexes.push_back(j);

				}
			}
			relations[i] = matchedIndexes;
		}
		//step 2 :计算出所有可能的三维点坐标以及齐依赖的左图索引和右图索引
		for (int i = 0; i < numLeft; i++)
		{
			cv::Point2f &PLeft = blobCentersLeft[i];
			std::vector<cv::Point3f> points3D;
			std::vector<cv::Point2f> right2D;
			for (int j = 0; j < relations[i].size(); j++)
			{

				cv::Point2f &PRight = blobCentersRight[relations[i][j]];
				float disparity = PLeft.x - PRight.x;
				float z = mStereoParams.fx * mStereoParams.b / disparity;
				float x = (PLeft.x - mStereoParams.cx) / mStereoParams.fx * z;
				float y = (PLeft.y - mStereoParams.cy) / mStereoParams.fy * z;
				right2D.push_back(PRight);
				points3D.push_back(cv::Point3f(x, y, z));
			}
			if (right2D.size() > 0)
			{
				stereoPaired.mBlobCenters3D.push_back(points3D);
				stereoPaired.mBlobCentersRightPaired.push_back(right2D);
				stereoPaired.mBlobCentersLeftPaired.push_back(PLeft);
			}
		}

		return stereoPaired;
	}

    void StereoBlob::update(cv::Mat& left, cv::Mat& right, Blob3DPoints& blobsReg)
    {
    
        blobsReg.points2D.clear();
        blobsReg.points3D.clear();
        std::vector<cv::Point2f> blobLeft;
        std::vector<cv::Point2f> blobRight;
        mBlob.getBlobCenters(right, left, blobLeft, blobRight);
        StereoBlobCentersPair stereoPaired = getStereoBlobsPairs(blobLeft, blobRight);
        for (int i = 0; i < stereoPaired.mBlobCentersLeftPaired.size(); i++)
        {
            std::vector<cv::Point2f> points2D;
            std::vector<cv::Point3f> points3D;
            for (int j = 0; j < stereoPaired.mBlobCenters3D[i].size(); j++)
            {
                float x = stereoPaired.mBlobCenters3D[i][j].x;
                float y = stereoPaired.mBlobCenters3D[i][j].y;
                float z = stereoPaired.mBlobCenters3D[i][j].z; 
                points2D.push_back(stereoPaired.mBlobCentersLeftPaired[i]);
                points3D.push_back(cv::Point3f(x, y, z));
            }
            if (points2D.size() > 0)
            {
                blobsReg.points2D.push_back(points2D);
                blobsReg.points3D.push_back(points3D);
            }
        }
        
    }

    Blob3DPointsRes StereoBlob::updateWithGeometryConstraint(cv::Mat& rawRGB, cv::Mat& depth)
    {

        Blob3DPointsRes blob3DPointsRes;
        std::vector<cv::Point2f> blobLeft;
        std::vector<cv::Point2f> blobRight;
        mBlob.getBlobCenters(depth, rawRGB,  blobLeft, blobRight);
        StereoBlobCentersPair stereoPaired = getStereoBlobsPairs(blobLeft, blobRight);

        Blob3DPoints blobsRegBeforeFilter;
        for (int i = 0; i < stereoPaired.mBlobCentersLeftPaired.size(); i++)
        {
            std::vector<cv::Point2f> points2D;
            std::vector<cv::Point3f> points3D;
            for (int j = 0; j < stereoPaired.mBlobCenters3D[i].size(); j++)
            {
                float x = stereoPaired.mBlobCenters3D[i][j].x;
                float y = stereoPaired.mBlobCenters3D[i][j].y;
                float z = stereoPaired.mBlobCenters3D[i][j].z;
                points2D.push_back(stereoPaired.mBlobCentersLeftPaired[i]);
                points3D.push_back(cv::Point3f(x, y, z));
            }
            if (points2D.size() > 0)
            {
                blobsRegBeforeFilter.points2D.push_back(points2D);
                blobsRegBeforeFilter.points3D.push_back(points3D);
            }
        }
        //进行空间约束找出设定数量的小球,暴力寻找三个之间的约束

        std::vector<cv::Point3f> points3DAll;
        std::vector<int> indexOfI;
        std::vector<int> indexOfJ;
        for (int i = 0; i < blobsRegBeforeFilter.points3D.size(); i++)
            for (int j = 0; j < blobsRegBeforeFilter.points3D[i].size(); j++)
            {
                points3DAll.push_back(blobsRegBeforeFilter.points3D[i][j]);
                indexOfI.push_back(i);
                indexOfJ.push_back(j);
            }
       //寻找第0个点
        std::vector<int> index0Res;
        std::vector<int> index1Res;
        std::vector<int> index2Res;
        for (int i = 0; i < points3DAll.size(); i++)
        {
            cv::Point3f p1 = points3DAll[i];
            std::vector<int> index01_j;
            std::vector<int> index02_j;
            for (int j = 0; j < points3DAll.size(); j++)
            {
                if (i == j) continue;
                cv::Point3f p2 = points3DAll[j];
                float dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
                if (abs(dist - mGeometryConstraint.dist01) < mGeometryConstraint.mJudgerThresh)
                {
                    index01_j.push_back(j);
                  //  std::cout << "find 01" << std::endl;
                }

                if (abs(dist - mGeometryConstraint.dist02) < mGeometryConstraint.mJudgerThresh)
                {
                    index02_j.push_back(j);
                //    std::cout << "find 02" << std::endl;
                }
            }
            if (index01_j.size() > 0 && index02_j.size()) 
            {
                //std::cout << "find 01 and 02" << std::endl;
                for (int m = 0; m < index01_j.size(); m++)
                {
                    int index1 = index01_j[m];
                    for (int n = 0; n < index02_j.size(); n++)
                    {
                        int index2 = index02_j[n];
                        cv::Point3f p1 = points3DAll[index1], p2 = points3DAll[index2];
                        float dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));

                        if (abs(dist - mGeometryConstraint.dist12) < mGeometryConstraint.mJudgerThresh)
                        {
                            index0Res.push_back(i);
                            index1Res.push_back(index1);
                            index2Res.push_back(index2);
                           // std::cout << " json1 object     find 01 and 02 and 12" << std::endl;
                        }
                    }
                }
            }
        }
        
        if (index0Res.size() == 1)
        {
            blob3DPointsRes.points2D.push_back(blobsRegBeforeFilter.points2D[indexOfI[index0Res[0]]][indexOfJ[index0Res[0]]]);
            blob3DPointsRes.points2D.push_back(blobsRegBeforeFilter.points2D[indexOfI[index1Res[0]]][indexOfJ[index1Res[0]]]);
            blob3DPointsRes.points2D.push_back(blobsRegBeforeFilter.points2D[indexOfI[index2Res[0]]][indexOfJ[index2Res[0]]]);

            blob3DPointsRes.points3D.push_back(blobsRegBeforeFilter.points3D[indexOfI[index0Res[0]]][indexOfJ[index0Res[0]]]);
            blob3DPointsRes.points3D.push_back(blobsRegBeforeFilter.points3D[indexOfI[index1Res[0]]][indexOfJ[index1Res[0]]]);
            blob3DPointsRes.points3D.push_back(blobsRegBeforeFilter.points3D[indexOfI[index2Res[0]]][indexOfJ[index2Res[0]]]);
        }
        return blob3DPointsRes;
      
    }

	Blob3DPointsRes StereoBlob::updateWithGeometryConstraint2(cv::Mat & rawRGB, cv::Mat & depth)
	{
		Blob3DPointsRes blob3DPointsRes;
		std::vector<cv::Point2f> blobLeft;
		std::vector<cv::Point2f> blobRight;
		mBlob2.getBlobCenters2(depth, rawRGB, blobLeft, blobRight);
		StereoBlobCentersPair stereoPaired = getStereoBlobsPairs2(blobLeft, blobRight);

		Blob3DPoints blobsRegBeforeFilter;
		for (int i = 0; i < stereoPaired.mBlobCentersLeftPaired.size(); i++)
		{
			std::vector<cv::Point2f> points2D;
			std::vector<cv::Point3f> points3D;
			for (int j = 0; j < stereoPaired.mBlobCenters3D[i].size(); j++)
			{
				float x = stereoPaired.mBlobCenters3D[i][j].x;
				float y = stereoPaired.mBlobCenters3D[i][j].y;
				float z = stereoPaired.mBlobCenters3D[i][j].z;
				points2D.push_back(stereoPaired.mBlobCentersLeftPaired[i]);
				points3D.push_back(cv::Point3f(x, y, z));
			}
			if (points2D.size() > 0)
			{
				blobsRegBeforeFilter.points2D.push_back(points2D);
				blobsRegBeforeFilter.points3D.push_back(points3D);
			}
		}
		//进行空间约束找出设定数量的小球,暴力寻找三个之间的约束

		std::vector<cv::Point3f> points3DAll;
		std::vector<int> indexOfI;
		std::vector<int> indexOfJ;
		for (int i = 0; i < blobsRegBeforeFilter.points3D.size(); i++)
			for (int j = 0; j < blobsRegBeforeFilter.points3D[i].size(); j++)
			{
				points3DAll.push_back(blobsRegBeforeFilter.points3D[i][j]);
				indexOfI.push_back(i);
				indexOfJ.push_back(j);
			}
		//寻找第0个点
		std::vector<int> index0Res;
		std::vector<int> index1Res;
		std::vector<int> index2Res;
		for (int i = 0; i < points3DAll.size(); i++)
		{
			cv::Point3f p1 = points3DAll[i];
			std::vector<int> index01_j;
			std::vector<int> index02_j;
			for (int j = 0; j < points3DAll.size(); j++)
			{
				if (i == j) continue;
				cv::Point3f p2 = points3DAll[j];
				float dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
				if (abs(dist - mGeometryConstraint2.dist01) < mGeometryConstraint2.mJudgerThresh)
				{
					index01_j.push_back(j);
					//std::cout << "find 01" << std::endl;
				}

				if (abs(dist - mGeometryConstraint2.dist02) < mGeometryConstraint2.mJudgerThresh)
				{
					index02_j.push_back(j);
					//std::cout << "find 02" << std::endl;
				}
			}
			if (index01_j.size() > 0 && index02_j.size())
			{
				//std::cout << "find 01 and 02" << std::endl;
				for (int m = 0; m < index01_j.size(); m++)
				{
					int index1 = index01_j[m];
					for (int n = 0; n < index02_j.size(); n++)
					{
						int index2 = index02_j[n];
						cv::Point3f p1 = points3DAll[index1], p2 = points3DAll[index2];
						float dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));

						if (abs(dist - mGeometryConstraint2.dist12) < mGeometryConstraint2.mJudgerThresh)
						{
							index0Res.push_back(i);
							index1Res.push_back(index1);
							index2Res.push_back(index2);
							//std::cout << "                   json2 glasses      find 01 and 02 and 12" << std::endl;
						}
					}
				}
			}
		}

		if (index0Res.size() == 1)
		{
			blob3DPointsRes.points2D.push_back(blobsRegBeforeFilter.points2D[indexOfI[index0Res[0]]][indexOfJ[index0Res[0]]]);
			blob3DPointsRes.points2D.push_back(blobsRegBeforeFilter.points2D[indexOfI[index1Res[0]]][indexOfJ[index1Res[0]]]);
			blob3DPointsRes.points2D.push_back(blobsRegBeforeFilter.points2D[indexOfI[index2Res[0]]][indexOfJ[index2Res[0]]]);

			blob3DPointsRes.points3D.push_back(blobsRegBeforeFilter.points3D[indexOfI[index0Res[0]]][indexOfJ[index0Res[0]]]);
			blob3DPointsRes.points3D.push_back(blobsRegBeforeFilter.points3D[indexOfI[index1Res[0]]][indexOfJ[index1Res[0]]]);
			blob3DPointsRes.points3D.push_back(blobsRegBeforeFilter.points3D[indexOfI[index2Res[0]]][indexOfJ[index2Res[0]]]);
		}
		return blob3DPointsRes;
	}

    StereoParams::StereoParams(float* setereoData, float* rgbData)
    {
        fx = setereoData[36];
        fy = setereoData[37];
        cx = setereoData[38];
        cy = setereoData[39];
        b =  setereoData[40]*-1.0;//mm

        fx_rgb = rgbData[18 + 18];
        fy_rgb = rgbData[19 + 18];
        cx_rgb = rgbData[20 + 18];
        cy_rgb = rgbData[21 + 18];

        R = cv::Mat::zeros(3,3,CV_32FC1);
        T = cv::Mat::zeros(3, 1, CV_32FC1);
        R.at<float>(0, 0) = rgbData[41];
        R.at<float>(0, 1) = rgbData[42];
        R.at<float>(0, 2) = rgbData[43];
        R.at<float>(1, 0) = rgbData[44];
        R.at<float>(1, 1) = rgbData[45];
        R.at<float>(1, 2) = rgbData[46];
        R.at<float>(2, 0) = rgbData[47];
        R.at<float>(2, 1) = rgbData[48];
        R.at<float>(2, 2) = rgbData[49];

        T.at<float>(0, 0) = rgbData[50];//mm
        T.at<float>(1, 0) = rgbData[51];
        T.at<float>(2, 0) = rgbData[52];


        std::cout << R << T << std::endl;
    }

    StereoParams::StereoParams(float f, float _cx, float _cy,float _b)
    {
        fx = f;
        fy = f;
        cx = _cx;
        cy = _cy;
        b = _b;//mm
    }

    void  StereoBlob::drawBlobRGB(cv::Mat& rawRGB, Blob3DPoints& blobsReg)
    {
        cv::Mat rgb;
        cv::cvtColor(rawRGB, rgb, cv::COLOR_GRAY2RGB);
        for (int i = 0; i < blobsReg.points2D.size(); i++)
        {
            for (int j = 0; j < blobsReg.points2D[i].size(); j++)
            {
                int u = (int)blobsReg.points2D[i][j].x;
                int v = (int)blobsReg.points2D[i][j].y;
                int x = blobsReg.points3D[i][j].x;
                int y = blobsReg.points3D[i][j].y;
                int z = blobsReg.points3D[i][j].z;
                cv::circle(rgb, cv::Point(u,v), 3,cv::Scalar(0,0,255),3 );
                cv::putText(rgb,std::to_string(i)+"_"+ std::to_string(j) + ":(" + std::to_string(x) +  ","+ std::to_string(y) + ","+ std::to_string(z) + ")", cv::Point(u, v - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0,255), 1.0);

            }
        }
        cv::imshow("blobDebug", rgb);
    }
    void StereoBlob::drawBlobRGB(cv::Mat& rawRGB, Blob3DPointsRes& Blob3DPointsRes)
    {
        cv::Mat rgb;
        cv::cvtColor(rawRGB, rgb, cv::COLOR_BayerGR2RGB);
        for (int i = 0; i < Blob3DPointsRes.points2D.size(); i++)
        {
                int u = (int)Blob3DPointsRes.points2D[i].x;
                int v = (int)Blob3DPointsRes.points2D[i].y;
                int x = Blob3DPointsRes.points3D[i].x;
                int y = Blob3DPointsRes.points3D[i].y;
                int z = Blob3DPointsRes.points3D[i].z;
                if(i == 0)
                 cv::circle(rgb, cv::Point(u, v), 3, cv::Scalar(0, 0, 255), 3);
                else if (i == 1)
                    cv::circle(rgb, cv::Point(u, v), 3, cv::Scalar(0, 255, 0), 3);
                else if (i == 2)
                    cv::circle(rgb, cv::Point(u, v), 3, cv::Scalar(255, 0, 0), 3);
                cv::putText(rgb, std::to_string(i)  + ":(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")", cv::Point(u, v - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.0);
        }
        cv::imshow("blobResDebug", rgb);
    }

};