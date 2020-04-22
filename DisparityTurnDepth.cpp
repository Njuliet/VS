///*https://www.cnblogs.com/riddick/p/8486223.html
//函数作用：视差图转深度图
//输入：
//　　dispMap ----视差图，8位单通道，CV_8UC1
//  　　K       ----内参矩阵，float类型
//	输出：
//	　　depthMap ----深度图，16位无符号单通道，CV_16UC1
//	  */
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K)
{
	int type = dispMap.type();
	imshow("dispMap", dispMap);
	float fx = K.at<float>(0, 0);//读取内参矩阵中的数
	
	float baseline = 33.267335558390052; //基线距离65mm

	if (type == CV_8UC1)
	{
		
		int height = dispMap.rows;
		int width = dispMap.cols;

		uchar* dispData = (uchar*)dispMap.data;
		ushort* depthData = (ushort*)depthMap.data;
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int id = i*width + j;
				if (!dispData[id])  continue;  //防止0除
				depthData[id] = ushort((float)fx *baseline / ((float)dispData[id]));
			}
		}
	}
	else
	{
		cout << "please confirm dispImg's type!" << endl;
		cv::waitKey(0);
	}
}

int main()
{
	cv::Mat disparity = imread("SGBM_RGB_Disparity.jpg", 0);
	cout << disparity.cols;
	cout << disparity.rows;
	Mat cameraMatrixL = (Mat_<double>(3, 3) << 530.0393889065845, 0, 338.8877128635598,
		0, 530.2174709010319, 232.337554842746,
		0, 0, 1);

	cv::Mat depthMap(480,640, CV_16UC1);
	
	disp2Depth(disparity, depthMap, cameraMatrixL);

	imshow("depth", depthMap);
	imwrite("depth.png", depthMap);
	
	//-------收尾------
	waitKey(0);
	return 0;
}

//
//int main()
//{
//	cv::Mat disparity = imread("SGBM_RGB_Disparity.jpg",0);
//
//	Mat cameraMatrixL = (Mat_<double>(3, 3) << 530.0393889065845, 0, 338.8877128635598,
//		0, 530.2174709010319, 232.337554842746,
//		0, 0, 1);
//
//	cv::Mat depthMap(640, 480, CV_16UC1);
//	
//	float fx = cameraMatrixL.at<float>(0, 0);//读取内参矩阵中的数
//	
//	float baseline = 33.267335558390052; //基线距离65mm
//	int height = disparity.rows;
//
//	int width = disparity.cols;
//	depthMap.data = ushort((float)fx *baseline / ((float)disparity.data);
//			uchar* dispData = (uchar*)dispMap.data;
//			ushort* depthData = (ushort*)depthMap.data;
//			for (int i = 0; i < height; i++)
//			{
//				for (int j = 0; j < width; j++)
//				{
//					int id = i*width + j;
//					if (!dispData[id])  continue;  //防止0除
//					depthData[id] = ushort((float)fx *baseline / ((float)dispData[id]));
//				}
//			}
//
//	imshow("depth", depthMap);
//	imwrite("depth.png", depthMap);
//
//	//-------收尾------
//	waitKey(0);
//	return 0;
//}