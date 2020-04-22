///*https://www.cnblogs.com/riddick/p/8486223.html
//�������ã��Ӳ�ͼת���ͼ
//���룺
//����dispMap ----�Ӳ�ͼ��8λ��ͨ����CV_8UC1
//  ����K       ----�ڲξ���float����
//	�����
//	����depthMap ----���ͼ��16λ�޷��ŵ�ͨ����CV_16UC1
//	  */
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K)
{
	int type = dispMap.type();
	imshow("dispMap", dispMap);
	float fx = K.at<float>(0, 0);//��ȡ�ڲξ����е���
	
	float baseline = 33.267335558390052; //���߾���65mm

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
				if (!dispData[id])  continue;  //��ֹ0��
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
	
	//-------��β------
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
//	float fx = cameraMatrixL.at<float>(0, 0);//��ȡ�ڲξ����е���
//	
//	float baseline = 33.267335558390052; //���߾���65mm
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
//					if (!dispData[id])  continue;  //��ֹ0��
//					depthData[id] = ushort((float)fx *baseline / ((float)dispData[id]));
//				}
//			}
//
//	imshow("depth", depthMap);
//	imwrite("depth.png", depthMap);
//
//	//-------��β------
//	waitKey(0);
//	return 0;
//}