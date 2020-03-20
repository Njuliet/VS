///*
//���������
//cam0 = [4152.073 0 1288.147; 0 4152.073 973.571; 0 0 1]
//cam1 = [4152.073 0 1501.231; 0 4152.073 973.571; 0 0 1]
//doffs = 213.084
//baseline = 176.252
//width = 2872
//height = 1984
//����ڲ�������
//K=[fx 0 u0; 0 fy v0; 0 0 1]
//
//doffs = |u1 - u0|
//*/
//
//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>  
//#include <pcl/io/io.h>  
//#include <pcl/io/pcd_io.h>  
//#include <opencv2/opencv.hpp>  
//
//using namespace cv;
//using namespace std;
//using namespace pcl;
//
//int user_data;
//// ����ڲ�
//const double u0 = 1288.147;
//const double v0 = 973.571;
//const double fx = 4152.073;
//const double fy = 4152.073;
//const double baseline = 176.252;
//const double doffs = 213.084;	// �����������������x�����ϵĲ��, doffs = |u1 - u0|
//
//void viewerOneOff(visualization::PCLVisualizer& viewer)
//{
//	viewer.setBackgroundColor(0.0, 0.0, 0.0);
//}
//
//int main()
//{
//	// ��������
//	Mat color = imread("b4-r.jpg"); // RGB
//	Mat depth = imread("test0.jpg", IMREAD_UNCHANGED);// depth
//	if (color.empty() || depth.empty())
//	{
//		cout << "The image is empty, please check it!" << endl;
//		return -1;
//	}
//
//	// �������ϵ�µĵ���
//	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
//
//	for (int row = 0; row < depth.rows; row++)
//	{
//		for (int col = 0; col < depth.cols; col++)
//		{
//			ushort d = depth.ptr<ushort>(row)[col];
//
//			if (d == 0)
//				continue;
//			PointXYZRGB p;
//
//			// depth			
//			p.z = fx * baseline / (d + doffs); // Zc = baseline * f / (d + doffs)
//			p.x = (col - u0) * p.z / fx; // Xc���ң�Yc����Ϊ��
//			p.y = (row - v0) * p.z / fy;
//
//			p.y = -p.y;  // Ϊ������ʾ����x����ά��ת180��
//			p.z = -p.z;
//
//			// RGB
//			p.b = color.ptr<uchar>(row)[col * 3];
//			p.g = color.ptr<uchar>(row)[col * 3 + 1];
//			p.r = color.ptr<uchar>(row)[col * 3 + 2];
//
//			cloud->points.push_back(p);
//		}
//	}
//
//	cloud->height = depth.rows;
//	cloud->width = depth.cols;
//	cloud->points.resize(cloud->height * cloud->width);
//
//	visualization::CloudViewer viewer("Cloud Viewer");
//	viewer.showCloud(cloud);
//	viewer.runOnVisualizationThreadOnce(viewerOneOff);
//
//	while (!viewer.wasStopped())
//	{
//		user_data = 9;
//	}
//	return 0;
//}
