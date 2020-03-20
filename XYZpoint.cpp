///*
//相机参数：
//cam0 = [4152.073 0 1288.147; 0 4152.073 973.571; 0 0 1]
//cam1 = [4152.073 0 1501.231; 0 4152.073 973.571; 0 0 1]
//doffs = 213.084
//baseline = 176.252
//width = 2872
//height = 1984
//相机内参数矩阵：
//K=[fx 0 u0; 0 fy v0; 0 0 1]
//
//doffs = |u1 - u0|
//*/
//
//// 与pointtest.cpp作用一致都是为了获得三维坐标值,结果同样很稀疏
//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>  
//#include <pcl/io/io.h>  
//#include <pcl/io/pcd_io.h>  
//#include <opencv2/opencv.hpp>  
//
//#include <pcl/point_types.h>  
//
//using namespace cv;
//using namespace std;
//using namespace pcl;
//
//int user_data;
//// 相机内参
///*const double u0 = 308.4038;
//const double v0 = 183.3596;
//const double fx = 486.8830;
//const double fy = 486.6497;
//const double baseline = 84;
//const double doffs = 23.6965;*/	// 代表两个相机主点在x方向上的差距, doffs = |u1 - u0|
//
//const double u0 = 313.8396;
//const double v0 = 190.4737;
//const double fx = 481.6309;
//const double fy = 481.5541;
//const double baseline = 86;//84
//const double doffs = 12.6203;
//
//void viewerOneOff(visualization::PCLVisualizer& viewer)
//{
//	viewer.setBackgroundColor(0.0, 0.0, 0.0);
//}
//
//int main()
//{
//	FILE *fp1;
//	
//	//fp1 = fopen("newpointXYZ.csv", "w");//对Excel文件进行写入操作，格式为CSV   
//	//if ((fp1 = fopen("newpointXYZ.csv", "w")) == NULL)
//	//{
//	//	printf("fail to open the file!\n");
//	//	exit(0);
//	//}
//
//	pcl::PointCloud<pcl::PointXYZ> cloud1;//在网上只找到PointXYZ类点云存储为pcd文件代码，不知道PointXYZRGB如何存，所以借助临时变量cloud1存储
//	size_t i = 0;
//
//	// 读入数据
//	Mat color = imread("a1.jpg"); // RGB
//	Mat depth = imread("SAD_RGB_Disparity.jpg", IMREAD_UNCHANGED);// depth
//	if (color.empty() || depth.empty())
//	{
//		cout << "The image is empty, please check it!" << endl;
//		return -1;
//	}
//
//	// 相机坐标系下的点云
//	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
//	cloud1.is_dense = false;
//	cloud1.points.resize(depth.rows * depth.cols);
//	cloud1.width = 1;
//	cloud1.height = cloud1.points.size();
//	
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
//			p.x = (col - u0) * p.z / fx; // Xc向右，Yc向下为正
//			p.y = (row - v0) * p.z / fy;
//
//			p.y = -p.y;  // 为便于显示，绕x轴三维旋转180°
//			p.z = -p.z;
//
//			cloud1.points[i].x = p.x ;//代码之前在这里中断，是因为cloud1.points.resize没有正确初始化
//			cloud1.points[i].y = p.y;
//			cloud1.points[i].z == p.z;
//			i++;
//
//			//fprintf(fp1, "p.x:%lf\t\t\tp.y:%lf\t\t\tp.z:%lf\n", p.x,p.y,p.z);//如果要给它的下一个同行单元格（第1行第2列）写数据，使用"\t" ;
//			//	如果要给它的下一个同列单元格（第2行第1列）写数据，使用"\n" 。
//			// RGB
//			
//
//			p.b = color.ptr<uchar>(row)[col * 3];
//			p.g = color.ptr<uchar>(row)[col * 3 + 1];
//			p.r = color.ptr<uchar>(row)[col * 3 + 2];
//
//			
//			
//
//			cloud->points.push_back(p);
//		}
//	}
//
//	pcl::io::savePCDFileASCII("point_pcd.pcd", cloud1);//只写入点云的坐标值，没有写入rgb值
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
