//by shuishui shiwenjun 20160926
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>  
#include <opencv2/opencv.hpp>  

using namespace cv;
using namespace std;
using namespace pcl;

int user_data;
//相机内参，根据输入改动 ,三维模型过于稀疏

const double u0 = 313.8396 ;//由于后面resize成原图的1/4所以有些参数要缩小相同倍数
const double v0 = 190.4737 ;
const double fx = 481.6309 ;
const double fy = 481.5541 ;
const double Tx = 84;              // 118.33;      //基线84毫米
const double doffs = 2.3442 ;//像素差    缺   // 代表两个相机主点在x方向上的差距, doffs = |u1 - u0|  分别是两个摄像头标定得到的内参中的U



void viewerOneOff(visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
}

int main()
{
	PointCloud<PointXYZRGB> cloud_a;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);



	Mat color1 = imread("Left.jpg");
	Mat depth = imread("Left_SAD_RGB_Disparity.jpg");
	////Resize
	//color1.resize();
	Mat color;
	resize(color1, color, Size(color1.cols , color1.rows ), 0, 0, CV_INTER_LINEAR);
	//resize(color1, color, Size(color1.cols / 4, color1.rows / 4), 0, 0, CV_INTER_LINEAR);
	//imshow("h",color);
	//imshow("h",color);
	//waitKey(0);

	int rowNumber = color.rows;
	int colNumber = color.cols;

	cloud_a.height = rowNumber;
	cloud_a.width = colNumber;
	cloud_a.points.resize(cloud_a.width * cloud_a.height);

	for (unsigned int u = 0; u < rowNumber; ++u)
	{
		for (unsigned int v = 0; v < colNumber; ++v)
		{
			/*unsigned int num = rowNumber*colNumber-(u*colNumber + v)-1;*/
			unsigned int num = u*colNumber + v;
			double Xw = 0, Yw = 0, Zw = 0;


			Zw = fx*Tx / (((double)depth.at<Vec3b>(u, v)[0]) + doffs);
			Xw = (v + 1 - u0) * Zw / fx;
			Yw = (u + 1 - v0) * Zw / fy;

			cloud_a.points[num].b = color.at<Vec3b>(u, v)[0];
			cloud_a.points[num].g = color.at<Vec3b>(u, v)[1];
			cloud_a.points[num].r = color.at<Vec3b>(u, v)[2];

			cloud_a.points[num].x = Xw;
			cloud_a.points[num].y = Yw;
			cloud_a.points[num].z = Zw;
			
	
		}
	}

	*cloud = cloud_a;
	pcl::io::savePCDFileASCII("point.pcd", *cloud);
	visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	while (!viewer.wasStopped())
	{
		user_data = 9;
	}

	//fclose(fp1);//关闭文件 
	return 0;
}