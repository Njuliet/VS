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
//����ڲΣ���������Ķ� ,��άģ�͹���ϡ��

//const double u0 = 327.76309 ;    //opencv�궨���ڲν��
//const double v0 = 196.35926656 ;
//const double fx = 480.1537 ;
//const double fy = 477.6767 ;
//const double Tx = 84;              // 118.33;      //����84����  �ı��������10->84->180��ͼ��ֲ����ⶼû�����Ա仯
//const double doffs = 13.876242 ;//���ز�    ȱ   // �����������������x�����ϵĲ��, doffs = |u1 - u0|  �ֱ�����������ͷ�궨�õ����ڲ��е�U

const double u0 = 341.74968377270648;    //    ���Ʒֲ����⻹��û�н��
const double v0 = 236.10021158917539;
const double fx = 533.70141137441306;
const double fy = 533.83894162717877;
const double Tx = 33.267335558390052;              // 118.33;      //����84����  �ı��������10->84->180��ͼ��ֲ����ⶼû�����Ա仯
const double doffs = 14.97398503103;//���ز� 

void viewerOneOff(visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
}

int main()
{
	PointCloud<PointXYZRGB> cloud_a;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);



	Mat color1 = imread("left01.jpg");
	Mat depth = imread("depth.png");
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

	//fclose(fp1);//�ر��ļ� 
	return 0;
}