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
//����ڲΣ���������Ķ�
const double u0 = 239.88 / 4;//���ں���resize��ԭͼ��1/4������Щ����Ҫ��С��ͬ����
const double v0 = 214.60 / 4;
const double fx = 319.59 / 4;
const double fy = 281.0499 / 4;
const double Tx = 118.33;      //����84����
const double doffs = 23 / 4;//���ز�    ȱ

//const double u0 = 1329.49 / 4;//���ں���resize��ԭͼ��1/4������Щ����Ҫ��С��ͬ����
//const double v0 = 954.485 / 4;
//const double fx = 6872.874 / 4;
//const double fy = 6872.874 / 4;
//const double Tx = 174.724;
//const double doffs = 293.97 / 4;


void viewerOneOff(visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
}

int main()
{
	PointCloud<PointXYZRGB> cloud_a;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);


	FILE *fp1;

	fp1 = fopen("pointXYZ.csv", "w");//��Excel�ļ�����д���������ʽΪCSV
	if ((fp1 = fopen("pointXYZ.csv", "w")) == NULL)
	{
		printf("fail to open the file!\n");
		exit(0);
	}
	/*Mat color1 = imread("b4-l.jpg");
	Mat depth = imread("test0.jpg");*/
	Mat color1 = imread("a1.jpg");
	Mat depth = imread("SAD_RGB_Disparity.jpg");
	////Resize
	//color1.resize();
	Mat color;
	resize(color1, color, Size(color1.cols / 4, color1.rows / 4), 0, 0, CV_INTER_LINEAR);
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
			fprintf(fp1, "Xw:%lf\t\t\tYw:%lf\t\t\tZw:%lf\n", Xw, Yw, Zw);//���Ҫ��������һ��ͬ�е�Ԫ�񣨵�1�е�2�У�д���ݣ�ʹ��"\t" ;
													 //	���Ҫ��������һ��ͬ�е�Ԫ�񣨵�2�е�1�У�д���ݣ�ʹ��"\n" ��

	
		}
	}

	*cloud = cloud_a;

	visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	while (!viewer.wasStopped())
	{
		user_data = 9;
	}

	fclose(fp1);//�ر��ļ� 
	return 0;
}