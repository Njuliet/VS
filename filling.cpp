//https://www.cnblogs.com/riddick/p/8486223.html
//�ն����ĺ����������£�
//��ұ�������Ϊ�������⣬������д�����32F��ȡֵ��0-1֮��ģ�����Ӳ�ͼ��0-255�ģ��������սᡣ
//���ɵ������ͼ�޷����棬��ΪOpenCVĬ�ϵ�ͼ���ʽΪCV_8UC3����ʱͼ��Ϊ3ͨ����8λRGBͼ��ÿ��ͨ�����ܱ��ĻҶȽ�Ϊ28=256 2^8=2562 
//OpenCVĬ�ϵ�ͼ���ʽΪCV_8UC3����ʱͼ��Ϊ3ͨ����8λRGBͼ��ÿ��ͨ�����ܱ��ĻҶȽ�Ϊ2 ^ 8 = 256��
//���Ӳ�ͼ��ΪCV_16S��CV_32S�ȣ����ֱ��ʹ��cv::imwrite()�����Ӳ�ͼ�����ͼ����ͼ�񽫱�ת��CV_8U��ʽ��������ֵ����255���ᱻת��255��

// https://blog.csdn.net/YunLaowang/article/details/86583351
// CV_32F��ͼƬ��α���???
//���棺
//
//�������ķ�ͼ��A8u��A32f��B8u��B32f
//
//��ôA8uת����A32f�ķ���Ϊ��A8u.comvertTo(A32f, CV_32F, 1.0 / 255);
//
//��ôA32fת����A8u�ķ���Ϊ��A32f.comvertTo(A8u, CV_8U, 255);

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;


void insertDepth32f(cv::Mat& depth)
{
	const int width = depth.cols;
	const int height = depth.rows;
	float* data = (float*)depth.data;
	cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
	cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
	double* integral = (double*)integralMap.data;
	int* ptsIntegral = (int*)ptsMap.data;
	memset(integral, 0, sizeof(double) * width * height);
	memset(ptsIntegral, 0, sizeof(int) * width * height);
	for (int i = 0; i < height; ++i)
	{
		int id1 = i * width;
		for (int j = 0; j < width; ++j)
		{
			int id2 = id1 + j;
			if (data[id2] > 1e-3)
			{
				integral[id2] = data[id2];
				ptsIntegral[id2] = 1;
			}
		}
	}
	// ��������
	for (int i = 0; i < height; ++i)
	{
		int id1 = i * width;
		for (int j = 1; j < width; ++j)
		{
			int id2 = id1 + j;
			integral[id2] += integral[id2 - 1];
			ptsIntegral[id2] += ptsIntegral[id2 - 1];
		}
	}
	for (int i = 1; i < height; ++i)
	{
		int id1 = i * width;
		for (int j = 0; j < width; ++j)
		{
			int id2 = id1 + j;
			integral[id2] += integral[id2 - width];
			ptsIntegral[id2] += ptsIntegral[id2 - width];
		}
	}
	int wnd;
	double dWnd = 2;
	while (dWnd > 1)
	{
		wnd = int(dWnd);
		dWnd /= 2;
		for (int i = 0; i < height; ++i)
		{
			int id1 = i * width;
			for (int j = 0; j < width; ++j)
			{
				int id2 = id1 + j;
				int left = j - wnd - 1;
				int right = j + wnd;
				int top = i - wnd - 1;
				int bot = i + wnd;
				left = max(0, left);
				right = min(right, width - 1);
				top = max(0, top);
				bot = min(bot, height - 1);
				int dx = right - left;
				int dy = (bot - top) * width;
				int idLeftTop = top * width + left;
				int idRightTop = idLeftTop + dx;
				int idLeftBot = idLeftTop + dy;
				int idRightBot = idLeftBot + dx;
				int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
				double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
				if (ptsCnt <= 0)
				{
					continue;
				}
				data[id2] = float(sumGray / ptsCnt);
			}
		}
		int s = wnd / 2 * 2 + 1;
		if (s > 201)
		{
			s = 201;
		}
		cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
	}
}

int main()
{
	cv::Mat depth = imread("depth.png", 0);
	imshow("depth", depth);

	cout << depth.cols;//640
	cout << depth.rows;//480

	depth.convertTo(depth, CV_32F, 1.0 / 255);

	insertDepth32f(depth);

	imshow("insertDepth32", depth);
	depth.convertTo(depth, CV_8U, 255);

	imshow("insertDepth8", depth);
	imwrite("insertDepth.bmp", depth);
	//cv::imwrite("*.exr", depth);
	//cvSaveImage("insertDepth.jpg", depth);
	//-------��β------
	waitKey(0);
	return 0;
}
