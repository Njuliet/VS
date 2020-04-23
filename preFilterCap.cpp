////https://www.cnblogs.com/riddick/p/8318997.html
////preFilterCap����ƥ��ͼ��Ԥ����
////��������ƥ���㷨��Ҫ�ȶ�����ͼ����Ԥ����OpenCVԴ�����е��ú��� 
////static void prefilterXSobel(const cv::Mat& src, cv::Mat& dst, int preFilterCap)��
////����������preFilterCap�ڴ˺������õ���
////�����������£�������Ҫ�����㣺���������������ܹ��ų��������ţ�
////���ڱ߽������ܹ���߽߱�������ԣ����ں�����ƥ����ۼ��㣺
//
////������ˮƽSobel����������ͼ��x�����΢��ֵValue��
////���Value<-preFilterCap, ��Value = 0;
////���Value>preFilterCap, ��Value = 2 * preFilterCap;
////���Value >= -preFilterCap &&Value <= preFilterCap, ��Value = Value + preFilterCap;
////���������ͼ����Ϊ��һ������ƥ����۵�����ͼ��
//
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//void mySobelX(cv::Mat srcImg, cv::Mat dstImg, int preFilterCap)
//{
//	assert(srcImg.channels() == 1);//ͼ���ͨ��ָ����ʲô���ǲ��ǻҶ�ͼ��ͨ����Ϊ1����ɫͼ��ͨ��Ϊ3����zhuker����ȷ��
//	int radius = 1;
//	int width = srcImg.cols;
//	int height = srcImg.rows;
//	uchar *pSrcData = srcImg.data;
//	uchar *pDstData = dstImg.data;
//	for (int i = 0; i < height; i++)
//	{
//		for (int j = 0; j < width; j++)
//		{
//			int idx = i*width + j;
//			if (i >= radius && i < height - radius && j >= radius && j < width - radius)
//			{
//				int diff0 = pSrcData[(i - 1)*width + j + 1] - pSrcData[(i - 1)*width + j - 1];
//				int diff1 = pSrcData[i*width + j + 1] - pSrcData[i*width + j - 1];
//				int diff2 = pSrcData[(i + 1)*width + j + 1] - pSrcData[(i + 1)*width + j - 1];
//
//				int value = diff0 + 2 * diff1 + diff2;
//				if (value < -preFilterCap)
//				{
//					pDstData[idx] = 0;
//				}
//				else if (value >= -preFilterCap && value <= preFilterCap)
//				{
//					pDstData[idx] = uchar(value + preFilterCap);
//				}
//				else
//				{
//					pDstData[idx] = uchar(2 * preFilterCap);
//				}
//
//			}
//			else
//			{
//				pDstData[idx] = 0;
//			}
//		}
//	}
//}
//


static void prefilterXSobel(const cv::Mat& src, cv::Mat& dst, int ftzero)
{
	int x, y;
	const int OFS = 256 * 4, TABSZ = OFS * 2 + 256;
	uchar tab[TABSZ];
	cv::Size size = src.size();

	for (x = 0; x < TABSZ; x++)
		tab[x] = (uchar)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero * 2 : x - OFS + ftzero);
	uchar val0 = tab[0 + OFS];

	for (y = 0; y < size.height - 1; y += 2)
	{
		const uchar* srow1 = src.ptr<uchar>(y);
		const uchar* srow0 = y > 0 ? srow1 - src.step : size.height > 1 ? srow1 + src.step : srow1;
		const uchar* srow2 = y < size.height - 1 ? srow1 + src.step : size.height > 1 ? srow1 - src.step : srow1;
		const uchar* srow3 = y < size.height - 2 ? srow1 + src.step * 2 : srow1;
		uchar* dptr0 = dst.ptr<uchar>(y);
		uchar* dptr1 = dptr0 + dst.step;
		//dptr0[0] = dptr0[size.width - 1] = dptr1[0] = dptr1[size.width - 1] = val0;

		dptr1[size.width - 1] = val0;
		dptr1[0] = val0;
		dptr0[size.width - 1] = val0;
		dptr0[0] = val0;

		x = 1;
		for (; x < size.width - 1; x++)
		{
			int d0 = srow0[x + 1] - srow0[x - 1], d1 = srow1[x + 1] - srow1[x - 1],
				d2 = srow2[x + 1] - srow2[x - 1], d3 = srow3[x + 1] - srow3[x - 1];
			int v0 = tab[d0 + d1 * 2 + d2 + OFS];
			int v1 = tab[d1 + d2 * 2 + d3 + OFS];
			dptr0[x] = (uchar)v0;
			dptr1[x] = (uchar)v1;
		}
	}

	for (; y < size.height; y++)
	{
		uchar* dptr = dst.ptr<uchar>(y);
		x = 0;
		for (; x < size.width; x++)
			dptr[x] = val0;
	}
}

int main()
{
	cv::Mat srcImg = imread("Left.jpg");
	imshow("depth", srcImg);

	//cout << srcImg.cols;//640
	//cout << srcImg.rows;//480
	cv::Mat dstImg;
	int preFilterCap = 1;
	//mySobelX(srcImg, dstImg, preFilterCap);
	prefilterXSobel(srcImg, dstImg, 100);
	imshow("dstImg", dstImg);
	//imwrite("insertDepth.bmp", depth);
	//cv::imwrite("*.exr", depth);
	//cvSaveImage("insertDepth.jpg", depth);
	//-------��β------
	waitKey(0);
	return 0;
}
