////https://www.cnblogs.com/riddick/p/8318997.html
////preFilterCap（）匹配图像预处理
////两种立体匹配算法都要先对输入图像做预处理，OpenCV源码中中调用函数 
////static void prefilterXSobel(const cv::Mat& src, cv::Mat& dst, int preFilterCap)，
////参数设置中preFilterCap在此函数中用到。
////函数步骤如下，作用主要有两点：对于无纹理区域，能够排除噪声干扰；
////对于边界区域，能够提高边界的区分性，利于后续的匹配代价计算：
//
////先利用水平Sobel算子求输入图像x方向的微分值Value；
////如果Value<-preFilterCap, 则Value = 0;
////如果Value>preFilterCap, 则Value = 2 * preFilterCap;
////如果Value >= -preFilterCap &&Value <= preFilterCap, 则Value = Value + preFilterCap;
////输出处理后的图像作为下一步计算匹配代价的输入图像。
//
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//void mySobelX(cv::Mat srcImg, cv::Mat dstImg, int preFilterCap)
//{
//	assert(srcImg.channels() == 1);//图像的通道指的是什么？是不是灰度图的通道数为1，彩色图的通道为3？（zhuker）正确！
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
	//-------收尾------
	waitKey(0);
	return 0;
}
