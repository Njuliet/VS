//https://www.cnblogs.com/riddick/p/8318997.html
//filterSpeckles（）视差图后处理
//两种立体匹配算法在算出初始视差图后会进行视差图后处理，包括中值滤波，连通域检测等。
//其中中值滤波能够有效去除视差图中孤立的噪点，而连通域检测能够检测出视差图中因噪声引起小团块(blob)。
//在BM和SGBM中都有speckleWindowSize和speckleRange这两个参数，speckleWindowSize是指设置检测出的连通域中像素点个数，也就是连通域的大小。
//speckleRange是指设置判断两个点是否属于同一个连通域的阈值条件。

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef cv::Point_<short> Point2s;
template <typename T> void myFilterSpeckles(cv::Mat &img, int newVal, int maxSpeckleSize, int maxDiff)
{
	int width = img.cols;
	int height = img.rows;
	int imgSize = width*height;
	int *pLabelBuf = (int*)malloc(sizeof(int)*imgSize);//标记值buffer
	Point2s *pPointBuf = (Point2s*)malloc(sizeof(short)*imgSize);//点坐标buffer
	uchar *pTypeBuf = (uchar*)malloc(sizeof(uchar)*imgSize);//blob判断标记buffer
	//初始化Labelbuffer
	int currentLabel = 0;
	memset(pLabelBuf, 0, sizeof(int)*imgSize);

	for (int i = 0; i < height; i++)
	{
		T *pData = img.ptr<T>(i);
		int *pLabel = pLabelBuf + width*i;
		for (int j = 0; j < width; j++)
		{
			if (pData[j] != newVal)
			{
				if (pLabel[j])
				{
					if (pTypeBuf[pLabel[j]])
					{
						pData[j] = (T)newVal;
					}
				}
				else
				{
					Point2s *pWave = pPointBuf;
					Point2s curPoint((T)j, (T)i);
					currentLabel++;
					int count = 0;
					pLabel[j] = currentLabel;
					while (pWave >= pPointBuf)
					{
						count++;
						T *pCurPos = &img.at<T>(curPoint.y, curPoint.x);
						T curValue = *pCurPos;
						int *pCurLabel = pLabelBuf + width*curPoint.y + curPoint.x;
						//bot
						if (curPoint.y < height - 1 && !pCurLabel[+width] && pCurPos[+width] != newVal  && abs(curValue - pCurPos[+width]) <= maxDiff)
						{
							pCurLabel[+width] = currentLabel;
							*pWave++ = Point2s(curPoint.x, curPoint.y + 1);
						}
						//top
						if (curPoint.y > 0 && !pCurLabel[-width] && pCurPos[-width] != newVal && abs(curValue - pCurPos[-width]) <= maxDiff)
						{
							pCurLabel[-width] = currentLabel;
							*pWave++ = Point2s(curPoint.x, curPoint.y - 1);
						}
						//right
						if (curPoint.x < width - 1 && !pCurLabel[+1] && pCurPos[+1] != newVal  && abs(curValue - pCurPos[+1]) <= maxDiff)
						{
							pCurLabel[+1] = currentLabel;
							*pWave++ = Point2s(curPoint.x + 1, curPoint.y);
						}
						//left
						if (curPoint.x > 0 && !pCurLabel[-1] && pCurPos[-1] != newVal && abs(curValue - pCurPos[-1]) <= maxDiff)
						{
							pCurLabel[-1] = currentLabel;
							*pWave++ = Point2s(curPoint.x - 1, curPoint.y);
						}

						--pWave;
						curPoint = *pWave;
					}

					if (count <= maxSpeckleSize)
					{
						pTypeBuf[pLabel[j]] = 1;
						pData[j] = (T)newVal;
					}
					else
					{
						pTypeBuf[pLabel[j]] = 0;
					}
				}
			}
		}
	}

	free(pLabelBuf);
	free(pPointBuf);
	free(pTypeBuf);
}
