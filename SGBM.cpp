//#include<iostream>
//#include<opencv2/opencv.hpp>
//
//using namespace std;
//using namespace cv;
//
//
//class SGBM
//{
//private:
//	enum mode_view { LEFT, RIGHT };
//	mode_view view;	//������Ӳ�ͼor���Ӳ�ͼ
//
//public:
//	SGBM() {};
//	SGBM(mode_view _mode_view) :view(_mode_view) {};
//	~SGBM() {};
//	Mat computersgbm(Mat &L, Mat &R);	//����SGBM
//};
//
//Mat SGBM::computersgbm(Mat &L, Mat &R)
///*SGBM_matching SGBM�㷨
//*@param Mat &left_image :��ͼ��
//*@param Mat &right_image:��ͼ��
//*/
//{
//	Mat disp;
//
//	int numberOfDisparities = ((L.size().width / 8) + 15)&-16;
//	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
//	sgbm->setPreFilterCap(32);
//
//	int SADWindowSize = 5;
//	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
//	sgbm->setBlockSize(sgbmWinSize);
//	int cn = L.channels();
//
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);
//	sgbm->setNumDisparities(numberOfDisparities);
//	sgbm->setUniquenessRatio(10);
//	sgbm->setSpeckleWindowSize(100);
//	sgbm->setSpeckleRange(32);
//	sgbm->setDisp12MaxDiff(1);
//
//
//	Mat left_gray, right_gray;
//	cvtColor(L, left_gray, CV_BGR2GRAY);
//	cvtColor(R, right_gray, CV_BGR2GRAY);
//
//	view = LEFT;
//	if (view == LEFT)	//�������Ӳ�ͼ
//	{
//		sgbm->compute(left_gray, right_gray, disp);
//
//		disp.convertTo(disp, CV_32F, 1.0 / 16);			//����16�õ���ʵ�Ӳ�ֵ
//
//		Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
//		normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
//		imwrite("results/SGBM.jpg", disp8U);
//
//		return disp8U;
//	}
//	else if (view == RIGHT)	//�������Ӳ�ͼ
//	{
//		sgbm->setMinDisparity(-numberOfDisparities);
//		sgbm->setNumDisparities(numberOfDisparities);
//		sgbm->compute(left_gray, right_gray, disp);
//
//		disp.convertTo(disp, CV_32F, 1.0 / 16);			//����16�õ���ʵ�Ӳ�ֵ
//
//		Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
//		normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
//		imwrite("results/SGBM.jpg", disp8U);
//
//		return disp8U;
//	}
//	else
//	{
//		return Mat();
//	}
//}
//
//
//int main()
//{
//
////Mat left = imread("Left.jpg");
////Mat right = imread("Right.jpg");
//Mat left = imread("Left.jpg");
//Mat right = imread("Right.jpg");
//	//-------ͼ����ʾ-----------
//	namedWindow("leftimag");
//	imshow("leftimag", left);
//
//	namedWindow("rightimag");
//	imshow("rightimag", right);
//	//--------��SAD��ȡ�Ӳ�ͼ-----
//	Mat Disparity;
//
//	SGBM mySGBM;
//	Disparity = mySGBM.computersgbm(left, right);
//
//	//-------�����ʾ------
//	namedWindow("Disparity");
//	imshow("Disparity", Disparity);
//	imwrite("SGBM_RGB_Disparity.jpg", Disparity);
////	imwrite("SGBM_Disparity.jpg", Disparity);
//	//-------��β------
//	waitKey(0);
//	return 0;
//}
