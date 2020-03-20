//// 同时调用两个摄像头，暂停并保存左右相机的标定棋盘图
//
//#include <opencv2/opencv.hpp>
//#include <iostream>
//#include <math.h>
//
//using namespace cv;
//using namespace std;                             //开头我是从教程学的，一般不变，直接用
//
//int main(int argc, char* argv[])
//{
//	VideoCapture cap(0);
//	VideoCapture cap1(1);                   //打开两个摄像头
//
//	if (!cap.isOpened())
//	{
//		printf("Could not open camera0...\n");
//		return -1;
//	}
//	if (!cap1.isOpened())
//	{
//		printf("Could not open camera1...\n");
//		return -2;
//
//	}                                                        //判断还是加上为好，便于调程序
//
//
//
//	Mat frame, frame1;
//	bool stop = false;
//	while (!stop)
//	{
//		cap.read(frame);
//		cap1.read(frame1);
//		imshow("camera0", frame);
//		imshow("camera1", frame1);
//
//		int delay = 30;
//		if (delay >= 0 && waitKey(delay) >= 0)
//		{
//			waitKey(0);                                           //实时拍摄暂停的程序
//		}
//		imwrite("C:/Users/Administrator/Desktop/11/left1.jpg", frame1);
//		imwrite("C:/Users/Administrator/Desktop/11/right1.jpg", frame);  //给个位置保存图片，注意图片到底是                                                                                                                 左相机还是右相机的（用手在摄像头前晃晃），我用                                                                                                        的笨方法，保存一下，再把（left1.jpg/right1.jpg）+1，接着保存
//	}
//
//	cap.release();
//	cap1.release();                             //两个摄像头数据释放  
//	return 0;
//}
//
////左右单目相机分别标定  
//
//#include "opencv2/core/core.hpp"  
//#include "opencv2/imgproc/imgproc.hpp"  
//#include "opencv2/calib3d/calib3d.hpp"  
//#include "opencv2/highgui/highgui.hpp"  
//
//#include <opencv2/opencv.hpp>  
//#include "cv.h"  
//#include <cv.hpp>  
//#include <iostream>  
//
//using namespace std;
//using namespace cv;                                      //人家这开头都长，遇到有红线标记的就删了，把你知道的开头加上，没问题
//
//const int boardWidth = 9;                               //横向的角点数目  
//const int boardHeight = 6;                              //纵向的角点数据  
//const int boardCorner = boardWidth * boardHeight;       //总的角点数据  
//const int frameNumber = 15;                             //相机标定时需要采用的图像帧数  
//const int squareSize = 25;                              //标定板黑白格子的大小 单位mm  
//const Size boardSize = Size(boardWidth, boardHeight);   //总的内角点
//
//Mat intrinsic;                                                //相机内参数  
//Mat distortion_coeff;                                   //相机畸变参数  
//vector<Mat> rvecs;                                        //旋转向量  
//vector<Mat> tvecs;                                        //平移向量  
//vector<vector<Point2f>> corners;                        //各个图像找到的角点的集合 和objRealPoint 一一对应  
//vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合  
//
//vector<Point2f> corner;                                   //某一副图像找到的角点  
//
///*计算标定板上模块的实际物理坐标*/
//void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
//{
//	vector<Point3f> imgpoint;
//	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
//	{
//		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
//		{
//			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
//		}
//	}
//	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
//	{
//		obj.push_back(imgpoint);
//	}
//}
//
//
///*设置相机的初始参数 也可以不估计*/
//void guessCameraParam(void)
//{
//	/*分配内存*/
//	intrinsic.create(3, 3, CV_64FC1);    //相机内参数
//	distortion_coeff.create(5, 1, CV_64FC1);  //畸变参数
//
//	/*
//	fx 0 cx
//	0 fy cy
//	0 0  1     内参数
//	*/
//	intrinsic.at<double>(0, 0) = 256.8093262;   //fx         
//	intrinsic.at<double>(0, 2) = 160.2826538;   //cx  
//	intrinsic.at<double>(1, 1) = 254.7511139;   //fy  
//	intrinsic.at<double>(1, 2) = 127.6264572;   //cy  
//
//	intrinsic.at<double>(0, 1) = 0;
//	intrinsic.at<double>(1, 0) = 0;
//	intrinsic.at<double>(2, 0) = 0;
//	intrinsic.at<double>(2, 1) = 0;
//	intrinsic.at<double>(2, 2) = 1;
//
//	/*
//	k1 k2 p1 p2 p3    畸变参数
//	*/
//	distortion_coeff.at<double>(0, 0) = -0.193740;  //k1  
//	distortion_coeff.at<double>(1, 0) = -0.378588;  //k2  
//	distortion_coeff.at<double>(2, 0) = 0.028980;   //p1  
//	distortion_coeff.at<double>(3, 0) = 0.008136;   //p2  
//	distortion_coeff.at<double>(4, 0) = 0;          //p3  
//}
//
//
//void outputCameraParam(void)
//{
//	/*保存数据*/
//	//cvSave("cameraMatrix.xml", &intrinsic);  
//	//cvSave("cameraDistoration.xml", &distortion_coeff);  
//	//cvSave("rotatoVector.xml", &rvecs);  
//	//cvSave("translationVector.xml", &tvecs);  
//	/*输出数据*/
//	//cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
//	//cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;//内参数
//	printf("fx:%lf...\n", intrinsic.at<double>(0, 0));
//	printf("fy:%lf...\n", intrinsic.at<double>(1, 1));
//	printf("cx:%lf...\n", intrinsic.at<double>(0, 2));
//	printf("cy:%lf...\n", intrinsic.at<double>(1, 2));                  //我学的是printf,就试着改了一下，都能用
//
//
//	//cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
//	//cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
//	//cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
//	//cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
//	//cout << "p3 :" << distortion_coeff.at<double>(4, 0) << endl;   //畸变参数
//	printf("k1:%lf...\n", distortion_coeff.at<double>(0, 0));
//	printf("k2:%lf...\n", distortion_coeff.at<double>(1, 0));
//	printf("p1:%lf...\n", distortion_coeff.at<double>(2, 0));
//	printf("p2:%lf...\n", distortion_coeff.at<double>(3, 0));
//	printf("p3:%lf...\n", distortion_coeff.at<double>(4, 0));
//}
//
//int main()
//{
//	int imageHeight;     //图像高度
//	int imageWidth;      //图像宽度
//	int goodFrameCount = 0;    //有效图像的数目
//
//	Mat rgbImage, grayImage;
//	Mat tImage = imread("C:/Users/Administrator/Desktop/11/right1.jpg");
//	if (tImage.empty())
//	{
//		printf("Could not load tImage...\n");
//		return -1;
//	}
//	imageHeight = tImage.rows;
//	imageWidth = tImage.cols;
//
//	grayImage = Mat::ones(tImage.size(), CV_8UC1);
//	while (goodFrameCount < frameNumber)
//	{
//		char filename[100];
//		sprintf_s(filename, "C:/Users/Administrator/Desktop/11/right%d.jpg", goodFrameCount + 1);
//
//		rgbImage = imread(filename);
//		if (rgbImage.empty())
//		{
//			printf("Could not load grayImage...\n");
//			return -2;
//		}
//
//		cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
//
//		imshow("Camera", grayImage);
//
//		bool isFind = findChessboardCorners(rgbImage, boardSize, corner, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
//		//bool isFind = findChessboardCorners( rgbImage, boardSize, corner,  
//		//CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);  
//
//		if (isFind == true) //所有角点都被找到 说明这幅图像是可行的  
//		{
//			//精确角点位置，亚像素角点检测
//			cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//			//绘制角点
//			drawChessboardCorners(rgbImage, boardSize, corner, isFind);
//			imshow("chessboard", rgbImage);
//			corners.push_back(corner);
//			goodFrameCount++;
//			/*cout << "The image" << goodFrameCount << " is good" << endl;*/
//			printf("The image %d is good...\n", goodFrameCount);
//		}
//		else
//		{
//			printf("The image is bad please try again...\n");
//		}
//
//
//		if (waitKey(10) == 'q')
//		{
//			break;
//		}
//	}
//
//	/*
//	图像采集完毕 接下来开始摄像头的校正
//	calibrateCamera()
//	输入参数 objectPoints  角点的实际物理坐标
//	imagePoints   角点的图像坐标
//	imageSize     图像的大小
//	输出参数
//	cameraMatrix  相机的内参矩阵
//	distCoeffs    相机的畸变参数
//	rvecs         旋转矢量(外参数)
//	tvecs         平移矢量(外参数）
//	*/
//
//
//	/*设置实际初始参数 根据calibrateCamera来 如果flag = 0 也可以不进行设置*/
//	guessCameraParam();
//	printf("guess successful...\n");
//	/*计算实际的校正点的三维坐标*/
//	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
//	printf("calculate real successful...\n");
//	/*标定摄像头*/
//	calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
//	printf("calibration successful...\n");
//	/*保存并输出参数*/
//	outputCameraParam();
//	printf("output successful...\n");
//
//	/*显示畸变校正效果*/
//	Mat cImage;
//	undistort(rgbImage, cImage, intrinsic, distortion_coeff);  //矫正相机镜头变形
//	imshow("Corret Image", cImage);
//	printf("Corret Image....\n");
//	printf("Wait for Key....\n");
//
//	waitKey(0);
//	return 0;
//
//}
//// 双目相机标定  
//
//#include "opencv2/core/core.hpp"  
//#include "opencv2/imgproc/imgproc.hpp"  
//#include "opencv2/calib3d/calib3d.hpp"  
//#include "opencv2/highgui/highgui.hpp"  
//
//#include <vector>  
//#include <string>  
//#include <algorithm>  
//#include <iostream>  
//#include <iterator>  
//#include <stdio.h>  
//#include <stdlib.h>  
//#include <ctype.h>  
//
//#include <opencv2/opencv.hpp>  
//#include "cv.h"  
//#include <cv.hpp>  
//
//using namespace std;
//using namespace cv;                                         //依旧很长的开头
//
//
//const int imageWidth = 640;                             //摄像头的分辨率  
//const int imageHeight = 480;
//const int boardWidth = 9;                               //横向的角点数目  
//const int boardHeight = 6;                              //纵向的角点数据  
//const int boardCorner = boardWidth * boardHeight;       //总的角点数据  
//const int frameNumber = 15;                             //相机标定时需要采用的图像帧数  
//const int squareSize = 25;                              //标定板黑白格子的大小 单位mm  
//const Size boardSize = Size(boardWidth, boardHeight);   //标定板的总内角点  
//Size imageSize = Size(imageWidth, imageHeight);
//
//Mat R, T, E, F;                                                  //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵  
//vector<Mat> rvecs;                                        //旋转向量  
//vector<Mat> tvecs;                                        //平移向量  
//vector<vector<Point2f>> imagePointL;                    //左边摄像机所有照片角点的坐标集合  
//vector<vector<Point2f>> imagePointR;                    //右边摄像机所有照片角点的坐标集合  
//vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合  
//
//vector<Point2f> cornerL;                              //左边摄像机某一照片角点坐标集合  
//
//vector<Point2f> cornerR;                              //右边摄像机某一照片角点坐标集合  
//
//Mat rgbImageL, grayImageL;
//Mat rgbImageR, grayImageR;
//
//Mat Rl, Rr, Pl, Pr, Q;                                  //校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）   
//Mat mapLx, mapLy, mapRx, mapRy;                         //映射表  
//Rect validROIL, validROIR;                              //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
//
///*
//事先标定好的左相机的内参矩阵
//fx 0 cx
//0 fy cy
//0 0  1
//*/
//Mat cameraMatrixL = (Mat_<double>(3, 3) << 462.279595, 0, 312.781587,
//	0, 460.220741, 208.225803,
//	0, 0, 1);                                                                           //这时候就需要你把左右相机单目标定的参数给写上
////获得的畸变参数
//Mat distCoeffL = (Mat_<double>(5, 1) << -0.054929, 0.224509, 0.000386, 0.001799, -0.302288);
///*
//事先标定好的右相机的内参矩阵
//fx 0 cx
//0 fy cy
//0 0  1
//*/
//Mat cameraMatrixR = (Mat_<double>(3, 3) << 463.923124, 0, 322.783959,
//	0, 462.203276, 256.100655,
//	0, 0, 1);
//Mat distCoeffR = (Mat_<double>(5, 1) << -0.049056, 0.229945, 0.001745, -0.001862, -0.321533);
//
///*计算标定板上模块的实际物理坐标*/
//void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
//{
//	vector<Point3f> imgpoint;
//	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
//	{
//		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
//		{
//			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
//		}
//	}
//	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
//	{
//		obj.push_back(imgpoint);
//	}
//}
//
//
//void outputCameraParam(void)
//{
//	/*保存数据*/
//	/*输出数据*/
//	FileStorage fs("intrinsics.yml", FileStorage::WRITE);  //文件存储器的初始化
//	if (fs.isOpened())
//	{
//		fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;
//		fs.release();
//		cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl;
//	}
//	else
//	{
//		cout << "Error: can not save the intrinsics!!!!!" << endl;
//	}
//
//
//	fs.open("extrinsics.yml", FileStorage::WRITE);
//	if (fs.isOpened())
//	{
//		fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
//		cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr=" << Rr << endl << "Pl=" << Pl << endl << "Pr=" << Pr << endl << "Q=" << Q << endl;
//		fs.release();
//	}
//	else
//		cout << "Error: can not save the extrinsic parameters\n";
//}
//
//int main(int argc, char* argv[])
//{
//	Mat img;
//	int goodFrameCount = 0;
//	while (goodFrameCount < frameNumber)
//	{
//		char filename[100];
//		/*读取左边的图像*/
//		sprintf_s(filename, "C:/Users/Administrator/Desktop/11/left%d.jpg", goodFrameCount + 1);
//		rgbImageL = imread(filename, CV_LOAD_IMAGE_COLOR);
//		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
//
//		/*读取右边的图像*/
//		sprintf_s(filename, "C:/Users/Administrator/Desktop/11/right%d.jpg", goodFrameCount + 1);
//		rgbImageR = imread(filename, CV_LOAD_IMAGE_COLOR);
//		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
//
//		bool isFindL, isFindR;
//
//		isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
//		isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
//		if (isFindL == true && isFindR == true)  //如果两幅图像都找到了所有的角点 则说明这两幅图像是可行的  
//		{
//			/*
//			Size(5,5) 搜索窗口的一半大小
//			Size(-1,-1) 死区的一半尺寸
//			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
//			*/
//			cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//			drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
//			imshow("chessboardL", rgbImageL);
//			imagePointL.push_back(cornerL);
//
//			cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//			drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
//			imshow("chessboardR", rgbImageR);
//			imagePointR.push_back(cornerR);
//
//
//			//string filename = "res\\image\\calibration";  
//			//filename += goodFrameCount + ".jpg";  
//			//cvSaveImage(filename.c_str(), &IplImage(rgbImage));       //把合格的图片保存起来  
//			goodFrameCount++;
//			cout << "The image" << goodFrameCount << " is good" << endl;
//		}
//		else
//		{
//			cout << "The image is bad please try again" << endl;
//		}
//
//		if (waitKey(10) == 'q')
//		{
//			break;
//		}
//	}
//
//	/*
//	计算实际的校正点的三维坐标
//	根据实际标定格子的大小来设置
//	*/
//	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
//	cout << "cal real successful" << endl;
//
//	/*
//	标定摄像头
//	由于左右摄像机分别都经过了单目标定
//	所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
//	*/
//	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
//		cameraMatrixL, distCoeffL,
//		cameraMatrixR, distCoeffR,
//		Size(imageWidth, imageHeight), R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
//		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5)); //需要注意，应该是版本的原因，该函数最                                                                                                                            后两个参数，我是调换过来后才显示不出错的
//
//	cout << "Stereo Calibration done with RMS error = " << rms << endl;
//
//	/*
//	立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
//	使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
//	stereoRectify 这个函数计算的就是从图像平面投影到公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
//	左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
//	其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
//	Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的时差
//	*/
//	//对标定过的图像进行校正
//	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
//		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
//	/*
//	根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
//	mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
//	ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
//	所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
//	*/
//	//摄像机校正映射
//	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
//	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
//
//	Mat rectifyImageL, rectifyImageR;
//	cvtColor(grayImageL, rectifyImageL, CV_GRAY2BGR);
//	cvtColor(grayImageR, rectifyImageR, CV_GRAY2BGR);
//
//	imshow("Rectify Before", rectifyImageL);
//	cout << "按Q1退出 ..." << endl;
//
//	/*
//	经过remap之后，左右相机的图像已经共面并且行对准了
//	*/
//	Mat rectifyImageL2, rectifyImageR2;
//	remap(rectifyImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
//	remap(rectifyImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
//	cout << "按Q2退出 ..." << endl;
//
//	imshow("rectifyImageL", rectifyImageL2);
//	imshow("rectifyImageR", rectifyImageR2);
//
//	/*保存并输出数据*/
//	outputCameraParam();
//
//	/*
//	把校正结果显示出来
//	把左右两幅图像显示到同一个画面上
//	这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来
//	*/
//	Mat canvas;
//	double sf;
//	int w, h;
//	sf = 600. / MAX(imageSize.width, imageSize.height);
//	w = cvRound(imageSize.width * sf);
//	h = cvRound(imageSize.height * sf);
//	canvas.create(h, w * 2, CV_8UC3);
//
//	/*左图像画到画布上*/
//	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
//	resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);        //把图像缩放到跟canvasPart一样大小  
//	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域    
//		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
//	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形  
//
//	cout << "Painted ImageL" << endl;
//
//	/*右图像画到画布上*/
//	canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分  
//	resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
//	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
//		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
//	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);
//
//	cout << "Painted ImageR" << endl;
//
//	/*画上对应的线条*/
//	for (int i = 0; i < canvas.rows; i += 16)
//		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
//
//	imshow("rectified", canvas);
//
//	cout << "wait key" << endl;
//	waitKey(0);
//	//system("pause");  
//	return 0;
//}
