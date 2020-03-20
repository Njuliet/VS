//// stereoCalibration.cpp : �������̨Ӧ�ó������ڵ㡣
////
////�ڽ���˫Ŀ����ͷ�ı궨֮ǰ��������ȷֱ����������ͷ���е�Ŀ�Ӿ��ı궨 
////�ֱ�ȷ����������ͷ���ڲξ���Ȼ���ٿ�ʼ����˫Ŀ����ͷ�ı궨
////�ڴ����������ȶ���������ͷ���е����궨(����һƪ��Ŀ�궨����)��Ȼ���ڽ�������궨
//
////#include "stdafx.h"
//#include <opencv2/opencv.hpp>
//#include <highgui.hpp>
//#include "cv.h"
//#include <cv.hpp>
//#include <iostream>
//
//using namespace std;
//using namespace cv;
//
//const int imageWidth = 640;								//����ͷ�ķֱ���
//const int imageHeight = 480;
//const int boardWidth = 9;								//����Ľǵ���Ŀ
//const int boardHeight = 6;								//����Ľǵ�����
//const int boardCorner = boardWidth * boardHeight;		//�ܵĽǵ�����
//const int frameNumber = 13;								//����궨ʱ��Ҫ���õ�ͼ��֡��
//const int squareSize = 20;								//�궨��ڰ׸��ӵĴ�С ��λmm
//const Size boardSize = Size(boardWidth, boardHeight);	//
//Size imageSize = Size(imageWidth, imageHeight);
//
//Mat R, T, E, F;											//R ��תʸ�� Tƽ��ʸ�� E�������� F��������
//vector<Mat> rvecs;									    //��ת����
//vector<Mat> tvecs;										//ƽ������
//vector<vector<Point2f>> imagePointL;				    //��������������Ƭ�ǵ�����꼯��
//vector<vector<Point2f>> imagePointR;					//�ұ������������Ƭ�ǵ�����꼯��
//vector<vector<Point3f>> objRealPoint;					//����ͼ��Ľǵ��ʵ���������꼯��
//
//
//vector<Point2f> cornerL;								//��������ĳһ��Ƭ�ǵ����꼯��
//vector<Point2f> cornerR;								//�ұ������ĳһ��Ƭ�ǵ����꼯��
//
//Mat rgbImageL, grayImageL;
//Mat rgbImageR, grayImageR;
//
//Mat Rl, Rr, Pl, Pr, Q;									//У����ת����R��ͶӰ����P ��ͶӰ����Q (�����о���ĺ�����ͣ�	
//Mat mapLx, mapLy, mapRx, mapRy;							//ӳ���
//Rect validROIL, validROIR;								//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������
//
///*
//���ȱ궨�õ���������ڲξ���
//fx 0 cx
//0 fy cy
//0 0  1
//*/
//Mat cameraMatrixL;/* = (Mat_<double>(3, 3) << 532.782, 0, 532.904,
//	0, 342.505, 233.876,
//	0, 0, 1);*/
//Mat distCoeffL;// = (Mat_<double>(5, 1) << -0.28095, 0.0255745, 0.00122226, -0.000137736, 0.162946);
///*
//���ȱ궨�õ���������ڲξ���
//fx 0 cx
//0 fy cy
//0 0  1
//*/
//Mat cameraMatrixR; /*= (Mat_<double>(3, 3) << 532.782, 0, 532.904,
//	0, 342.505, 233.876,
//	0, 0, 1);*/
//Mat distCoeffR;// = (Mat_<double>(5, 1) << -0.28095, 0.0255745, 0.00122226, -0.000137736, 0.162946);
//
//
///*����궨����ģ���ʵ����������*/
//void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
//{
//	//	Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));
//	vector<Point3f> imgpoint;
//	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
//	{
//		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
//		{
//			//	imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);
//			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
//		}
//	}
//	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
//	{
//		obj.push_back(imgpoint);
//	}
//}
//
//void outputCameraParam(void)
//{
//	/*��������*/
//	/*�������*/
//	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
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
//void StereoTo3D(vector<Point2f> ptsL, vector<Point2f> ptsR, vector<Point3f> &pts3D, Mat Q,Mat T)
//{
//	//1���ǲ���
//	//if (ptsL.size() != ptsR.size())
//	//{
//	//	cout << "ƥ����������ִ���" << endl;
//	//	return;
//	//}
//	//float cx = -Q.at<double>(0, 3);
//	//float cy = -Q.at<double>(1, 3);
//	//float f = Q.at<double>(2, 3);
//
//	//float B = sqrt(T.at<double>(0,0)* T.at<double>(0, 0) + T.at<double>(1, 0) * T.at<double>(1, 0) + T.at<double>(2, 0) * T.at<double>(2, 0));
//	//pts3D.resize(ptsL.size());
//	//for (int i = 0; i < ptsL.size(); i++)
//	//{
//	//	pts3D[i].x = (ptsL[i].x - cx) * B / (ptsL[i].x - ptsR[i].x);
//	//	pts3D[i].y = (ptsL[i].y - cy) * B / (ptsL[i].x - ptsR[i].x);
//	//	pts3D[i].z = f         * B / (ptsL[i].x - ptsR[i].x);
//	//}
//
//	//2����cvPerspectiveTransform����
//	CvMat *src = cvCreateMat(1, 1, CV_32FC3);//���任��  
//	CvMat *dst = cvCreateMat(1, 1, CV_32FC3);//�任���  
//	CvMat *perMat = cvCreateMat(4, 4, CV_32FC1);//͸��ͶӰ�任����  
//	*perMat = Q;
//	pts3D.resize(ptsL.size());
//	for (int i = 0; i < ptsL.size(); i++)
//	{
//		cvZero(src);
//		cvZero(dst);
//		for (int j = 0; j < 3; j++)
//		{
//			float *p = (float*)cvPtr2D(src, 0, 0);
//			*p++ = ptsL[i].x;
//			*p++ = ptsL[i].y;
//			*p = ptsL[i].x - ptsR[i].x;
//		}
//		cvPerspectiveTransform(src, dst, perMat);
//		float *pp = (float*)cvPtr2D(dst, 0, 0);
//		pts3D[i].x = *pp++;
//		pts3D[i].y = *pp++;
//		pts3D[i].z = *pp;
//	}
//}
//
//int main()
//{
//	Mat img;
//	int goodFrameCount = 0;
//	namedWindow("ImageL");
//	namedWindow("ImageR");
//	cout << "��Q�˳� ..." << endl;
//	while (goodFrameCount < frameNumber)
//	{
//		char filename[100];
//		/*��ȡ��ߵ�ͼ��*/
//		sprintf_s(filename, "F:/�Ӿ����/opencv/sources/samples/data/left%02d.jpg", goodFrameCount + 1);
//		rgbImageL = imread(filename, CV_LOAD_IMAGE_COLOR);
//		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
//
//		/*��ȡ�ұߵ�ͼ��*/
//		sprintf_s(filename, "F:/�Ӿ����/opencv/sources/samples/data/right%02d.jpg", goodFrameCount + 1);
//		rgbImageR = imread(filename, CV_LOAD_IMAGE_COLOR);
//		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
//
//		bool isFindL, isFindR;
//
//		isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
//		isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
//		if (isFindL == true && isFindR == true)	 //�������ͼ���ҵ������еĽǵ� ��˵��������ͼ���ǿ��е�
//		{
//			/*
//			Size(5,5) �������ڵ�һ���С
//			Size(-1,-1) ������һ��ߴ�
//			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)������ֹ����
//			*/
//			cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//			drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
//			imshow("chessboardL", rgbImageL);
//			imagePointL.push_back(cornerL);
//
//
//			cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//			drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
//			imshow("chessboardR", rgbImageR);
//			imagePointR.push_back(cornerR);
//
//			/*
//			����Ӧ���ж�������ͼ���ǲ��Ǻõģ��������ƥ��Ļ��ſ��������궨
//			������������̵��У��õ�ͼ����ϵͳ�Դ���ͼ�񣬶��ǿ���ƥ��ɹ��ġ�
//			���������û���ж�
//			*/
//			//string filename = "res\\image\\calibration";
//			//filename += goodFrameCount + ".jpg";
//			//cvSaveImage(filename.c_str(), &IplImage(rgbImage));		//�Ѻϸ��ͼƬ��������
//			goodFrameCount++;
//			cout << "The image is good" << endl;
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
//	����ʵ�ʵ�У�������ά����
//	����ʵ�ʱ궨���ӵĴ�С������
//	*/
//	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
//	cout << "cal real successful" << endl;
//
//
//	/*
//	�궨����ͷ
//	��������������ֱ𶼾����˵�Ŀ�궨
//	�����ڴ˴�ѡ��flag = CALIB_USE_INTRINSIC_GUESS
//	*/
//	//��Ŀ
//	cameraMatrixL = initCameraMatrix2D(objRealPoint, imagePointL, imageSize, 0);
//	cameraMatrixR = initCameraMatrix2D(objRealPoint, imagePointR, imageSize, 0);
//	//˫Ŀ
//	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
//								 cameraMatrixL, distCoeffL,
//								 cameraMatrixR, distCoeffR,
//								 Size(imageWidth, imageHeight), R, T, E, F,
//								 CALIB_USE_INTRINSIC_GUESS,
//								 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//
//	cout << "Stereo Calibration done with RMS error = " << rms << endl;
//
//	/*
//	����У����ʱ����Ҫ����ͼ���沢���ж�׼ ��ʹ������ƥ����ӵĿɿ�
//	ʹ������ͼ����ķ������ǰ���������ͷ��ͼ��ͶӰ��һ�������������ϣ�����ÿ��ͼ��ӱ�ͼ��ƽ��ͶӰ������ͼ��ƽ�涼��Ҫһ����ת����R
//	stereoRectify �����������ľ��Ǵ�ͼ��ƽ��ͶӰ����������ƽ�����ת����Rl,Rr�� Rl,Rr��Ϊ�������ƽ���ж�׼��У����ת����
//	���������Rl��ת�����������Rr��ת֮������ͼ����Ѿ����沢���ж�׼�ˡ�
//	����Pl,PrΪ���������ͶӰ�����������ǽ�3D�������ת����ͼ���2D�������:P*[X Y Z 1]' =[x y w]
//	Q����Ϊ��ͶӰ���󣬼�����Q���԰�2άƽ��(ͼ��ƽ��)�ϵĵ�ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ���ʱ��
//	*/
//	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
//		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
//	/*
//	����stereoRectify ���������R �� P ������ͼ���ӳ��� mapx,mapy
//	mapx,mapy������ӳ�����������Ը�remap()�������ã���У��ͼ��ʹ������ͼ���沢���ж�׼
//	ininUndistortRectifyMap()�Ĳ���newCameraMatrix����У����������������openCV���棬У����ļ��������Mrect�Ǹ�ͶӰ����Pһ�𷵻صġ�
//	�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
//	*/
//	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
//	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
//
//
//	//����
//	rgbImageL = imread("F:/�Ӿ����/opencv/sources/samples/data/left01.jpg", CV_LOAD_IMAGE_COLOR);
//	rgbImageR = imread("F:/�Ӿ����/opencv/sources/samples/data/right01.jpg", CV_LOAD_IMAGE_COLOR);
//
//	cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
//	cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
//
//	//imshow("ImageL Before Rectify", grayImageL);
//	//imshow("ImageR Before Rectify", grayImageR);
//
//	/*
//	����remap֮�����������ͼ���Ѿ����沢���ж�׼��
//	*/
//
//	Mat rectifyImageL, rectifyImageR;
//	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
//	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
//
//	//imshow("ImageL", rectifyImageL);
//	//imshow("ImageR", rectifyImageR);
//
//	/*���沢�������*/
//	outputCameraParam();
//
//
//
//	///*
//	//��У�������ʾ����
//	//����������ͼ����ʾ��ͬһ��������
//	//����ֻ��ʾ�����һ��ͼ���У���������û�а����е�ͼ����ʾ����
//	//*/
//
//	//Mat rgbRectifyImageL, rgbRectifyImageR;
//	//cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);  //α��ɫͼ
//	//cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);
//	//Mat canvas;
//	//double sf;
//	//int w, h;
//	//sf = 600. / MAX(imageSize.width, imageSize.height);
//	//w = cvRound(imageSize.width * sf);
//	//h = cvRound(imageSize.height * sf);
//	//canvas.create(h, w * 2, CV_8UC3);
//	///*��ͼ�񻭵�������*/
//	//Mat canvasPart = canvas(Rect(w * 0, 0, w, h));								//�õ�������һ����
//	//resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);		//��ͼ�����ŵ���canvasPartһ����С
//	//Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),				//��ñ���ȡ������	
//	//	cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
//	//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);						//����һ������
//	//cout << "Painted ImageL" << endl;
//	///*��ͼ�񻭵�������*/
//	//canvasPart = canvas(Rect(w, 0, w, h));										//��û�������һ����
//	//resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
//	//Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
//	//	cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
//	//rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);
//	//cout << "Painted ImageR" << endl;
//	///*���϶�Ӧ������*/
//	//for (int i = 0; i < canvas.rows; i += 16)
//	//	line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
//	//imshow("rectified", canvas);
//	//cout << "wait key" << endl;
//	//waitKey(0);
//	//system("pause");
//	//return 0;
//
//	//ȫͼ��ά�����ؽ�
//
//	//int blockSize = 1, uniquenessRatio = 15, numDisparities = 4;
//	//Mat xyz;              //��ά����
//	//Ptr<StereoBM> bm = StereoBM::create(16, 9);
//	//bm->setBlockSize(2 * blockSize + 5);     //SAD���ڴ�С��5~21֮��Ϊ��
//	//bm->setROI1(validROIL);
//	//bm->setROI2(validROIR);
//	//bm->setPreFilterCap(31);
//	//bm->setMinDisparity(0);  //��С�ӲĬ��ֵΪ0, �����Ǹ�ֵ��int��
//	//bm->setNumDisparities(numDisparities * 16 + 16);//�Ӳ�ڣ�������Ӳ�ֵ����С�Ӳ�ֵ֮��,���ڴ�С������16����������int��
//	//bm->setTextureThreshold(10);
//	//bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio��Ҫ���Է�ֹ��ƥ��
//	//bm->setSpeckleWindowSize(100);
//	//bm->setSpeckleRange(32);
//	//bm->setDisp12MaxDiff(1);
//	//Mat disp, disp8;
//	//bm->compute(rectifyImageL, rectifyImageR, disp);
//	//disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//��������Ӳ���CV_16S��ʽ
//	//reprojectImageTo3D(disp, xyz, Q, true); //��ʵ�������ʱ��ReprojectTo3D������X / W, Y / W, Z / W��Ҫ����16(Ҳ����W����16)�����ܵõ���ȷ����ά������Ϣ��
//	//xyz = xyz * 16;
//	//imshow("disparity", disp8);
//
//	//��������ά�ؽ�.
//	bool isFindLp, isFindRp;
//	vector<Point2f> cornerLp,cornerRp;
//	isFindLp = findChessboardCorners(rectifyImageL, boardSize, cornerLp);
//	isFindRp = findChessboardCorners(rectifyImageR, boardSize, cornerRp);
//	cornerSubPix(rectifyImageL, cornerLp, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//	cornerSubPix(rectifyImageR, cornerRp, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
//
//	vector<Point3f> xyz;
//	StereoTo3D(cornerLp, cornerRp, xyz, Q, T);
//
//	//������άλ����Ϣmm
//	ofstream area_statistics_file;
//	area_statistics_file.open("point3d.txt");
//	for (int i = 0; i < xyz.size(); i++)
//	{
//		area_statistics_file << xyz[i].x<<" "<< xyz[i].y<<" "<<xyz[i].z << endl;
//	}
//	//
//	//waitKey(0);
//	system("pause");
//	//return 0;
//}
