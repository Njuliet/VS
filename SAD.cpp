//#include<iostream>
//#include<opencv2/opencv.hpp>
//
//using namespace std;
//using namespace cv;
//
//
//class SAD
//{
//private:
//	int winSize;//�����˳ߴ�
//	int DSR;//�Ӳ�������Χ
//public:
//	SAD() :winSize(7), DSR(30){}
//	SAD(int _winSize, int _DSR) :winSize(_winSize), DSR(_DSR){}
//	Mat computerSAD(Mat&L, Mat&R);//����SAD
//};
//
//Mat SAD::computerSAD(Mat&L, Mat&R)
//{
//	int Height = L.rows;
//	int Width = L.cols;
//	Mat Kernel_L(Size(winSize, winSize), CV_8U, Scalar::all(0));
//	//CV_8U:0~255��ֵ�������ͼ��/��Ƶ�ĸ�ʽ���ö�����ȫ0����
//	Mat Kernel_R(Size(winSize, winSize), CV_8U, Scalar::all(0));
//	Mat Disparity(Height, Width, CV_8U, Scalar(0));
//
//
//	for (int i = 0; i < Width - winSize; ++i){
//		for (int j = 0; j < Height - winSize; ++j){
//			Kernel_L = L(Rect(i, j, winSize, winSize));//LΪ��ͼ��KernelΪ�����Χ�ڵ���ͼ
//			Mat MM(1, DSR, CV_32F, Scalar(0));//����ƥ�䷶Χ
//
//			for (int k = 0; k < DSR; ++k){
//				int x = i - k;
//				if (x >= 0){
//					Kernel_R = R(Rect(x, j, winSize, winSize));
//					Mat Dif;
//					absdiff(Kernel_L, Kernel_R, Dif);
//					Scalar ADD = sum(Dif);
//					float a = ADD[0];
//					MM.at<float>(k) = a;
//				}
//				Point minLoc;
//				minMaxLoc(MM, NULL, NULL, &minLoc, NULL);
//
//				int loc = minLoc.x;
//				Disparity.at<char>(j, i) = loc * 16;
//			}
//			double rate = double(i) / (Width);
//			//cout << "�����" << setprecision(2) << rate * 100 << "%" << endl;
//		}
//	}
//	return Disparity;
//}
//
//int main()
//{
//	Mat left = imread("a1.jpg");
//	Mat right = imread("b1.jpg");
//	/*Mat left = imread("Left1.jpg");
//	Mat right = imread("Right1.jpg");*/
//	//-------ͼ����ʾ-----------
//	namedWindow("leftimag");
//	imshow("leftimag", left);
//	//imwrite("left.jpg", left);
//
//	namedWindow("rightimag");
//	imshow("rightimag", right);
//	//imwrite("right.jpg", right);
//	//--------��SAD��ȡ�Ӳ�ͼ-----
//	Mat Disparity;
//
//	SAD mySAD(7, 30);
//	Disparity = mySAD.computerSAD(left, right);
//	//-------�����ʾ------
//	namedWindow("Disparity");
//	imshow("Disparity", Disparity);
//	imwrite("SAD_RGB_Disparity.jpg", Disparity);
//	//imwrite("SAD_Disparity.jpg", Disparity);//imwrite()������������ͼƬ����һ��������ʾ��Ҫд����ļ���������Ҫ���Ϻ�׺�����硰123.png����
//	//-------��β------
//	waitKey(0);
//	return 0;
//}