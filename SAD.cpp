//#include<iostream>//输入左右图求视差图
//#include<opencv2/opencv.hpp>
//
//using namespace std;
//using namespace cv;
//
//
//class SAD
//{
//private:
//	int winSize;//卷积核尺寸
//	int DSR;//视差搜索范围
//public:
//	SAD() :winSize(7), DSR(30){}
//	SAD(int _winSize, int _DSR) :winSize(_winSize), DSR(_DSR){}
//	Mat computerSAD(Mat&L, Mat&R);//计算SAD
//};
//
//Mat SAD::computerSAD(Mat&L, Mat&R)
//{
//	int Height = L.rows;
//	int Width = L.cols;
//	Mat Kernel_L(Size(winSize, winSize), CV_8U, Scalar::all(0));
//	//CV_8U:0~255的值，大多数图像/视频的格式，该段设置全0矩阵
//	Mat Kernel_R(Size(winSize, winSize), CV_8U, Scalar::all(0));
//	Mat Disparity(Height, Width, CV_8U, Scalar(0));
//
//
//	for (int i = 0; i < Width - winSize; ++i){
//		for (int j = 0; j < Height - winSize; ++j){
//			Kernel_L = L(Rect(i, j, winSize, winSize));//L为做图像，Kernel为这个范围内的左图
//			Mat MM(1, DSR, CV_32F, Scalar(0));//定义匹配范围
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
//			//cout << "已完成" << setprecision(2) << rate * 100 << "%" << endl;
//		}
//	}
//	return Disparity;
//}
//
//int main()
//{
//	Mat left = imread("Left.jpg");
//	Mat right = imread("Right.jpg");
//	/*Mat left = imread("Left1.jpg");
//	Mat right = imread("Right1.jpg");*/
//	//-------图像显示-----------
//	namedWindow("leftimag");
//	imshow("leftimag", left);
//	//imwrite("left.jpg", left);
//
//	namedWindow("rightimag");
//	imshow("rightimag", right);
//	//imwrite("right.jpg", right);
//	//--------由SAD求取视差图-----
//	Mat Disparity;
//
//	SAD mySAD(7, 30);
//	Disparity = mySAD.computerSAD(left, right);
//	//-------结果显示------
//	namedWindow("Disparity");
//	imshow("Disparity", Disparity);
//	imwrite("Left_SAD_RGB_Disparity.jpg", Disparity);
//	//imwrite("SAD_Disparity.jpg", Disparity);//imwrite()函数用来保存图片，第一个参数表示需要写入的文件名，必须要加上后缀，比如“123.png”。
//	//-------收尾------
//	waitKey(0);
//	return 0;
//}
