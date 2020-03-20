//////#include <pcl/point_types.h>
//////#include <pcl/io/pcd_io.h>
//////#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
//////#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
//////
//////int
//////main(int argc, char** argv)
//////{
//////	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//////	pcl::io::loadPCDFile("point_pcd.pcd", *cloud);
//////
//////	// 创建 KD-Tree
//////	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//////
//////	// Output has the PointNormal type in order to store the normals calculated by MLS
//////	pcl::PointCloud<pcl::PointNormal> mls_points;
//////
//////	// 定义最小二乘实现的对象mls
//////	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//////
//////	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计
//////
//////	// Set parameters
//////	mls.setInputCloud(cloud);
//////	mls.setPolynomialFit(true);
//////	mls.setSearchMethod(tree);
//////	mls.setSearchRadius(0.03);
//////
//////	// Reconstruct
//////	mls.process(mls_points);
//////
//////	// Save output
//////	pcl::io::savePCDFile("point_pcd-MLS.pcd", mls_points);
//////}
////
////
////
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
//#include <pcl/console/time.h>
//#include <pcl/point_cloud.h>
//using namespace std;
//typedef pcl::PointXYZ point;
//typedef pcl::PointCloud<point> pointcloud;
//
//
//int main(int argc, char **argv)
//{
//	pointcloud::Ptr cloud(new pointcloud);
//	pcl::io::loadPCDFile("point_pcd.pcd", *cloud);
//	cout << "points size is:" << cloud->size() << endl;
//	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
//
//	//创建存储的mls对象
//	//    pcl::PointCloud<pcl::PointNormal> mls_points;
//	pcl::PointCloud<point> mls_points;
//
//	//创建mls对象
//	//  pcl::MovingLeastSquares<point,pcl::PointNormal> mls;
//
//	pcl::MovingLeastSquares<point, point> mls;
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true); //设置为true则在平滑过程中采用多项式拟合来提高精度
//	mls.setPolynomialOrder(2); //MLS拟合的阶数，默认是2
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(1.1);  //这个值越大，输出的点越多
//
//	mls.process(mls_points);
//
//	cout << "mls poits size is: " << mls_points.size() << endl;
//
//	// Save output
//	pcl::io::savePCDFile("mid-mls.pcd", mls_points);
//
//}
////return 0;
//
/////****************************
////* 给定一个融合后的点云，对其进行下采样和滤波。
////* 再进行平滑（输出结果），然后计算法线，并讲法线显示在平滑后的点云上。
////****************************/
////
////#include <pcl/point_types.h>
////#include <pcl/io/io.h>
////#include <pcl/io/pcd_io.h>
////#include <pcl/visualization/cloud_viewer.h>
////#include <pcl/filters/radius_outlier_removal.h>
////#include <pcl/filters/voxel_grid.h>
////#include <pcl/filters/statistical_outlier_removal.h>
////#include <pcl/surface/mls.h>
////#include <pcl/features/normal_3d.h>
////
////typedef pcl::PointXYZRGB PointT;
////
////int main(int argc, char** argv)
////{
////
////	// Load input file
////	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
////	pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
////	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
////	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
////	if (pcl::io::loadPCDFile("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/fusedCloud.pcd", *cloud) == -1)
////	{
////		cout << "点云数据读取失败！" << endl;
////	}
////
////	std::cout << "Orginal points number: " << cloud->points.size() << std::endl;
////
////	// 下采样，同时保持点云形状特征
////	pcl::VoxelGrid<PointT> downSampled;  //创建滤波对象
////	downSampled.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
////	downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
////	downSampled.filter(*cloud_downSampled);           //执行滤波处理，存储输出
////
////	pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/downsampledPC.pcd", *cloud_downSampled);
////
////	// 统计滤波
////	pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       //创建滤波器对象
////	statisOutlierRemoval.setInputCloud(cloud_downSampled);            //设置待滤波的点云
////	statisOutlierRemoval.setMeanK(50);                                //设置在进行统计时考虑查询点临近点数
////	statisOutlierRemoval.setStddevMulThresh(3.0);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
////	statisOutlierRemoval.filter(*cloud_filtered);                     //滤波结果存储到cloud_filtered
////
////	pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/filteredPC.pcd", *cloud_filtered);
////	// ----------------------开始你的代码--------------------------//
////	// 请参考PCL官网实现以下功能
////	// 对点云重采样
////	pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
////	pcl::PointCloud<PointT> mls_point;    //输出MLS
////	pcl::MovingLeastSquares<PointT, PointT> mls; // 定义最小二乘实现的对象mls
////	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
////	mls.setInputCloud(cloud_filtered);         //设置待处理点云
////	mls.setPolynomialOrder(2);            // 拟合2阶多项式拟合
////	mls.setPolynomialFit(false);     // 设置为false可以 加速 smooth
////	mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
////	mls.setSearchRadius(0.05);           // 单位m.设置用于拟合的K近邻半径
////	mls.process(mls_point);                 //输出
////
////	// 输出重采样结果
////	cloud_smoothed = mls_point.makeShared();
////	std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;
////
////	//save cloud_smoothed
////	pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/cloud_smoothed.pcd", *cloud_smoothed);
////
////
////	// 法线估计
////	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;             //创建法线估计的对象
////	normalEstimation.setInputCloud(cloud_smoothed);                         //输入点云
////	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
////	normalEstimation.setSearchMethod(tree);
////	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 定义输出的点云法线
////	// K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
////	normalEstimation.setKSearch(10);// 使用当前点周围最近的10个点
////	//normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
////	normalEstimation.compute(*normals);               //计算法线
////
////	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
////	pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/normals.pcd", *normals);
////
////
////
////	// 显示结果
////	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
////	viewer->setBackgroundColor(0, 0, 0);
////	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_smoothed);
////	viewer->addPointCloud<PointT>(cloud_smoothed, rgb, "smooth cloud");
////	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "smooth cloud");
////	viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud_smoothed, normals, 20, 0.05, "normals");
////
////	viewer->initCameraParameters();
////
////	while (!viewer->wasStopped())
////	{
////		viewer->spinOnce(100);
////		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
////	}
////
////	return 1;
////}
////
