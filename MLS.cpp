//////#include <pcl/point_types.h>
//////#include <pcl/io/pcd_io.h>
//////#include <pcl/kdtree/kdtree_flann.h>  //kd-tree����������ඨ���ͷ�ļ�
//////#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�
//////
//////int
//////main(int argc, char** argv)
//////{
//////	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//////	pcl::io::loadPCDFile("point_pcd.pcd", *cloud);
//////
//////	// ���� KD-Tree
//////	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//////
//////	// Output has the PointNormal type in order to store the normals calculated by MLS
//////	pcl::PointCloud<pcl::PointNormal> mls_points;
//////
//////	// ������С����ʵ�ֵĶ���mls
//////	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//////
//////	mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���
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
//	//�����洢��mls����
//	//    pcl::PointCloud<pcl::PointNormal> mls_points;
//	pcl::PointCloud<point> mls_points;
//
//	//����mls����
//	//  pcl::MovingLeastSquares<point,pcl::PointNormal> mls;
//
//	pcl::MovingLeastSquares<point, point> mls;
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true); //����Ϊtrue����ƽ�������в��ö���ʽ�������߾���
//	mls.setPolynomialOrder(2); //MLS��ϵĽ�����Ĭ����2
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(1.1);  //���ֵԽ������ĵ�Խ��
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
////* ����һ���ںϺ�ĵ��ƣ���������²������˲���
////* �ٽ���ƽ��������������Ȼ����㷨�ߣ�����������ʾ��ƽ����ĵ����ϡ�
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
////	if (pcl::io::loadPCDFile("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/fusedCloud.pcd", *cloud) == -1)
////	{
////		cout << "�������ݶ�ȡʧ�ܣ�" << endl;
////	}
////
////	std::cout << "Orginal points number: " << cloud->points.size() << std::endl;
////
////	// �²�����ͬʱ���ֵ�����״����
////	pcl::VoxelGrid<PointT> downSampled;  //�����˲�����
////	downSampled.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
////	downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
////	downSampled.filter(*cloud_downSampled);           //ִ���˲������洢���
////
////	pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/downsampledPC.pcd", *cloud_downSampled);
////
////	// ͳ���˲�
////	pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       //�����˲�������
////	statisOutlierRemoval.setInputCloud(cloud_downSampled);            //���ô��˲��ĵ���
////	statisOutlierRemoval.setMeanK(50);                                //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
////	statisOutlierRemoval.setStddevMulThresh(3.0);                     //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ:��ֵ+1.0*��׼��
////	statisOutlierRemoval.filter(*cloud_filtered);                     //�˲�����洢��cloud_filtered
////
////	pcl::io::savePCDFile("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/filteredPC.pcd", *cloud_filtered);
////	// ----------------------��ʼ��Ĵ���--------------------------//
////	// ��ο�PCL����ʵ�����¹���
////	// �Ե����ز���
////	pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>);// �������������������KD-Tree
////	pcl::PointCloud<PointT> mls_point;    //���MLS
////	pcl::MovingLeastSquares<PointT, PointT> mls; // ������С����ʵ�ֵĶ���mls
////	mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
////	mls.setInputCloud(cloud_filtered);         //���ô��������
////	mls.setPolynomialOrder(2);            // ���2�׶���ʽ���
////	mls.setPolynomialFit(false);     // ����Ϊfalse���� ���� smooth
////	mls.setSearchMethod(treeSampling);         // ����KD-Tree��Ϊ��������
////	mls.setSearchRadius(0.05);           // ��λm.����������ϵ�K���ڰ뾶
////	mls.process(mls_point);                 //���
////
////	// ����ز������
////	cloud_smoothed = mls_point.makeShared();
////	std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;
////
////	//save cloud_smoothed
////	pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/cloud_smoothed.pcd", *cloud_smoothed);
////
////
////	// ���߹���
////	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;             //�������߹��ƵĶ���
////	normalEstimation.setInputCloud(cloud_smoothed);                         //�������
////	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// �������������������KD-Tree
////	normalEstimation.setSearchMethod(tree);
////	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // ��������ĵ��Ʒ���
////	// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
////	normalEstimation.setKSearch(10);// ʹ�õ�ǰ����Χ�����10����
////	//normalEstimation.setRadiusSearch(0.03);            //����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
////	normalEstimation.compute(*normals);               //���㷨��
////
////	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
////	pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/��ҵ15-����ƽ�������߹��Ƽ���ʾ/data/normals.pcd", *normals);
////
////
////
////	// ��ʾ���
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
