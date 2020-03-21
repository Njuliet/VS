////没有足够的邻居被考虑：ffn或sfn超出范围！考虑增加nnn。。。设置R=29424为边界！
////边缘邻居邻域大小增加请求数：1   将62行中的参数由2.5改为1.5后提示消失
////在平面区域的三角化过程中用到了基于Delaunay的空间区域增长算法，该方法通过选取一个样本三角片作为初始曲面，不断扩张曲面边界，
////最后形成一张完整的三角网格曲面，最后根据投影点云的连接关系确定各原始三维点间的拓扑连接，所得的三角网格即为重建得到的曲面模型。
////https://blog.csdn.net/qq_35768238/article/details/80577623
//#include <pcl/point_types.h>  
//#include <pcl/io/pcd_io.h>  
//#include <pcl/io/ply_io.h>  
//#include <pcl/io/obj_io.h>
//#include <pcl/PolygonMesh.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/kdtree/kdtree_flann.h>  
//#include <pcl/features/normal_3d.h>  
//#include <pcl/surface/gp3.h>  
//#include <pcl/visualization/pcl_visualizer.h>  
//#include <boost/thread/thread.hpp>  
//#include <fstream>  
//#include <iostream>  
//#include <stdio.h>  
//#include <string.h>  
//#include <string>  
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//int main(int argc, char** argv)
//{
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCLPointCloud2 cloud_blob;
//	pcl::io::loadPCDFile("dragonUpRight_transformed_icp.pcd", cloud_blob);
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//		//* the data should be available in cloud
//
//	// 估计法向量  
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals); //计算法线，结果存储在normals中  
//	//* normals 不能同时包含点的法向量和表面的曲率  
//	std::cout << "\n法向量ok "  << std::endl;
//	//将点云和法线放到一起  
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals  
//	std::cout << "\n点云+法向量ok " << std::endl;
//	//创建搜索树  
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//	std::cout << "\n搜索树ok " << std::endl;
//	//初始化GreedyProjectionTriangulation对象，并设置参数  
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//	//创建多变形网格，用于存储结果  
//	pcl::PolygonMesh triangles;
//
//	//设置GreedyProjectionTriangulation对象的参数  
//	//第一个参数影响很大  
//	gp3.setSearchRadius(200.0f);           //设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径【默认值 0】  
//	gp3.setMu(1.5);                       //设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】  
//	gp3.setMaximumNearestNeighbors(100);   //设置搜索的最近邻点的最大数量  
//	gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees（pi）最大平面角  
//	gp3.setMinimumAngle(M_PI / 18);        // 10 degrees 每个三角的最小角度  
//	gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees 每个三角的最大角度  
//	gp3.setNormalConsistency(false);       //如果法向量一致，设置为true  
//	std::cout << "\n多边形网格ok " << std::endl;
//	//设置搜索方法和输入点云  
//	gp3.setInputCloud(cloud_with_normals);
//	gp3.setSearchMethod(tree2);
//	std::cout << "\n搜索方法和输入点云ok " << std::endl;
//	//执行重构，结果保存在triangles中  
//	gp3.reconstruct(triangles);//这个函数会提示说有问题
//	std::cout << "\n重构ok " << std::endl;
//	//保存网格图  
//	//pcl::io::saveOBJFile("result.obj", triangles);
//	pcl::io::savePLYFile("triangulation.ply", triangles);
//	std::cout << "\n保存网格图ok " << std::endl;
//	// Additional vertex information  
//	//std::vector<int> parts = gp3.getPartIDs();  
//	//std::vector<int> states = gp3.getPointStates();  
//
//	// 显示结果图  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);                  //设置背景  
//	viewer->addPolygonMesh(triangles, "my");              //设置显示的网格
//	std::cout << "\n显示结果图ok " << std::endl;
//	//设置网格模型显示模式
//	//viewer->setRepresentationToSurfaceForAllActors();   //网格模型以面片形式显示
//	//viewer->setRepresentationToPointsForAllActors();    //网格模型以点形式显示
//	viewer->setRepresentationToWireframeForAllActors();   //网格模型以线框图模式显示
//	viewer->addCoordinateSystem(1.0);                     //设置坐标系  
//	viewer->initCameraParameters();
//	while (!viewer->wasStopped()){
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//	std::cout << "\n设置网络模型显示模式ok " << std::endl;
//	return (0);
//}
