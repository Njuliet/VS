#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile("dragonUpRight_transformed_icp.pcd", cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	// 估计法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //计算法线，结果存储在normals中
	//* normals 不能同时包含点的法向量和表面的曲率

	//将点云和法线放到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals


	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//初始化MarchingCubes对象，并设置参数
	pcl::MarchingCubes<pcl::PointNormal> *mc;
	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
	/*
	if (hoppe_or_rbf == 0)
	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
	else
	{
	mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
	(reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
	}
	*/

	//创建多变形网格，用于存储结果
	pcl::PolygonMesh mesh;

	//设置MarchingCubes对象的参数
	mc->setIsoLevel(0.0f);
	mc->setGridResolution(50, 50, 50);
	mc->setPercentageExtendGrid(0.0f);

	//设置搜索方法
	mc->setInputCloud(cloud_with_normals);

	//执行重构，结果保存在mesh中
	mc->reconstruct(mesh);

	//保存网格图
	pcl::io::savePLYFile("marchingCubes.ply", mesh);

	// 显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //设置背景
	viewer->addPolygonMesh(mesh, "my"); //设置显示的网格
	viewer->addCoordinateSystem(1.0); //设置坐标系
	viewer->initCameraParameters();
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}

