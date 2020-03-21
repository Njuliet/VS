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

	// ���Ʒ�����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //���㷨�ߣ�����洢��normals��
	//* normals ����ͬʱ������ķ������ͱ��������

	//�����ƺͷ��߷ŵ�һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals


	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//��ʼ��MarchingCubes���󣬲����ò���
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

	//����������������ڴ洢���
	pcl::PolygonMesh mesh;

	//����MarchingCubes����Ĳ���
	mc->setIsoLevel(0.0f);
	mc->setGridResolution(50, 50, 50);
	mc->setPercentageExtendGrid(0.0f);

	//������������
	mc->setInputCloud(cloud_with_normals);

	//ִ���ع������������mesh��
	mc->reconstruct(mesh);

	//��������ͼ
	pcl::io::savePLYFile("marchingCubes.ply", mesh);

	// ��ʾ���ͼ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //���ñ���
	viewer->addPolygonMesh(mesh, "my"); //������ʾ������
	viewer->addCoordinateSystem(1.0); //��������ϵ
	viewer->initCameraParameters();
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}

