////û���㹻���ھӱ����ǣ�ffn��sfn������Χ����������nnn����������R=29424Ϊ�߽磡
////��Ե�ھ������С������������1   ��62���еĲ�����2.5��Ϊ1.5����ʾ��ʧ
////��ƽ����������ǻ��������õ��˻���Delaunay�Ŀռ����������㷨���÷���ͨ��ѡȡһ����������Ƭ��Ϊ��ʼ���棬������������߽磬
////����γ�һ�������������������棬������ͶӰ���Ƶ����ӹ�ϵȷ����ԭʼ��ά�����������ӣ����õ���������Ϊ�ؽ��õ�������ģ�͡�
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
//	// ���Ʒ�����  
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals); //���㷨�ߣ�����洢��normals��  
//	//* normals ����ͬʱ������ķ������ͱ��������  
//	std::cout << "\n������ok "  << std::endl;
//	//�����ƺͷ��߷ŵ�һ��  
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals  
//	std::cout << "\n����+������ok " << std::endl;
//	//����������  
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//	std::cout << "\n������ok " << std::endl;
//	//��ʼ��GreedyProjectionTriangulation���󣬲����ò���  
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//	//����������������ڴ洢���  
//	pcl::PolygonMesh triangles;
//
//	//����GreedyProjectionTriangulation����Ĳ���  
//	//��һ������Ӱ��ܴ�  
//	gp3.setSearchRadius(200.0f);           //�������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶��Ĭ��ֵ 0��  
//	gp3.setMu(1.5);                       //��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶��Ĭ��ֵ 0��  
//	gp3.setMaximumNearestNeighbors(100);   //��������������ڵ���������  
//	gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees��pi�����ƽ���  
//	gp3.setMinimumAngle(M_PI / 18);        // 10 degrees ÿ�����ǵ���С�Ƕ�  
//	gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees ÿ�����ǵ����Ƕ�  
//	gp3.setNormalConsistency(false);       //���������һ�£�����Ϊtrue  
//	std::cout << "\n���������ok " << std::endl;
//	//���������������������  
//	gp3.setInputCloud(cloud_with_normals);
//	gp3.setSearchMethod(tree2);
//	std::cout << "\n�����������������ok " << std::endl;
//	//ִ���ع������������triangles��  
//	gp3.reconstruct(triangles);//�����������ʾ˵������
//	std::cout << "\n�ع�ok " << std::endl;
//	//��������ͼ  
//	//pcl::io::saveOBJFile("result.obj", triangles);
//	pcl::io::savePLYFile("triangulation.ply", triangles);
//	std::cout << "\n��������ͼok " << std::endl;
//	// Additional vertex information  
//	//std::vector<int> parts = gp3.getPartIDs();  
//	//std::vector<int> states = gp3.getPointStates();  
//
//	// ��ʾ���ͼ  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);                  //���ñ���  
//	viewer->addPolygonMesh(triangles, "my");              //������ʾ������
//	std::cout << "\n��ʾ���ͼok " << std::endl;
//	//��������ģ����ʾģʽ
//	//viewer->setRepresentationToSurfaceForAllActors();   //����ģ������Ƭ��ʽ��ʾ
//	//viewer->setRepresentationToPointsForAllActors();    //����ģ���Ե���ʽ��ʾ
//	viewer->setRepresentationToWireframeForAllActors();   //����ģ�����߿�ͼģʽ��ʾ
//	viewer->addCoordinateSystem(1.0);                     //��������ϵ  
//	viewer->initCameraParameters();
//	while (!viewer->wasStopped()){
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//	std::cout << "\n��������ģ����ʾģʽok " << std::endl;
//	return (0);
//}
