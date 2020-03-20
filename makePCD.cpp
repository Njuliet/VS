//#include <iostream>  //
//#include <pcl/io/pcd_io.h>  
//#include <pcl/point_types.h>  
//pcl/io/pcd_io.h 是包含了PCD文件IO操作定义的头文件，pcl/point_types.h 是包含了若干PointT数据结构定义的头文件。
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//
//	 Fill in the cloud data  
//	cloud.width = 5;
//	cloud.height = 1;
//	cloud.is_dense = false;
//	cloud.points.resize(cloud.width * cloud.height);
//
//	for (size_t i = 0; i < cloud.points.size(); ++i)
//	{
//		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//	}
//
//	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
//	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
//
//	for (size_t i = 0; i < cloud.points.size(); ++i)
//		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
//
//	return (0);
//}
