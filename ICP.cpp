//输入两个.ply文件然后输出配准结果为pcd文件
//P和Q是同一个场景有交集的不同位置处扫描的点云。
//http://pointclouds.org/documentation/tutorials/interactive_icp.php#interactive-icp


#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/filter.h>   //removeNaNFromPointCloud

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void
print4x4Matrix(const Eigen::Matrix4d & matrix)//刚性变换
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int
main()
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud  变换后的点云
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud


	int iterations = 1;  // Default number of ICP iterations


	pcl::console::TicToc time;
	time.tic();
	if (pcl::io::loadPLYFile("dragonUpRight_0.ply", *cloud_in) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n");
		return (-1);
	}
	std::cout << "\nLoaded file " << "dragonUpRight_0.ply" << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;


	if (pcl::io::loadPLYFile("dragonUpRight_24.ply", *cloud_tr) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n");
		return (-1);
	}
	std::cout << "\nLoaded file " << "dragonUpRight_24.ply" << " (" << cloud_tr->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	
	//去除NAN点   去除无效点之后程序不再报错
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices_src);//在使用PCL进行数据处理时候，很多算法都会考虑无效点,其主要是通过判断PointCloud类里的数据成员是否包含NaN。
	std::cout << "remove *cloud_in nan" << "in " << time.toc() << " ms\n"  << " valid points have " << cloud_in->size() << endl;
	
	std::vector<int> indices_src1; 
	pcl::removeNaNFromPointCloud(*cloud_tr, *cloud_tr, indices_src1);//
	std::cout << "remove *cloud_tr nan" << "in " << time.toc() << " ms\n"  << " valid points have " << cloud_tr->size() << endl;


	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	//double theta = M_PI / 4;  // The angle of rotation in radians   M_PI       3.14159265358979323846   以弧度表示的旋转角度
	//transformation_matrix(0, 0) = cos(theta);
	//transformation_matrix(0, 1) = -sin(theta);
	//transformation_matrix(1, 0) = sin(theta);
	//transformation_matrix(1, 1) = cos(theta);

	//// A translation on Z axis (0.4 meters)
	//transformation_matrix(2, 3) = 0.4;//Z轴的平移向量 (0.4 meters)

	// Display in terminal the transformation matrix
	//std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;//将这个严格的转换应用到:cloud_in -> cloud_icp
	//print4x4Matrix(transformation_matrix);//打印转换矩阵

	// Executing the transformation
	//pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	//使用刚性矩阵变换来变换原始点云。
	//cloud_in包含原始点云。 cloud_tr和cloud_icp包含平移/旋转的点云。 cloud_tr是我们将用于显示的备份（绿点云）。
	//*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
	*cloud_icp = *cloud_tr;
	// The Iterative Closest Point algorithm
	time.tic();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	//这是ICP对象的创建。我们设置ICP算法的参数。 setMaximumIterations（iterations）设置要执行的初始迭代次数（默认值为1）。
	//然后，我们将点云转换为cloud_icp。第一次对齐后，我们将在下一次使用此ICP对象时（当用户按下“空格”时）将ICP max迭代次数设置为1。
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		pcl::io::savePCDFileASCII("dragonUpRight_transformed_icp.pcd", *cloud_icp);
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (-1);
	}
	//检查ICP算法是否收敛；否则退出程序。如果成功，我们将转换矩阵存储在4x4矩阵中，然后打印刚性矩阵转换。
	// Visualization可视化
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two vertically separated viewports
	int v1(0);// 创建两个观察视点
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	//对于可视化，我们在可视化器中创建两个垂直分隔的视口。bckgr_gray_level和txt_gray_lvl是可轻松从白色背景和黑色文本/点云切换到黑色背景和白色文本/点云的变量。
	// Original point cloud is white  原始的点云设置为白色的
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	// Transformed point cloud is green// 转换后的点云显示为绿色
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red  ICP配准后的点云为红色
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
	//我们在2个视口中添加原始点云，并将其显示为与txt_gray_lvl相同的颜色。我们在绿色的左侧视口中添加使用矩阵进行变换的点云，在红色的右侧视口中添加与ICP对齐的点云。
	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;   //输入的迭代的次数
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
	//我们在每个视口中添加了对点云的描述，以便用户知道什么。需要使用字符串流ss将整数迭代转换为字符串。
	// Set background color   设置背景颜色
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation   设置相机的坐标和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Register keyboard callback :  注册按键回调函数
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
	//我们根据bckgr_gray_level设置两个视口的背景色。要获取相机参数，我只需在查看器中按“ C”即可。然后，我将参数复制到此函数中，以保存相机的位置/方向/焦点。函数registerKeyboardCallback允许我们在查看器窗口位于顶部时每当用户按下键盘键时调用函数。
	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		//如果没有按键，这是正常现象。观众等待退出。
		// The user pressed "space" :
		if (next_iteration)
		{
			// The Iterative Closest Point algorithm
			time.tic();
			//如果用户按下键盘，功能任意键keyboardEventOccurred被调用; 此功能检查键是否为“空格”。如果是，则全局布尔值next_iteration 设置为true，从而允许查看器循环输入代码的下一部分：调用ICP对象以对齐网格。记住，我们已经配置了该对象输入/输出云，并且在第90-93行中将最大迭代次数设置为1。
			icp.align(*cloud_icp);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				pcl::io::savePCDFileASCII("dragonUpRight_transformed_icp.pcd", *cloud_icp);
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose打印原始位姿和当前位姿之间的转换

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}
	return (0);
}



