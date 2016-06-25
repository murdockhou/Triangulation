#include "triangulation.h"

Triangulation::Triangulation(){

}

Triangulation::~Triangulation(){

}
/***********************************************************
*showTriangulation函数，对点云进行快速三角网格化
*输入：不带有颜色信息的点云数据
*输出：显示出快速三角网格化后的结果
*      生成个ply(vtk,stl)文件
*
***********************************************************/
void Triangulation::showTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	std::cout << "Loaded " << cloud->width * cloud->height << " data points from data." << std::endl;
	//下面就是进行快速三角网格化过程
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//Concatenate the XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
	//Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloudWithNormals);
	//Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	//set parameters
	gp3.setSearchRadius(20.0);
	gp3.setMu(3);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI/4);
	gp3.setMinimumAngle(M_PI/18);
	gp3.setMaximumAngle(M_PI/3);
	gp3.setNormalConsistency(true);
	//Get result
	gp3.setInputCloud(cloudWithNormals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	//create ply file, you can create .ply or .vtk or .stl files.
	std::string polygonFileName = "polygon.ply";
	//std::string polygonFileName = "polygon.vtk";
	//std::string polygonFileName = "polygon.stl";
	pcl::io::savePolygonFile(polygonFileName,triangles);
	//Viewer
	pcl::visualization::PCLVisualizer viewer("triangulation");
	viewer.addPolygonMesh(triangles);
	viewer.spin();

	return ;

}
/***********************************************************
*showTriangulationRGB函数，对点云进行快速三角网格化
*输入：带有颜色信息的点云数据
*输出：显示出快速三角网格化后的结果
*      生成个ply(vtk,stl)文件
*
***********************************************************/
void Triangulation::showTriangulationRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	std::cout << "Loaded " << cloud->width * cloud->height << " data points from data." << std::endl;
	//下面就是进行快速三角网格化过程
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//Concatenate the XYZ and normal fields
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
	//Create search tree
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloudWithNormals);
	//Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;
	//set parameters
	gp3.setSearchRadius(20.0);
	gp3.setMu(3);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI/4);
	gp3.setMinimumAngle(M_PI/18);
	gp3.setMaximumAngle(M_PI/3);
	gp3.setNormalConsistency(true);
	//Get result
	gp3.setInputCloud(cloudWithNormals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	//create ply file, you can create .ply or .vtk or .stl files.
	std::string polygonFileName = "polygon.ply";
	//std::string polygonFileName = "polygon.vtk";
	//std::string polygonFileName = "polygon.stl";
	pcl::io::savePolygonFile(polygonFileName,triangles);
	//Viewer
	pcl::visualization::PCLVisualizer viewer("triangulation");
	viewer.addPolygonMesh(triangles);
	viewer.spin();

	return ;
}