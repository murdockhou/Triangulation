#include <iostream>
#include "triangulation.h"

int main(){
	//read pcd files
	std::string fname = "input.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(fname,*cloud) == -1){
		std::cout << "open failed." << std::endl;
		exit(0);
	}
	Triangulation testTriangulation;
	testTriangulation.showTriangulation(cloud);
	return 0;
}