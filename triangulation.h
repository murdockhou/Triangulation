#pragma once
#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include<pcl/io/pcd_io.h>		
#include<pcl/point_types.h>	
#include<pcl/features/normal_3d.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/surface/gp3.h>
#include<pcl/io/vtk_lib_io.h>
#include<iostream>
/**********************************************
本文件用于对点云pcd文件的一些处理
Edited by HSW 2016.06.25:
	对pcd文件进行读取并进行快速三角网格化。
**********************************************/
class Triangulation{
public:
	Triangulation();
	~Triangulation();
	// read pcd files
	void readPCDFiles(std::string &fileName);
	// save points data using .pcd formate
	void writePCDFiles(std::string &fileName);
	//这个是不带颜色信息的点云快速三角网格化
	void showTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	//这个是带颜色信息的点云快速三角网格化
	void showTriangulationRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};
#endif

