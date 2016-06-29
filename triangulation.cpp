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
/***********************************************************
*readPCDFiles函数，读取pcd文件
*输入：string类型的文件名
*输出：无
*
***********************************************************/
void Triangulation::readPCDFiles(std::string &fileName){
	// load points without rgb
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloud) == -1){
		std::cout << "Load file error!";
		return ;
	}*/

	// load points with rgb
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName, *cloudRGB) == -1){
		std::cout << "Load file error!";
		return ;
	}

	return ;
}

/***********************************************************
*writePCDFiles函数，保存写入pcd文件
*输入：string类型的文件名
*输出：已string为名字的pcd文件
*
***********************************************************/
void Triangulation::writePCDFiles(std::string &fileName){
	// we assume that we already have a points data named cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(fileName,*cloud,false);
	//pcl::io::savePCDFile(fileName,*cloud,false);

	return ;
}
/***********************************************************
*vtkSmooth函数，根据vtk来平滑点云数据
*输入：指向点云数据的cloud指针
*输出：smooth过后的点云数据
*
***********************************************************/
void Triangulation::vtkSmooth(pcl::PointCloud<pcl::PointXYZ>::Ptr finalcloud){
	//Create parabola over cloud of points
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for (size_t i=0; i<finalcloud->points.size(); ++i){
		double x = (double)finalcloud->points[i].x;
		double y = (double)finalcloud->points[i].y;
		double z = (double)finalcloud->points[i].z;	
		points->InsertNextPoint(x,y,z);
		
	}
	
	//Add the  points to a polydata object
	vtkSmartPointer<vtkPolyData> inputPolyData = vtkSmartPointer<vtkPolyData>::New();

	inputPolyData->SetPoints(points);

	//Triangulate the grid points
	vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
#if VTK_MAJOR_VERSION <= 5
	delaunay->SetInput(inputPolyData);
#else
	delaunay->SetInputData(inputPolyData);
#endif
	delaunay->Update();
	//Do smooth,最重要的是SetNumberOfIterations函数，参数意思为迭代次数，越大，拟合的越光滑，但会失去更多的信息
	vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
	smoothFilter->SetInputConnection(delaunay->GetOutputPort());
	smoothFilter->SetNumberOfIterations(10);
	smoothFilter->SetRelaxationFactor(0.1);
	smoothFilter->FeatureEdgeSmoothingOff();
	smoothFilter->BoundarySmoothingOn();
	smoothFilter->Update();

	//Update normals on newly smoothed polydata
	vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
	normalGenerator->SetInputConnection(smoothFilter->GetOutputPort());
	normalGenerator->ComputePointNormalsOn();
	normalGenerator->ComputeCellNormalsOn();
	normalGenerator->Update();


	//Make colors by hsw
	//update 2016-4-6 20:45PM
	vtkPolyData* outputPolyData = normalGenerator->GetOutput();

	double bounds[6];
	outputPolyData->GetBounds(bounds);

	// Find min and max z
	double minz = bounds[4];
	double maxz = bounds[5];

	/*std::cout << "minz: " << minz << std::endl;
	std::cout << "maxz: " << maxz << std::endl;*/

	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(minz, maxz);
	colorLookupTable->Build();

	// Generate the colors for each point based on the color map
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	//make color base on the z
	for(int i = 0; i < outputPolyData->GetNumberOfPoints(); i++)
	{
		double p[3];
		outputPolyData->GetPoint(i,p);
		
		double dcolor[3];
		colorLookupTable->GetColor(p[2], dcolor);
		unsigned char color[3];
		for(unsigned int j = 0; j < 3; j++)
		{
			color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
		}
		
#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(color);
#else
		colors->InsertNextTypedTuple(color);
#endif
	}

	outputPolyData->GetPointData()->SetScalars(colors);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInputConnection(outputPolyData->GetProducerPort());
#else
	mapper->SetInputData(outputPolyData);
#endif
	//写ply文件
	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetArrayName("Colors");
	writer->SetInputConnection(outputPolyData->GetProducerPort());
	writer->SetFileName("GroundPointsDEM");
	writer->Update();
	writer->Write();

	std::string plyName = "vtkSmooth.ply";
	//z_GroundPoints2(finalcloud,plyName,outputPolyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// Create a renderer, render window, and interactor
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000,562);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	// Add the actor to the scene
	renderer->AddActor(actor);
	renderer->SetBackground(.1, .2, .3);

	// Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();

	
	return ;
}
