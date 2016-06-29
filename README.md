# Triangulation
利用pcl点云库函数对pcd文件的点云进行一些处理，包括但不仅限于：

  1.showTriangulation函数，快速三角网格化处理并并将三角网格保存下来
  
  2.read and write function （it's very simply, maybe it should not be a member function at all.）
  
  3.利用vtkSmoothPointer来对点云数据进行平滑处理

系统环境：

  vs2010 + pcl 1.6.0 64位,其它版本的vs应该也可以
