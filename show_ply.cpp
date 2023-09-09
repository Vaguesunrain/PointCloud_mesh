#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
int user_data;
using namespace std;
using pcl::PolygonMesh;
using pcl::io::loadPolygonFilePLY;
using pcl::visualization::PCLVisualizer;


int main()
{
	PolygonMesh human_1_mesh;
  loadPolygonFilePLY("demo.ply", human_1_mesh);
  std::cerr << "点云读入   完成" << std::endl;

  PCLVisualizer* viewer = new PCLVisualizer("Mesh viewer");
  viewer->addPolygonMesh(human_1_mesh);
  while (!viewer->wasStopped()) { viewer->spinOnce(100); }
  viewer->close();
  delete viewer;



	return 0;
}