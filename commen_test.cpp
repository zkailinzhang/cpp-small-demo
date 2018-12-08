// commen_test.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataReader.h>
#include <pcl/features/board.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <sstream>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
/*
pcl::io::savePCDFile("save_ply2vtk2pcd2.pcd",*cloud,false);  false ascii码 能用文本打开 存储速度慢； true 二进制 不能用txt打开 存储快 
*/

int _tmain(int argc, _TCHAR* argv[])
{
	//pcd文件显示
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::io::loadPCDFile("model1.pcd",*cloud);



	//pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh()) ;
	//vtkPolyData* mesh =   pcl::PLYReader("2.ply");// LoadPolyData()  PLYReader
	//pcl::io::loadPolygonfileply
	//	pcl::io::loadPLYFile("2.ply",*mesh);



	//ply文件显示
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();

	stringstream ss_tou;
	ss_tou<<"E://datasets//SHOT//dataset4 kinect cvlab//3D models scenes//CVLab//xx//";  // 最后一定要zai加个   //

	for (int i = 1; i<24; i++)
	{
    stringstream ss;
	ss<<ss_tou.str()<<i<<".ply";


	if (pcl::io::loadPolygonFilePLY (ss.str(), mesh) < 0)
		return -1;


	// ply另存vtk
	pcl::io::saveVTKFile("temp.vtk", mesh);
	pcl::io::mesh2vtk(mesh, polydata);
	//viewer.addModelFromPolyData(polydata); // To check that the conversion until here is fine



	pcl::io::vtkPolyDataToPointCloud (polydata, *cloud);
	//std::cout << "Cloud: \n" << *cloud << std::endl;


	//cout<<"点云数目："<< cloud->points.size() ;


	//两种存贮方式 pcd另存pcd
	pcl::PCDWriter pcdwriter;
	//pcdwriter.write<pcl::PointXYZRGBA>("save_ply2vtk2pcd.pcd", *cloud);
	ss.str("");
	ss<<ss_tou.str()<<i<<".pcd";
	pcl::io::savePCDFileASCII(ss.str(),*cloud);
}

	pcl::PLYWriter plywriter;
	//pcd另存ply 显示仍然是 点云形式 ，木有渲染， 也有两种方式    
	//plywriter.write("save_ply2vtk2pcd2.ply",*cloud);
	//pcl::io::savePLYFile("save_ply2vtk2pcd22.ply",*cloud);

	//mesh到ply  mesh到vtk  实心 不是点云形式
	//pcl::io::savePLYFile("save_ply2vtk2pcd22.ply",mesh); 
	pcl::io::saveVTKFile("save_ply2vtk2pcd22.ply",mesh);




	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud,centroid);
	pcl::demeanPointCloud<pcl::PointXYZRGBA>(*cloud,centroid, *cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("ss"));
	viewer->initCameraParameters();



	//viewer->setBackgroundColor(0.3,0.3,0.3);
	viewer->addCoordinateSystem(1.0f);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color(cloud);



	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, color, "cloud");
	


	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,3, "cloud");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

	}






	return 0;
}

