// ConsoleApplication2.cpp : 定义控制台应用程序的入口点。
//和PCD文件读取是一样的，只要把头文件的pcd改为ply,再把读取文件函数也改为LoadPLY,不知道对你是否有帮助 有人试过 可以

#include "stdafx.h"
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

int _tmain(int argc, _TCHAR* argv[])
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ply_read(new pcl::PointCloud<pcl::PointXYZRGBA>);




	pcl::PCLPointCloud2 cloud;
	pcl::PLYReader reader;
	//reader.read ("Scene0_1-8.ply", cloud);

	pcl::io::loadPLYFile("Armadillo_vres2_small_scaled.ply",cloud);
	pcl::io::savePCDFile("Scene0_1.pcd",cloud);

	pcl::io::loadPCDFile("Scene0_1.pcd",*ply_read);

	PCL_INFO("%d ", ply_read->points.size());

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  pcl::PLYReader reader;
  tt.tic ();
  if (reader.read (filename, cloud) < 0)
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::PCDWriter writer;
  writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);






	/*
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud(cloud_out);
	while (!viewer.wasStopped ())
	{
	}
	 
	 
	 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	while (!viewer->wasStopped ())
	{
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
	

	pcl::visualization::PCLVisualizer viewer("xxx");

	viewer.addPointCloud(ply_read);
	int s=0;
	while (!viewer.wasStopped())
	{
	s++;
	viewer.spinOnce(100);
	//pcl_sleep(0.01);
	}

	
	**/


	return 0;
}

