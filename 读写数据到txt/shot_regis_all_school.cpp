// shot_regis_all_school.cpp : 定义控制台应用程序的入口点。
//

// shot_regis.cpp : 定义控制台应用程序的入口点。


// 20151106

#include "stdafx.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/PolygonMesh.h>

#include <pcl/io/boost.h>
#include <boost/thread/thread.hpp>

#include <pcl/correspondence.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>


//using namespace cv;
using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT1344 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (true);
float model_ss_ (0.01f);    ///两者若一样， 则出现两对匹配的实例，，还是大点  下采样的半径
float scene_ss_ (0.015f);   //原文0.03 0.02
float rf_rad_ (0.015f);     //表示坐标轴支持域 空间半径
float descr_rad_ (0.03f);   //原文0.02
float cg_size_ (0.01f);     //聚类 霍夫空间单元格 大小  聚类  分辨率  ？？
float cg_thresh_ (5.0f);    // 聚类最小 5个 投票 ？？？

float neigh_sqr_dists_thresh (0.3);


/*  debug release 下调试 ，  第一个参数是模板，第二个是场景，模板内就一个对象，场景可多个
 *   带颜色不待颜色调试，，  特征描述子是 SHOT  
 描述子也是点云类型的  局部坐标轴也是点云类型的  这样描述不准确   
 聚类输入是关键点
 局部坐标轴也是在关键点
 kdtree 匹配对应对的 描述子距离 Find Model-Scene Correspondences with KdTree
 下采样（均匀采样）结果存为 索引 	pcl::PointCloud<int> sampled_indices;
 视口 点云 字体 的id标签可以是 变量stringstream


 模板就整个一个对象，单个物体

 //   each instance of the model found into the scene
      the model keypoints descriptor cloud
 
 *	omp 调试

定义了两个对应对， 两者的属性 都是一维数组？
第一个 有误匹配 输入是描述子  
pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());//  对应对匹配是 根据描述子距离匹配  方法是kdtree flann  
第二个是  聚类算法 输入关键点和第一个匹配对  输出  没有误判的  模板与场景中正确对象的匹配对 
std::vector<pcl::Correspondences> clustered_corrs;  //  以对应对为元素的数组  聚类 关键点

过多的用 pcl::copyPointCloud     pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);

 pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
 viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
 viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");


关键点显示 可以关闭，不然全是蓝色  覆盖模型颜色
警告：1.8.0\include\flann\algorithms\kmeans_index.h(469): warning C4291: “void *operator new(size_t,flann::PooledAllocator &)”: 未找到匹配的删除运算符；如果初始化引发异常，则不会释放内存

1788 78 
3324  31 476  两个变化矩阵很相近
5474 528

 */
 //输出变换矩阵，模板中的对象 instance到场景 和 对应对数组 

void
	showHelp (char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                     Show this help." << std::endl;
	std::cout << "     -k:                     Show used keypoints." << std::endl;
	std::cout << "     -c:                     Show used correspondences." << std::endl;
	std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
	std::cout << "                             each radius given by that value." << std::endl;
	std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
	std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
	std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
	std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
	std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
	std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
	std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}





//处理点云序列的结构定义
struct PCD
{
	PointCloud::Ptr cloud;
	std::string f_name;    //???
	PCD() : cloud (new PointCloud) {};
};
struct PCDComparator
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};

////////////////////////////////////////////////////////////////////////////////
/**加载一组我们想要匹配在一起的PCD文件                               pcd    ply  mesh  obj
* 参数argc是参数的数量 (pass from main ())
*参数 argv 实际的命令行参数 (pass from main ())
*参数models点云数据集的合成矢量
*/
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension (".pcd");
	//假定第一个参数是实际测试模型
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string (argv[i]);
		// 至少需要5个字符长（因为.plot就有 5个字符）
		if (fname.size () <= extension.size ())                                        //!!!!!
			continue;  //执行for 

		std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
		//检查参数是一个pcd文件
		if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)   //扩展名比较
		{
			//加载点云并保存在总体的模型列表中
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile (argv[i], *m.cloud);
			//从点云中移除NAN点
			std::vector<int> indices22;
			pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices22);
			models.push_back (m);
		}
	}
}

/************************************************************************/
/* 说明：The next function performs the spatial resolution computation for a given point cloud 
averaging the distance between each cloud point and its nearest neighbor.                                                                     */
/************************************************************************/
double
	computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))  //查询点云中 x坐标
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	//创建3D窗口并添加点云其包括法线  
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("shot__noraml"));

	int v1(0),v2(0);
	viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
	viewer->setBackgroundColor(0.0,0.0,0.0,v1);
	viewer->addText("orgin_cloud",10,10,"v1_txt",v1);

	     
	viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
	viewer->setBackgroundColor(0.3,0.3,0.3,v2);
	viewer->addText("noraml_cloud",10,10,"v2_txt",v2);

	viewer->setBackgroundColor (0, 0, 0);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud",v1);
	   
	//  vfh中的数据库10  0.05每10个点显示一个法矢，长度5cm； shot中的数据库ply pcd ， 10，1；
	viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals,10, 5, "normals",v2); 
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud",v1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "normals",v2);

	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	return (viewer);
}

void demean_for_show(pcl::PointCloud<PointType>::Ptr &src, pcl::PointCloud<PointType>::Ptr &dst )
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*src,centroid);
	pcl::demeanPointCloud<PointType>(*src,centroid, *dst);
}



int _tmain(int argc, _TCHAR* argv[])
{
	pcl::PointCloud<PointType>::Ptr result_all (new pcl::PointCloud<PointType>);  
	pcl::PointCloud<PointType>::Ptr result_two (new pcl::PointCloud<PointType>); 
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > all_rototranslations;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();

	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>);      //输入带颜色的点云数据
	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());  //关键点也是带颜色的
	pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());  //关键点也是带颜色的
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	char* arg[] = {"","chef_view1-22_ply\\chef_view1.pcd",
		"chef_view1-22_ply\\chef_view2.pcd",
		"chef_view1-22_ply\\chef_view3.pcd",
		"chef_view1-22_ply\\chef_view4.pcd",
		"chef_view1-22_ply\\chef_view5.pcd",
		"chef_view1-22_ply\\chef_view6.pcd",
		"chef_view1-22_ply\\chef_view7.pcd",
		"chef_view1-22_ply\\chef_view8.pcd",
		"chef_view1-22_ply\\chef_view9.pcd",
		"chef_view1-22_ply\\chef_view10.pcd",
		"chef_view1-22_ply\\chef_view11.pcd",
		"chef_view1-22_ply\\chef_view12.pcd",
		"chef_view1-22_ply\\chef_view13.pcd",
		"chef_view1-22_ply\\chef_view14.pcd",
		"chef_view1-22_ply\\chef_view15.pcd",
		"chef_view1-22_ply\\chef_view16.pcd"};
/*		"chef_view1-22_ply\\chef_view17.pcd",
		"chef_view1-22_ply\\chef_view17.pcd",
		"chef_view1-22_ply\\chef_view18.pcd",
		"chef_view1-22_ply\\chef_view19.pcd",
		"chef_view1-22_ply\\chef_view20.pcd",
		"chef_view1-22_ply\\chef_view21.pcd",
		"chef_view1-22_ply\\chef_view22.pcd"};  */

	int arc= 17;

	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;   //点云序列
	loadData (arc, arg, data);

	if (data.empty ()){
		return (-1);
	}
	PCL_INFO ("Loaded %d datasets.", (int)data.size ());


for (size_t k = 1; k < data.size (); ++k)
{

		pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>);      //输入带颜色的点云数据
		pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());  //关键点也是带颜色的
		pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());  //关键点也是带颜色的
		pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
		pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
		pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
		pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

		scene = data[k-1].cloud;
		model = data[k].cloud;
	    PCL_INFO("  model : %d Points \n",model->points.size());
	    PCL_INFO("  scene : %d Points \n",scene->points.size());

	  if (true)   // 调整分辨率
	  {
		float resolution = static_cast<float> (computeCloudResolution (model));       //从ply转换过来的pcd 分辨率变成了 0.103143
		float resolution_scen = static_cast<float> (computeCloudResolution (scene));   //0.105135
		cout<<"点云分辨率是："<<resolution<<" "<<resolution_scen<<endl;

		if (resolution != 0.0f) {
			model_ss_    = resolution *2.5;    //  即关键点数目   bunny 2.5
			scene_ss_    = resolution *2.5;     //    6     bunny 2.5 
			rf_rad_      = resolution *10;   
			descr_rad_   = resolution *10;    // bunny 20
			cg_size_     = resolution *5; 
		}
		//std::cout << "Model resolution:       " << resolution << std::endl;  //不是默认输出，都特别小
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
	  }
	
	//
	//  Compute Normals                               法向量计算
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est(4);        //omp加速
	//pcl::NormalEstimation<PointType, NormalType> norm_est;            //好多成员函数没用呢？？、/
	norm_est.setKSearch (15);                                           //13 有好多nan点
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);
	//pcl::io::savePCDFileASCII("scen0e_norm.pcd",*scene_normals);
	//PCL_INFO("  model norm : %d Points \n",model_normals->points.size());
	//PCL_INFO("  scene norm : %d Points \n",scene_normals->points.size());

	demean_for_show(scene,scene);  //质心移到坐标原点  方便最终显示 
	demean_for_show(model,model);
/*	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_norm;  
	//demean_for_show_xyz(cloud_filter,cloud_filter);
	viewer_norm =  normalsVis(scene, scene_normals);
	while  (!viewer_norm->wasStopped ())   // while 一直判断  if 就判断一次
	{
		viewer_norm->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer_norm->close();
	
*/

	//
	//  Downsample Clouds to Extract keypoints        关键点 均匀采样  多点好
	//
	pcl::PointCloud<int> sampled_indices;

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.compute (sampled_indices);

	pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);
	std::cout << "Model Selected Keypoints: " << model_keypoints->size () << std::endl;

	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.compute (sampled_indices);

	pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
	std::cout << "Scene Selected Keypoints: " << scene_keypoints->size () << std::endl;

	//
	//  Compute Descriptor for keypoints                描述子 shot
	//
	//pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est(4); //OMP  没有编译OpenMP 能正常运行不？？、、

	pcl::SHOTColorEstimationOMP<PointType, NormalType, DescriptorType> descr_est(4); 
	descr_est.setRadiusSearch (descr_rad_);

	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);    //!!!!!
// 	descr_est.setIndices(indices);
// 	descr_est.setSearchMethod(tree);
// 	descr_est.setRadiusSearch(0.06);

	descr_est.compute (*model_descriptors);

	descr_est.setInputCloud (scene_keypoints);
	descr_est.setInputNormals (scene_normals);
	descr_est.setSearchSurface (scene);
	descr_est.compute (*scene_descriptors);

	PCL_INFO("  model shot : %d Points \n",model_descriptors->points.size());    //23407  7587
	PCL_INFO("  scene shot : %d Points \n",scene_descriptors->points.size());


	//  对应对匹配是 根据描述子距离匹配  方法是kdtree 
	//  Find Model-Scene Correspondences with KdTree 模板为source，根据模板在场景目标中找对应对       根据 描述子 找 对应对 KdTree flann
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);      //切记 搜索空间

	//  For each scene key point descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t j = 0; j < scene_descriptors->size (); ++j) //i 是 场景中多少个描述子
	{
		std::vector<int> neigh_indices (1);            //就要一个 近邻索引
		std::vector<float> neigh_sqr_dists (1);

		if (!pcl_isfinite (scene_descriptors->at (j).descriptor[0])) //skipping NaNs  看第一个元素!!!!!!??? 有限小数跳过if语句
		{
			//scene_descriptors->at(i).descriptor[12];  访问描述子向量 的值  i是第几个描述子  [x] 描述子向量中的某个维度值
			//scene_descriptors->at(i).descriptorSize(); //当前描述子的维度
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (j), 1, neigh_indices, neigh_sqr_dists);// 搜索找到则1 反之0 ；设置k 则返回k？？		
		
		if(found_neighs == 1 && neigh_sqr_dists[0] < neigh_sqr_dists_thresh) //0.15  0.25  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			//第二个参数 ？  两个点 及距离  。索引是在搜素空间搜到最近邻的那个点的索引值
			/**  Index of the matching (target) point. Set to -1 if no correspondence found. */
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (j), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	//关键点 描述子 对应对 相同的  1788 有正确的  有误匹配的 ，模板对象与场景其他物体
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

	//
	//  Actual Clustering                         聚类  full Clustering Pipeline:
	//

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;//?????什么类型   数组元素 及相应内存
	std::vector<pcl::Correspondences> clustered_corrs;  //  以对应对为元素的数组






	//  Using Hough3D           霍夫3D 方法
	if (use_hough_) 
	{
		//
		//  Compute (Key points) Reference Frames only for Hough  计算关键点处的局部坐标轴
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());                //!!!!!!
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;  //此方法对 RF估计 较好  对点云边缘和处理空洞，但好多成员函数没用呢？？
		rf_est.setFindHoles (true);   //考虑孔洞因素
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering   根据关键点
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;                          //重要 ！！！！！！！
		clusterer.setHoughBinSize (cg_size_);  //0.01  构造 3Dhough空间  几维的？？
		clusterer.setHoughThreshold (cg_thresh_); // 5
		clusterer.setUseInterpolation (true);     //考虑插值   测量值是准确的，木有误差，考虑插值，反之拟合
		clusterer.setUseDistanceWeight (false);   //距离权重

		clusterer.setInputCloud (model_keypoints);   //
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);

		clusterer.setModelSceneCorrespondences (model_scene_corrs);  // 两个不同阶段的对应对 联系节点

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);             //输出变换矩阵，模板中的对象 instance到场景 和 对应对数组 		
	}
	else // Using GeometricConsistency     用几何一致性方法
	{
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (cg_size_);
		gc_clusterer.setGCThreshold (cg_thresh_);

		gc_clusterer.setInputCloud (model_keypoints);
		gc_clusterer.setSceneCloud (scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
	}



	//
	//  Output results
	//  模型实例 在场景中 找到 1个 也就是1个变换矩阵  对应对 找到1788对  但属于该实例的是84对         变换矩阵 输出显示

	//   each instance of the model found into the scene
	std::cout << "-------打印第"<<k<<"转到"<<k-1<<"坐标下的变换矩阵--------"<<std::endl;
	std::cout << "Model instances found: " << rototranslations.size () << std::endl;  //Model instances ??? 是的 1 


	 

	
	 std::stringstream ss_cloud;
	 ss_cloud << "-------打印第"<<k<<"转到"<<k-1<<"坐标下的变换矩阵--------"<<"\n"<< "Model instances found: " << rototranslations.size () <<"\n";
	
	 cout<<ss_cloud.str();  //out.write(str1,strlen(str1));


	for (size_t i = 0; i < rototranslations.size(); ++i)  //rototranslations.size ()
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;


		// Print the rotation matrix and translation vector
		//inline Block<Derived, BlockRows, BlockCols> block(Index startRow, Index startCol) 鼠标拉大 从一点啦成方形
/*		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);    //???  每一个模板实例都要输出一个变换矩阵？、 是
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);   //??? 0行 第四列  <>表示 大小 3*3 3*1列向量
	


		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf ("\n");
		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));		*/
	}




	if (rototranslations.size ()==0)
	{
		neigh_sqr_dists_thresh +=0.08;
		k--;
		continue;
	}
	if (rototranslations.size ()>=5)
	{
		neigh_sqr_dists_thresh -=0.05;
		
	}



	pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());

    pcl::transformPointCloud (*model, *rotated_model, rototranslations[0]);

	std::vector<int> indicesnan,indeicesnan2;
	pcl::removeNaNFromPointCloud(*rotated_model,*rotated_model,indicesnan);	//可以本地运算
	pcl::removeNaNFromPointCloud(*scene,*scene,indeicesnan2);

	*scene += *rotated_model;    //所有转到第一src坐标系下 相加
	//保存配准对，转换到第一个点云框架中
	std::stringstream ss;
	ss <<"mali_20151106_"<< k<<"_"<<k+1 << "_.ply";
	pcl::io::savePLYFile(ss.str (), *scene, true);     //两两存本地


	//保存ALL
	//*result_all += *rotated_model;            // 0'，    1'，  2' ,  3'

	all_rototranslations.push_back(GlobalTransform) ;   //单位阵， T1， T1*T2，T1*T2*T3

	pcl::transformPointCloud (*rotated_model, *rotated_model, all_rototranslations[k-1]);      //可以本地运算
	*result_all += *rotated_model; 
	GlobalTransform *= rototranslations[0];              // T1， T1*T2，T1*T2*T3


}

	*result_all += *data[0].cloud; 

	pcl::io::savePLYFile("mali_result_all_20151106_1-16.ply", *result_all, true);     //ALL存本地
	

	//可视化
	pcl::visualization::PCLVisualizer viewer ("SHOT_ICP");
	viewer.addCoordinateSystem();
	viewer.setBackgroundColor(0.3,0.3,0.3);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (result_all, 255, 255, 255);
	viewer.addPointCloud (result_all, rotated_model_color_handler,"all");    

	while (!viewer.wasStopped ()){		
		viewer.spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}





	return (0);
}


/*
	//
	//  Visualization                                                                 结果可视化
	//If key point visualization is enabled, keypoints are displayed as blue dots 
	// and if correspondence visualization has been enabled they are shown as a green line 
	// for each correspondence which survived the clustering process.
	//  关键点 蓝色     对应对 绿线
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Correspondence Grouping") );
	   pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	   viewer.addCoordinateSystem();
	   viewer.setBackgroundColor(0.3,0.3,0.3);
	  // pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_scene(scene);
	   pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler (scene, 255,255,128);
	    viewer.addPointCloud<PointType> (scene,scene_color_handler,"scene_cloud");    //场景加入

       pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
     	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	 //把模型点云 添加窗口内      模型 场景 公用一个视口
	
		//  We are translating the model so that it doesn't end in the middle of the scene representation    模板转移 关键点也转移
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (0,150,0), Eigen::Quaternionf (1, 0, 0, 0));   //沿x轴负方向平移1个单位，
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (0,150,0), Eigen::Quaternionf (1, 0, 0, 0));


		//测试以下两句不同
		//pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128); //模型黄色
		//pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_model(off_scene_model);
		viewer.addPointCloud <PointType>(off_scene_model, "off_scene_model");  // 模板加入
		

         //false                                          显示关键点
		if(false)
		{
			pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
		}

		//viewer.addCorrespondences(scene,off_scene_model,clustered_corrs,"correspondences",0);  直接添加对应对，但不知效果


	//一个变换矩阵对应场景中的一个实例，  一个实例又对应 相应的 对应对                  
	// i是第几个变换矩阵，j是当前实例的对应对
	pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr rotated_model_removenan (new pcl::PointCloud<PointType> ());

	for (size_t i = 0; i < rototranslations.size (); ++i)          // 把模板 根据求出来的变换矩阵 变换到 场景中对象的位置       重叠 红色                         
	{	
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 0, 0, 255);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler,ss_cloud.str ());     //没显示红色  
		if (show_correspondences_)                      //  显示 对应线  绿色 
		{
			for (size_t j = 0; j < clustered_corrs[i].size (); j++)   // 78
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;    // 二元组 作为一个元素 
				PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);  //访问函数不一样
				PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				if( 0 == j%5)
				viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}

*/