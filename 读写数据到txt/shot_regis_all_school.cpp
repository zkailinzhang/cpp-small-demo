// shot_regis_all_school.cpp : �������̨Ӧ�ó������ڵ㡣
//

// shot_regis.cpp : �������̨Ӧ�ó������ڵ㡣


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
float model_ss_ (0.01f);    ///������һ���� ���������ƥ���ʵ���������Ǵ��  �²����İ뾶
float scene_ss_ (0.015f);   //ԭ��0.03 0.02
float rf_rad_ (0.015f);     //��ʾ������֧���� �ռ�뾶
float descr_rad_ (0.03f);   //ԭ��0.02
float cg_size_ (0.01f);     //���� ����ռ䵥Ԫ�� ��С  ����  �ֱ���  ����
float cg_thresh_ (5.0f);    // ������С 5�� ͶƱ ������

float neigh_sqr_dists_thresh (0.3);


/*  debug release �µ��� ��  ��һ��������ģ�壬�ڶ����ǳ�����ģ���ھ�һ�����󣬳����ɶ��
 *   ����ɫ������ɫ���ԣ���  ������������ SHOT  
 ������Ҳ�ǵ������͵�  �ֲ�������Ҳ�ǵ������͵�  ����������׼ȷ   
 ���������ǹؼ���
 �ֲ�������Ҳ���ڹؼ���
 kdtree ƥ���Ӧ�Ե� �����Ӿ��� Find Model-Scene Correspondences with KdTree
 �²��������Ȳ����������Ϊ ���� 	pcl::PointCloud<int> sampled_indices;
 �ӿ� ���� ���� ��id��ǩ������ ����stringstream


 ģ�������һ�����󣬵�������

 //   each instance of the model found into the scene
      the model keypoints descriptor cloud
 
 *	omp ����

������������Ӧ�ԣ� ���ߵ����� ����һά���飿
��һ�� ����ƥ�� ������������  
pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());//  ��Ӧ��ƥ���� ���������Ӿ���ƥ��  ������kdtree flann  
�ڶ�����  �����㷨 ����ؼ���͵�һ��ƥ���  ���  û�����е�  ģ���볡������ȷ�����ƥ��� 
std::vector<pcl::Correspondences> clustered_corrs;  //  �Զ�Ӧ��ΪԪ�ص�����  ���� �ؼ���

������� pcl::copyPointCloud     pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);

 pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
 viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
 viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");


�ؼ�����ʾ ���Թرգ���Ȼȫ����ɫ  ����ģ����ɫ
���棺1.8.0\include\flann\algorithms\kmeans_index.h(469): warning C4291: ��void *operator new(size_t,flann::PooledAllocator &)��: δ�ҵ�ƥ���ɾ��������������ʼ�������쳣���򲻻��ͷ��ڴ�

1788 78 
3324  31 476  �����仯��������
5474 528

 */
 //����任����ģ���еĶ��� instance������ �� ��Ӧ������ 

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





//����������еĽṹ����
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
/**����һ��������Ҫƥ����һ���PCD�ļ�                               pcd    ply  mesh  obj
* ����argc�ǲ��������� (pass from main ())
*���� argv ʵ�ʵ������в��� (pass from main ())
*����models�������ݼ��ĺϳ�ʸ��
*/
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension (".pcd");
	//�ٶ���һ��������ʵ�ʲ���ģ��
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string (argv[i]);
		// ������Ҫ5���ַ�������Ϊ.plot���� 5���ַ���
		if (fname.size () <= extension.size ())                                        //!!!!!
			continue;  //ִ��for 

		std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
		//��������һ��pcd�ļ�
		if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)   //��չ���Ƚ�
		{
			//���ص��Ʋ������������ģ���б���
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile (argv[i], *m.cloud);
			//�ӵ������Ƴ�NAN��
			std::vector<int> indices22;
			pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices22);
			models.push_back (m);
		}
	}
}

/************************************************************************/
/* ˵����The next function performs the spatial resolution computation for a given point cloud 
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
		if (! pcl_isfinite ((*cloud)[i].x))  //��ѯ������ x����
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
	//����3D���ڲ���ӵ������������  
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
	   
	//  vfh�е����ݿ�10  0.05ÿ10������ʾһ����ʸ������5cm�� shot�е����ݿ�ply pcd �� 10��1��
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

	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>);      //�������ɫ�ĵ�������
	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());  //�ؼ���Ҳ�Ǵ���ɫ��
	pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());  //�ؼ���Ҳ�Ǵ���ɫ��
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

	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;   //��������
	loadData (arc, arg, data);

	if (data.empty ()){
		return (-1);
	}
	PCL_INFO ("Loaded %d datasets.", (int)data.size ());


for (size_t k = 1; k < data.size (); ++k)
{

		pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>);      //�������ɫ�ĵ�������
		pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());  //�ؼ���Ҳ�Ǵ���ɫ��
		pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());  //�ؼ���Ҳ�Ǵ���ɫ��
		pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
		pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
		pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
		pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

		scene = data[k-1].cloud;
		model = data[k].cloud;
	    PCL_INFO("  model : %d Points \n",model->points.size());
	    PCL_INFO("  scene : %d Points \n",scene->points.size());

	  if (true)   // �����ֱ���
	  {
		float resolution = static_cast<float> (computeCloudResolution (model));       //��plyת��������pcd �ֱ��ʱ���� 0.103143
		float resolution_scen = static_cast<float> (computeCloudResolution (scene));   //0.105135
		cout<<"���Ʒֱ����ǣ�"<<resolution<<" "<<resolution_scen<<endl;

		if (resolution != 0.0f) {
			model_ss_    = resolution *2.5;    //  ���ؼ�����Ŀ   bunny 2.5
			scene_ss_    = resolution *2.5;     //    6     bunny 2.5 
			rf_rad_      = resolution *10;   
			descr_rad_   = resolution *10;    // bunny 20
			cg_size_     = resolution *5; 
		}
		//std::cout << "Model resolution:       " << resolution << std::endl;  //����Ĭ����������ر�С
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
	  }
	
	//
	//  Compute Normals                               ����������
	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est(4);        //omp����
	//pcl::NormalEstimation<PointType, NormalType> norm_est;            //�ö��Ա����û���أ�����/
	norm_est.setKSearch (15);                                           //13 �кö�nan��
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);
	//pcl::io::savePCDFileASCII("scen0e_norm.pcd",*scene_normals);
	//PCL_INFO("  model norm : %d Points \n",model_normals->points.size());
	//PCL_INFO("  scene norm : %d Points \n",scene_normals->points.size());

	demean_for_show(scene,scene);  //�����Ƶ�����ԭ��  ����������ʾ 
	demean_for_show(model,model);
/*	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_norm;  
	//demean_for_show_xyz(cloud_filter,cloud_filter);
	viewer_norm =  normalsVis(scene, scene_normals);
	while  (!viewer_norm->wasStopped ())   // while һֱ�ж�  if ���ж�һ��
	{
		viewer_norm->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer_norm->close();
	
*/

	//
	//  Downsample Clouds to Extract keypoints        �ؼ��� ���Ȳ���  ����
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
	//  Compute Descriptor for keypoints                ������ shot
	//
	//pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est(4); //OMP  û�б���OpenMP ���������в���������

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


	//  ��Ӧ��ƥ���� ���������Ӿ���ƥ��  ������kdtree 
	//  Find Model-Scene Correspondences with KdTree ģ��Ϊsource������ģ���ڳ���Ŀ�����Ҷ�Ӧ��       ���� ������ �� ��Ӧ�� KdTree flann
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);      //�м� �����ռ�

	//  For each scene key point descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t j = 0; j < scene_descriptors->size (); ++j) //i �� �����ж��ٸ�������
	{
		std::vector<int> neigh_indices (1);            //��Ҫһ�� ��������
		std::vector<float> neigh_sqr_dists (1);

		if (!pcl_isfinite (scene_descriptors->at (j).descriptor[0])) //skipping NaNs  ����һ��Ԫ��!!!!!!??? ����С������if���
		{
			//scene_descriptors->at(i).descriptor[12];  �������������� ��ֵ  i�ǵڼ���������  [x] �����������е�ĳ��ά��ֵ
			//scene_descriptors->at(i).descriptorSize(); //��ǰ�����ӵ�ά��
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (j), 1, neigh_indices, neigh_sqr_dists);// �����ҵ���1 ��֮0 ������k �򷵻�k����		
		
		if(found_neighs == 1 && neigh_sqr_dists[0] < neigh_sqr_dists_thresh) //0.15  0.25  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			//�ڶ������� ��  ������ ������  �������������ؿռ��ѵ�����ڵ��Ǹ��������ֵ
			/**  Index of the matching (target) point. Set to -1 if no correspondence found. */
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (j), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	//�ؼ��� ������ ��Ӧ�� ��ͬ��  1788 ����ȷ��  ����ƥ��� ��ģ������볡����������
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

	//
	//  Actual Clustering                         ����  full Clustering Pipeline:
	//

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;//?????ʲô����   ����Ԫ�� ����Ӧ�ڴ�
	std::vector<pcl::Correspondences> clustered_corrs;  //  �Զ�Ӧ��ΪԪ�ص�����






	//  Using Hough3D           ����3D ����
	if (use_hough_) 
	{
		//
		//  Compute (Key points) Reference Frames only for Hough  ����ؼ��㴦�ľֲ�������
		//
		pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());                //!!!!!!
		pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;  //�˷����� RF���� �Ϻ�  �Ե��Ʊ�Ե�ʹ���ն������ö��Ա����û���أ���
		rf_est.setFindHoles (true);   //���ǿ׶�����
		rf_est.setRadiusSearch (rf_rad_);

		rf_est.setInputCloud (model_keypoints);
		rf_est.setInputNormals (model_normals);
		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (scene_keypoints);
		rf_est.setInputNormals (scene_normals);
		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering   ���ݹؼ���
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;                          //��Ҫ ��������������
		clusterer.setHoughBinSize (cg_size_);  //0.01  ���� 3Dhough�ռ�  ��ά�ģ���
		clusterer.setHoughThreshold (cg_thresh_); // 5
		clusterer.setUseInterpolation (true);     //���ǲ�ֵ   ����ֵ��׼ȷ�ģ�ľ�������ǲ�ֵ����֮���
		clusterer.setUseDistanceWeight (false);   //����Ȩ��

		clusterer.setInputCloud (model_keypoints);   //
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (scene_keypoints);
		clusterer.setSceneRf (scene_rf);

		clusterer.setModelSceneCorrespondences (model_scene_corrs);  // ������ͬ�׶εĶ�Ӧ�� ��ϵ�ڵ�

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);             //����任����ģ���еĶ��� instance������ �� ��Ӧ������ 		
	}
	else // Using GeometricConsistency     �ü���һ���Է���
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
	//  ģ��ʵ�� �ڳ����� �ҵ� 1�� Ҳ����1���任����  ��Ӧ�� �ҵ�1788��  �����ڸ�ʵ������84��         �任���� �����ʾ

	//   each instance of the model found into the scene
	std::cout << "-------��ӡ��"<<k<<"ת��"<<k-1<<"�����µı任����--------"<<std::endl;
	std::cout << "Model instances found: " << rototranslations.size () << std::endl;  //Model instances ??? �ǵ� 1 


	 

	
	 std::stringstream ss_cloud;
	 ss_cloud << "-------��ӡ��"<<k<<"ת��"<<k-1<<"�����µı任����--------"<<"\n"<< "Model instances found: " << rototranslations.size () <<"\n";
	
	 cout<<ss_cloud.str();  //out.write(str1,strlen(str1));


	for (size_t i = 0; i < rototranslations.size(); ++i)  //rototranslations.size ()
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;


		// Print the rotation matrix and translation vector
		//inline Block<Derived, BlockRows, BlockCols> block(Index startRow, Index startCol) ������� ��һ�����ɷ���
/*		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);    //???  ÿһ��ģ��ʵ����Ҫ���һ���任���󣿡� ��
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);   //??? 0�� ������  <>��ʾ ��С 3*3 3*1������
	


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
	pcl::removeNaNFromPointCloud(*rotated_model,*rotated_model,indicesnan);	//���Ա�������
	pcl::removeNaNFromPointCloud(*scene,*scene,indeicesnan2);

	*scene += *rotated_model;    //����ת����һsrc����ϵ�� ���
	//������׼�ԣ�ת������һ�����ƿ����
	std::stringstream ss;
	ss <<"mali_20151106_"<< k<<"_"<<k+1 << "_.ply";
	pcl::io::savePLYFile(ss.str (), *scene, true);     //�����汾��


	//����ALL
	//*result_all += *rotated_model;            // 0'��    1'��  2' ,  3'

	all_rototranslations.push_back(GlobalTransform) ;   //��λ�� T1�� T1*T2��T1*T2*T3

	pcl::transformPointCloud (*rotated_model, *rotated_model, all_rototranslations[k-1]);      //���Ա�������
	*result_all += *rotated_model; 
	GlobalTransform *= rototranslations[0];              // T1�� T1*T2��T1*T2*T3


}

	*result_all += *data[0].cloud; 

	pcl::io::savePLYFile("mali_result_all_20151106_1-16.ply", *result_all, true);     //ALL�汾��
	

	//���ӻ�
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
	//  Visualization                                                                 ������ӻ�
	//If key point visualization is enabled, keypoints are displayed as blue dots 
	// and if correspondence visualization has been enabled they are shown as a green line 
	// for each correspondence which survived the clustering process.
	//  �ؼ��� ��ɫ     ��Ӧ�� ����
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Correspondence Grouping") );
	   pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	   viewer.addCoordinateSystem();
	   viewer.setBackgroundColor(0.3,0.3,0.3);
	  // pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_scene(scene);
	   pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler (scene, 255,255,128);
	    viewer.addPointCloud<PointType> (scene,scene_color_handler,"scene_cloud");    //��������

       pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
     	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	 //��ģ�͵��� ��Ӵ�����      ģ�� ���� ����һ���ӿ�
	
		//  We are translating the model so that it doesn't end in the middle of the scene representation    ģ��ת�� �ؼ���Ҳת��
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (0,150,0), Eigen::Quaternionf (1, 0, 0, 0));   //��x�Ḻ����ƽ��1����λ��
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (0,150,0), Eigen::Quaternionf (1, 0, 0, 0));


		//�����������䲻ͬ
		//pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128); //ģ�ͻ�ɫ
		//pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_model(off_scene_model);
		viewer.addPointCloud <PointType>(off_scene_model, "off_scene_model");  // ģ�����
		

         //false                                          ��ʾ�ؼ���
		if(false)
		{
			pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
		}

		//viewer.addCorrespondences(scene,off_scene_model,clustered_corrs,"correspondences",0);  ֱ����Ӷ�Ӧ�ԣ�����֪Ч��


	//һ���任�����Ӧ�����е�һ��ʵ����  һ��ʵ���ֶ�Ӧ ��Ӧ�� ��Ӧ��                  
	// i�ǵڼ����任����j�ǵ�ǰʵ���Ķ�Ӧ��
	pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr rotated_model_removenan (new pcl::PointCloud<PointType> ());

	for (size_t i = 0; i < rototranslations.size (); ++i)          // ��ģ�� ����������ı任���� �任�� �����ж����λ��       �ص� ��ɫ                         
	{	
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 0, 0, 255);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler,ss_cloud.str ());     //û��ʾ��ɫ  
		if (show_correspondences_)                      //  ��ʾ ��Ӧ��  ��ɫ 
		{
			for (size_t j = 0; j < clustered_corrs[i].size (); j++)   // 78
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;    // ��Ԫ�� ��Ϊһ��Ԫ�� 
				PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);  //���ʺ�����һ��
				PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				if( 0 == j%5)
				viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	}

*/