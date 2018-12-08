// shot_regis.cpp : �������̨Ӧ�ó������ڵ㡣
//

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
#include <pcl/registration/icp_nl.h>


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

float neigh_sqr_dists_thresh (0.1); //0.3  ����ƽ��


int icp_max_iter_ (30);
float icp_corr_distance_ (0.25f);  //    �����Ǿ���ƽ��

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
//
void demean_for_show(const pcl::PointCloud<PointType>::Ptr &src,const pcl::PointCloud<PointType>::Ptr &dst )
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*src,centroid);
	pcl::demeanPointCloud<PointType>(*src,centroid, *dst);
}

//����shot���������Ӧ�Բ����
void comCorrespondences_base_SHOT(const PointCloud::Ptr &scene, const PointCloud::Ptr &model,pcl::CorrespondencesPtr model_scene_corrs,
								  std::vector<pcl::Correspondences>& clustered_corrs,
								  vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations)
{
	pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());  //�ؼ���Ҳ�Ǵ���ɫ��
	pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());  //�ؼ���Ҳ�Ǵ���ɫ��
	pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	//
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est(4);        //omp����
	//pcl::NormalEstimation<PointType, NormalType> norm_est;            //�ö��Ա����û���أ�����/
	norm_est.setKSearch (15);                                           //13 �кö�nan��
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);

	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

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
	viewer_norm->close();*/


	//  Downsample Clouds to Extract keypoints        �ؼ��� ���Ȳ���  ����
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

	
	//  Compute Descriptor for keypoints                ������ shot	
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
	//pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);      //�м� �����ռ�
	//  For each scene key point descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.

	bool  bIterate =true;
	while(bIterate ){
	for (size_t j = 0; j < scene_descriptors->size (); ++j) //i �� �����ж��ٸ�������
	{
		std::vector<int> neigh_indices (1);            //��Ҫһ�� ��������
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (scene_descriptors->at (j).descriptor[0])) //skipping NaNs  ����һ��Ԫ��!!!!!!??? ����С������if���
		{
			//scene_descriptors->at(i).descriptor[12];  �������������� ��ֵ  i�ǵڼ���������  [x] �����������е�ĳ��ά��ֵ
			//scene_descriptors->at(i).descriptorSize(); //��ǰ�����ӵ�ά��
			continue;  //������for
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

	//  Actual Clustering                         ����  full Clustering Pipeline:
	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;//?????ʲô����   ����Ԫ�� ����Ӧ�ڴ�
	//std::vector<pcl::Correspondences> clustered_corrs;  //  �Զ�Ӧ��ΪԪ�ص�����

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

	if (rototranslations.size ()==0){
		neigh_sqr_dists_thresh +=0.08;
		bIterate =true;
		continue; //�Ƿ���while����
	}
	bIterate =false;
	if (rototranslations.size ()>=5){
		neigh_sqr_dists_thresh -=0.05;

	}


	}

}

//������ʼ��׼
void CouarseAlignPair (const PointCloud::Ptr scene, const PointCloud::Ptr model, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{




}
//��������׼  Դ���Ƶ�Ŀ�����   ICPtransformation *  rotated_model  = scene
void LMICP(const PointCloud::Ptr &scene, const PointCloud::Ptr &rotated_model, PointCloud::Ptr &icp_model,Eigen::Matrix4f &ICPtransformation,string filename,int k )
{	
	cout <<"ICP����׼��ı任����   "<<k<<"ת��"<<k-1<<"\n"<< "--- ICP ---------" << endl;
	pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;
	icp.setMaximumIterations (icp_max_iter_);
	icp.setMaxCorrespondenceDistance (icp_corr_distance_);
	icp.setInputTarget (scene);
	icp.setInputSource (rotated_model);
	pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
	icp.align (*registered);   //����Ŀ����� 

	
	 ICPtransformation = icp.getFinalTransformation (); //������

	 cout<<icp.getFitnessScore() <<endl;

	if (icp.hasConverged ())      {
		cout << "Aligned!" << endl;
	}
	else      {
		cout << "Not Aligned!" << endl;
	}

	*icp_model = *registered;


	ofstream fout(filename,ios::out|ios::app);
	fout<<"\n"<<"ICP����׼��ı任���� "<<k<<"ת��"<<k-1<<"\n";
	fout<<"  ��ת����R��"<<"\n";
	fout<<"    "<<setw(8)<< ICPtransformation (0,0)<<"  "<<ICPtransformation (0,1)<<"  "<<ICPtransformation (0,2)<<"\n";        //setprecision(2)����С����λ�������������    fixed ��?
	fout<<"    "<<setw(8)<< ICPtransformation (1,0)<<"  "<<ICPtransformation (1,1)<<"  "<<ICPtransformation (1,2)<<"\n"; 
	fout<<"    "<<setw(8)<< ICPtransformation (2,0)<<"  "<<ICPtransformation (2,1)<<"  "<<ICPtransformation (2,2)<<"\n"; 
	fout<<"  ƽ������T��";
	fout<<"    "<< ICPtransformation (3,0)<<"  "<<ICPtransformation (3,1)<<"  "<<ICPtransformation (3,2)<<"\n";
	fout <<"  ������"<<icp.getFitnessScore()<<"\n";

	fout.close();
}


void showTwo(string filename,int k,const PointCloud::Ptr &scene, const PointCloud::Ptr &model,vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,std::vector<pcl::Correspondences> &clustered_corrs,bool show)
{
	ofstream fout(filename,ios::out|ios::app);

	std::stringstream ss_cloud;
	ss_cloud << "-------��ӡ��"<<k<<"ת��"<<k-1<<"�����µı任����--------"<<"\n"<< "  Model instances found: " << rototranslations.size () <<"\n";
	fout<<ss_cloud.str(); 
	fout<<"��С������ֵ�ǣ�"<<neigh_sqr_dists_thresh<<"\n";
	for (size_t i = 0; i < rototranslations.size(); ++i)  //rototranslations.size ()
	{
		fout << " Instance " << i + 1 << ":" <<"\n";
		fout << "  Correspondences belonging to this instance: " << clustered_corrs[i].size () <<"\n";

		// Print the rotation matrix and translation vector
		//inline Block<Derived, BlockRows, BlockCols> block(Index startRow, Index startCol) ������� ��һ�����ɷ���
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);    //???  ÿһ��ģ��ʵ����Ҫ���һ���任���󣿡� ��
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);   //??? 0�� ������  <>��ʾ ��С 3*3 3*1������

		fout<<"  ��ת����R��"<<"\n";
		fout<<"    "<<setw(8)<< rotation (0,0)<<"  "<<rotation (0,1)<<"  "<<rotation (0,2)<<"\n";        //setprecision(2)����С����λ�������������    fixed ��?
		fout<<"    "<<setw(8)<< rotation (1,0)<<"  "<<rotation (1,1)<<"  "<<rotation (1,2)<<"\n"; 
		fout<<"    "<<setw(8)<< rotation (2,0)<<"  "<<rotation (2,1)<<"  "<<rotation (2,2)<<"\n"; 
		fout<<"  ƽ������T��";
		fout<<"    "<< translation (3,0)<<"  "<<translation (3,1)<<"  "<<translation (3,2)<<"\n";
	}
	fout.close();
	
	show =false;
	if (show)
	{
		pcl::PointCloud<PointType>::Ptr rotated_modelxx (new pcl::PointCloud<PointType> ());
		pcl::visualization::PCLVisualizer viewerxx ("SHOT_ICPxx");
		viewerxx.setBackgroundColor(0.3,0.3,0.3);
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler (scene, 255,0,0);
		viewerxx.addPointCloud<PointType> (scene,scene_color_handler,"scene_cloud"); 
		pcl::transformPointCloud (*model, *rotated_modelxx, rototranslations[0]);
		std::stringstream ss_cloud;
		ss_cloud << "instance" ;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_modelxx, 255, 255, 255);
		viewerxx.addPointCloud (rotated_modelxx, rotated_model_color_handler,ss_cloud.str ());     //û��ʾ��ɫ  

		while (!viewerxx.wasStopped ()){		
			viewerxx.spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}




}


int _tmain(int argc, _TCHAR* argv[])
{
	pcl::PointCloud<PointType>::Ptr result_all (new pcl::PointCloud<PointType>);  
	pcl::PointCloud<PointType>::Ptr result_two (new pcl::PointCloud<PointType>); 
		 PointCloud::Ptr icp_model(new pcl::PointCloud<PointType>);

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > all_rototranslations;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();

	pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>);      //�������ɫ�ĵ�������

	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

	char* arg[] = {"","chef_view1-22_ply\\chef_view1.pcd","chef_view1-22_ply\\chef_view2.pcd",
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
		"chef_view1-22_ply\\chef_view15.pcd"};
/*		"chef_view1-22_ply\\chef_view16.pcd"
		"chef_view1-22_ply\\chef_view17.pcd",
		"chef_view1-22_ply\\chef_view18.pcd",
		"chef_view1-22_ply\\chef_view19.pcd",
		"chef_view1-22_ply\\chef_view20.pcd",
		"chef_view1-22_ply\\chef_view21.pcd",
		"chef_view1-22_ply\\chef_view22.pcd"};  */

	int arc= 16;

	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;   //��������
	loadData (arc, arg, data);
	if (data.empty ()){return (-1);}
	PCL_INFO ("Loaded %d datasets.", (int)data.size ());
	string file ="mat.txt";
for (size_t k = 1; k < data.size (); ++k)
{

		pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>);      //�������ɫ�ĵ�������
		pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType>);


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


	//  ��Ӧ��ƥ���� ���������Ӿ���ƥ��  ������kdtree 
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	comCorrespondences_base_SHOT(scene, model,model_scene_corrs,clustered_corrs,rototranslations);

	//showTwo(file,k,scene, model,rototranslations,clustered_corrs,false);

	pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[0]);

	Eigen::Matrix4f ICPtransformation = Eigen::Matrix4f::Identity ();

	LMICP(scene, rotated_model,icp_model,ICPtransformation,file,k);
	



	std::vector<int> indicesnan,indeicesnan2;
	pcl::removeNaNFromPointCloud(*icp_model,*icp_model,indicesnan);	//���Ա�������
	pcl::removeNaNFromPointCloud(*scene,*scene,indeicesnan2);

/*	*scene += *icp_model;    //����ת����һsrc����ϵ�� ���
	//������׼�ԣ�ת������һ�����ƿ����
	std::stringstream ss;
	ss <<"mali_20151123_"<< k<<"_"<<k+1 << "_.ply";
	pcl::io::savePLYFile(ss.str (), *scene, true);     //�����汾��
*/

	//����ALL
	//*result_all += *rotated_model;            // 0'��    1'��  2' ,  3'

	all_rototranslations.push_back(GlobalTransform) ;   //��λ�� T1�� T1*T2��T1*T2*T3

	pcl::transformPointCloud (*icp_model, *icp_model, all_rototranslations[k-1]);      //���Ա�������
	*result_all += *icp_model; 
	GlobalTransform *= ICPtransformation;              // T1�� T1*T2��T1*T2*T3


}

	*result_all += *data[0].cloud; 

	pcl::io::savePLYFileASCII("mali_result_all_20151123_1-3.ply", *result_all);     //ALL�汾��
	

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
