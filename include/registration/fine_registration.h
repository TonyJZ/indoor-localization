#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>


#ifdef _WIN32
#ifdef REGISTRATION_EXPORTS
#define  _reg_Lib_  __declspec(dllexport)
#else
#define  _reg_Lib_  __declspec(dllimport)
#endif
#else
#  define _reg_Lib_
#endif

//indoor point cloud fine registering (by ICP)
class _reg_Lib_ ipcFineRegistering
{
public:
	typedef pcl::search::Search <pcl::PointXYZRGBNormal> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;


	ipcFineRegistering();
	~ipcFineRegistering();


	void set_ref_pts(const pcl::PointCloud<pcl::PointXYZRGBNormal> *ceiling_cloud, const pcl::PointCloud<pcl::PointXYZRGBNormal> *wall_cloud);
	
	void set_tar_pts(const pcl::PointCloud<pcl::PointXYZRGBNormal> *ceiling_cloud, const pcl::PointCloud<pcl::PointXYZRGBNormal> *wall_cloud);

	void set_init_reg_parameters(const float tar_org[3], const Eigen::Affine3f init_trans);

	void set_search_bufsize(float bsize = 2.0);
	void set_sampling_rate(float rate);

	bool do_ICP(double stddev_disTh);

	Eigen::Affine3f  get_fine_transform() {
		return fine_transform_;
	};

	Eigen::Affine3f get_final_transform();

	void transform_pointcloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &org_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &dst_cloud, 
		const Eigen::Affine3f transfom);

	double get_ICP_score() { return icp_fit_score_; };

protected:

	

private:
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal> > ref_ceiling_cloud_;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal> > ref_wall_cloud_;

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal> > tar_ceiling_cloud_;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal> > tar_wall_cloud_;


	float target_org_[3];		//the origin of the target dataset
	Eigen::Affine3f  init_transform_;  //the initial matching result
	Eigen::Affine3f  fine_transform_;    //the ICP matching result


	float ceiling_lifted_;    //lift the ceiling for ICP

	float search_buf_size_;
	float sampling_rate_;					//sampling rate for target points  which is ranged from 0 to 1
//	KdTreePtr clipped_ref_ceiling_search_;  //kd-tree for clipped ref. ceiling points
//	KdTreePtr clipped_ref_wall_search_;		//kd-tree for clipped ref. wall points

	double icp_fit_score_;
};

