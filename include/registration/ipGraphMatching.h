#pragma once

#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include "shape_detection/structDef.h"
#include "shape_detection/ptSegment.h"

#ifdef _WIN32
#ifdef REGISTRATION_EXPORTS
#define  _reg_Lib_  __declspec(dllexport)
#else
#define  _reg_Lib_  __declspec(dllimport)
#endif
#else
#  define _reg_Lib_
#endif



using namespace std;
class _reg_Lib_ ipGraphMatching
{
public:
//	typedef pcl::PointXYZ pcl::PointXYZ;

	typedef pcl::search::Search <pcl::PointXYZ> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > PointCloudPtr;

	ipGraphMatching(string refname, string targetname);
	ipGraphMatching();
	~ipGraphMatching();

	void set_ipf(string refname, string targetname);
	void set_search_radius(float r);

	double get_match_cost() {
		return cur_match_cost_; 
	};

	Eigen::Affine3f  get_transform() {
		return transform_;
	};

	void get_normalized_org(float org[3]) {
		org[0] = target_ipts_[0].p[0];
		org[1] = target_ipts_[0].p[1];
		org[2] = target_ipts_[0].p[2];
	};

	//匹配前必须先调用
	bool start_match();

	bool match_next();

// 	template <typename PointT> void
// 		transformPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out);

//	bool do_ICP();

protected:
	inline void transform_ipts(vector<iglIntersectionPoint> src_ipts, vector<iglIntersectionPoint> &dst_ipts, Eigen::Affine3f  trans_mat);

private:
	//ipf name
	string ref_ipfname_;
	string target_ipfname_;

	//seg names
	vector<string> ref_segfiles_;
	vector<string> target_segfiles_;

	//ipt
	vector<iglIntersectionPoint> ref_ipts_;
	vector<iglIntersectionPoint> target_ipts_;

	//the principal point of unregistering ipt
	iglIntersectionPoint target_principal_;
	Eigen::Vector2f princ_normal_;


	//segs
	vector<ptSegment<pcl::PointXYZ>> ref_segments_;
	vector<ptSegment<pcl::PointXYZ>> target_segments_;

	//ref candidates
	KdTreePtr search_;  //for all ref ipts
	PointCloudPtr  ipt_cloud_;

	float search_radius_;
	std::vector<int> cand_indices_;

	int cur_matchID_;  //对应的ref ipt ID
	double cur_match_cost_;   //stddev of the distances between corresponding points

	//coarse to fine registration
//	bool bfine_trans;
	Eigen::Affine3f  transform_;  //the intersection matching result
//	Eigen::Affine3f  fine_transform_;    //the icp matching result

//	Eigen::Affine3f icp_transform_;  //fine registering result by ICP
};

//#include <registration/impl/ipGraphMatching.hpp>
