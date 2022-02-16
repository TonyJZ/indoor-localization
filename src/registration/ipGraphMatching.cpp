#pragma once

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "pcl/impl/instantiate.hpp"

#include "shape_detection/impl/ptSegment.hpp"
#include "registration/ipGraphMatching.h"
#include "filesupport/AuxFileIO.h"
#include "filesupport/filename_support.h"
#include "geometry/rotateMat.h"

//PCL_INSTANTIATE(ptSegment, pcl::PointXYZ);
template class ptSegment<pcl::PointXYZ>;

ipGraphMatching::ipGraphMatching()
{
	search_radius_ = 5.0;

	cur_matchID_ = -1;
	cur_match_cost_ = std::numeric_limits<double>::max();

//	bfine_trans = false;
	transform_ = Eigen::Matrix4f::Identity();
//	icp_transform_ = Eigen::Matrix4f::Identity();
//	fine_transform_ = Eigen::Matrix4f::Identity();
}

ipGraphMatching::ipGraphMatching(string refname, string targetname)
{
	ref_ipfname_ = refname;
	target_ipfname_ = targetname;

	ipGraphMatching();
}

ipGraphMatching::~ipGraphMatching()
{
	ref_ipts_.clear();
	target_ipts_.clear();

	ref_segfiles_.clear();
	target_segfiles_.clear();

	ref_segments_.clear();
	target_segments_.clear();

	if (search_ != 0)
		search_.reset();

	if (ipt_cloud_ != 0)
		ipt_cloud_.reset();

	cand_indices_.clear();
}

void ipGraphMatching::set_ipf(string refname, string targetname)
{
	ref_ipfname_ = refname;
	target_ipfname_ = targetname;
}

void ipGraphMatching::set_search_radius(float r)
{
	search_radius_ = r;
}

bool ipGraphMatching::start_match()
{
	bool bRefOK, btargetOK;

	//1. load ipt
	bRefOK = Load_ipfFile(ref_ipfname_.c_str(), ref_segfiles_, ref_ipts_);

	btargetOK = Load_ipfFile(target_ipfname_.c_str(), target_segfiles_, target_ipts_);

	if (!bRefOK || !btargetOK)
		return false;

	//2. load segments
	int nRefSegs = ref_segfiles_.size();
	ref_segments_.resize(nRefSegs);

	for (int i = 0; i < nRefSegs; i++)
	{
		//提取分割点云和模型参数
		char rname[128], sname[128];
		GetFileName(ref_segfiles_[i].c_str(), rname, sname);
		string paraname = rname;
		paraname += ".param";

		pcl::ModelCoefficients mCoef;
		sModelType type;
		Load_ParamFile(paraname.c_str(), type, mCoef);
		//		planeCoeffs.push_back(mCoef);

		//提取分割点云
		pcl::PCDReader pcdReader;
		pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud <pcl::PointXYZ>);
		pcdReader.read(ref_segfiles_[i], *seg_cloud);
		
		ptSegment<pcl::PointXYZ> seg;

		seg.setInputCloud(seg_cloud);

		seg.setModelCoef(type, mCoef);

		ref_segments_[i] = seg;
	}

	int ntargetSegs = target_segfiles_.size();
	target_segments_.resize(ntargetSegs);

	for (int i = 0; i < ntargetSegs; i++)
	{
		//提取分割点云和模型参数
		char rname[128], sname[128];
		GetFileName(target_segfiles_[i].c_str(), rname, sname);
		string paraname = rname;
		paraname += ".param";

		pcl::ModelCoefficients mCoef;
		sModelType type;
		Load_ParamFile(paraname.c_str(), type, mCoef);
		//		planeCoeffs.push_back(mCoef);

		//提取分割点云
		pcl::PCDReader pcdReader;
		pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud <pcl::PointXYZ>);
		pcdReader.read(target_segfiles_[i], *seg_cloud);

		ptSegment<pcl::PointXYZ> seg;

		seg.setInputCloud(seg_cloud);

		seg.setModelCoef(type, mCoef);

		target_segments_[i] = seg;
	}

	//3. build ref ipt tree
	if(ipt_cloud_ == 0)
		ipt_cloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);

	int n_refipt = ref_ipts_.size();
	ipt_cloud_->points.resize(n_refipt);
	for (int i = 0; i < n_refipt; i++)
	{
		memcpy(ipt_cloud_->points[i].data, ref_ipts_[i].p, sizeof(float)*4);
	}

	if (search_ == 0)
		search_ = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);

	search_->setInputCloud(ipt_cloud_);

	//4.find the principal point in unregistering ipt
	target_principal_ = target_ipts_[0];
	
	//find the principal edge of target ip
	double maxEdge = std::numeric_limits<double>::lowest();
	int princEdgeID = -1;
	for (int i = 0; i < target_principal_.connIndices.size(); i++)
	{//选择长边作为principal edge
		int edgeId = target_principal_.connIndices[i];

		double edge_len = target_segments_[edgeId].get2DDiagonal();
		if (maxEdge < edge_len)
		{
			maxEdge = edge_len;
			princEdgeID = edgeId;
		}
	}

	//确定principal edge 的方向向量
	pcl::PointXYZ org;
//	Eigen::Vector3f pe_normal;//principal edge's direction
	org.x = target_principal_.p[0];
	org.y = target_principal_.p[1];
	org.z = target_principal_.p[2];

	target_segments_[princEdgeID].getProjectionLineDirection(org, princ_normal_);

	//5. clip candidates
	pcl::PointXYZ searchPoint;
	memcpy(searchPoint.data, target_ipts_[0].p, sizeof(float) * 4);  //the first point is the principal point

	std::vector<float> pointNKNSquaredDistance;
	cand_indices_.clear();
	search_->radiusSearch(searchPoint, search_radius_, cand_indices_, pointNKNSquaredDistance);
	
	cur_matchID_ = -1;
	cur_match_cost_ = std::numeric_limits<double>::max();

	transform_ = Eigen::Matrix4f::Identity();
//	icp_transform_ = Eigen::Matrix4f::Identity();

	return true;
}

bool ipGraphMatching::match_next()
{
	if (cur_matchID_ < -1 || cur_matchID_ >= static_cast<int> (cand_indices_.size()-1))
		return false;

	cur_matchID_++;
	
	iglIntersectionPoint   cand_ip;
	cand_ip = ref_ipts_[cur_matchID_];

	float xOffset, yOffset, zOffset;
	xOffset = cand_ip.p[0];
	yOffset = cand_ip.p[1];
	zOffset = cand_ip.p[2];

	
	cur_match_cost_ = std::numeric_limits<double>::max();
	double mCur_Cost;
	pcl::PointXYZ org;
	org.x = cand_ip.p[0]; org.y = cand_ip.p[1]; org.z = cand_ip.p[2];
	for (int i = 0; i < cand_ip.connIndices.size(); i++)
	{//和每一条edge匹配，并计算匹配代价，将匹配代价最小最为当前点的最佳匹配
		int edgeId = cand_ip.connIndices[i];

		//计算当前参考边的方向
		Eigen::Vector2f cur_eNormal;//current edge's direction
		ref_segments_[edgeId].getProjectionLineDirection(org, cur_eNormal);
		
		//计算绕Z轴的旋转角
		double kappa = get2DRotateAngle(princ_normal_, cur_eNormal);

		//构造转换矩阵
		Eigen::Affine3f  cur_transform = Eigen::Affine3f::Identity();
//		Eigen::Matrix3d  rotate_mat;

// 		RotateMat_Z(kappa, rotate_mat);
// 
// 		cur_transform(0, 0) = rotate_mat(0, 0);
// 		cur_transform(0, 1) = rotate_mat(0, 1);
// 		cur_transform(0, 2) = rotate_mat(0, 2);
// 		cur_transform(1, 0) = rotate_mat(1, 0);
// 		cur_transform(1, 1) = rotate_mat(1, 1);
// 		cur_transform(1, 2) = rotate_mat(1, 2);
// 		cur_transform(2, 0) = rotate_mat(2, 0);
// 		cur_transform(2, 1) = rotate_mat(2, 1);
// 		cur_transform(2, 2) = rotate_mat(2, 2);
// 
// 		cur_transform(0, 3) = xOffset;
// 		cur_transform(1, 3) = yOffset;
// 		cur_transform(2, 3) = zOffset;

//		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		cur_transform.translation() << xOffset, yOffset, zOffset;
		cur_transform.rotate(Eigen::AngleAxisf(kappa, Eigen::Vector3f::UnitZ()));

		vector<iglIntersectionPoint> trans_ipts;
		transform_ipts(target_ipts_, trans_ipts, cur_transform);

		//计算当前转换矩阵下的匹配代价
		mCur_Cost = 0;
		for (int j = 0; j < trans_ipts.size(); j++)
		{
			pcl::PointXYZ searchPoint;
			memcpy(searchPoint.data, trans_ipts[j].p, sizeof(float) * 4);  //the first point is the principal point

			std::vector<int> k_indices;
			std::vector<float> k_sqr_distances;
			search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

			mCur_Cost += k_sqr_distances[0];
		}
		mCur_Cost /= trans_ipts.size();
		mCur_Cost = sqrt(mCur_Cost);

		if (cur_match_cost_ > mCur_Cost)
		{
			cur_match_cost_ = mCur_Cost;
			transform_ = cur_transform;
		}
	}

	return true;
}

inline void ipGraphMatching::transform_ipts(vector<iglIntersectionPoint> src_ipts,
	vector<iglIntersectionPoint> &dst_ipts, Eigen::Affine3f trans_mat)
{
	pcl::PointCloud<pcl::PointXYZ> input, output;

	input.points.resize(src_ipts.size());
	input.width = input.size();
	input.height = 1;
	for (int i = 0; i < src_ipts.size(); i++)
	{
		input.points[i].x = src_ipts[i].p[0] - src_ipts[0].p[0];
		input.points[i].y = src_ipts[i].p[1] - src_ipts[0].p[1];
		input.points[i].z = src_ipts[i].p[2] - src_ipts[0].p[2];
	}

	pcl::transformPointCloud(input, output, trans_mat);

	dst_ipts = src_ipts;
	for (int i = 0; i < dst_ipts.size(); i++)
	{
		dst_ipts[i].p[0] = output.points[i].x;
		dst_ipts[i].p[1] = output.points[i].y;
		dst_ipts[i].p[2] = output.points[i].z;
	}
}

// template <typename PointT> void
// ipGraphMatching::transformPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
// {
// 	for (int i = 0; i < cloud_in.size(); i++)
// 	{
// 		cloud_in.points[i].data[0] -= target_ipts_[0].p[0];
// 		cloud_in.points[i].data[1] -= target_ipts_[0].p[1];
// 		cloud_in.points[i].data[2] -= target_ipts_[0].p[2];
// 	}
// 
// 	pcl::transformPointCloud(cloud_in, cloud_out, transform_);
// }




