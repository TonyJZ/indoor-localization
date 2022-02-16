#include "registration/fine_registration.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>


ipcFineRegistering::ipcFineRegistering()
{
	target_org_[0] = target_org_[1] = target_org_[2] = 0.0;
	init_transform_ = Eigen::Matrix4f::Identity();
	fine_transform_ = Eigen::Matrix4f::Identity();

	search_buf_size_ = 0.0;
	sampling_rate_ = 1.0;

	ceiling_lifted_ = 10.0;

	icp_fit_score_ = std::numeric_limits<double>::min();
}

ipcFineRegistering::~ipcFineRegistering()
{
// 	if (clipped_ref_ceiling_search_ != 0)
// 		clipped_ref_ceiling_search_.reset();
// 
// 	if (clipped_ref_wall_search_ != 0)
// 		clipped_ref_wall_search_.reset();

}

void ipcFineRegistering::set_ref_pts(const pcl::PointCloud<pcl::PointXYZRGBNormal> *ceiling_cloud, const pcl::PointCloud<pcl::PointXYZRGBNormal> *wall_cloud)
{
	if (ref_ceiling_cloud_.get() == 0)
		ref_ceiling_cloud_ = boost::shared_ptr<pcl::PointCloud <pcl::PointXYZRGBNormal>>(new pcl::PointCloud <pcl::PointXYZRGBNormal>);
	
	*ref_ceiling_cloud_ = *ceiling_cloud;

	if(ref_wall_cloud_.get() == 0)
		ref_wall_cloud_ = boost::shared_ptr<pcl::PointCloud <pcl::PointXYZRGBNormal>>(new pcl::PointCloud <pcl::PointXYZRGBNormal>);

	*ref_wall_cloud_ = *wall_cloud;

	//lift ceiling
	for (int i = 0; i < ref_ceiling_cloud_->size(); i++)
	{
		ref_ceiling_cloud_->points[i].z += ceiling_lifted_;
	}

}

void ipcFineRegistering::set_tar_pts(const pcl::PointCloud<pcl::PointXYZRGBNormal> *ceiling_cloud, const pcl::PointCloud<pcl::PointXYZRGBNormal> *wall_cloud)
{
	if (tar_ceiling_cloud_.get() == 0)
		tar_ceiling_cloud_ = boost::shared_ptr<pcl::PointCloud <pcl::PointXYZRGBNormal>>(new pcl::PointCloud <pcl::PointXYZRGBNormal>);

	*tar_ceiling_cloud_ = *ceiling_cloud;

	if (tar_wall_cloud_.get() == 0)
		tar_wall_cloud_ = boost::shared_ptr<pcl::PointCloud <pcl::PointXYZRGBNormal>>(new pcl::PointCloud <pcl::PointXYZRGBNormal>);

	*tar_wall_cloud_ = *wall_cloud;

	//lift ceiling
	for (int i = 0; i < tar_ceiling_cloud_->size(); i++)
	{
		tar_ceiling_cloud_->points[i].z += ceiling_lifted_;
	}
}

void ipcFineRegistering::set_init_reg_parameters(const float tar_org[3], const Eigen::Affine3f init_trans)
{
	target_org_[0] = tar_org[0];
	target_org_[1] = tar_org[1];
	target_org_[2] = tar_org[2];

	init_transform_ = init_trans;
}

void ipcFineRegistering::set_search_bufsize(float bsize /* = 2.0 */)
{
	search_buf_size_ = bsize;
}

void ipcFineRegistering::set_sampling_rate(float rate)
{
	if(rate > 0 && rate <= 1.0)
		sampling_rate_ = rate;
}

bool ipcFineRegistering::do_ICP(double stddev_disTh)
{
	Eigen::Vector4f min_pt, max_pt;
	pcl::PointCloud<pcl::PointXYZRGBNormal> trans_ceiling_cloud, trans_wall_cloud;
//	pcl::PointCloud<pcl::PointXYZRGBNormal> clipped_cloud;

	pcl::PointCloud<pcl::PointXYZRGBNormal> src_cloud, dst_cloud;

	///////////////////////////////////////////////////
	//0. clip reference point cloud
	/////////////////////////////////////////////////////
	transform_pointcloud(*tar_ceiling_cloud_, trans_ceiling_cloud, init_transform_);
	pcl::getMinMax3D(trans_ceiling_cloud, min_pt, max_pt);

	min_pt[0] -= search_buf_size_; min_pt[1] -= search_buf_size_; min_pt[2] -= search_buf_size_;
	max_pt[0] += search_buf_size_; max_pt[1] += search_buf_size_; max_pt[2] += search_buf_size_;

	//clip ref. ceiling
	for (int i = 0; i < ref_ceiling_cloud_->size(); i++)
	{
		if (ref_ceiling_cloud_->points[i].x > min_pt[0]
			&& ref_ceiling_cloud_->points[i].y > min_pt[1]
			&& ref_ceiling_cloud_->points[i].x < max_pt[0]
			&& ref_ceiling_cloud_->points[i].y < max_pt[1])
		{
			dst_cloud.points.push_back(ref_ceiling_cloud_->points[i]);
		}
	}


	//clip ref. wall
	transform_pointcloud(*tar_wall_cloud_, trans_wall_cloud, init_transform_);
	pcl::getMinMax3D(trans_wall_cloud, min_pt, max_pt);

	min_pt[0] -= search_buf_size_; min_pt[1] -= search_buf_size_; min_pt[2] -= search_buf_size_;
	max_pt[0] += search_buf_size_; max_pt[1] += search_buf_size_; max_pt[2] += search_buf_size_;

	for (int i = 0; i < ref_wall_cloud_->size(); i++)
	{
		if (ref_wall_cloud_->points[i].x > min_pt[0]
			&& ref_wall_cloud_->points[i].y > min_pt[1]
			&& ref_wall_cloud_->points[i].x < max_pt[0]
			&& ref_wall_cloud_->points[i].y < max_pt[1])
		{
			dst_cloud.points.push_back(ref_wall_cloud_->points[i]);
		}
	}
	dst_cloud.height = 1;
	dst_cloud.width = dst_cloud.size();

	printf("ref. point num: %d\n", dst_cloud.size());


	//////////////////////////////////
	//1. resample the target points
	////////////////////////////////////
	int n_wall = tar_wall_cloud_->size();
	int n_resampling_wall = ceil(n_wall * sampling_rate_);

	//analyse the distribution of normals 
	int norm_quard[4];  
	norm_quard[0] = norm_quard[1] = norm_quard[2] = norm_quard[3] = 0;

	Eigen::Vector2f  directX = Eigen::Vector2f(1, 0);

	float th_pi_4 = cosf(M_PI/4);

	std::vector<std::vector<int>> quard_indices;
	quard_indices.resize(4);
	for (int i = 0; i < trans_wall_cloud.size(); i++)
	{
		Eigen::Map<Eigen::Vector2f> cur_normal(static_cast<float*> (trans_wall_cloud.points[i].normal));

		float c_angle = fabsf(cur_normal.dot(directX));

		if (c_angle > th_pi_4)
		{
			norm_quard[0]++;
			quard_indices[0].push_back(i);
		}
		else if (c_angle > 0)
		{
			norm_quard[1]++;
			quard_indices[1].push_back(i);
		}
		else if (c_angle > -th_pi_4)
		{
			norm_quard[2]++;
			quard_indices[2].push_back(i);
		}
		else if (c_angle > -1)
		{
			norm_quard[3]++;
			quard_indices[3].push_back(i);
		}
	}

	printf("4 Quadrant samples: ");
	for (int i = 0; i < 4; i++)
	{
		if (norm_quard[i] > n_resampling_wall / 4)
			norm_quard[i] = n_resampling_wall / 4;

		printf("%d ", norm_quard[i]);
// 		if (norm_quard[1] > n_resampling_wall / 4)
// 			norm_quard[1] = n_resampling_wall / 4;
// 		if (norm_quard[2] > n_resampling_wall / 4)
// 			norm_quard[2] = n_resampling_wall / 4;
// 		if (norm_quard[3] > n_resampling_wall / 4)
// 			norm_quard[3] = n_resampling_wall / 4;
	}

	printf("\n");
	
	std::vector<bool> sel_flags;
	for (int i = 0; i < 4; i++)
	{
		if(quard_indices[i].size()==0)
			continue;

		sel_flags.resize(quard_indices[i].size(), false);

		int istep = floor(quard_indices[i].size() / norm_quard[i]);
		for (int j = 0; j < quard_indices[i].size(); j += istep)
		{
			int id = quard_indices[i].at(j);
			src_cloud.points.push_back(trans_wall_cloud.points[id]);
		}
	}

	printf("wall resamples: %d\n", src_cloud.size());
	
	int n_ceiling = trans_ceiling_cloud.size();
	int n_resampling_ceiling = ceil(src_cloud.size() / 2);  //wall:ceiling = 2:1
	
	if (n_resampling_ceiling >= n_ceiling)
		n_resampling_ceiling = n_ceiling;

	printf("ceiling samples: %d\n", n_resampling_ceiling);

	int istep = floor(n_ceiling / n_resampling_ceiling);
	for (int i = 0; i < n_ceiling; i += istep)
	{
		src_cloud.points.push_back(trans_ceiling_cloud.points[i]);
	}

	src_cloud.height = 1;
	src_cloud.width = src_cloud.size();

	/////////////////////////////////////
	//2. ICP
	//////////////////////////////////////////
#define Scalar float
#define PointT pcl::PointXYZRGBNormal
	using namespace pcl;
	using namespace pcl::registration;

	TransformationEstimationPointToPlane<PointT, PointT, Scalar>::Ptr te(new TransformationEstimationPointToPlane<PointT, PointT, Scalar>);
	//TransformationEstimationLM<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationLM<PointNormal, PointNormal, Scalar>);
	//TransformationEstimationSVD<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationSVD<PointNormal, PointNormal, Scalar>);
	CorrespondenceEstimation<PointT, PointT, Scalar>::Ptr cens(new CorrespondenceEstimation<PointT, PointT, Scalar>);
	//CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>);
	//CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>);
	cens->setInputSource(src_cloud.makeShared());
	cens->setInputTarget(dst_cloud.makeShared());
	//cens->setSourceNormals (src);

	CorrespondenceRejectorSurfaceNormal::Ptr cor_rej_sn(new CorrespondenceRejectorSurfaceNormal);
	cor_rej_sn->initializeDataContainer<pcl::PointXYZ, pcl::Normal>();
	// 	cor_rej_sn->setInputSource<PointT> (org_cloud.makeShared());
	// 	cor_rej_sn->setInputNormals<PointT, PointT> (org_cloud.makeShared());
	// 	cor_rej_sn->setInputTarget<PointT> (tar_cloud.makeShared());
	// 	cor_rej_sn->setTargetNormals<PointT, PointT> (tar_cloud.makeShared());
	cor_rej_sn->setThreshold(cos(M_PI/12));  //default: 15 degree


	CorrespondenceRejectorMedianDistance::Ptr cor_rej_med(new CorrespondenceRejectorMedianDistance);
	cor_rej_med->setMedianFactor(2.0);
	// 	cor_rej_med->setInputSource<PointT> (org_cloud.makeShared());
	// 	cor_rej_med->setInputTarget<PointT> (tar_cloud.makeShared());

	CorrespondenceRejectorSampleConsensus<PointT>::Ptr cor_rej_sac(new CorrespondenceRejectorSampleConsensus<PointT>);
	// 	cor_rej_sac->setInputSource (org_cloud.makeShared());
	// 	cor_rej_sac->setInputTarget (tar_cloud.makeShared());
	cor_rej_sac->setInlierThreshold(0.005);
	cor_rej_sac->setMaximumIterations(1000);

	CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var(new CorrespondenceRejectorVarTrimmed);
	// 	cor_rej_var->setInputSource<PointT> (org_cloud.makeShared());
	// 	cor_rej_var->setInputTarget<PointT> (tar_cloud.makeShared());

	CorrespondenceRejectorTrimmed::Ptr cor_rej_tri(new CorrespondenceRejectorTrimmed);

	IterativeClosestPointNonLinear<PointT, PointT, Scalar> icp;
	icp.setCorrespondenceEstimation(cens);
	icp.setTransformationEstimation(te);

	icp.addCorrespondenceRejector(cor_rej_sn);
	icp.addCorrespondenceRejector(cor_rej_med);
	icp.addCorrespondenceRejector(cor_rej_var);

	//icp.addCorrespondenceRejector (cor_rej_tri);
	//icp.addCorrespondenceRejector (cor_rej_sac);
	icp.setInputSource(src_cloud.makeShared());
	icp.setInputTarget(dst_cloud.makeShared());

	double search_dis = 1.0;
	icp.setMaxCorrespondenceDistance(search_dis);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations(100);

	icp.getConvergeCriteria()->setAbsoluteMSE(1e-8);

	icp.setTransformationEpsilon(1e-10);
	//	PointCloud<PointNormal> output;

	pcl::PointCloud<pcl::PointXYZRGBNormal> final_cloud;
	icp.align(final_cloud);

	icp_fit_score_ = icp.getFitnessScore(search_dis);

	Eigen::Matrix<Scalar, 4, 4> trans_param = icp.getFinalTransformation();

	fine_transform_.matrix() = trans_param;
	//or fine_transform_ = trans_param;

#undef Scalar

//	return icp.getCorrespondencesPtr();

	if (icp_fit_score_ < stddev_disTh)
		return true;
	else
		return false;
}


void ipcFineRegistering::transform_pointcloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &org_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &dst_cloud,
	const Eigen::Affine3f transfom)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> normalized_cloud;

	pcl::copyPointCloud(org_cloud, normalized_cloud);

	for (int i = 0; i < normalized_cloud.size(); i++)
	{
		normalized_cloud.points[i].data[0] -= target_org_[0];
		normalized_cloud.points[i].data[1] -= target_org_[1];
		normalized_cloud.points[i].data[2] -= target_org_[2];
	}
	//pcl::PointCloud<pcl::PointXYZRGBNormal> trans_cloud;
	pcl::transformPointCloud(normalized_cloud, dst_cloud, transfom);
}

Eigen::Affine3f ipcFineRegistering::get_final_transform()
{
	Eigen::Affine3f final_transform_ = init_transform_*fine_transform_;

	return final_transform_;
}

