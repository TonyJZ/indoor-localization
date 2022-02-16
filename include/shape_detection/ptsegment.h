#ifndef _pointcloud_segment_define_h_createdbyTony_2017_June_27_
#define _pointcloud_segment_define_h_createdbyTony_2017_June_27_

#include <pcl/pcl_macros.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>

#include <vector>

#include "shape_detection/structDef.h"


template <typename PointT>
class ptSegment : public pcl::PCLBase<PointT>
{
public:
	typedef pcl::PointCloud< PointT > PointCloud;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;	typedef typename PointCloud::Ptr PointCloudPtr;

	using pcl::PCLBase<PointT>::initCompute;
	using pcl::PCLBase<PointT>::deinitCompute;
	using pcl::PCLBase<PointT>::indices_;
//	using pcl::PCLBase<PointT>::input_;

	typedef pcl::search::Search <PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;

public:
	ptSegment();
	~ptSegment();

	virtual void
		setInputCloud(const PointCloudConstPtr &cloud);

	void setModelCoef(sModelType type, pcl::ModelCoefficients coef);

	const sModelType getModelType()
	{
		return mt_;
	};

	const pcl::ModelCoefficients getModelCoef()
	{
		return coef_;
	};

	void getBBox(Eigen::Vector3f &bbmin, Eigen::Vector3f &bbmax);

	double get3DDiagonal();

	double get2DDiagonal();

	double getFurthestPointsByPCL(PointT &pmin, PointT &pmax);

	double getFurthestPointsApproximate(PointT &pmin, PointT &pmax);

	//nsample: 随机采样点数
	double getProjectionDensity(int nsample, double dc); 

	//计算联通度，分析segment的空间分布情况
	inline double get2DConnectivity(float vSize, double &maxConnDis);

	//提取投影射线的方向向量, orgPt为给定射线原点
	void getProjectionLineDirection(PointT orgPt, Eigen::Vector2f &normal);

protected:
	PointCloudPtr input_;


private:
	KdTreePtr search_;

	sModelType mt_;
	pcl::ModelCoefficients coef_;  

	bool bFindFurthestPair;
	PointT ptmin_, ptmax_;
	double maxDis_;

	//feaures
	bool bFindbbox;
	std::vector<double> bbox_seg_; //xmin,ymin,zmin,xmax,ymax,zmax
	Eigen::Vector3f low_pt_, high_pt_;//, lb_pt_, rt_pt_; //low, high, leftbottom, righttop 
};

#ifdef PCL_NO_PRECOMPILE
#include <shape_detection/impl/ptSegment.hpp>
#endif



#endif
