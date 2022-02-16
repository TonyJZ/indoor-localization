#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//weights和cloud对应
//line_segs: 顺序排列的点链
template <typename PointT>
int alpha_shape_2d(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, bool has_weight,
	std::vector<float> &weights,
	pcl::PointCloud<PointT> *line_segs,
	float alpha=-1);



#ifdef PCL_NO_PRECOMPILE
#include <shape_detection/impl/alpha_shapes_2d.hpp>
#endif
