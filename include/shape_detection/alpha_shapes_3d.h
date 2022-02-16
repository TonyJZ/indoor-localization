#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//weights∫Õcloud∂‘”¶
template <typename PointT>
int /*PCL_EXPORTS*/ alpha_shape_3d(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, bool has_weight,
	std::vector<float> weights, float alphaW = 10000);



#ifdef PCL_NO_PRECOMPILE
#include <shape_detection/impl/alpha_shapes.hpp>
#endif