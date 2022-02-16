#ifndef _Geometry_DETECTION_By_JLinkage_H_createdbyTony_2016_June_19_
#define _Geometry_DETECTION_By_JLinkage_H_createdbyTony_2016_June_19_

#include <pcl/point_cloud.h>



enum geoModelType
{
	geoModel_line = 2,
	geoModel_plane = 3
};
//modelType: 2 for line, 3 for plane
template <typename PointT>
int geometry_detect_by_JLinkage(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int>> &clusters, geoModelType modelType,
	float mInlierThreshold, int nDNeigh = 10, float nPClose = 0.8, float nPFar = 0.2, int nSampleModels = 5000);

#ifdef PCL_NO_PRECOMPILE
#include <shape_detection/impl/geometry_detection_by_Jlinkage.hpp>
#endif

#endif