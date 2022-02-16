#pragma once

#ifdef _WIN32
#ifdef CLUSTERING_EXPORTS
#define  _cluster_Lib_  __declspec(dllexport)
#else
#define  _cluster_Lib_  __declspec(dllimport)
#endif
#else
#  define _cluster_Lib_
#endif

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//indices: indices[0], wall; indices[1], ceiling; indices[2], others
int _cluster_Lib_ cluster_wall_ceiling(pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud, std::vector<std::vector <int> > &indices,
	float ver_DegTh = 15.0, float hor_DegTh = 80.0, float hei_interval = 0.1);


