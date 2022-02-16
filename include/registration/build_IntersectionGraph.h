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

int _reg_Lib_ detect_intersection_points(char *wall_dir, char *file_ext, std::vector<std::string> &filenames, 
	std::vector<ptSegment<pcl::PointXYZRGBNormal>> &segments,
	std::vector<iglIntersectionPoint> &ipts);


