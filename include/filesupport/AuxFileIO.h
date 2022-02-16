#pragma once


#ifdef _WIN32
#ifdef FILESUPPORT_EXPORTS
#define  _fs_Lib_  __declspec(dllexport)
#else
#define  _fs_Lib_  __declspec(dllimport)
#endif

#ifndef FILESUPPORT_EXPORTS
#ifdef _DEBUG
#pragma comment(lib,"filesupportD.lib")
#else
#pragma comment(lib,"filesupport.lib")
#endif
#endif
#else
#  define _fs_Lib_
#endif



#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>

#include "shape_detection/structDef.h"


//*.param
bool _fs_Lib_ Load_ParamFile(const char* filename, sModelType &mType, pcl::ModelCoefficients &mCoef);
bool _fs_Lib_ Save_ParamFile(const char* filename, sModelType mType, pcl::ModelCoefficients mCoef);

//*.ipf
bool _fs_Lib_ Load_ipfFile(const char* filename, std::vector<std::string> &pointnames, std::vector<iglIntersectionPoint> &ipts);
bool _fs_Lib_ Save_ipfFile(const char* filename, std::vector<std::string> pointnames, std::vector<iglIntersectionPoint> ipts);

//trans.txt
bool _fs_Lib_ Load_TransParameFile(const char* filename, float orgin[3], Eigen::Matrix4f &trans_param);
bool _fs_Lib_ Save_TransParameFile(const char* filename, float orgin[3], Eigen::Matrix4f trans_param);

//bundle.out
bool _fs_Lib_ Load_bundleFile(const char* filename, std::vector<simpleCameraModel> &cams, std::vector<tiePoint> &tps);


