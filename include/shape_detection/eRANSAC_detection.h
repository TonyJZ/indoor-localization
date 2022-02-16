#ifndef _Geometric_Shape_Detection_Using_EfficientRANSAC_H_createdbyTony_2017_June_16_
#define _Geometric_Shape_Detection_Using_EfficientRANSAC_H_createdbyTony_2017_June_16_



#ifdef _WIN32
#ifdef SHAPE_DETECTION_EXPORTS
#define  _sd_Lib_  __declspec(dllexport)
#else
#define  _sd_Lib_  __declspec(dllimport)
#endif
#else
#  define _sd_Lib_
#endif


//#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>


//minModelPts: 最小模型点数
//epsTh: 模型拟合阈值
//epsConnective: 点之间的连通性阈值
//normal_threshold: 法向阈值
//note：当前cgal算法会进行内部排序，但不支持索引方式和扩展的点类型，采用数据回传的方式来对应原始数据. sorted_cloudptr
//template <typename PCLPointT>
int _sd_Lib_ plane_detection_CGAL(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, std::vector<int> *indices, bool has_normal,
	pcl::PointCloud<pcl::PointXYZRGBNormal> *sorted_cloudptr,
	std::vector<std::vector<int> > &inliers,
	std::vector<pcl::ModelCoefficients> &planeCoeffs,
	int minModelPts = 200, double epsTh = 0.05, double epsConnective = 0.2, double normalTh = 0.9);



// #ifdef PCL_NO_PRECOMPILE
// #include <shape_detection/impl/eRANSAC_detection.hpp>
// #end

#endif
