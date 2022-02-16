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


//minModelPts: ��Сģ�͵���
//epsTh: ģ�������ֵ
//epsConnective: ��֮�����ͨ����ֵ
//normal_threshold: ������ֵ
//note����ǰcgal�㷨������ڲ����򣬵���֧��������ʽ����չ�ĵ����ͣ��������ݻش��ķ�ʽ����Ӧԭʼ����. sorted_cloudptr
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
