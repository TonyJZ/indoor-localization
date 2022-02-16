#include "clustering/cluster_wall_ceiling.h"

#include <pcl/common/angles.h>

int cluster_wall_ceiling(pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud, std::vector<std::vector<int> > &indices,
	float ver_DegTh /* = 15.0 */, float hor_DegTh /* = 80.0 */, float hei_interval /* = 0.1 */)
{
	//按法方向分类，提取顶面和立面

	std::vector <int> idx_hor, idx_ver;  //水平面, 垂直面
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

//	float vDegThreshold_ = 15.0, fDegThreshold_ = 80.0;
	float threshold_r = cosf(pcl::deg2rad(ver_DegTh));
	float threshold_f = cosf(pcl::deg2rad(hor_DegTh));

	double minr_h, maxr_h;
	minr_h = std::numeric_limits<double>::max();
	maxr_h = std::numeric_limits<double>::lowest();

	for (int i = 0; i < cloud->size(); ++i)
	{
		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (cloud->points[i].normal));
		float dot_product = fabsf(nghbr_normal.dot(nVertical));
		if (dot_product > threshold_r)
		{
			idx_hor.push_back(i);

			if (cloud->points[i].z > maxr_h)
				maxr_h = cloud->points[i].z;
			if (cloud->points[i].z < minr_h)
				minr_h = cloud->points[i].z;
		}
		else if (dot_product < threshold_f)
		{
			idx_ver.push_back(i);
		}
	}

// 	int nstep = 30;
// 	double interval = (maxr_h - minr_h) / nstep;

	minr_h = minr_h - 0.5*hei_interval;
	int nstep = ceil((maxr_h - minr_h) / hei_interval);

	std::vector<int> zHist;
	zHist.resize(nstep + 2, 0);

	for (int i = 0; i < idx_hor.size(); i++)
	{
		int id = idx_hor[i];
		double z = cloud->points[id].z;

		int iStep = static_cast<int> (floor((z - minr_h) / hei_interval));
		zHist[iStep]++;
	}

	int imax, maxPts = 0;
	for (int i = 0; i < zHist.size(); i++)
	{
		if (zHist[i] > maxPts)
		{
			maxPts = zHist[i];
			imax = i;
		}
	}

	double sec_floor, sec_ceil; //屋顶的高度区间

	double roof_meanZ = minr_h + imax*hei_interval + 0.5*hei_interval;
	double buf_size = 0.1;  //0.1m
	sec_floor = roof_meanZ - buf_size;
	sec_ceil = roof_meanZ + buf_size;

	std::vector <int> idx_ceiling, idx_unclassified; //屋顶面索引
	for (int i = 0; i < idx_hor.size(); i++)
	{
		int id = idx_hor[i];
		double z = cloud->points[id].z;

		if (z > sec_floor && z < sec_ceil)
			idx_ceiling.push_back(id);
		else
			idx_unclassified.push_back(id);
	}

	indices.push_back(idx_ver);
	indices.push_back(idx_ceiling);
	indices.push_back(idx_unclassified);

	return (1);
}

