#ifndef _PointCloud_Density_Estimation_H_CreatedByTony_2017_Jun_10_
#define _PointCloud_Density_Estimation_H_CreatedByTony_2017_Jun_10_

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

//cutoff kernel
template <typename PointT>
int getLocalDensity_cutoff(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, double dc,
std::vector<double> &rhos)
{
	int nSamples;
	
	if (indices == NULL)
		nSamples = cloud.size();
	else
		nSamples = indices->size();

	rhos.resize(nSamples, 0.0);

	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	//const boost::shared_ptr<const pcl::PointCloud<PointT>>  shptr(&cloud);
	boost::shared_ptr <std::vector<int> > indptr(new std::vector<int>);

	if (indices)
	{
		indptr->assign(indices->begin(), indices->end());
		kdtree->setInputCloud(cloud.makeShared(), indptr);
	}
	else
		kdtree->setInputCloud(cloud.makeShared());

	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;

	for (int i = 0; i<nSamples; i++)
	{
		int id;
		if (indices)
			id = indices->at(i);
		else
			id = i;

		PointT pt = cloud.points[id];

		kdtree->radiusSearch(pt, dc, k_indices, k_sqr_distances);

		int nn = k_indices.size() - 1; //去掉查找点本身
	
		assert(nn >= 0);

		rhos[i] += nn;
	}

	return 0;
}

//gaussian kernel
template <typename PointT>
int getLocalDensity_gaussian(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, double dc,
	std::vector<double> &rhos)
{
	int nSamples;

	if (indices == NULL)
		nSamples = cloud.size();
	else
		nSamples = indices->size();

	rhos.resize(nSamples, 0.0);

	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	boost::shared_ptr <std::vector<int> > indptr(new std::vector<int>);

	if (indices)
	{
		indptr->assign(indices->begin(), indices->end());
		kdtree->setInputCloud(cloud.makeShared(), indptr);
	}
	else
		kdtree->setInputCloud(cloud.makeShared());


	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;
	double dc2 = dc*dc;

	for (int i = 0; i<nSamples; i++)
	{
		int id;
		if (indices)
			id = indices->at(i);
		else
			id = i;

		PointT pt = cloud.points[id];

		kdtree->radiusSearch(pt, dc, k_indices, k_sqr_distances);

		for (int j=1; j < k_indices.size(); ++j)
		{
			double dis_ij2 = k_sqr_distances[j];

			//gaussian kernel
			rhos[i] += exp(-dis_ij2 / dc2);
		}
	}

	return 0;
}

// http://eric-yuan.me/clustering-fast-search-find-density-peaks/
template <typename PointT>
int getLocalDensity_AverageKnnDistance_Ryan(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, double dc, 
	std::vector<double> &rhos)
{
	//M = ratio*N;
	double ratio = 0.015;  //经验参数

	int nSamples;

	if (indices == NULL)
		nSamples = cloud.points.size();
	else
		nSamples = indices->size();

	int M = nSamples * ratio;

	rhos.resize(nSamples, 0.0);

	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	boost::shared_ptr <std::vector<int> > indptr(new std::vector<int>);

	if (indices)
	{
		indptr->assign(indices->begin(), indices->end());
		kdtree->setInputCloud(cloud.makeShared(), indptr);
	}
	else
		kdtree->setInputCloud(cloud.makeShared());


	for (int i = 0; i<nSamples; i++)
	{
		int id;
		if (indices)
			id = indices->at(i);
		else
			id = i;

		PointT pt = cloud.points[id];

		std::vector<int> NN_Idx;
		std::vector<float> NN_Distance;
		double sum_dis = 0;

		kdtree.nearestKSearch(pt, M, NN_Idx, NN_Distance);

		for (int j = 1; i < NN_Idx.size(); ++j)
		{
			sum_dis += sqrt(NN_Distance[j]);
		}

		if (NN_Idx.size() > 1)
			rhos[i] = sum_dis / (NN_Idx.size() - 1);
	}

	return 0;
}




#endif
