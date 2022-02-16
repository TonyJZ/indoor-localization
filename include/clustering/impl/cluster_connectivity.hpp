#pragma once

#include "clustering/cluster_connectivity.h"

using namespace igl;

namespace pcl
{

	namespace octree
	{
		//unordered_map中自定义类型的hash函数，必须自己实现
		static size_t hash_value(const OctreeKey& b)
		{
			return boost::hash_value(b.key_);
		}

	}

}

template <typename PointT>
ClusterConnectivity<PointT>::ClusterConnectivity()
{
	pID_cID_table_ = nullptr;

	voxel_size_ = 0.0f;
	vNumX_ = vNumY_ = vNumZ_ = 0;
	
	cluster_amount_ = 0;
}

template <typename PointT>
ClusterConnectivity<PointT>::~ClusterConnectivity()
{
	if (adj_graph_ != 0)
		adj_graph_.reset();
}

template <typename PointT> void
ClusterConnectivity<PointT>::setPointClusterLut(std::vector<int>* p2c_Lut)
{
	pID_cID_table_ = p2c_Lut;
	//cluster ID: from 0 to n, consecutive numbering

	int ptNum = p2c_Lut->size();
	cluster_amount_ = p2c_Lut->at(ptNum-1) + 1;
}

template <typename PointT> void
ClusterConnectivity<PointT>::getBBox(Eigen::Vector3f &bbmin, Eigen::Vector3f &bbmax)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return;
	}

	std::vector<double> bbox_seg_;
	bbox_seg_.resize(6, 0);
	bbox_seg_[0] = bbox_seg_[1] = bbox_seg_[2] = std::numeric_limits<float>::max();
	bbox_seg_[3] = bbox_seg_[4] = bbox_seg_[5] = std::numeric_limits<float>::lowest();

	int nSample;
	if (indices_.get() == 0)
		nSample = input_->size();
	else
		nSample = indices_->size();

	for (int i = 0; i < nSample; i++)
	{
		int id = i;
		if (indices_.get())
			id = indices_->at(i);

		PointT pt = input_->points[id];
		if (pt.x < bbox_seg_[0])
			bbox_seg_[0] = pt.x;
		if (pt.x > bbox_seg_[3])
			bbox_seg_[3] = pt.x;

		if (pt.y < bbox_seg_[1])
			bbox_seg_[1] = pt.y;
		if (pt.y > bbox_seg_[4])
			bbox_seg_[4] = pt.y;

		if (pt.z < bbox_seg_[2])
		{
			bbox_seg_[2] = pt.z;
		}
		if (pt.z > bbox_seg_[5])
		{
			bbox_seg_[5] = pt.z;
		}
	}

	bbmin[0] = bbox_seg_[0];
	bbmin[1] = bbox_seg_[1];
	bbmin[2] = bbox_seg_[2];

	bbmax[0] = bbox_seg_[3];
	bbmax[1] = bbox_seg_[4];
	bbmax[2] = bbox_seg_[5];
	return;
}


template <typename PointT> const bool
ClusterConnectivity<PointT>::isClusterInVoxel3D(const int cID_arg, const PointT& point_arg) 
{
	int vID = getVoxelIDAtPoint(point_arg);
	if (vID == -1)
		return false;

	std::vector<int> cID_indices = vID_cID_table_[vID];

	bool bFind = false;
	for (int i = 0; i < cID_indices.size(); i++)
	{
		if (cID_arg == cID_indices[i])
		{
			bFind = true;
			break;
		}
	}

	return bFind;
}

template <typename PointT> const bool
ClusterConnectivity<PointT>::isClusterInVoxel2D(const int cID_arg, const PointT& point_arg) 
{
	double x, y;
	x = point_arg.x;
	y = point_arg.y;
//	z = point_arg.z;

	int ijk0 = static_cast<int> (floor((x - min_p[0]) * inverse_leaf_size_[0]));
	int ijk1 = static_cast<int> (floor((y - min_p[1]) * inverse_leaf_size_[1]));
//	int ijk2 = static_cast<int> (floor((z - min_p[2]) * inverse_leaf_size_[2]));

	pcl::octree::OctreeKey	key_arg;
	key_arg.x = ijk0; key_arg.y = ijk1;

	if (key_arg.x < 0 || key_arg.x > vNumX_ - 1
		|| key_arg.y < 0 || key_arg.y > vNumY_ - 1)
		return false;

//	bool bFind = false;
	for (int ijk2 = 0; ijk2 < vNumZ_; ++ijk2)
	{
		key_arg.z = ijk2;

		VoxelMap::iterator it_vm;
		it_vm = vID_map_.find(key_arg);
		if (it_vm == vID_map_.end())
			continue;  //null voxel
		else
		{
			int vID = it_vm->second;

			std::vector<int> cID_indices = vID_cID_table_[vID];
			for (int i = 0; i < cID_indices.size(); i++)
			{
				if (cID_arg == cID_indices[i])
				{
					return true;
				}
			}
		}
	}
	
	return false;
}

template <typename PointT> const int
ClusterConnectivity<PointT>::getVoxelIDAtPoint(const PointT& point_arg)
{
	double x, y, z;
	x = point_arg.x;
	y = point_arg.y;
	z = point_arg.z;

	int ijk0 = static_cast<int> (floor((x - min_p[0]) * inverse_leaf_size_[0]));
	int ijk1 = static_cast<int> (floor((y - min_p[1]) * inverse_leaf_size_[1]));
	int ijk2 = static_cast<int> (floor((z - min_p[2]) * inverse_leaf_size_[2]));

	pcl::octree::OctreeKey	key_arg;
	key_arg.x = ijk0; key_arg.y = ijk1; key_arg.z = ijk2;

	if (key_arg.x < 0 || key_arg.x > vNumX_ - 1
		|| key_arg.y < 0 || key_arg.y > vNumY_ - 1
		|| key_arg.z < 0 || key_arg.z > vNumZ_ - 1)
		return -1;

	VoxelMap::iterator it_vm;
	it_vm = vID_map_.find(key_arg);
	if (it_vm == vID_map_.end())
		return -1;  //null voxel
	else
		return it_vm->second;
}

template <typename PointT> void
ClusterConnectivity<PointT>::voxelize(float vsize)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return;
	}

	if (vsize < 0)
		return;

	voxel_size_ = vsize;


	leaf_size_[0] = vsize; leaf_size_[1] = vsize; leaf_size_[2] = vsize;
	// Avoid division errors
//	if (leaf_size_[3] == 0)
		leaf_size_[3] = 1;
	// Use multiplications instead of divisions
	inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
	

	// Get the minimum and maximum dimensions
	pcl::getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

// 	m_bbmin[0] = min_p[0]; m_bbmin[1] = min_p[1]; m_bbmin[2] = min_p[2];
// 	m_bbmax[0] = max_p[0]; m_bbmax[1] = max_p[1]; m_bbmax[2] = max_p[2];

	// Check that the leaf size is not too small, given the size of the data
	int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
	int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

	if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
	{
		PCL_WARN("[pcl::UrbanRec::VoxelFCGraph] Leaf size is too small for the input dataset. Integer indices would overflow.");
		//output = *input_;
		return;
	}

	vNumX_ = dx; vNumY_ = dy; vNumZ_ = dz;
//	vLayerNum_ = vNumX_ * vNumY_;

//	VoxelContainerPointIndices cell;
//	cell.voxel_type = null_voxel;
//	cell.voxel_att = NULL;
//	cell.father = NULL;
//	vNeighbourhood_.resize(vNumX_*vNumY_*vNumZ_, cell);

//	m_boundMark = Eigen::MatrixXd::Constant(vNumY_, vNumX_, -1.0);
	//	m_boundMark.resize(vNumX_, vNumY_);

	// Compute the minimum and maximum bounding box values
	// 	min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
	// 	max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
	// 	min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
	// 	max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
	// 	min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
	// 	max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

	// Compute the number of divisions needed along all axis
	//	div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
	//	div_b_[3] = 0;

	// Set up the division multiplier
	//	divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);


	// First pass: go over all points and insert them into the index_vector vector
	// with calculated idx. Points with the same idx value will contribute to the
	// same point of resulting CloudPoint
//	std::vector<VoxelCell>::iterator vIter = vNeighbourhood_.begin();
//	num_of_occupied_ = 0;

	voxel_list_.clear();
	vID_map_.clear();
	int vID;
	for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
	{
		double x, y, z;
		x = input_->points[*it].x;
		y = input_->points[*it].y;
		z = input_->points[*it].z;
// 		if (!input_->is_dense)
// 			// Check if the point is invalid
// 			if (!pcl_isfinite(x) ||
// 				!pcl_isfinite(y) ||
// 				!pcl_isfinite(z))
// 				continue;

		int ijk0 = static_cast<int> (floor((x - min_p[0]) * inverse_leaf_size_[0]));
		int ijk1 = static_cast<int> (floor((y - min_p[1]) * inverse_leaf_size_[1]));
		int ijk2 = static_cast<int> (floor((z - min_p[2]) * inverse_leaf_size_[2]));

		pcl::octree::OctreeKey	key_arg;
		key_arg.x = ijk0; key_arg.y = ijk1; key_arg.z = ijk2;

		VoxelMap::iterator it_vm;
		it_vm = vID_map_.find(key_arg);

		if(it_vm != vID_map_.end())
		{//exist voxel
			vID = it_vm->second;
			voxel_list_[vID].addPointIndex(*it);
		}
		else
		{//add new voxel
			pcl::UrbanRec::VoxelContainerPointIndices newVoxel;
// 			newVoxel.occupyFlag = pcl::UrbanRec::occupied_voxel;
// 			newVoxel.voxel_key = key_arg;
 			newVoxel.addPointIndex(*it);

			vID_map_.insert(std::make_pair(key_arg, voxel_list_.size()));
			voxel_list_.push_back(newVoxel);
		}
	}

	//build vID_cID table
	vID_cID_table_.clear();
	vID_cID_table_.resize(voxel_list_.size());
	for (int i = 0; i < voxel_list_.size(); i++)
	{
		std::vector<int>* vPtIndices = voxel_list_[i].getPointIndices();

		std::vector<int> cID_list;
		int cID;
		for (std::vector<int>::iterator it = vPtIndices->begin();
			it != vPtIndices->end();
			++it)
		{
			cID = pID_cID_table_->at(*it);
			cID_list.push_back(cID);
		}

		std::sort(cID_list.begin(), cID_list.end());

		std::vector<int>  vCIndices;  //voxel - cluster indices

		cID = cID_list[0];
		vCIndices.push_back(cID);
		for (int j = 0; j < cID_list.size(); j++)
		{
			if (cID != cID_list[j])
			{
				cID = cID_list[j];
				vCIndices.push_back(cID);
			}
		}

		vID_cID_table_[i] = vCIndices;
	}

}

template <typename PointT> CCGraph* 
ClusterConnectivity<PointT>::build_voxelConnectivity(float vsize)
{
	voxelize(vsize);

	adj_graph_.reset();
	adj_graph_ = boost::shared_ptr< CCGraph >(new CCGraph(/*cluster_amount_*/));

	VertexDescriptor vertex_descriptor(0);
	vertices_.clear();
	vertices_.resize(cluster_amount_, vertex_descriptor);

	for (int ic = 0; ic < cluster_amount_; ic++)
		vertices_[ic] = boost::add_vertex(*adj_graph_);

//	igl::VertexDescriptor cID1, cID2;

	std::vector<int> nvoxel_indices; //邻接voxel ID
	
	for (VoxelMap::iterator vmIter = vID_map_.begin();
		vmIter != vID_map_.end();
		++vmIter)
	{//遍历每个voxel
		pcl::octree::OctreeKey key_arg = vmIter->first;
		int vID1 = vmIter->second;
		std::vector<int> v1_cID_indices = vID_cID_table_[vID1];

		//voxel中有多个cluster时，连接所有的cluster
		for (int i = 0; i < v1_cID_indices.size() - 1; i++)
		{
			for (int j = i + 1; j < v1_cID_indices.size(); j++)
			{
// 				cID1 = boost::vertex(v1_cID_indices[i], *adj_graph_);
// 				cID2 = boost::vertex(v1_cID_indices[j], *adj_graph_);
// 				one = vertex(1, undigraph);
// 				two = vertex(2, undigraph);
// 				add_edge(zero, one, undigraph);
// 				add_edge(zero, two, undigraph);
// 				add_edge(one, two, undigraph);

				boost::add_edge(vertices_[v1_cID_indices[i]], vertices_[v1_cID_indices[j]], *adj_graph_);
			}
		}
		
		search_Voxel_Neighbours(key_arg, nvoxel_indices);
		for (std::vector<int>::iterator vIter = nvoxel_indices.begin();
			vIter != nvoxel_indices.end();
			++vIter)
		{
			int vID2 = *vIter;
			std::vector<int> v2_cID_indices = vID_cID_table_[vID2];
			//连接相邻voxel中的所有cluster

			for (int i = 0; i < v1_cID_indices.size(); i++)
			{
				for (int j = 0; j < v2_cID_indices.size(); j++)
				{
// 					cID1 = boost::vertex(v1_cID_indices[i], *adj_graph_);
// 					cID2 = boost::vertex(v2_cID_indices[j], *adj_graph_);
					if(v1_cID_indices[i] == v2_cID_indices[j]) continue;

					boost::add_edge(vertices_[v1_cID_indices[i]], vertices_[v2_cID_indices[j]], *adj_graph_);
				}
			}
		}
		
	}
	
	return adj_graph_.get();
}

template <typename PointT> int
ClusterConnectivity<PointT>::search_Voxel_Neighbours(pcl::octree::OctreeKey key_arg, std::vector<int> &Voxel_indices)
{
	Voxel_indices.clear();

	std::vector<pcl::octree::OctreeKey> neighbors;
	int nn = 10;  //邻域数
//	neighbors = Eigen::MatrixXd::Constant(nn, 3, 0);

	neighbors.resize(nn);
	//10邻域定义
	neighbors[0].x = key_arg.x - 1;		neighbors[0].y = key_arg.y;			neighbors[0].z = key_arg.z;
	neighbors[1].x = key_arg.x + 1;		neighbors[1].y = key_arg.y;			neighbors[1].z = key_arg.z;
	neighbors[2].x = key_arg.x;			neighbors[2].y = key_arg.y - 1;		neighbors[2].z = key_arg.z;
	neighbors[3].x = key_arg.x;			neighbors[3].y = key_arg.y + 1;		neighbors[3].z = key_arg.z;
	neighbors[4].x = key_arg.x;			neighbors[4].y = key_arg.y;			neighbors[4].z = key_arg.z - 1;
	neighbors[5].x = key_arg.x;			neighbors[5].y = key_arg.y;			neighbors[5].z = key_arg.z + 1;

	neighbors[6].x = key_arg.x - 1;		neighbors[6].y = key_arg.y - 1;		neighbors[6].z = key_arg.z;
	neighbors[7].x = key_arg.x - 1;		neighbors[7].y = key_arg.y + 1;		neighbors[7].z = key_arg.z;
	neighbors[8].x = key_arg.x + 1;		neighbors[8].y = key_arg.y - 1;		neighbors[8].z = key_arg.z;
	neighbors[9].x = key_arg.x + 1;		neighbors[9].y = key_arg.y + 1;		neighbors[9].z = key_arg.z;


	for (int i = 0; i < nn; i++)
	{
		if (neighbors[i].x < 0 || neighbors[i].x > vNumX_ - 1
			|| neighbors[i].y < 0 || neighbors[i].y > vNumY_ - 1
			|| neighbors[i].z < 0 || neighbors[i].z > vNumZ_ - 1)
			continue;

		VoxelMap::iterator it_vm;
		it_vm = vID_map_.find(neighbors[i]);
		if (it_vm == vID_map_.end())
			continue;  //null voxel

		Voxel_indices.push_back(it_vm->second);

	}

	return Voxel_indices.size();
}