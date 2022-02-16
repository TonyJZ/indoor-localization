#ifndef _cluster_connectivity_h_createdbyTony_2017_July_04_
#define _cluster_connectivity_h_createdbyTony_2017_July_04_

#include <pcl/pcl_macros.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "voxelmap/voxel_container.h"

#include <boost/unordered_map.hpp>
#include <utility> // for std::pair
#include <algorithm> // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "shape_detection/structDef.h"


namespace igl
{//indoor geolocation

typedef typename boost::property <boost::edge_weight_t, std::vector<float> > Weight;
typedef typename boost::adjacency_list <boost::hash_setS, boost::vecS, boost::undirectedS, boost::no_property, Weight> CCGraph;
typedef typename boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::undirectedS> Traits;
typedef Traits::vertex_descriptor VertexDescriptor;
typedef typename boost::graph_traits< CCGraph >::edge_descriptor EdgeDescriptor;
typedef typename boost::graph_traits< CCGraph >::out_edge_iterator OutEdgeIterator;
typedef typename boost::graph_traits< CCGraph >::in_edge_iterator InEdgeIterator;
typedef typename boost::graph_traits< CCGraph >::vertex_iterator VertexIterator;
typedef typename boost::graph_traits< CCGraph >::adjacency_iterator AdjacencyIterator;


//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> CCGraph;  //cluster connectivity graph

template <typename PointT>
class ClusterConnectivity : public pcl::PCLBase<PointT>
{
public:
	typedef pcl::PointCloud< PointT > PointCloud;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;
	typedef boost::unordered_map<pcl::octree::OctreeKey, int> VoxelMap;

	using pcl::PCLBase<PointT>::initCompute;
	using pcl::PCLBase<PointT>::deinitCompute;
	using pcl::PCLBase<PointT>::indices_;
	using pcl::PCLBase<PointT>::input_;

public:
	ClusterConnectivity();
	~ClusterConnectivity();

// 	virtual void
// 		setInputCloud(const PointCloudConstPtr &cloud);

	void setPointClusterLut(std::vector<int>* p2c_Lut);

	void getBBox(Eigen::Vector3f &bbmin, Eigen::Vector3f &bbmax);

	CCGraph* build_voxelConnectivity(float vsize);

	//根据查询点坐标，提取voxel ID.  no voxel return -1 
	const int getVoxelIDAtPoint(const PointT& point_arg);

	const bool isClusterInVoxel2D(const int cID_arg, const PointT& point_arg) ;

	const bool isClusterInVoxel3D(const int cID_arg, const PointT& point_arg) ;

protected:
	void voxelize(float vsize);
	
	int search_Voxel_Neighbours(pcl::octree::OctreeKey key_arg, std::vector<int> &Voxel_indices);


private:
	//lut
	const std::vector<int>*  pID_cID_table_;  //point-cluster lut
	std::vector<std::vector<int>> vID_cID_table_;  //每个voxel中的cluster编号

	//graph
	std::vector< VertexDescriptor > vertices_; //唯一的顶点数组，避免插入重复顶点
	boost::shared_ptr<CCGraph> adj_graph_;	//adjacency graph for clusters


	//voxels
	std::vector<pcl::UrbanRec::VoxelContainerPointIndices> voxel_list_;
	VoxelMap   vID_map_; //voxel的八叉树编码 - vID

	float voxel_size_;
	int vNumX_, vNumY_, vNumZ_;
	Eigen::Vector4f leaf_size_;
	Eigen::Array4f inverse_leaf_size_;
	Eigen::Vector4f min_p, max_p;

	//聚类总数 从point-cluster lut中确定
	int		cluster_amount_;
	
};

}
#ifdef PCL_NO_PRECOMPILE
#include <clustering/impl/cluster_connectivity.hpp>
#endif

#endif
