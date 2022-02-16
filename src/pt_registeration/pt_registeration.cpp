// pt_registeration.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
//stl
#include <vector>
#include <limits>
#include <math.h>
#include <algorithm>
#include <fstream>
//#include <filesystem>

//pcl
#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/common/io.h>
#include <pcl/segmentation/extract_clusters.h>

//cgal
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>

//self developed
#include "filesupport/filename_support.h"
#include "segmentation/DBSCAN.h"
#include "features/density_estimation.h"
#include "shape_detection/geometry_detection_by_Jlinkage.h"
#include "shape_detection/eRANSAC_detection.h"
#include "shape_detection/alpha_shapes_2d.h"
#include "segmentation/ptSegment.h"
#include "segmentation/cluster_connectivity.h"
#include "filesupport/AuxFileIO.h"

#include "registration/ipGraphMatching.h"


bool descending_vector(const std::vector<int> &elem1, const std::vector<int> &elem2)
{
// 	std::vector<std::vector<int>>::iterator it1, it2;
// 	it1 = elem1;
// 	it2 = elem2;
	return elem1.size() > elem2.size();
}

template <typename PointT>
int clustering_by_DBSCAN(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, double Eps, int MinPts,
	/*std::vector<int> &cluster_center_idx, */std::vector<int> &labels, std::vector<double> &rhosout)
{
	using namespace DBS;

	int nSamples;
	if(indices == NULL)
		nSamples = cloud.size();
	else
		nSamples = indices->size();

	point_t *points = new point_t[nSamples];

	for (int i = 0; i < nSamples; i++)
	{
		int id;
		if (indices)
			id = indices->at(i);
		else
			id = i;

		points[i].x = cloud.points[id].x;
		points[i].y = cloud.points[id].y;
		points[i].z = cloud.points[id].z/*0*/;
		points[i].cluster_id = UNCLASSIFIED;
	}


	dbscan(points, nSamples, Eps, MinPts, euclidean_dist);


	labels.resize(nSamples);
	for (int i = 0; i < nSamples; i++)
	{
		labels[i] = points[i].cluster_id;
	}

	rhosout.assign(nSamples, 0);

	if (points)	delete[] points;   points = NULL;

	return 0;
}



#define PointT pcl::PointXYZRGBNormal

bool descendingIntersectionPointbyWeight(const iglIntersectionPoint &elem1, const iglIntersectionPoint &elem2)
{
	return elem1.weight > elem2.weight;
}

//
int detect_inflection_points(char *inputDir, char *file_ext, std::vector<std::string> &filenames, std::vector<ptSegment<PointT>> &segments, std::vector<iglIntersectionPoint> &ipts)
{
	char *eRansac_dir = inputDir;
//	vector <string> filenames;
	TraversFolderForCertainFiles(eRansac_dir, file_ext, filenames);

	pcl::PCDReader pcdReader;
	pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_in_all(new pcl::PointCloud <PointT>); //合并所有seg点云
																			 //	std::vector<pcl::ModelCoefficients> planeCoeffs;
	std::vector<std::vector<int>> segGroup_indices;
	int nSegs = filenames.size();
	if (nSegs)
		segGroup_indices.resize(nSegs);

	//segNames = filenames;

	//std::vector<ptSegment<PointT>>  segments;

	segments.clear();

	std::vector<int>   p2cLut;  //点云对应的聚类号
	int curptId = 0;
	for (int i = 0; i < nSegs; i++)
	{
		//提取分割点云和模型参数
		char rname[128], sname[128];
		GetFileName(filenames[i].c_str(), rname, sname);
		string paraname = rname;
		paraname += ".param";

		pcl::ModelCoefficients mCoef;
		sModelType type;
		Load_ParamFile(paraname.c_str(), type, mCoef);
		//		planeCoeffs.push_back(mCoef);

		//提取分割点云
		pcdReader.read(filenames[i], *seg_cloud);
		for (int j = 0; j < seg_cloud->size(); j++)
		{//合并点云并记录每个分割的索引
			cloud_in_all->points.push_back(seg_cloud->points[j]);

			p2cLut.push_back(i);

			segGroup_indices[i].push_back(curptId);
			curptId++;
		}

		ptSegment<PointT> seg;

		seg.setInputCloud(seg_cloud);

		seg.setModelCoef(sMT_plane, mCoef);

		segments.push_back(seg);
	}

	
//	segments[0].get2DConnectivity(1.0);

	cloud_in_all->height = 1;
	cloud_in_all->width = cloud_in_all->size();

	//建立voxel connectivity
	boost::shared_ptr<ClusterConnectivity<PointT>> voxelConn(new ClusterConnectivity<PointT>);

	voxelConn->setInputCloud(cloud_in_all);
	voxelConn->setPointClusterLut(&p2cLut);

	// 	Eigen::Vector3f bbmin, bbmax;
	// 	voxelConn->getBBox(bbmin, bbmax);
	// 
	// 	bbmin[0] -= (bbmax[0] - bbmin[0])*0.5;
	// 	bbmin[1] -= (bbmax[1] - bbmin[1])*0.5;
	// 	bbmax[0] += (bbmax[0] - bbmin[0])*0.5;
	// 	bbmax[1] += (bbmax[1] - bbmin[1])*0.5;

	float vsize = 1.0;
	igl::CCGraph* adjSegs = voxelConn->build_voxelConnectivity(vsize);

	//找到相邻的segments并计算交点
	typedef CGAL::Exact_predicates_exact_constructions_kernel K;
	typedef K::Point_2 Point_2;
	typedef K::Segment_2 Segment_2;
	typedef K::Line_2 Line_2;
	typedef K::Intersect_2 Intersect_2;

	typedef K::Line_3	Line_3;
	typedef K::Plane_3	Plane_3;

	//std::vector<iglIntersectionPoint>  ipts;

	ipts.clear();

	igl::VertexIterator vi, vi_end;
	int nvertices = 0;
	for (boost::tie(vi, vi_end) = boost::vertices(*adjSegs); vi != vi_end; ++vi)
	{//遍历顶点
		nvertices++;

		igl::VertexDescriptor vd = *vi;

		igl::AdjacencyIterator ai, ai_end;
		boost::tie(ai, ai_end) = boost::adjacent_vertices(vd, *adjSegs);
		for (; ai != ai_end; ++ai)
		{//遍历邻接节点
			VertexDescriptor vt = *ai;

			if (vd < vt)
			{//计算交点

				std::cout << "connected: (" << vd << ", " << vt << ")" << std::endl;

				pcl::ModelCoefficients vdCeof, vtCeof;

				vdCeof = segments[vd].getModelCoef();
				vtCeof = segments[vt].getModelCoef();

				Line_2 lin1(vdCeof.values[0], vdCeof.values[1], vdCeof.values[3]);
				Line_2 lin2(vtCeof.values[0], vtCeof.values[1], vtCeof.values[3]);

				CGAL::cpp11::result_of<Intersect_2(Line_2, Line_2)>::type
					result = intersection(lin1, lin2);
				if (result)
				{
					if (const Line_2* l = boost::get<Line_2>(&*result))
					{
						std::cout << "line:" << *l << std::endl;
					}
					else
					{
						const Point_2* p = boost::get<Point_2 >(&*result);
						std::cout << "point:" << *p << std::endl;

						PointT pt;
						pt.x = to_double(p->x());
						pt.y = to_double(p->y());
						pt.z = projected2D_zVal;

						bool bFindvd = voxelConn->isClusterInVoxel2D(vd, pt);
						bool bFindvt = voxelConn->isClusterInVoxel2D(vt, pt);

						if (bFindvd || bFindvt)
						{//交点所在的voxel中包含目标分割

							iglIntersectionPoint ipt;
							ipt.p[0] = pt.x;
							ipt.p[1] = pt.y;
							ipt.p[2] = projected2D_zVal;
							ipt.p[3] = 1.0;

							ipt.connIndices.push_back(vd);
							ipt.connIndices.push_back(vt);

							//计算交点的权重
							PointT vd_pmin, vd_pmax, vt_pmin, vt_pmax;
							double vd_dis_2D, vt_dis_2D;

							segments[vd].getFurthestPointsApproximate(vd_pmin, vd_pmax);
							segments[vt].getFurthestPointsApproximate(vt_pmin, vt_pmax);

							vd_dis_2D = std::sqrt((vd_pmax.x - vd_pmin.x)*(vd_pmax.x - vd_pmin.x) 
								+ (vd_pmax.y - vd_pmin.y)*(vd_pmax.y - vd_pmin.y));
							vt_dis_2D = std::sqrt((vt_pmax.x - vt_pmin.x)*(vt_pmax.x - vt_pmin.x)
								+ (vt_pmax.y - vt_pmin.y)*(vt_pmax.y - vt_pmin.y));


							//maxEdge = 1.0;
							
							Eigen::Vector2f vd_line_normal, vt_line_normal;
							vd_line_normal[0] = -vdCeof.values[1];
							vd_line_normal[1] = vdCeof.values[0];

							vt_line_normal[0] = -vtCeof.values[1];
							vt_line_normal[1] = vtCeof.values[0];

							float cos_theta = vd_line_normal.dot(vt_line_normal);
							float sin_theta = sqrt(1.0 - cos_theta*cos_theta);

							ipt.weight = 0.5*(vd_dis_2D + vt_dis_2D*sin_theta);

							ipts.push_back(ipt);
						}

					}
				}
			}

		}
	}

	return 1;
}


double intersection_graph_match(const char *ref_g_name, const char *unreg_g_name, float buf_radius, double mTh, Eigen::Affine3f &init_trans, float norm_org[3])
{
	ipGraphMatching ip_matcher;
	

	ip_matcher.set_ipf(ref_g_name, unreg_g_name);
	ip_matcher.set_search_radius(buf_radius);

	double bestM_cost = std::numeric_limits<double>::max();
	Eigen::Affine3f bestM_trans = Eigen::Affine3f::Identity();
	init_trans = bestM_trans;

	if (!ip_matcher.start_match())
		return bestM_cost;

	while (ip_matcher.match_next())
	{
		double cost = ip_matcher.get_match_cost();
		if (cost < bestM_cost)
		{
			bestM_cost = cost;
			bestM_trans = ip_matcher.get_transform();
		}

		if (bestM_cost < mTh)
			break;
	}

	ip_matcher.get_normalized_org(norm_org);
	init_trans = bestM_trans;
	return bestM_cost;
}


int main(int argc, char* argv[])
{
// 	char *pTestName = argv[1];
// 	char *pOutDir = argv[2];

//	CreateMultipleDirectory(pOutDir);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
	pcl::PCDWriter writer;

#ifdef NEED_ROOFWALL_SEGMETATION
	pcl::PCDReader Reader;
	string filename = "D:/indoor_data/test/to_register/test2/cloud0.pcd";
	Reader.read(filename, *cloud);

	//按法方向分类，提取顶面和立面

	std::vector <int> idx_hor, idx_ver;  //水平面, 垂直面
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

	float vDegThreshold_ = 15.0, fDegThreshold_ = 80.0;
	float threshold_r = cosf(pcl::deg2rad(vDegThreshold_));
	float threshold_f = cosf(pcl::deg2rad(fDegThreshold_));

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
		else if(dot_product < threshold_f)
		{
			idx_ver.push_back(i);
		}
	}

	int nstep = 30;
	double interval = (maxr_h - minr_h) / nstep;

	minr_h = minr_h - 0.5*interval;
	std::vector<int> zHist;
	zHist.resize(nstep + 2, 0);
	
	for (int i = 0; i < idx_hor.size(); i++)
	{
		int id = idx_hor[i];
		double z = cloud->points[id].z;

		int iStep = static_cast<int> (floor((z - minr_h) / interval));
		zHist[iStep]++;
	}

	int imax, maxPts = 0;
	for (int i = 0; i<zHist.size(); i++)
	{
		if (zHist[i] > maxPts)
		{
			maxPts = zHist[i];
			imax = i;
		}
	}

	double sec_floor, sec_ceil; //屋顶的高度区间

	double roof_meanZ = minr_h + imax*interval + 0.5*interval;
	double buf_size = 0.1;  //0.1m
	sec_floor = roof_meanZ - buf_size;
	sec_ceil = roof_meanZ + buf_size;

	std::vector <int> idx_refine_roof; //屋顶面索引
	for (int i = 0; i < idx_hor.size(); i++)
	{
		int id = idx_hor[i];
		double z = cloud->points[id].z;

		if (z > sec_floor && z < sec_ceil)
			idx_refine_roof.push_back(id);
	}

	writer.write<PointT> ("D:/indoor_data/test/to_register/test2/hor.pcd", *cloud, idx_hor, true);
	writer.write<PointT>("D:/indoor_data/test/to_register/test2/ver.pcd", *cloud, idx_ver, true);
	writer.write<PointT>("D:/indoor_data/test/to_register/test2/roof_refine.pcd", *cloud, idx_refine_roof, true);
#endif

#ifdef NEED_DENSITYRANKING
	//local density ranking
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, idx_ver, *cloud2D);
	
	// XYplane projection
	for (int i = 0; i < cloud2D->size(); i++)
	{
		cloud2D->points[i].z = 0;
	}

	double dc = 0.1;
	std::vector<double> rhos;
	getLocalDensity_cutoff(*cloud2D, NULL, dc, rhos);

	std::vector<double>  temp_rhos = rhos;

	std::sort(temp_rhos.begin(), temp_rhos.end(), less<double>());  //ascend

	double mean, median, maxrho, minrho;
	int ncount = temp_rhos.size();
	int nmid = ncount / 2 + 1;

	maxrho = temp_rhos[ncount-1];
	minrho = temp_rhos[0];
	median = temp_rhos[nmid];
	
	mean = 0;
	for (int i = 0; i < ncount; i++)
	{
		mean += temp_rhos[i];
	}
	mean /= ncount;

// 	nstep = 10;
// 	interval = (maxrho - mean) / nstep;
// 
// 	nstep = ceil((maxrho - minrho) / interval);
// 	minrho -= 0.5*interval;
// 
// 	std::vector<int> rhoHist;
// 	rhoHist.resize(nstep + 2, 0);

	std::vector<std::vector<int>>  idx_rhoRanking;  //密度分级  暂时分成两级
	idx_rhoRanking.resize(2);

	double lowrhoTh, highrhoTh;  //用来过滤低密度点(非稳定的墙面)
	double vSize = 0.02; //原始数据的voxel size
	double lowZTh = 0.5, highZTh = 2.0; //可接受的最小墙面高

	lowrhoTh = 2*dc*lowZTh / (vSize*vSize);
	highrhoTh = 2*dc*highZTh / (vSize*vSize);

	for (int i = 0; i < rhos.size(); i++)
	{
		int id = idx_ver[i];
		double cur_rho = rhos[i];

		if(cur_rho < lowrhoTh)
			continue;
		
		if (cur_rho > highrhoTh)
			idx_rhoRanking[0].push_back(id);
		else if (cur_rho > lowrhoTh)
			idx_rhoRanking[1].push_back(id);
	}

	std::sort(idx_rhoRanking.begin(), idx_rhoRanking.end(), descending_vector);

	char result_name[128], suffix_name[8];
	GetPureFileName(pTestName, result_name, suffix_name);
	for (int i = 0; i < idx_rhoRanking.size(); i++)
	{
		if (idx_rhoRanking[i].size() > 0)
		{
			string out_name;
			char buf[32];
			sprintf(buf, "_ver_rhoRank_%d.pcd", i);

			out_name += pOutDir;
			out_name += "/";
			out_name += result_name;
			out_name += buf;
	
			writer.write<PointT>(out_name, *cloud, idx_rhoRanking[i]/*, false*/);
		}
	}
#endif

#ifdef NEED_EuclideanClustering
	//clustering
	char rankingfile[256] = "D:/indoor_data/test/ref_good/ver.pcd";
	pcl::PCDReader pcdReader;
	pcdReader.read(rankingfile, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, *cloud2D);

	// XYplane projection
	for (int i = 0; i < cloud2D->size(); i++)
	{
		cloud2D->points[i].z = 0;
	}

	double dTolerance = 0.2;
	int minSize = 500;
	int maxSize = 100000;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud2D);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(/*0.02*/dTolerance); // 2cm
	ec.setMinClusterSize(/*100*/minSize);
	ec.setMaxClusterSize(/*25000*/maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud2D);
	ec.extract(cluster_indices);

	string pOutDir = "D:/indoor_data/test/ref_good";
	char result_name[128], suffix_name[8];
	GetPureFileName(rankingfile, result_name, suffix_name);
	for (int i = 0; i < cluster_indices.size(); i++)
	{
		if (cluster_indices[i].indices.size() > 0)
		{
			string out_name;
			char buf[32];
			sprintf(buf, "_cluster_%d.pcd", i);

			out_name += pOutDir;
			out_name += "/";
			out_name += result_name;
			out_name += buf;

			writer.write<PointT>(out_name, *cloud, cluster_indices[i].indices, true);
		}
	}
#endif

#ifdef NEED_JLinkage
	char rankingfile[256] = "D:/indoor_data/test_ref/test_jlinkage.pcd";
	pcl::PCDReader pcdReader;
	pcdReader.read(rankingfile, *cloud);

	pcl::PointCloud<PointT>::Ptr sort_cloud(new pcl::PointCloud <PointT>);
	std::vector<std::vector<int>> inliers;

	geometry_detect_by_JLinkage(*cloud, inliers, geoModel_line, 0.3, 10, 0.8, 0.2, 50);
	string pOutDir = "D:/indoor_data/test_ref/jlinkage";
	char result_name[128], suffix_name[8];
	GetPureFileName(rankingfile, result_name, suffix_name);
	for (int i = 0; i < inliers.size(); i++)
	{
		if (inliers[i].size() > 0)
		{
			string out_name, out_param;
			char buf[32], buf_para[32];
			sprintf(buf, "_eRansac_%d.pcd", i);
			sprintf(buf_para, "_eRansac_%d.param", i);

			out_name += pOutDir;
			out_name += "/";
			out_name += result_name;
			out_name += buf;

			writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
		}
	}
#endif


#ifdef NEED_EfficientRANSAC
	//plane detection
	char rankingfile[256] = "D:/indoor_data/test/to_register/test2/ver.pcd";
	pcl::PCDReader pcdReader;
	pcdReader.read(rankingfile, *cloud);

	pcl::PointCloud<PointT>::Ptr sort_cloud(new pcl::PointCloud <PointT>);
 	std::vector<std::vector<int>> inliers;
 	std::vector<pcl::ModelCoefficients> planeCoeffs;
 	plane_detection_CGAL(*cloud, NULL, true, sort_cloud.get(), inliers, planeCoeffs);

	//filtering planes
/*	std::vector<ptSegment<PointT>>  segments;
	pcl::PointIndicesPtr segIndices(new pcl::PointIndices);
	for (int i = 0; i < inliers.size(); i++)
	{
		segIndices->indices = inliers[i];
		ptSegment<PointT> seg;

		seg.setInputCloud(sort_cloud);
		seg.setIndices(segIndices);
		seg.setModelCoef(sMT_plane, planeCoeffs[i]);

		segments.push_back(seg);
	}*/
	
	string pOutDir = "D:/indoor_data/test/to_register/test2/eRansac";
	CreateMultipleDirectory(pOutDir.c_str());

	char result_name[128], suffix_name[8];
	GetPureFileName(rankingfile, result_name, suffix_name);
	for (int i = 0; i < inliers.size(); i++)
	{
		if (inliers[i].size() > 0)
		{
			string out_name, out_param;
			char buf[32], buf_para[32];
			sprintf(buf, "_eRansac_%d.pcd", i);
			sprintf(buf_para, "_eRansac_%d.param", i);

			out_name += pOutDir;
			out_name += "/";
			out_name += result_name;
			out_name += buf;

			out_param += pOutDir;
			out_param += "/";
			out_param += result_name;
			out_param += buf_para;

			writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);

			Save_ParamFile(out_param.c_str(), sMT_plane, planeCoeffs[i]);

// 			std::ofstream  s_param;
// 			s_param.open(out_param);
// 			
// 			s_param << "plane" << endl;
// 			for (int j = 0; j < planeCoeffs[i].values.size(); j++)
// 			{
// 				s_param << planeCoeffs[i].values[j] << endl;
// 			}
// 			
// 			s_param.close();
		}
	}
#endif


#ifdef NEED_planefiltering
	char eRansac_dir[] = "D:/indoor_data/test/ref_good/eRansac/";
	char filtering_dir[] = "D:/indoor_data/test/ref_good/filteredplanes/";

	CreateMultipleDirectory(filtering_dir);

	vector <string> filenames;
	TraversFolderForCertainFiles(eRansac_dir, ".pcd", filenames);

	pcl::PCDReader pcdReader;
	pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
	//pcl::PointCloud<PointT>::Ptr cloud_in_all(new pcl::PointCloud <PointT>); //合并所有seg点云\

	double dc = 0.1;
	double lowrhoTh, highrhoTh;  //用来过滤低密度点(非稳定的墙面)
	double vSize = 0.02; //原始数据的voxel size
	double lowZTh = 0.5, highZTh = 2.0; //可接受的最小墙面高

	lowrhoTh = 2 * dc*lowZTh / (vSize*vSize);
	highrhoTh = 2 * dc*highZTh / (vSize*vSize);

	ptSegment<PointT>  seg;
	int nSegs = filenames.size();
	for (int i = 0; i < nSegs; i++)
	{
		//提取分割点云
		pcdReader.read(filenames[i], *seg_cloud);
		seg.setInputCloud(seg_cloud);

		double diagLen = seg.get2DDiagonal();
		int nsample = 20;
		double dens = seg.getProjectionDensity(nsample, dc);

		if(diagLen < 2.0)
			continue;

		if (diagLen < 3.0 && dens < lowrhoTh)
			continue;


		double maxConnDis;
		double connp = seg.get2DConnectivity(0.2, maxConnDis);
		if(connp  < 0.5 && maxConnDis < 2.0)
			continue;


		//accept
		 //提取分割模型参数
		char rname[128], sname[128];
		GetPureFileName(filenames[i].c_str(), rname, sname);
		string newptname = filtering_dir;
		newptname += rname;
		string newparamname = newptname;
		newptname += ".pcd";
		newparamname += ".param";

		GetFileName(filenames[i].c_str(), rname, sname);
		string paramname = rname;
		paramname += ".param";

		boost::filesystem::copy_file(filenames[i].c_str(), newptname.c_str(),
			boost::filesystem::copy_option::overwrite_if_exists);
		boost::filesystem::copy_file(paramname.c_str(), newparamname.c_str(),
			boost::filesystem::copy_option::overwrite_if_exists);

	}

#endif  



#ifdef NEED_voxelConnectivity
	char eRansac_dir[] = "D:/indoor_data/test/ref_good/filteredplanes/";

	std::vector<ptSegment<PointT>> segments;
	std::vector<iglIntersectionPoint> ipts;
	std::vector<std::string> filenames;

	detect_inflection_points(eRansac_dir, ".pcd", filenames, segments, ipts);
	
	std::sort(ipts.begin(), ipts.end(), descendingIntersectionPointbyWeight);

	char ipf_name[] = "D:/indoor_data/test/ref_good/intersection.ipf";

	Save_ipfFile(ipf_name, filenames, ipts);

	//用于检查的点云文件
	pcl::PointCloud<PointT>::Ptr intersection_cloud(new pcl::PointCloud <PointT>);

	for (int i = 0; i < ipts.size(); i++)
	{
		PointT pt;
		pt.x = ipts[i].p[0];
		pt.y = ipts[i].p[1];
		pt.z = ipts[i].p[2];

		intersection_cloud->points.push_back(pt);
	}
	intersection_cloud->height = 1;
	intersection_cloud->width = intersection_cloud->size();
	writer.write<PointT>("D:/indoor_data/test/ref_good/intersection.pcd", *intersection_cloud, true);

#endif

//#ifdef NEED_ipGraphMatching

	char ref_g_name[] = "D:/indoor_data/test/ref_good/intersection.ipf";
	char unreg_g_name[] = "D:/indoor_data/test/to_register/test2/intersection.ipf";

	float init_r = 25.0;
	double init_mTh = 0.2;
	Eigen::Affine3f init_trans;
	float norm_org[3];

	double mCost = intersection_graph_match(ref_g_name, unreg_g_name, init_r, init_mTh, init_trans, norm_org);

	//test initial transformation
//	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
	pcl::PCDReader Reader;
	string filename = "D:/indoor_data/test/to_register/test2/cloud0.pcd";
	Reader.read(filename, *cloud);

	for (int i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].data[0] -= norm_org[0];
		cloud->points[i].data[1] -= norm_org[1];
		cloud->points[i].data[2] -= norm_org[2];
	}
	pcl::PointCloud<PointT> trans_cloud;
	pcl::transformPointCloud(*cloud, trans_cloud, init_trans);

//	pcl::PCDWriter writer;
	writer.write<PointT>("D:/indoor_data/test/to_register/test2/init_trans.pcd", trans_cloud, true);


//#endif

	//test alpha shape 2d
#ifdef NEED_alphaShape
	char rankingfile[256] = "D:/indoor_data/test/un_reg_subsample.pcd";
 	pcl::PCDReader pcdReader;
 	pcdReader.read(rankingfile, *cloud);

	std::vector<float> weights;
	pcl::PointCloud<PointT>::Ptr exEdges(new pcl::PointCloud <PointT>);
	alpha_shape_2d(*cloud, NULL, false, weights, exEdges.get(), 0.1);

	string out_name = "D:/indoor_data/test/un_reg_subsample_alphashape.pcd";
	writer.write<PointT>(out_name, *exEdges,  true);
#endif

	//clustering by density and local connectivity
//	std::vector<int> slabels;
//	std::vector<double> rhos;

/*	clustering_by_DBSCAN(*cloud, &idx_ver, 0.05, 10,  slabels, rhos);

	//统计聚类数
	int max_ClsID = 0;
	for (int i = 0; i < idx_ver.size(); i++)
	{
		int clsID = slabels[i];
		if (clsID > max_ClsID)
			max_ClsID = clsID;
	}

	std::vector<std::vector<int>>  cls_idx;
	std::vector<int> uncls_idx;  //未分类点索引

	cls_idx.resize(max_ClsID + 1);
	for (int i = 0; i < idx_ver.size(); i++)
	{
		int clsID = slabels[i];

		if (clsID == UNCLASSIFIED)
		{
			uncls_idx.push_back(i);
		}
		else
		{
			cls_idx[clsID].push_back(i);
		}
	}*/

	
    return 0;
}

