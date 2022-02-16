#include "registration/build_IntersectionGraph.h"
#include "filesupport/filename_support.h"
#include "filesupport/AuxFileIO.h"
#include "shape_detection/impl/ptSegment.hpp"
#include "clustering/cluster_connectivity.h"
#include "clustering/impl/cluster_connectivity.hpp"

//pcl
#include <pcl/io/pcd_io.h>

//cgal
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>

int detect_intersection_points(char *wall_dir, char *file_ext, std::vector<std::string> &filenames, 
	std::vector<ptSegment<pcl::PointXYZRGBNormal>> &segments, std::vector<iglIntersectionPoint> &ipts)
{
	typedef pcl::PointXYZRGBNormal PointT;

	TraversFolderForCertainFiles(wall_dir, file_ext, filenames);

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
	boost::shared_ptr<ClusterConnectivity<PointT> > voxelConn(new ClusterConnectivity<PointT>);

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

