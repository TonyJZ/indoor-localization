// utest_shapedetection.cpp : Defines the entry point for the console application.
//
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "filesupport/filename_support.h"
#include "filesupport/AuxFileIO.h"
#include "shape_detection/eRANSAC_detection.h"
#include "shape_detection/impl/ptSegment.hpp"


void usage(bool wait = false)
{
	printf("wall detection V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_shapedetection wall_pts.pcd eRANSAC_dir filtered_dir --doRANSAC(optional)\n");
	printf("---------------------------------------------\n");
	if (wait)
	{
		printf("<press ENTER>\n");
		getc(stdin);
	}
	exit(1);
}

static void byebye(bool wait = false)
{
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	/*	exit(1);*/
}


typedef pcl::PointXYZRGBNormal PointT;
//template class ptSegment<PointT>;

//#define PointT pcl::PointXYZRGBNormal
int main(int argc, char * argv[])
{
	if (argc < 4)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *pWallFileName = argv[i]; i++;
	char *pRansacOutDir = argv[i]; i++;
	char *pFilterOutDir = argv[i]; i++;
	
	bool doRansac = false;

	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--doRANSAC") == 0)
			doRansac = true;
// 		else if (strcmp(argv[i], "-z0") == 0 || strcmp(argv[i], "-Z0") == 0) {
// 			i++;
// 			z0 = atof(argv[i]);
// 		}
// 		else if (strcmp(argv[i], "-from") == 0)
// 		{
// 			i++;
// 			prjFrom = argv[i];
// 		}
// 		else if (strcmp(argv[i], "-from") == 0)
// 		{
// 			i++;
// 			prjTo = argv[i];
// 		}
	}

	pcl::PCDReader pcdReader;
	if (doRansac) {

	CreateMultipleDirectory(/*pOutDir*/pRansacOutDir);

	printf("plane detection by efficient RANSAC: \n");

	//plane detection
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
	
	pcdReader.read(pWallFileName, *cloud);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sort_cloud(new pcl::PointCloud <PointT>);
	std::vector<std::vector<int>> inliers;
	std::vector<pcl::ModelCoefficients> planeCoeffs;

	plane_detection_CGAL(*cloud, NULL, true, sort_cloud.get(), inliers, planeCoeffs);
	int iModel = 0;

	pcl::PCDWriter writer;
	for (int i = 0; i < inliers.size(); i++)
	{
		if(inliers[i].size()==0)
			continue;

		//accept
		string out_name, out_param;
		char buf[32], buf_para[32];
		sprintf(buf, "wall_model_%04d.pcd", iModel);
		sprintf(buf_para, "wall_model_%04d.param", iModel);

		out_name = /*pOutDir*/pRansacOutDir;
		out_name += "/";
		//out_name += result_name;
		out_name += buf;

		out_param = /*pOutDir*/pRansacOutDir;
		out_param += "/";
		//out_param += result_name;
		out_param += buf_para;

		writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);

		Save_ParamFile(out_param.c_str(), sMT_plane, planeCoeffs[i]);

		iModel++;
	}
	}


	CreateMultipleDirectory(pFilterOutDir);

	std::vector <std::string> filenames;
	char pExterntion[] = ".pcd";
	TraversFolderForCertainFiles(pRansacOutDir, pExterntion, filenames);

	pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
	ptSegment<PointT>  seg;

	double dc = 0.1;
	double lowrhoTh, highrhoTh;  //用来过滤低密度点(非稳定的墙面)
	double vSize = 0.02; //原始数据的voxel size
	double lowZTh = 0.5, highZTh = 2.0; //可接受的最小墙面高

	lowrhoTh = 2 * dc*lowZTh / (vSize*vSize);
	highrhoTh = 2 * dc*highZTh / (vSize*vSize);

	printf("plane filtering: \n");
	
	int nSegs = filenames.size();
	for (int i = 0; i < nSegs; i++)
	{
		//提取分割点云
		pcdReader.read(filenames[i], *seg_cloud);
		seg.setInputCloud(seg_cloud);

		double diagLen = seg.get2DDiagonal();
		int nsample = 20;
		double dens = seg.getProjectionDensity(nsample, dc);

		if (diagLen < 2.0)
			continue;

		if (diagLen < 3.0 && dens < lowrhoTh)
			continue;


		double maxConnDis;
		double connp = seg.get2DConnectivity(0.2, maxConnDis);
		if (connp < 0.5 && maxConnDis < 2.0)
			continue;


		//accept
		//提取分割模型参数
		char rname[128], sname[128];
		GetPureFileName(filenames[i].c_str(), rname, sname);

		printf("accept: %s\n", rname);

		string newptname = pFilterOutDir;
		newptname += '/';
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
	
	byebye(argc == 1);
    return 0;
}

