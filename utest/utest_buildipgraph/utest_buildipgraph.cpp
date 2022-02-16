// utest_buildipgraph.cpp : Defines the entry point for the console application.
//
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "registration/build_IntersectionGraph.h"
#include "filesupport/AuxFileIO.h"
#include "filesupport/filename_support.h"

void usage(bool wait = false)
{
	printf("build intersection graph V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_buildipgraph wall_dir ipfFile --outputforcheck\n");
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

bool descendingIntersectionPointbyWeight(const iglIntersectionPoint &elem1, const iglIntersectionPoint &elem2)
{
	return elem1.weight > elem2.weight;
}


int main(int argc, char * argv[])
{
	if (argc < 3)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *pInputDir = argv[i]; i++;
	char *pOutFileName = argv[i]; i++;

	bool bCheck = false;
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--outputforcheck") == 0)
			bCheck = true;
	}


//	char eRansac_dir[] = "D:/indoor_data/test/ref_good/filteredplanes/";

	std::vector<ptSegment<PointT>> segments;
	std::vector<iglIntersectionPoint> ipts;
	std::vector<std::string> filenames;

	detect_intersection_points(pInputDir, ".pcd", filenames, segments, ipts);

	std::sort(ipts.begin(), ipts.end(), descendingIntersectionPointbyWeight);

	//char ipf_name[] = "D:/indoor_data/test/ref_good/intersection.ipf";

	Save_ipfFile(pOutFileName, filenames, ipts);

	//用于检查的点云文件
	if (bCheck)
	{  
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

		char rname[128], sname[128];
		
		GetFileName(pOutFileName, rname, sname);
		std::string paramname = rname;
		paramname += ".pcd";
		pcl::PCDWriter writer;
		writer.write<PointT>(paramname, *intersection_cloud, true);
	}
	
	byebye(argc == 1);
    return 0;
}

