// utest_clustering.cpp : Defines the entry point for the console application.
//

#include "filesupport/filename_support.h"
#include "clustering/cluster_wall_ceiling.h"

#include <pcl/io/pcd_io.h>

void usage(bool wait = false)
{
	printf("indoor pointcloud clustering V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_clustering input.pcd output_dir\n");
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
	char *pInputFileName = argv[i]; i++;
	char *pOutDir = argv[i]; i++;

	for (; i < argc; i++)
	{
		;
	}

	CreateMultipleDirectory(pOutDir);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGBNormal>);
	pcl::PCDReader Reader;
	//string filename = "D:/indoor_data/test/to_register/test2/cloud0.pcd";
	Reader.read(pInputFileName, *cloud);

	//按法方向分类，提取顶面和立面
	std::vector<std::vector<int>> indices;
	cluster_wall_ceiling(cloud.get(), indices);

	pcl::PCDWriter writer;
	string out_name;
	char buf[32];

	sprintf(buf, "wall_pts.pcd");
	out_name = pOutDir;
	out_name += "/";
	out_name += buf;
	writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, indices[0], true);

	sprintf(buf, "ceiling_pts.pcd");
	out_name = pOutDir;
	out_name += "/";
	out_name += buf;
	writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, indices[1], true);
	
	sprintf(buf, "outlier_pts.pcd");
	out_name = pOutDir;
	out_name += "/";
	out_name += buf;
	writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, indices[2], true);
	
	byebye(argc == 1);
	return 0;
}

