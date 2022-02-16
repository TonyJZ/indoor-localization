// utest_registration.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "registration/ipGraphMatching.h"
#include "registration/fine_registration.h"
#include "filesupport/AuxFileIO.h"
#include "filesupport/filename_support.h"


void usage(bool wait = false)
{
	printf("indoor registration V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("ref.ipf target.ipf trans_file --search_radius 10.0 --doICP(optional) ref_ceiling.pcd ref_wall.pcd target_ceiling.pcd target_wall.pcd --doTrans\n");
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

double intersection_graph_match(const char *ref_g_name, const char *unreg_g_name, float buf_radius, double mTh, 
	Eigen::Affine3f &init_trans, float norm_org[3])
{
	ipGraphMatching ip_matcher;


	ip_matcher.set_ipf(ref_g_name, unreg_g_name);
	ip_matcher.set_search_radius(buf_radius);

	double bestM_cost = std::numeric_limits<double>::max();
	Eigen::Affine3f bestM_trans = Eigen::Affine3f::Identity();
	init_trans = bestM_trans;

	if (!ip_matcher.start_match())
		return bestM_cost;

	int iter = 0;
	while (ip_matcher.match_next())
	{
		double cost = ip_matcher.get_match_cost();
		
		printf("iter: %d, cost: %f \n", iter++, cost);

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

typedef pcl::PointXYZRGBNormal PointT;

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
	char *ref_ipf_name = argv[i]; i++;
	char *tar_ipf_name = argv[i]; i++;
	char *trans_name = argv[i]; i++;
	char *ref_ceiling_name = NULL;
	char *ref_wall_name = NULL;
	char *tar_ceiling_name = NULL;
	char *tar_wall_name = NULL;

	float init_r = 10.0;
	double init_mTh = 0.2;

	bool doICP = false;
	bool doTrans = false;
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--search_radius") == 0)
		{
			i++;
			init_r = atof(argv[i]);
		}
		else if (strcmp(argv[i], "--doICP") == 0)
		{
			doICP = true;
			i++;
			ref_ceiling_name = argv[i]; i++;
			ref_wall_name = argv[i]; i++;
			tar_ceiling_name = argv[i]; i++;
			tar_wall_name = argv[i]; /*i++;*/
		}
		else if (strcmp(argv[i], "--doTrans") == 0)
		{
			doTrans = true;
			/*i++;*/
		}
	}
	
	

	Eigen::Affine3f init_trans, final_trans;
	float norm_org[3];

	printf("intersection graph matching: \n");
	double mCost = intersection_graph_match(ref_ipf_name, tar_ipf_name, init_r, init_mTh, init_trans, norm_org);

	final_trans = init_trans;

	if (doICP)
	{
		printf("do ICP: \n");

		ipcFineRegistering  ipcReg;
		pcl::PointCloud<PointT>::Ptr ref_ceiling_cloud(new pcl::PointCloud <PointT>);
		pcl::PointCloud<PointT>::Ptr ref_wall_cloud(new pcl::PointCloud <PointT>);
		pcl::PointCloud<PointT>::Ptr tar_ceiling_cloud(new pcl::PointCloud <PointT>);
		pcl::PointCloud<PointT>::Ptr tar_wall_cloud(new pcl::PointCloud <PointT>);

		pcl::PCDReader Reader;
		Reader.read(ref_ceiling_name, *ref_ceiling_cloud);
		Reader.read(ref_wall_name, *ref_wall_cloud);
		Reader.read(tar_ceiling_name, *tar_ceiling_cloud);
		Reader.read(tar_wall_name, *tar_wall_cloud);

		ipcReg.set_ref_pts(ref_ceiling_cloud.get(), ref_wall_cloud.get());
		ipcReg.set_tar_pts(tar_ceiling_cloud.get(), tar_wall_cloud.get());


		ipcReg.set_init_reg_parameters(norm_org, init_trans);

		ipcReg.set_search_bufsize(2.0);
		ipcReg.set_sampling_rate(0.2);

		double stddev_disTh = 0.05;
		bool bSuccess = ipcReg.do_ICP(stddev_disTh);
		double score = ipcReg.get_ICP_score();

		printf("ICP = %d, fitscore = %f\n", bSuccess, score);

		final_trans = ipcReg.get_final_transform();
	}

	if(doTrans)
	{
	//test initial transformation
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
		pcl::PCDReader Reader;
		if (tar_ceiling_name)
		{
			Reader.read(tar_ceiling_name, *cloud);

			for (int i = 0; i < cloud->size(); i++)
			{
				cloud->points[i].data[0] -= norm_org[0];
				cloud->points[i].data[1] -= norm_org[1];
				cloud->points[i].data[2] -= norm_org[2];
			}
			pcl::PointCloud<PointT> trans_cloud;
			pcl::transformPointCloud(*cloud, trans_cloud, final_trans);

			char rname[128], sname[128];

			GetFileName(tar_ceiling_name, rname, sname);
			std::string paramname = rname;
			paramname += "_trans.pcd";
			//paramname += ".pcd";
			pcl::PCDWriter writer;
			writer.write<PointT>(paramname, trans_cloud, true);
		}
		
		if (tar_wall_name)
		{
			Reader.read(tar_wall_name, *cloud);

			for (int i = 0; i < cloud->size(); i++)
			{
				cloud->points[i].data[0] -= norm_org[0];
				cloud->points[i].data[1] -= norm_org[1];
				cloud->points[i].data[2] -= norm_org[2];
			}
			pcl::PointCloud<PointT> trans_cloud;
			pcl::transformPointCloud(*cloud, trans_cloud, final_trans);

			char rname[128], sname[128];

			GetFileName(tar_wall_name, rname, sname);
			std::string paramname = rname;
			paramname += "_trans.pcd";
			//paramname += ".pcd";
			pcl::PCDWriter writer;
			writer.write<PointT>(paramname, trans_cloud, true);
		}
	}

	if (trans_name)
		Save_TransParameFile(trans_name, norm_org, final_trans.matrix());

	byebye(argc == 1);
    return 0;
}

