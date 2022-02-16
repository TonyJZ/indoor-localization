// utest_geoTransform.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <ctime>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation.h>
#ifdef _WIN32
#include <pcl/registration/transformation_estimation_dq.h>
#else
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#endif // _WIN32


#include <pcl/registration/transformation_estimation_lm.h>
//#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>


#include <boost/random.hpp>


#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

using namespace Eigen;

#define sqr(x)  ((x)*(x))
double statDistanceStddev(pcl::PointCloud<pcl::PointXYZ> *srcCloud, pcl::PointCloud<pcl::PointXYZ> *dstCloud)
{
	double dis_stddev = 0;
	int num = srcCloud->size();

	if (srcCloud->size() != dstCloud->size())
		return std::numeric_limits<double>::max();

	for (int i = 0; i < num; i++)
	{
		pcl::PointXYZ pt1, pt2;
		pt1 = srcCloud->points[i];
		pt2 = dstCloud->points[i];

		double dis = sqrt(sqr(pt1.x - pt2.x) + sqr(pt1.y - pt2.y) + sqr(pt1.z - pt2.z));

		dis_stddev += dis;
	}

	dis_stddev /= num;
	return dis_stddev;
}

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransformType;
typedef std::vector<Eigen::Vector3d>                PointsType;


int main(int argc, char * argv[])
{
	if (argc < 4)
		return 1;

	char *pFileName = argv[1];
	char *pOutName = argv[2];
	int num_simulate = atoi(argv[3]);

	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_r_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PCDReader reader;
	reader.read(pFileName, *src_cloud);

	Eigen::Matrix<double, 4, 1> centroid_src, centroid_tgt;
	// Estimate the centroids of source, target
	pcl::compute3DCentroid(*src_cloud, centroid_src);

	std::ofstream  ofs;
	ofs.open(pOutName);

	if (!ofs.is_open())
	{
		printf("can't open %s!\n", pOutName);
		return 1;
	}

	boost::mt19937 rng(time(0));
	// 1. uniform_int  
	boost::uniform_int<> ui_angle(0, 72);  //interval: 5 degree 
	boost::uniform_real<> ui_offset(-100.0, 100.0);

	double angle_interval = 5.0 / 180.0*M_PI;
	double roll, pitch, yaw;
	double xoff, yoff, zoff;
	

	ofs << "File name: " << pFileName << std::endl;
	ofs << "point number: " << src_cloud->size() << std::endl;

	ofs << "simulation times: " ;
	ofs << num_simulate << std::endl;

	pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, double> TE_LM;

#ifdef _WIN32
	pcl::registration::TransformationEstimationDQ<pcl::PointXYZ, pcl::PointXYZ, double> TE_DQ;
#else
	pcl::registration::TransformationEstimationDualQuaternion<pcl::PointXYZ, pcl::PointXYZ, double> TE_DQ;
#endif // _WIN32
		
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double> TE_SVD(false);

	Eigen::Matrix<double, 4, 4> siml_transMat;
	Eigen::Matrix<double, 4, 4> esti_transMat;

	Eigen::Affine3d set_trans;
	Eigen::Affine3d esti_trans;

	Eigen::Vector3d esti_eulerAngle;
	double esti_xoff, esti_yoff, esti_zoff;

	printf("simulate times = %d\n", num_simulate);
	for (int i = 0; i < num_simulate; i++)
	{
		printf("time: %05d\n", i);
		roll = ui_angle(rng)*angle_interval;
		pitch = ui_angle(rng)*angle_interval;
		yaw = ui_angle(rng)*angle_interval;
		xoff = ui_offset(rng);
		yoff = ui_offset(rng);
		zoff = ui_offset(rng);

// 		Matrix3d m;
// 
// 		m = AngleAxisd(yaw, Vector3d::UnitZ())
// 			* AngleAxisd(pitch, Vector3d::UnitY())
// 			* AngleAxisd(roll, Vector3d::UnitX());



// 		siml_transMat = Eigen::Matrix<double, 4, 4>::Identity();
//  		Eigen::Affine3d set_trans;
// // 		set_trans.translation() << xoff, yoff, zoff;
// // 		set_trans.rotation() = m;
// 		siml_transMat.block<3, 3>(0, 0) = m;
// 		siml_transMat.block<1, 3>(3, 0) << xoff, yoff, zoff;
// 
// 		std::cout << m << std::endl << std::endl;
// 		std::cout << siml_transMat << std::endl;

// 		siml_transMat(0, 0) = m(0, 0); siml_transMat(0, 1) = m(0, 1); siml_transMat(0, 2) = m(0, 2); siml_transMat(0, 3) = xoff;
// 		siml_transMat(1, 0) = m(1, 0); siml_transMat(1, 1) = m(1, 1); siml_transMat(1, 2) = m(1, 2); siml_transMat(1, 3) = yoff;
// 		siml_transMat(2, 0) = m(2, 0); siml_transMat(2, 1) = m(2, 1); siml_transMat(2, 2) = m(2, 2); siml_transMat(2, 3) = zoff;

// 		pcl::transformPointCloud(*src_cloud, *dst_cloud, siml_transMat);
// 
// 		double dis = statDistanceStddev(src_cloud.get(), dst_cloud.get());

		
		set_trans = Eigen::Affine3d::Identity();
		set_trans.translation() << xoff, yoff, zoff;
		set_trans.rotate(AngleAxisd(yaw, Vector3d::UnitZ())
			* AngleAxisd(pitch, Vector3d::UnitY())
			* AngleAxisd(roll, Vector3d::UnitX()));

		pcl::transformPointCloud(*src_cloud, *dst_cloud, set_trans);


		ofs << "time " << i << ":" << std::endl;
		ofs << "setting: " << pcl::rad2deg(roll) << ',' << pcl::rad2deg(pitch) << ',' << pcl::rad2deg(yaw) << ',' << xoff << ',' << yoff << ',' << zoff << std::endl;
		ofs << "rotateMat" << std::endl;
		ofs << set_trans.matrix();
		ofs << std::endl;

		//std::cout << set_trans.matrix().block<3, 3>(0, 0) << std::endl << std::endl;
		std::cout << set_trans.matrix() << std::endl;

		double dis = statDistanceStddev(src_cloud.get(), dst_cloud.get());

		ofs << "dis: " << dis << std::endl;

		//1. SVD
		ofs << "method 1: SVD" << std::endl;
		esti_transMat = Eigen::Matrix<double, 4, 4>::Identity();
		TE_SVD.estimateRigidTransformation(*dst_cloud, *src_cloud, esti_transMat);

		Eigen::Affine3d esti_trans;
		esti_trans.matrix() = esti_transMat;

		std::cout << esti_trans.matrix() << std::endl;

		esti_eulerAngle = esti_trans.rotation().eulerAngles(0, 1, 2);
		esti_xoff = esti_transMat(0, 3);
		esti_yoff = esti_transMat(1, 3);
		esti_zoff = esti_transMat(2, 3);

		pcl::compute3DCentroid(*dst_cloud, centroid_tgt);

		Eigen::Matrix<double, 3, 1> Rc(esti_transMat.topLeftCorner(3,3) * centroid_tgt.head(3));
		Eigen::Matrix<double, 3, 1> t(esti_trans.translation() + Rc - centroid_tgt.head(3));

		//t *= -1;

/*		Eigen::Affine3d test_rot;
		test_rot.rotate(AngleAxisd(esti_eulerAngle(2), Vector3d::UnitZ())
			* AngleAxisd(esti_eulerAngle(1), Vector3d::UnitY())
			* AngleAxisd(esti_eulerAngle(0), Vector3d::UnitX()));

		std::cout << test_rot.matrix() << std::endl;*/

		ofs<<"estimated: "<< pcl::rad2deg(esti_eulerAngle(0)) << ',' << pcl::rad2deg(esti_eulerAngle(1)) << ',' << pcl::rad2deg(esti_eulerAngle(2)) << ',' << t(0) << ',' << t(1) << ','
			<<t(2) << std::endl;

		
		ofs << "rotateMat" << std::endl;
		//ofs << esti_trans.rotation();
		ofs << esti_trans.matrix().block<4, 4>(0, 0);
		ofs << std::endl;

		pcl::transformPointCloud(*dst_cloud, *trans_cloud, esti_transMat);
		dis = statDistanceStddev(src_cloud.get(), trans_cloud.get());
		ofs << "dis: " << dis << std::endl;

		//2. DQ
		ofs << "method 2: DQ" << std::endl;
		esti_transMat = Eigen::Matrix<double, 4, 4>::Identity();
		TE_DQ.estimateRigidTransformation(*dst_cloud, *src_cloud, esti_transMat);

		esti_trans.matrix() = esti_transMat;

		std::cout << esti_trans.matrix() << std::endl;

		esti_eulerAngle = esti_trans.rotation().eulerAngles(0, 1, 2);
		esti_xoff = esti_transMat(0, 3);
		esti_yoff = esti_transMat(1, 3);
		esti_zoff = esti_transMat(2, 3);

		//pcl::compute3DCentroid(*dst_cloud, centroid_tgt);

		Rc = esti_transMat.topLeftCorner(3, 3) * centroid_tgt.head(3);
		t = esti_trans.translation() + Rc - centroid_tgt.head(3);

		ofs << "estimated: " << pcl::rad2deg(esti_eulerAngle(0)) << ',' << pcl::rad2deg(esti_eulerAngle(1)) << ',' << pcl::rad2deg(esti_eulerAngle(2)) << ',' << t(0) << ',' << t(1) << ','
			<< t(2) << std::endl;


		ofs << "rotateMat" << std::endl;
		//ofs << esti_trans.rotation();
		ofs << esti_trans.matrix().block<4, 4>(0, 0);
		ofs << std::endl;

		pcl::transformPointCloud(*dst_cloud, *trans_cloud, esti_transMat);
		dis = statDistanceStddev(src_cloud.get(), trans_cloud.get());
		ofs << "dis: " << dis << std::endl;

		//3. LM
		ofs << "method 3: LM" << std::endl;
		esti_transMat = Eigen::Matrix<double, 4, 4>::Identity();
		TE_LM.estimateRigidTransformation(*dst_cloud, *src_cloud, esti_transMat);


		esti_trans.matrix() = esti_transMat;

		std::cout << esti_trans.matrix() << std::endl;

		esti_eulerAngle = esti_trans.rotation().eulerAngles(0, 1, 2);
		esti_xoff = esti_transMat(0, 3);
		esti_yoff = esti_transMat(1, 3);
		esti_zoff = esti_transMat(2, 3);

		Rc = esti_transMat.topLeftCorner(3, 3) * centroid_tgt.head(3);
		t = esti_trans.translation() + Rc - centroid_tgt.head(3);

		ofs << "estimated: " << pcl::rad2deg(esti_eulerAngle(0)) << ',' << pcl::rad2deg(esti_eulerAngle(1)) << ',' << pcl::rad2deg(esti_eulerAngle(2)) << ',' << t(0) << ',' << t(1) << ','
			<< t(2) << std::endl;


		ofs << "rotateMat" << std::endl;
		//ofs << esti_trans.rotation();
		ofs << esti_trans.matrix().block<4, 4>(0, 0);
		ofs << std::endl;

		pcl::transformPointCloud(*dst_cloud, *trans_cloud, esti_transMat);
		dis = statDistanceStddev(src_cloud.get(), trans_cloud.get());
		ofs << "dis: " << dis << std::endl;
	}

	ofs.close();
	return 0;
}

