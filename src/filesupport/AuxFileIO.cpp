#include "filesupport/AuxFileIO.h"

#include <fstream>

bool Load_ParamFile(const char* filename, sModelType &mType, pcl::ModelCoefficients &mCoef)
{
	std::ifstream  ifs;
	ifs.open(filename);

	if (!ifs.is_open())
	{
		return false;
	}

	std::string model_flag;
	ifs >> model_flag;

	if (model_flag.compare(planeFlag) == 0)
	{
		mType = sMT_plane;
		mCoef.values.resize(4, 0.0);
		ifs >> mCoef.values[0] >> mCoef.values[1] >> mCoef.values[2] >> mCoef.values[3];
	}
	else if (model_flag.compare(lineFlag) == 0)
	{
		mType = sMT_line;
		mCoef.values.resize(3, 0.0);
		ifs >> mCoef.values[0] >> mCoef.values[1] >> mCoef.values[2];
	}
	else if(model_flag.compare(unknownModelFlag) == 0)
	{
		//unknown model type
		mType = sMT_undefined;
		mCoef.values.clear();

		while (!ifs.eof())
		{
			float v;
			ifs >> v;
			mCoef.values.push_back(v);
		}

	}

	ifs.close();
	
	return true;
}

bool Save_ParamFile(const char* filename, sModelType mType, pcl::ModelCoefficients mCoef)
{
	std::ofstream  ofs;
	ofs.open(filename);

	if (!ofs.is_open())
	{
		return false;
	}

	if (mType == sMT_line)
	{
		ofs << lineFlag << std::endl;
		
		for (int j = 0; j < mCoef.values.size(); j++)
		{
			ofs << mCoef.values[j] << std::endl;
		}
	}
	else if (mType == sMT_plane)
	{
		ofs << planeFlag << std::endl;
		for (int j = 0; j < mCoef.values.size(); j++)
		{
			ofs << mCoef.values[j] << std::endl;
		}
	}
	else
	{
		ofs << unknownModelFlag << std::endl;
		for (int j = 0; j < mCoef.values.size(); j++)
		{
			ofs << mCoef.values[j] << std::endl;
		}
	}
	ofs.close();

	return true;
}

bool Load_ipfFile(const char* filename, std::vector<std::string> &pointnames, std::vector<iglIntersectionPoint> &ipts)
{
	std::ifstream  ifs;
	ifs.open(filename);

	if (!ifs.is_open())
	{
		return false;
	}

//	std::string file_desc;
//	file_desc.resize(100);
	char file_desc[1024];
	ifs.getline(file_desc, 1024);
//	ifs >> file_desc;

	int n_pts; //number of point cloud 
	ifs.getline(file_desc, 1024);
	n_pts = atoi(file_desc);
//	ifs >> n_pts;

	pointnames.resize(n_pts);
	for (int i = 0; i < n_pts; i++)
	{
		ifs.getline(file_desc, 1024);
		pointnames[i] = file_desc;
		//ifs >> pointnames[i];
	}

	int n_ipt; //number of ipt
	ifs >> n_ipt;

	ipts.resize(n_ipt);
	for (int i = 0; i < n_ipt; i++)
	{
		ifs >> ipts[i].p[0] >> ipts[i].p[1] >> ipts[i].p[2] >> ipts[i].p[3];

		int n_conn;
		ifs >> n_conn;

		ipts[i].connIndices.resize(n_conn);
		for (int j = 0; j < n_conn; j++)
		{
			ifs >> ipts[i].connIndices[j];
		}
		
		ifs >> ipts[i].weight;
	}
	ifs.close();

	return true;
}

bool Save_ipfFile(const char* filename, std::vector<std::string> pointnames, std::vector<iglIntersectionPoint> ipts)
{
	std::ofstream  ofs;
	ofs.open(filename);

	if (!ofs.is_open())
	{
		return false;
	}

	ofs << "This is an intersection point file" << std::endl;

	int n_pts = pointnames.size(); //number of point cloud 
	ofs << n_pts << std::endl;

	for (int i = 0; i < n_pts; i++)
	{
		ofs << pointnames[i] << std::endl;
	}

	int n_ipt = ipts.size(); //number of ipt
	ofs << n_ipt << std::endl;
	
	for (int i = 0; i < n_ipt; i++)
	{
		ofs << ipts[i].p[0] << " " << ipts[i].p[1] << " " << ipts[i].p[2] << " " << ipts[i].p[3] << std::endl;
		
		int n_conn = ipts[i].connIndices.size();
		ofs << n_conn << std::endl;

		for (int j = 0; j < n_conn; j++)
		{
			ofs << ipts[i].connIndices[j] << " ";
		}
		ofs << std::endl;
		ofs << ipts[i].weight << std::endl;
	}
	ofs.close();

	return true;
}

bool Load_TransParameFile(const char* filename, float orgin[3], Eigen::Matrix4f &trans_param)
{
	std::ifstream  ifs;
	ifs.open(filename);

	if (!ifs.is_open())
	{
		return false;
	}

	//	std::string file_desc;
	//	file_desc.resize(100);
	char file_desc[1024];
	ifs.getline(file_desc, 1024);
	ifs >> orgin[0] >> orgin[1] >> orgin[2];

	ifs.getline(file_desc, 1024);

	for (int i = 0; i < 16; i++)
		ifs >> trans_param(i);

	ifs.close();

	return true;
}

bool Save_TransParameFile(const char* filename, float orgin[3], Eigen::Matrix4f trans_param)
{
	std::ofstream  ofs;
	ofs.open(filename);

	if (!ofs.is_open())
	{
		return false;
	}

	ofs << "This is the transformation origin:" << std::endl;
	ofs << orgin[0] << " " << orgin[1] << " " << orgin[2] << std::endl;
	
	ofs << "This is the transformation matrix:" << std::endl;
	ofs << trans_param;
	ofs << std::endl;

	ofs.close();

	return true;
}


bool Load_bundleFile(const char* filename, std::vector<simpleCameraModel> &cams, std::vector<tiePoint> &tps)
{
	std::ifstream  ifs;
	ifs.open(filename);

	if (!ifs.is_open())
	{
		return false;
	}

	cams.clear();
	tps.clear();

	char file_desc[1024];
	ifs.getline(file_desc, 1024);

	int num_cams, num_tps;

	ifs >> num_cams >> num_tps;

	for (int i = 0; i < num_cams; i++)
	{
		simpleCameraModel cam;

		ifs >> cam.f >> cam.k1 >> cam.k2;

		cam.mRC = Eigen::Matrix<double, 4, 4>::Identity();
		ifs >> cam.mRC.block<3,3>(0,0)(0, 0)
			>> cam.mRC.block<3, 3>(0, 0)(0, 1)
			>> cam.mRC.block<3, 3>(0, 0)(0, 2);
			
		ifs >> cam.mRC.block<3, 3>(0, 0)(1, 0)
			>> cam.mRC.block<3, 3>(0, 0)(1, 1)
			>> cam.mRC.block<3, 3>(0, 0)(1, 2);
		
		ifs >> cam.mRC.block<3, 3>(0, 0)(2, 0)
			>> cam.mRC.block<3, 3>(0, 0)(2, 1)
			>> cam.mRC.block<3, 3>(0, 0)(2, 2);

		ifs >> cam.mRC.block<3, 1>(0, 3)(0)
			>> cam.mRC.block<3, 1>(0, 3)(1)
			>> cam.mRC.block<3, 1>(0, 3)(2);

//		std::cout << cam.mRC << std::endl;

		cams.push_back(cam);
	}

	for (int i = 0; i < num_tps; i++)
	{
		tiePoint tp;

		ifs >> tp.pos[0]
			>> tp.pos[1]
			>> tp.pos[2];

		ifs >> tp.color[0]
			>> tp.color[1]
			>> tp.color[2];

		int nv = 0;
		ifs >> nv;

		for (int j = 0; j < nv; j++)
		{
			imagePoint ipt;

			ifs >> ipt.imgID >> ipt.keyID >> ipt.img_coord[0] >> ipt.img_coord[1];

			tp.viewList.push_back(ipt);
		}

		tps.push_back(tp);
	}

	ifs.close();

	return true;
}
