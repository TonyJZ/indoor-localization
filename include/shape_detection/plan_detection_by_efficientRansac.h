#pragma once
//use efficient_Ransac_v1.1


#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

template <typename PCLPointT>
int plane_detection_eRansac(const pcl::PointCloud<PCLPointT> &cloud, std::vector<int> *indices, bool has_normal,
	std::vector<std::vector<int>> &inliers, std::vector<pcl::ModelCoefficients> &planeCoeffs)
{
	//	using namespace MiscLib;
	eRansac::PointCloud pc;

	Vec3f cbbMin, cbbMax;
	cbbMin[0] = std::numeric_limits<float>::max();
	cbbMin[1] = std::numeric_limits<float>::max();
	cbbMin[2] = std::numeric_limits<float>::max();

	cbbMax[0] = std::numeric_limits<float>::lowest();
	cbbMax[1] = std::numeric_limits<float>::lowest();
	cbbMax[2] = std::numeric_limits<float>::lowest();

	size_t nSample = 0;
	if (indices)
		nSample = indices->size();
	else
		nSample = cloud.size();

	for (int i = 0; i < nSample; i++)
	{
		int id = i;
		if (indices)
			id = indices->at(i);

		eRansac::Point Pt;
		//default normal
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;

		Pt.pos[0] = static_cast<float>(cloud.points[id].x);
		Pt.pos[1] = static_cast<float>(cloud.points[id].y);
		Pt.pos[2] = static_cast<float>(cloud.points[id].z);

		if (has_normal)
		{
			Pt.normal[0] = static_cast<float>(cloud.points[id].normal_x);
			Pt.normal[1] = static_cast<float>(cloud.points[id].normal_y);
			Pt.normal[2] = static_cast<float>(cloud.points[id].normal_z);
		}
		pc.push_back(Pt);

		if (cbbMin[0] > Pt.pos[0])
			cbbMin[0] = Pt.pos[0];
		if (cbbMin[1] > Pt.pos[1])
			cbbMin[1] = Pt.pos[1];
		if (cbbMin[2] > Pt.pos[2])
			cbbMin[2] = Pt.pos[2];

		if (cbbMax[0] < Pt.pos[0])
			cbbMax[0] = Pt.pos[0];
		if (cbbMax[1] < Pt.pos[1])
			cbbMax[1] = Pt.pos[1];
		if (cbbMax[2] < Pt.pos[2])
			cbbMax[2] = Pt.pos[2];

	}

	// don't forget to set the bbox in pc
	pc.setBBox(cbbMin, cbbMax);

	const float scale = pc.getScale();

	if (!has_normal)
		pc.calcNormals(.01f * scale);


	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .01f * pc.getScale(); // set distance threshold to .01f of bounding box width
													// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
														  // NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 500; // this is the minimal numer of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

												 // set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());
	// 	detector.Add(new SpherePrimitiveShapeConstructor());
	// 	detector.Add(new CylinderPrimitiveShapeConstructor());
	// 	detector.Add(new ConePrimitiveShapeConstructor());
	// 	detector.Add(new TorusPrimitiveShapeConstructor());

	typedef std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > DetectedShape;
	MiscLib::Vector< DetectedShape > shapes; // stores the detected shapes
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection

	if (remaining == nSample)
	{
		printf("Segmentation failed! \n");
		return -1;
	}


	if (shapes.size() > 0)
	{
		for (MiscLib::Vector<DetectedShape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
		{
			const PrimitiveShape* shape = it->first;
			unsigned shapePointsCount = static_cast<unsigned>(it->second);


			std::string desc;
			shape->Description(&desc);

			if (shape->Identifier() != 0) //plane
			{
				continue;
			}

			//
			//shape->

			const PlanePrimitiveShape* plane = static_cast<const PlanePrimitiveShape*>(shape);
			Vec3f G = plane->Internal().getPosition();
			Vec3f N = plane->Internal().getNormal();
			//			Vec3f X = plane->getXDim();
			//			Vec3f Y = plane->getYDim();

			pcl::ModelCoefficients coef;
			coef.values.resize(4);
			coef.values[0] = N[0];
			coef.values[1] = N[1];
			coef.values[2] = N[2];
			coef.values[3] = -N.dot(G);

			planeCoeffs.push_back(coef);

		}

	}

	return 0;
}
