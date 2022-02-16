#pragma once

#include <string.h>
#include <vector>
#include <Eigen/Dense>
//#include <eigen/src/Core/Matrix.h>


//segment model type
enum sModelType
{
	sMT_undefined = 0,
	sMT_line,
	sMT_plane
};

const char unknownModelFlag[] = "undefined";
const char planeFlag[] = "plane";
const char lineFlag[] = "line";


typedef struct
{
	float  p[4];   //coordinates

	std::vector<int> connIndices; //connected model's ID
 
	float weight;    //the weight of intersection point. 
} iglIntersectionPoint;

//2D ͶӰ���ͳһ���ֵ
const float projected2D_zVal = 10.0; 

//based on bundle.out
typedef struct
{
	double f;
	double k1, k2;

	Eigen::Matrix<double, 4, 4> mRC; //��ת��ƽ��  ͶӰ���� P = KR[I  -C]�� KΪ���������

} simpleCameraModel;

typedef struct  
{
	int imgID;
	int keyID;
	double img_coord[2]; //ȥ��������������
} imagePoint;

typedef struct
{
	double pos[3];  //position
	int color[3]; //RGB
	std::vector<imagePoint> viewList;  //a list of views the point is visible in

} tiePoint;

