#pragma once

#include <math.h>

#include <Eigen/Dense>

/* 计算两个向量的旋转角，是指从向量v1开始，逆时针旋转，转到向量v2时，所转过的角度，范围是 0 ~360度
* 算法如下：
* 首先通过点乘和arccosine的得到两个向量之间的夹角
* 然后判断通过差乘来判断两个向量之间的位置关系
* 如果v2在v1的逆时针方向, 返回arccose的角度值, 范围0 ~180.0(根据右手定理, 可以构成正的面积)
* 否则返回 360.0 - arecose的值, 返回180到360(根据右手定理, 面积为负)
* 返回弧度值
*/
double get2DRotateAngle(Eigen::Vector2f v1, Eigen::Vector2f v2)
{
	const double epsilon = 1.0e-6;
	const double nyPI = acos(-1.0);
	double angle;

	double dd = v1.dot(v2);

	if (fabs(dd - 1.0) <= epsilon)
		angle = 0.0;
	else if (fabs(dd + 1.0) <= epsilon)
		angle = nyPI;
	else
	{
		//Eigen::Vector2f cross;

		angle = acos(dd);
		//cross product
		//cross = v1.cross(v2);
		double cross = v1[0] * v2[1] - v1[1] * v2[0];
		// vector p2 is clockwise from vector p1 
		// with respect to the origin (0.0)
		if (cross < 0) {
			angle = 2 * nyPI - angle;
		}
	}

//	degree = angle *  180.0 / nyPI;
	return angle;
}

void RotateMat_X(double angle, Eigen::Matrix3d &Mat)
{
	Mat = Eigen::Matrix3d::Identity();
	
	Mat(1, 1) = cos(angle);
	Mat(1, 2) = sin(angle);
	Mat(2, 1) = -sin(angle);
	Mat(2, 2) = cos(angle);
}

void RotateMat_Y(double angle, Eigen::Matrix3d &Mat)
{
	Mat = Eigen::Matrix3d::Identity();

	Mat(0, 0) = cos(angle);
	Mat(0, 2) = -sin(angle);
	Mat(2, 0) = sin(angle);
	Mat(2, 2) = cos(angle);
}

void RotateMat_Z(double angle, Eigen::Matrix3d &Mat)
{
	Mat = Eigen::Matrix3d::Identity();

	Mat(0, 0) = cos(angle);
	Mat(0, 1) = sin(angle);
	Mat(1, 0) = -sin(angle);
	Mat(1, 1) = cos(angle);
}

void RotateMat_XYZ(double alpha, double beta, double gamma, Eigen::Matrix3d &Mat)
{
	Eigen::Matrix3d matX, matY, matZ;
	RotateMat_X(alpha, matX);
	RotateMat_Y(beta, matY);
	RotateMat_Z(gamma, matZ);

	Mat = matX*matY*matZ;
}

