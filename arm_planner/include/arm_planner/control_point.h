#ifndef CPOINT_HPP_
#define CPOINT_HPP_

#include "Eigen/Dense"

namespace arm_planner{

using namespace Eigen;

class ControlPoint
{
public:
	ControlPoint(){}
	ControlPoint(int link, double d, double r){this->link=link; this->d=d; this->radius=r; position(0)=0; position(1)=0; position(2)=0;}
	ControlPoint(double x, double y, double z, double r){position(0)=x; position(1)=y; position(2)=z;this->radius=r; link=0; d=0;}
	~ControlPoint(){};

	Vector3d position;
	double radius;
	int link;
	double d;
};

}

#endif
