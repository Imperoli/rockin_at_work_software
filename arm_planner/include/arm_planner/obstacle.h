#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

#include "Eigen/Dense"

namespace arm_planner{

using namespace Eigen;

class Obstacle
{
public:
	Obstacle(){}
	Obstacle(Vector3d pos, double r){position=pos; radius=r;}
	Obstacle(double x, double y, double z, double r){position(0)=x; position(1)=y; position(2)=z; radius=r;}
	Obstacle(Vector3d min_pos, Vector3d max_pos){min_position=min_pos; max_position=max_pos; radius=-1;}
	~Obstacle(){}

	Vector3d position;
	Vector3d min_position;
	Vector3d max_position;
	Vector3d normal;
	double radius;
};

}

#endif
