#ifndef STUFF_CPP
#define STUFF_CPP

#include <math.h>
#include "simulator.h"
#include "Eigen/Core"
#include "Eigen/LU"

// computes the absolute value of the given input double
static double d_abs(double input){
	return (input < 0 ? -input : input);
}

static double normalizeAngle(double angle) {
	double absangle = (angle > 0 ? angle : -angle); // absolute value of angle

	// now, bring the angle in the interval [-2pi, 2pi]
	if (absangle > 2 * M_PI) {
		if (angle > 0) { // must decrease the value
			while (angle > 2 * M_PI) angle -= 2 * M_PI; // any better idea to do the module operation between two doubles?
		} else { // must increase the value
			while (angle < -2 * M_PI) angle += 2 * M_PI;
		}
	}

	// now, bring the angle in the [-pi, pi] interval
	absangle = (angle > 0 ? angle : -angle);
	if (absangle > M_PI) {
		(angle > 0 ? angle -= 2 * M_PI : angle += 2 * M_PI);
	}
	return angle;
}

static double computeAnglesDifference(double ang1, double ang2){
	if(d_abs(ang1-ang2) > M_PI){	// this passes through the discontinuous point π
		if(ang1 > 0){	// ang2 is negative
			return -(2*M_PI - (ang1-ang2));
		}
		else{	// ang1 negative, ang2 positive
			return (2*M_PI - (ang2-ang1));
		}
	}
	return (ang1-ang2);
}

static double computeAngle(Landmark* point, RobotPose* pose){
	double dx = point->x - pose->x;
	double dy = point->y - pose->y;
	double pure_angle = atan2(dy,dx);

	return computeAnglesDifference(pure_angle, pose->theta);
}

// RobotPose to Matrix
static Eigen::Matrix3d r2m(RobotPose rob){
  Eigen::Matrix3d ret;
  ret(0,0) = cos(rob.theta);
  ret(0,1) = -sin(rob.theta);
  ret(1,0) = sin(rob.theta);
  ret(1,1) = cos(rob.theta);
  ret(0,2) = rob.x;
  ret(1,2) = rob.y;
  ret(2,0) = 0;
  ret(2,1) = 0;
  ret(2,2) = 1;
  return ret;
}

// Matrix to RobotPose
static RobotPose m2r(Eigen::Matrix3d mat){
  RobotPose ret;
  ret.x = mat(0,2);
  ret.y = mat(1,2);
  ret.theta = atan2(mat(1,0), mat(0,0));
  return ret;
}

#endif // STUFF_CPP
