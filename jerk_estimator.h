#ifndef JERKESTIMATOR_V12020_H
#define JERKESTIMATOR_V12020_H

#include <iostream>
#include <Eigen/Geometry>

class JerkEstimator
{
public:
	JerkEstimator(double mass, double gravity, double drag_coeff);
	virtual ~ JerkEstimator();
	Eigen::Vector3d computeJerkEstimation(const Eigen::Quaterniond q, const Eigen::Vector3d a_imu,
		const Eigen::Vector3d angular_velocity_imu, double thrust, double thrust_dot);
private:

	double mass_, gravity_, drag_coeff_, gamma_ = 0.0;

	// basis vector generating R^3
	Eigen::Vector3d e1_;
	Eigen::Vector3d e2_;
	Eigen::Vector3d e3_;
};

#endif /* JERKESTIMATOR_V12020_H */