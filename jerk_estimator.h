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
		const Eigen::Vector3d angular_velocity_imu, const Eigen::Vector3d v, double thrust, 
		double thrust_dot);
	void setBiases(const Eigen::Vector3d ba, const Eigen::Vector3d bg);
	bool initialized();
private:

	double mass_, gravity_, drag_coeff_, gamma_ = 0.0;
	bool bias_initialized_;
	const double k_ = 0.2;
	const double b_ = 1.1;
	const int n_ = 4;

	// basis vector generating R^3
	Eigen::Vector3d e1_;
	Eigen::Vector3d e2_;
	Eigen::Vector3d e3_;

	// IMU biases
	Eigen::Vector3d ba_;
	Eigen::Vector3d bg_;

	double computeGamma(double thrust);
	double computeGammaDot(double thrust_dot);
};

#endif /* JERKESTIMATOR_V12020_H */