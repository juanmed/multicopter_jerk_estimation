#include "jerk_estimator.h"

JerkEstimator::JerkEstimator(double mass, double gravity, double drag_coeff)
{
	mass_ = mass;
	gravity_ = gravity;
	drag_coeff_ = drag_coeff;
	gamma_ = 2.0 * drag_coeff_ * std::sqrt(mass_ * gravity_);
	bias_initialized_ = false;

	e1_ << 1., 0., 0.;
	e2_ << 0., 1., 0.;
	e3_ << 0., 0., 1.;

	ba_ << 0., 0., 0.;
	bg_ << 0., 0., 0.;
}

Eigen::Vector3d JerkEstimator::computeJerkEstimation(const Eigen::Quaterniond q, const Eigen::Vector3d a_imu,
	const Eigen::Vector3d angular_velocity_imu, double thrust, double thrust_dot)
{
	// from body to world rotation
	Eigen::Matrix3d Rbw = q.toRotationMatrix();	
	Eigen::Vector3d j, a, zbw, ybw, xbw, angular_velocity = Eigen::Vector3d::Zero();

	zbw = Rbw * e3_; 
	ybw = Rbw * e2_;
	xbw = Rbw * e1_;

	a = Rbw * (a_imu - ba_) - gravity_ * e3_;
	angular_velocity = angular_velocity_imu - bg_;
	j = (thrust_dot * zbw + thrust * (angular_velocity(1) * xbw - angular_velocity(0) * ybw) - gamma_ * a) / mass_ ;
	return j;
}

void JerkEstimator::setBiases(const Eigen::Vector3d ba, const Eigen::Vector3d bg)
{
	ba_ = ba;
	bg_ = bg;
	if(!bias_initialized_){
		bias_initialized_ = true;
		std::cout << "Jerk Estimator Initialized!" << std::endl;
	}
}

bool JerkEstimator::initialized()
{
	return bias_initialized_;
}

JerkEstimator::~JerkEstimator()
{
  //Destructor
}