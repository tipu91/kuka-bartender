// Author: Enrico Corvaglia
// skew_symmetric() takes a vector as input and apply it the skew-symmetric operator
// returns the related skew-symmetric matrix

#ifndef SKEW_SYMMETRIC_H
#define SKEW_SYMMETRIC_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>


inline void skew_symmetric(KDL::Vector &v_, Eigen::Matrix<double,3,3> &skew_mat_)
{
	skew_mat_ = Eigen::Matrix<double,3,3>::Zero();
	
	skew_mat_(0,1) = -v_(2);
	skew_mat_(0,2) =  v_(1);
	skew_mat_(1,0) =  v_(2);
	skew_mat_(1,2) = -v_(0);
	skew_mat_(2,0) = -v_(1);
	skew_mat_(2,1) =  v_(0);
}

inline void EulerToQuaternion(float R, float P, float Y)
{
  static double q_[4];
  double t0 = std::cos((3.142 * Y/ 180)*0.5f);
  double t1 = std::sin((3.142 * Y/ 180)*0.5f);
  double t2 = std::cos((3.142 * R/ 180)*0.5f);
  double t3 = std::sin((3.142 * R/ 180)*0.5f);
  double t4 = std::cos((3.142 * P/ 180)*0.5f);
  double t5 = std::sin((3.142 * P/ 180)*0.5f);

  q_[0] = t0 * t2 * t4 + t1 * t3 * t5;
  q_[1] = t0 * t3 * t4 - t1 * t2 * t5;
  q_[2] = t0 * t2 * t5 + t1 * t3 * t4;
  q_[3] = t1 * t2 * t4 - t0 * t3 * t5;

  return q_;
}

inline void QuaternionToEuler(double q1, double q2, double q3, double q4)
{
	static float R, P, Y;
	// roll (x-axis rotation)
	double t0 = +2.0 * (q4* q1 + q2 * q3);
	double t1 = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
	R = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q4* q2 - q3 * q1;
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	P = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q4 * q3 + q1 * q2);
	double t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);  
	Y = std::atan2(t3, t4);

	return double angles[3] = {R, P, Y};

}

#endif