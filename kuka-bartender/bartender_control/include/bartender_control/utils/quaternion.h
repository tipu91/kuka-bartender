// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose between damped and not)
// returns the pseudo inverted matrix M_pinv_

#ifndef QUATERNION_H
#define QUATERNION_H

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

}

/*inline void bamba_symmetric(KDL::Vector &v_, Eigen::Matrix<double,3,3> &bamba_mat_)
{
  bamba_mat_ = Eigen::Matrix<double,3,3>::Zero();
  
  bamba_mat_(0,1) = -v_(2);
  bamba_mat_(0,2) =  v_(1);
  bamba_mat_(1,0) =  v_(2);
  bamba_mat_(1,2) = -v_(0);
  bamba_mat_(2,0) = -v_(1);
  bamba_mat_(2,1) =  v_(0);
}*/


#endif
