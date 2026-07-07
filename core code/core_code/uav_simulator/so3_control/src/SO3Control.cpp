#include <iostream>
#include <so3_control/SO3Control.h>

#include <ros/ros.h>

namespace
{
bool hasValidVectorCommand(const Eigen::Vector3d& command)
{
  return !(std::isnan(command(0)) || std::isnan(command(1)) || std::isnan(command(2)));
}

Eigen::Vector3d adaptiveAccelerationGain(const Eigen::Vector3d& total_error)
{
  return Eigen::Vector3d(fabs(total_error[0]) > 3 ? 0 : (fabs(total_error[0]) * 0.2),
                         fabs(total_error[1]) > 3 ? 0 : (fabs(total_error[1]) * 0.2),
                         fabs(total_error[2]) > 3 ? 0 : (fabs(total_error[2]) * 0.2));
}
} // namespace

SO3Control::SO3Control()
  : mass_(0.5)
  , g_(9.81)
{
  acc_.setZero();
}

void
SO3Control::setMass(const double mass)
{
  mass_ = mass;
}

void
SO3Control::setGravity(const double g)
{
  g_ = g;
}

void
SO3Control::setPosition(const Eigen::Vector3d& position)
{
  pos_ = position;
}

void
SO3Control::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

void
SO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)
{
  //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
  //           (des_vel - vel_).norm(), (des_acc - acc_).norm());

  bool flag_use_pos = hasValidVectorCommand(des_pos);
  bool flag_use_vel = hasValidVectorCommand(des_vel);
  bool flag_use_acc = hasValidVectorCommand(des_acc);

  Eigen::Vector3d totalError(Eigen::Vector3d::Zero());
  if ( flag_use_pos ) totalError.noalias() += des_pos - pos_;
  if ( flag_use_vel ) totalError.noalias() += des_vel - vel_;
  if ( flag_use_acc ) totalError.noalias() += des_acc - acc_;

  Eigen::Vector3d ka = adaptiveAccelerationGain(totalError);

  // std::cout << des_pos.transpose() << std::endl;
  // std::cout << des_vel.transpose() << std::endl;
  // std::cout << des_acc.transpose() << std::endl;
  // std::cout << des_yaw << std::endl;
  // std::cout << pos_.transpose() << std::endl;
  // std::cout << vel_.transpose() << std::endl;
  // std::cout << acc_.transpose() << std::endl;
  

  force_ = mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  if ( flag_use_pos ) force_.noalias() += kx.asDiagonal() * (des_pos - pos_);
  if ( flag_use_vel ) force_.noalias() += kv.asDiagonal() * (des_vel - vel_);
  if ( flag_use_acc ) force_.noalias() += mass_ * ka.asDiagonal() * (des_acc - acc_) + mass_ * (des_acc);

  // Limit control angle to 45 degree
  double          theta = M_PI / 2;
  double          c     = cos(theta);
  Eigen::Vector3d f;
  f.noalias() = force_ - mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
  {
    double nf        = f.norm();
    double A         = c * c * nf * nf - f(2) * f(2);
    double B         = 2 * (c * c - 1) * f(2) * mass_ * g_;
    double C         = (c * c - 1) * mass_ * mass_ * g_ * g_;
    double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  }
  // Limit control angle to 45 degree

  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

  if (force_.norm() > 1e-6)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;
  R << b1c, b2c, b3c;

  orientation_ = Eigen::Quaterniond(R);
}

const Eigen::Vector3d&
SO3Control::getComputedForce(void)
{
  return force_;
}

const Eigen::Quaterniond&
SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

void
SO3Control::setAcc(const Eigen::Vector3d& acc)
{
  acc_ = acc;
}
