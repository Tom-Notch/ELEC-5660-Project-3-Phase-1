#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;
using namespace Eigen;

ros::Publisher odom_pub;

ros::Time tImuLastTime = ros::Time::ZERO;

/**
 * @brief [
 *          p     position x 3
 *          p_dot linear velocity x 3
 *          q     row pitch yaw, rotation by x, y, z axis
 *          b_a   accelerometer bias x 3
 *          b_g   gyroscope bias x 3
 *        ]
 *
 */
VectorXd vX = VectorXd::Zero(15);

MatrixXd Q = MatrixXd::Identity(15, 15);
MatrixXd Rt = MatrixXd::Identity(6, 6);

/**
 * @brief Generate a random Gaussian noise with respect to the given mean and standard deviation
 *
 * @param dMean
 * @param dStdDev
 * @return double
 */
double generateGaussianNoise(const double& dMean, const double& dStdDev)
{
  static mt19937 generator;
  normal_distribution<double> ndDist(dMean, dStdDev);
  return ndDist(generator);
}

/**
 * @brief Generate a random Gaussian noise with respect to the given covariance matrix assuming zero mean
 *
 * @param mCovariance
 * @return VectorXd
 */
VectorXd generateGaussianNoise(const MatrixXd& mCovariance)
{
  assert(mCovariance.cols() == mCovariance.rows());
  // cholesky decomposition A=LL^T. That's how Eigen named llt.
  Eigen::LLT<Eigen::MatrixXd> llt(mCovariance);
  if (llt.info() == Eigen::NumericalIssue)
  {
    ROS_ERROR("Lower Cholesky decomposition failed");
  }
  Eigen::MatrixXd L = llt.matrixL();

  // Note, Zero() returns an expression object, which is an rvalue reference.
  Eigen::VectorXd vNoise = Eigen::VectorXd::Zero(mCovariance.rows());
  for (unsigned int i = 0; i < vNoise.size(); ++i)
  {
    // construct vector of 3 normally distributed variables. Why?
    vNoise[i] = generateGaussianNoise(0, 1);
  }

  return L * vNoise;
}

/**
 * @brief Generate Quaternion from Euler angles in XYZ order
 *
 * @param vEulerAngles
 * @return Quaterniond
 */
Quaterniond euler2Quaternion(const Vector3d& vEulerAngles)
{
  return AngleAxisd(vEulerAngles(0), Vector3d::UnitX()) * AngleAxisd(vEulerAngles(1), Vector3d::UnitY()) * AngleAxisd(vEulerAngles(2), Vector3d::UnitZ());
}

/**
 * @brief Generate Euler angles from Quaternion in XYZ order
 *
 * @param q
 * @return VectorXd
 */
VectorXd quaternion2Euler(const Quaterniond& q)
{
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

VectorXd predictFunction(const VectorXd& vXLast, const VectorXd& vU, const VectorXd& vN, const double& dDt)
{
  VectorXd vX = vXLast;

  vX.segment<3>(0) = vXLast.segment<3>(0) + vXLast.segment<3>(3) * dDt + 0.5 * (vU.segment<3>(0) - vXLast.segment<3>(9)) * dDt * dDt + vN.segment<3>(0);  // update position
  vX.segment<3>(3) = vXLast.segment<3>(3) + (vU.segment<3>(0) - vXLast.segment<3>(9)) * dDt + vN.segment<3>(3);                                           // update linear velocity

  const Quaterniond& qLast = euler2Quaternion(vXLast.segment<3>(6));
  const Quaterniond& qImu = euler2Quaternion((vU.segment<3>(3) - vXLast.segment<3>(12)) * dDt);
  const Quaterniond& qN = euler2Quaternion(vN.segment<3>(6));

  Quaterniond q = qLast * qImu * qN;
  q.normalize();

  vX.segment<3>(6) = quaternion2Euler(q);  // update orientation

  vX.segment<3>(9) = vXLast.segment<3>(9) + vN.segment<3>(9);     // update accelerometer bias with random walk model
  vX.segment<3>(12) = vXLast.segment<3>(12) + vN.segment<3>(12);  // update gyroscope bias with random walk model

  return vX;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // your code for propagation

  // Calculate dt
  if (tImuLastTime.isZero())  // Process initial condition
  {
    tImuLastTime = msg->header.stamp;
    return;
  }
  ros::Time tImuCurTime = msg->header.stamp;
  double dDt = (tImuCurTime - tImuLastTime).toSec();
  tImuLastTime = tImuCurTime;

  // Get x_{t - 1}
  const VectorXd vXLast = vX;

  // Get u vector
  VectorXd vU(6);
  vU << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  // Generate random Gaussian noise with respect to Q
  VectorXd vN = generateGaussianNoise(Q);

  // Prediction
}

// Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //  your code for update
  //  camera position in the IMU frame = (0.05, 0.05, 0)
  //  camera orientation in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
  //					   RotationMatrix << 1,  0,  0,
  //							                 0, -1,  0,
  //                               0,  0, -1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf");
  ros::NodeHandle n("~");
  ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
  ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
  odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
  Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
  cout << "R_cam" << endl
       << Rcam << endl;
  // Q imu covariance matrix; Rt visual odomtry covariance matrix
  // You should also tune these parameters
  Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
  Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
  Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
  Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
  Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

  ros::spin();
}
