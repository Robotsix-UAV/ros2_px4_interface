#include <Eigen/Core>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>


tf2::Vector3 eigenNedToTf2Nwu(const Eigen::Vector3f & eigen_vector)
{
  return tf2::Vector3(eigen_vector.x(), -eigen_vector.y(), -eigen_vector.z());
}


Eigen::Vector3f tf2FwuToEigenNed(const tf2::Vector3 & tf2_vector)
{
  return Eigen::Vector3f(tf2_vector.x(), -tf2_vector.y(), -tf2_vector.z());
}


tf2::Quaternion eigenNedToTf2Nwu(const Eigen::Quaternionf & eigen_quaternion)
{
  return tf2::Quaternion(
    eigen_quaternion.x(), -eigen_quaternion.y(),
    -eigen_quaternion.z(), eigen_quaternion.w());
}


Eigen::Quaternionf tf2FwuToEigenNed(const tf2::Quaternion & tf2_quaternion)
{
  return Eigen::Quaternionf(
    tf2_quaternion.w(), tf2_quaternion.x(),
    -tf2_quaternion.y(), -tf2_quaternion.z());
}
