#ifndef __BASIC_GEOMETRY_HPP_
#define __BASIC_GEOMETRY_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>

namespace Geometry{


/**
 * @brief Creates a rotation matrix from roll, pitch, and yaw angles.
 * 
 * \param roll The roll angle (in radian) around the X-axis.
 * \param pitch The pitch angle (in radian) around the Y-axis.
 * \param yaw The yaw angle (in radian) around the Z-axis.
 * 
 * \return A 3x3 rotation matrix representing the combined rotation.
 */
const Eigen::Matrix3d createRotationMatrix(double roll, double pitch, double yaw);


/**
 * @brief Creates a 4x4 Transformation matrix combining rotation and translation.
 * 
 * @param roll The roll angle (in radian) around the X-axis.
 * @param pitch The pitch angle (in radian) around the Y-axis.
 * @param yaw The yaw angle (in radian) around the Z-axis.
 * @param translation A 3D vector representing the translation in the X, Y, and Z directions.
 * 
 * @return A 4x4 transformation matrix that includes both the rotation and translation.
 */
Eigen::Matrix4d createTransformationMatrix(double roll, double pitch, double yaw, const Eigen::Vector3d& translation);


}



#endif