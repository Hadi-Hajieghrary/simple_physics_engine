#include <basic_geometry.hpp>

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
const Eigen::Matrix3d createRotationMatrix(double roll, double pitch, double yaw) {
    // Rotation matrix for roll (rotation around the X-axis)
    Eigen::Matrix3d R_roll;
    R_roll << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);

    // Rotation matrix for pitch (rotation around the Y-axis)
    Eigen::Matrix3d R_pitch;
    R_pitch << cos(pitch), 0, sin(pitch),
               0, 1, 0,
               -sin(pitch), 0, cos(pitch);

    // Rotation matrix for yaw (rotation around the Z-axis)
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0,
             sin(yaw), cos(yaw), 0,
             0, 0, 1;

    // Combine rotations in Z-Y-X order (Yaw -> Pitch -> Roll)
    Eigen::Matrix3d rotationMatrix = R_yaw * R_pitch * R_roll;

    return rotationMatrix;
}


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
Eigen::Matrix4d createTransformationMatrix(double roll, double pitch, double yaw, const Eigen::Vector3d& translation) {
    // Create the rotation matrix using the given roll, pitch, yaw angles
    Eigen::Matrix3d rotation_matrix = createRotationMatrix(roll, pitch, yaw);

    // Create the 4x4 transformation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();  // Start with identity matrix

    // Set the upper-left 3x3 part to the rotation matrix
    transformation_matrix.block<3,3>(0,0) = rotation_matrix;

    // Set the translation part (first three elements of the last column)
    transformation_matrix.block<3,1>(0,3) = translation;

    return transformation_matrix;
}



} // namespace Geometry
