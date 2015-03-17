#ifndef MINKINDR_CONVERSIONS_KINDR_TF_H
#define MINKINDR_CONVERSIONS_KINDR_TF_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <tf/transform_datatypes.h>

namespace tf {

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
void poseKindrToTf(const kindr::minimal::QuatTransformation& kindr,
                   tf::Pose* tf_type);
void poseTfToKindr(const tf::Pose& tf_type,
                   kindr::minimal::QuatTransformation* kindr);

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::Transform.
void transformKindrToTf(const kindr::minimal::QuatTransformation& kindr,
                        tf::Transform* tf_type);
void transformTfToKindr(const tf::Transform& tf_type,
                        kindr::minimal::QuatTransformation* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToTf(const kindr::minimal::RotationQuaternion& kindr,
                         tf::Quaternion* tf_type);
void quaternionTfToKindr(const tf::Quaternion& tf_type,
                         kindr::minimal::RotationQuaternion* kindr);
// Also the Eigen implementation version of this.
void quaternionKindrToTf(const  Eigen::Quaterniond& kindr,
                         tf::Quaternion* tf_type);
void quaternionTfToKindr(const tf::Quaternion& tf_type,
                         Eigen::Quaterniond* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void pointKindrToTf(const Eigen::Vector3d& kindr, tf::Vector3* tf_type);
void pointTfToKindr(const tf::Vector3& tf_type, Eigen::Vector3d* kindr);

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_TF_H
