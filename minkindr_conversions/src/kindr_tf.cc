#include "minkindr_conversions/kindr_tf.h"

#include <tf_conversions/tf_eigen.h>
#include <glog/logging.h>

namespace tf {

void poseKindrToTf(const kindr::minimal::QuatTransformation& kindr,
                   tf::Pose* tf_type) {
  transformKindrToTf(kindr, tf_type);
}

void poseTfToKindr(const tf::Pose& tf_type,
                   kindr::minimal::QuatTransformation* kindr) {
  transformTfToKindr(tf_type, kindr);
}

void transformKindrToTf(const kindr::minimal::QuatTransformation& kindr,
                        tf::Transform* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf::Vector3 origin;
  tf::Quaternion rotation;
  pointKindrToTf(kindr.getPosition(), &origin);
  quaternionKindrToTf(kindr.getRotation(), &rotation);
  tf_type->setOrigin(origin);
  tf_type->setRotation(rotation);
}

void transformTfToKindr(const tf::Transform& tf_type,
                        kindr::minimal::QuatTransformation* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;

  quaternionTfToKindr(tf_type.getRotation(), &rotation);
  pointTfToKindr(tf_type.getOrigin(), &position);

  *kindr = kindr::minimal::QuatTransformation(rotation, position);
}

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToTf(const kindr::minimal::RotationQuaternion& kindr,
                         tf::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  quaternionEigenToTF(kindr.toImplementation(), *tf_type);
}

void quaternionTfToKindr(const tf::Quaternion& tf_type,
                         kindr::minimal::RotationQuaternion* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond quat;
  quaternionTFToEigen(tf_type, quat);
  *kindr = kindr::minimal::RotationQuaternion(quat);
}

void quaternionKindrToTf(const  Eigen::Quaterniond& kindr,
                         tf::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  quaternionEigenToTF(kindr, *tf_type);
}

void quaternionTfToKindr(const tf::Quaternion& tf_type,
                         Eigen::Quaterniond* kindr) {
  CHECK_NOTNULL(kindr);
  quaternionTFToEigen(tf_type, *kindr);
}

void pointKindrToTf(const Eigen::Vector3d& kindr, tf::Vector3* tf_type) {
  CHECK_NOTNULL(tf_type);
  vectorEigenToTF(kindr, *tf_type);
}

void pointTfToKindr(const tf::Vector3& tf_type, Eigen::Vector3d* kindr) {
  CHECK_NOTNULL(kindr);
  vectorTFToEigen(tf_type, *kindr);
}

}  // namespace tf
