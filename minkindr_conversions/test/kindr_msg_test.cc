#include "minkindr_conversions/kindr_msg.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "testing_predicates.h"

namespace tf {

TEST(KindrMsgTest, poseKindrToMsgToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();
  Eigen::Vector3d position(Eigen::Vector3d::Random());
  kindr::minimal::QuatTransformation kindr_transform(rotation, position);

  geometry_msgs::Pose msg;
  poseKindrToMsg(kindr_transform, &msg);
  kindr::minimal::QuatTransformation output_transform;
  poseMsgToKindr(msg, &output_transform);

  EXPECT_NEAR_EIGEN(output_transform.getRotation().toImplementation().coeffs(),
                    rotation.coeffs(), 1e-6);
  EXPECT_NEAR_EIGEN(output_transform.getPosition(), position, 1e-6);
}

TEST(KindrMsgTest, transformKindrToMsgToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();
  Eigen::Vector3d position(Eigen::Vector3d::Random());
  kindr::minimal::QuatTransformation kindr_transform(rotation, position);

  geometry_msgs::Transform msg;
  transformKindrToMsg(kindr_transform, &msg);
  kindr::minimal::QuatTransformation output_transform;
  transformMsgToKindr(msg, &output_transform);

  EXPECT_NEAR_EIGEN(output_transform.getRotation().toImplementation().coeffs(),
                    rotation.coeffs(), 1e-6);
  EXPECT_NEAR_EIGEN(output_transform.getPosition(), position, 1e-6);
}

TEST(KindrMsgTest, quaternionKindrToMsgToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();

  geometry_msgs::Quaternion msg;
  quaternionKindrToMsg(rotation, &msg);
  Eigen::Quaterniond output_rotation;
  quaternionMsgToKindr(msg, &output_rotation);

  EXPECT_NEAR_EIGEN(output_rotation.coeffs(), rotation.coeffs(), 1e-6);
}

TEST(KindrMsgTest, vectorKindrToMsgToKindr) {
  Eigen::Vector3d position(Eigen::Vector3d::Random());

  geometry_msgs::Vector3 msg;
  vectorKindrToMsg(position, &msg);
  Eigen::Vector3d output_position;
  vectorMsgToKindr(msg, &output_position);

  EXPECT_NEAR_EIGEN(output_position, position, 1e-6);
}

}  // namespace tf

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
