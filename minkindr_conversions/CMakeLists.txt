cmake_minimum_required(VERSION 2.8.3)
project(minkindr_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/kindr_msg.cc
  src/kindr_tf.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
