cmake_minimum_required(VERSION 2.8.3)
project(gnss_cal)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++11 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  gnss_comm
  message_generation
)

file(GLOB_RECURSE message_files ${PROJECT_SOURCE_DIR}/msg/*.msg)
add_message_files(
   FILES
   gnssCal.msg
   gnssToU.msg
   detect_planes.msg
   single_plane.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/gnss_cal_node.cpp 
                               )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GLOG_LIBRARIES})
