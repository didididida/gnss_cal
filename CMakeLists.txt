cmake_minimum_required(VERSION 2.8.3)
project(gnss_cal)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++17 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  gnss_comm
  message_generation
  pcl_ros
  geodesy
  pcl_conversions
  eigen_conversions
  message_filters
  std_srvs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/algorithmParameters.cfg
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

add_executable(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/src/gnss_cal_node.cpp )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${GLOG_LIBRARIES} )

add_executable(check_plane_node ${PROJECT_SOURCE_DIR}/src/plane_check_node.cpp )
target_link_libraries(check_plane_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES})



add_executable(ekf_node ${PROJECT_SOURCE_DIR}/src/ekf_filter_node.cpp 
                        ${PROJECT_SOURCE_DIR}/src/ekf.cpp 
                        ${PROJECT_SOURCE_DIR}/src/ekf_filter.cpp)

target_link_libraries(ekf_node ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR})


add_executable(test_node ${PROJECT_SOURCE_DIR}/src/test.cpp 
                   ${PROJECT_SOURCE_DIR}/src/satellite_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/plane_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/gnss_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/imu_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/particle_filter.cpp
                   ${PROJECT_SOURCE_DIR}/src/gps_publisher.cpp
                   ${PROJECT_SOURCE_DIR}/src/imu_publisher.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES}  ${EIGEN3_INCLUDE_DIR})

add_executable(sync_node ${PROJECT_SOURCE_DIR}/src/sync_node.cpp 
                   ${PROJECT_SOURCE_DIR}/src/sync_test.cpp 
                   ${PROJECT_SOURCE_DIR}/src/satellite_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/plane_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/gnss_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/imu_subscriber.cpp
                   ${PROJECT_SOURCE_DIR}/src/particle_filter.cpp
                   ${PROJECT_SOURCE_DIR}/src/gps_publisher.cpp
                   ${PROJECT_SOURCE_DIR}/src/imu_publisher.cpp)
target_link_libraries(sync_node ${catkin_LIBRARIES}  ${EIGEN3_INCLUDE_DIR})
