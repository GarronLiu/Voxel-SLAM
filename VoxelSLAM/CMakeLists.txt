cmake_minimum_required(VERSION 3.0.2)
project(voxel_slam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver2
  gnss_comm
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Glog REQUIRED)
# find_package(Ceres REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs gnss_comm
  DEPENDS EIGEN3 PCL GTSAM
  INCLUDE_DIRS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIR}
  # ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(voxelslam src/voxelslam.cpp src/BTC.cpp src/tools.cpp include/GNSS_Assignment.cpp include/GNSS_Initialization.cpp include/GNSS_Processing_fg.cpp)
target_link_libraries(voxelslam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_LIBRARIES} glog gtsam)


