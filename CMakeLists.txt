cmake_minimum_required(VERSION 2.8.3)
project(topomap_localizer)
set(CMAKE_CXX_STANDARD 17)
add_definitions(-w)

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  inekf_msgs
  message_filters
  velodyne_pointcloud
  roslib
  dynamic_reconfigure
)
find_package(PCL 1.10 REQUIRED)
generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Eigen3
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    visualization_msgs
    inekf_msgs
)

##########################
######## Build ###########
##########################

include_directories(
  include
  external/cereal/include
  /usr/local/include/
  /usr/local/lib
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ${PROJECT_NAME}_node
add_executable(maplite_node
    src/maplite_node.cpp
    src/maplite.cpp
    src/kdtree.cpp
    src/particle_filter.cpp
)

add_dependencies(maplite_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# ${PROJECT_NAME}_node
target_link_libraries(maplite_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
)
