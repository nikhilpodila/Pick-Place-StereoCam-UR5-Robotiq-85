CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(ur5_pickplace_task)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig)

FIND_PACKAGE(catkin REQUIRED
  COMPONENTS
    std_srvs
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_ros_perception
    moveit_visual_tools
    rviz_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

# generate_messages(
#   DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs
# )

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    src
    launch
    include
    ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    std_srvs
  DEPENDS
    EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(pickplace
  src/pickplace_node.cpp
  src/PickPlaceTask.cpp
)

target_compile_features(pickplace INTERFACE cxx_std_11)
add_dependencies(pickplace
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pickplace ${catkin_LIBRARIES})

#############
## Install ##
#############
# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark executables and/or libraries for installation
install(
  TARGETS pickplace #${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

