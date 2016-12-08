cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  image_transport
  pods
  roscpp
  sensor_msgs
  tf
)

project(perception_ros_lcm_translators)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    image_transport
    pods
    roscpp
    sensor_msgs
    tf
  DEPENDS opencv lcm
)

###########
## Build ##
###########

set(TARGETS
    ros2lcm_kinect
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros2lcm_kinect src/ros2lcm_kinect.cpp)
target_link_libraries(ros2lcm_kinect ${catkin_LIBRARIES} z)
pods_use_pkg_config_packages(ros2lcm_kinect lcm bot2-core opencv)

#############
## Install ##
#############

install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
