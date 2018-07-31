cmake_minimum_required(VERSION 2.8.3)
project(cv_bridge_cpp)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cv_bridge_cpp
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_converter src/image_converter.cpp)
add_dependencies(image_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} catkin_EXPORTED_TARGETS})
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############


#############
## Testing ##
#############
