cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nodelet
  roscpp
  std_msgs
  tf
  cv_bridge
  nav_msgs
  image_transport
)
find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  geometry_msgs
  nodelet
  roscpp
  std_msgs
  tf
  DEPENDS
  EIGEN3
)

# include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
  include/multi_robot_tracking
)

add_library(multi_robot_tracking src/hungarian_alg.cpp src/SimpleKalman.cpp  src/track.cpp src/jpdaf_filter.cpp src/phd_filter.cpp src/kalman.cpp src/multi_robot_tracking_nodelet.cpp)
target_include_directories(multi_robot_tracking PUBLIC include ${catkin_LIBRARIES})

target_link_libraries(multi_robot_tracking PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(multi_robot_tracking ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(multi_robot_tracking_node src/multi_robot_tracking_node.cpp)
add_dependencies(multi_robot_tracking_node ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(multi_robot_tracking_node multi_robot_tracking ${catkin_LIBRARIES})


install(
  TARGETS multi_robot_tracking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
