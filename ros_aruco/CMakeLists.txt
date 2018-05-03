cmake_minimum_required(VERSION 2.8.3)
project(ros_aruco)

add_definitions(-std=c++11)
find_package(catkin REQUIRED
  cv_bridge
  image_transport
  image_geometry
  roscpp
  ros_aruco_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_detector_node src/nodes/marker_detector_node.cpp)
target_link_libraries(marker_detector_node
  ${catkin_LIBRARIES}
)

add_executable(board_generator src/board_generator.cpp)
target_link_libraries(board_generator
  ${catkin_LIBRARIES}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

