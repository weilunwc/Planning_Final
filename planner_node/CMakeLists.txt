cmake_minimum_required(VERSION 2.8.3)
project(planner_node)
set (CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs rospy std_msgs)


find_path(FCL_INCLUDE_DIRS
NAMES fcl/collision.h
HINTS ${PC_FCL_INCLUDEDIR}
PATHS "${CMAKE_INSTALL_PREFIX}/include")

add_message_files(
  FILES
  planner_pub.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner planner_node_generate_messages_cpp)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(planner ${OCTOMAP_LIBRARIES})
#find_package(octomap_ros REQUIRED)

#adding route for fcl
find_package(fcl REQUIRED)
include_directories(${fcl_INCLUDE_DIRS})
target_link_libraries(planner ${fcl_LIBRARIES})

#adding route for boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(planner ${Boost_LIBRARIES})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories ( "$ENV{EIGEN3_INCLUDE_DIR}" )
#INCLUDE_DIRECTORIES ( /usr/include/eigen3 )
#find_package (Eigen3 REQUIRED NO_MODULE)
#target_link_libraries (planner Eigen3::Eigen)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


install(TARGETS planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
