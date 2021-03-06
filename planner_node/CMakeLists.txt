cmake_minimum_required(VERSION 2.8.3)
project(planner_node)

set (CMAKE_CXX_STANDARD 11)
#add_compile()
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs rospy std_msgs geometry_msgs nav_msgs)


add_message_files(
  FILES
  planner_pub.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs geometry_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(planner src/planner.cpp src/kdTree.cpp src/RRT.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner planner_node_generate_messages_cpp)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(planner ${OCTOMAP_LIBRARIES})
#find_package(octomap_ros REQUIRED)

#add_library(rrtLib src/kdTree.cpp src/RRT.cpp)
#target_link_libraries(rrtLib ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})


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
