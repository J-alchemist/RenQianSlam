cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")
include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake) 

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(matching_node src/apps/matching_node.cpp ${ALL_SRCS})
add_dependencies(matching_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(matching_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})




