cmake_minimum_required(VERSION 2.8.3)
project(map_server_pcd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g ")


find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  cloud_msgs
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_executable(map_server_pcd
    src/map_server_pcd.cpp)

add_dependencies(map_server_pcd ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(map_server_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS})

