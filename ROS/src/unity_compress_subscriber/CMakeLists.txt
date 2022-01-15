cmake_minimum_required(VERSION 2.8.3)
project(unity_compress_subscriber)

## Compile as C++14 for pcl
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs 
  geometry_msgs
  pcl_ros
  pcl_conversions
  message_filters  
  tf2
  tf2_ros
  std_msgs
  compress_depth_image
  compress_image
  depth_image_to_pc2
  image_geometry
  rosparam_shortcuts
)

find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  include
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(unity_compress_subscriber nodes/unity_compress_subscriber.cpp)
target_link_libraries(unity_compress_subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(unity_compress_subscriber ${catkin_EXPORTED_TARGETS})
