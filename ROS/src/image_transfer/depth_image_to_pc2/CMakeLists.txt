cmake_minimum_required(VERSION 3.0.2)
project(depth_image_to_pc2)

find_package(catkin REQUIRED
  cmake_modules
  image_geometry
  sensor_msgs
)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
find_package(OpenCV REQUIRED)
include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/point_cloud_xyz.cpp 
                             #src/convert_metric.cpp
                             #src/crop_foremost.cpp
                             #src/register.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)