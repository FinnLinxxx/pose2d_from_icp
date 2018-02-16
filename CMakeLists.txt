cmake_minimum_required(VERSION 2.8.3)
project(pose2d_from_icp)

## Find catkin and any catkin packages on which
## this package depends at build time

find_package(catkin REQUIRED COMPONENTS roscpp roslib laser_filters laser_geometry tf eigen_stl_containers eigen_conversions cmake_modules pcl_conversions )
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)
set(OpenCV_DIR "/opt/ros/kinetic/share/OpenCV-3.3.1")
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

## Declare things to be passed to other projects
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib laser_filters tf pcl_ros
    DEPENDS eigen 
)

add_compile_options(-std=c++11)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/laserscan_to_pointcloud_node.cpp src/matrixcalc.cpp src/fieldmap.cpp src/Colors.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})


#install execs
install(TARGETS
	${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the yaml files.
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml" 
)

