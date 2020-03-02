cmake_minimum_required(VERSION 2.8.3)
project(nuscenes_add)  

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
nuscenes_pose
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(nuscenes_addition src/nuscenes_add.cpp)
target_link_libraries(nuscenes_addition
  ${catkin_LIBRARIES}
)
