cmake_minimum_required(VERSION 2.8.3)
project(nuscenes_add)  

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

target_link_libraries(bin2pcd
  ${catkin_LIBRARIES}
)


add_executable(nuscenes_add src/nuscenes_add.cpp)
target_link_libraries(nuscenes_add
  ${catkin_LIBRARIES}
)
