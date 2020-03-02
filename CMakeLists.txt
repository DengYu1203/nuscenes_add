cmake_minimum_required(VERSION 2.8.3)
project(nuscenes)  

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
#OpenCV
cv_bridge
#esr_msgs
conti_radar
image_transport
)
find_package( OpenCV REQUIRED )

find_package(PCL REQUIRED COMPONENTS)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME}
   src/bin2pcd.cpp
 )
add_executable(bin2pcd src/bin2pcd.cpp)


target_link_libraries(bin2pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(radar src/radar.cpp)


target_link_libraries(radar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(lidar src/lidar.cpp)


target_link_libraries(lidar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(json_read src/json_read.cpp)


target_link_libraries(json_read
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(combineSensor src/combineSensor.cpp)


target_link_libraries(combineSensor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(bag_sweeps src/bag_sweeps.cpp)


target_link_libraries(bag_sweeps
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(bag_samples src/bag_samples.cpp)


target_link_libraries(bag_samples
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(bag_test src/bag_test.cpp)


target_link_libraries(bag_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#add_dependencies (combineSensor esr_msge_gencpp)
add_dependencies (combineSensor conti_radare_gencpp)

add_executable(annotation src/annotation.cpp)


target_link_libraries(annotation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(bag_sample_camera src/bag_sample_camera.cpp)


target_link_libraries(bag_sample_camera
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_test src/image_test.cpp)


target_link_libraries(image_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_sub src/image_sub.cpp)


target_link_libraries(image_sub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(nuscenes_add src/nuscenes_add.cpp)


target_link_libraries(nuscenes_add
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)