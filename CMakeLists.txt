cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uncertain_pointcloud_msgs REQUIRED)   
find_package(realsense2 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)





include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${realsense_INCLUDE_DIR}
)


link_directories(
  include
)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
  )
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================
  
add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(realsense_d455
  src/realsense_d455/realsense_d455.cpp
  src/realsense_d455/codec.hpp
  src/realsense_d455/codec.cpp
  src/realsense_d455/realsense_d455-slam-node.hpp
  src/realsense_d455/realsense_d455-slam-node.cpp
)

target_link_libraries(realsense_d455 ${realsense2_LIBRARY})



ament_target_dependencies(realsense_d455 
rclcpp tf2 tf2_ros sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV uncertain_pointcloud_msgs realsense2  )

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(wrapped_realsense
  src/realsense_d455/wrapped_realsense_node.cpp
  src/realsense_d455/wrapped_realsense.cpp
)

target_link_libraries(wrapped_realsense ${realsense2_LIBRARY})

ament_target_dependencies(wrapped_realsense 
rclcpp tf2 tf2_ros sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV uncertain_pointcloud_msgs realsense2)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(simulation_rgbd
  src/simulation/simulation_rgbd_node.cpp
  src/simulation/simulation_rgbd.cpp
)

ament_target_dependencies(simulation_rgbd 
rclcpp tf2 tf2_ros sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV uncertain_pointcloud_msgs)


# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================


add_executable(simulation_rgbd_evaluator
  src/simulation/simulation_rgbd_evaluator_node.cpp
  src/simulation/simulation_rgbd_evaluator.cpp
)

ament_target_dependencies(simulation_rgbd_evaluator
rclcpp tf2 tf2_ros sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV uncertain_pointcloud_msgs visualization_msgs)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(fake_rgbd
  src/simulation/fake_rgbd_node.cpp
  src/simulation/fake_rgbd.cpp
)

ament_target_dependencies(fake_rgbd 
rclcpp tf2 tf2_ros sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV uncertain_pointcloud_msgs px4_msgs)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(stereo
src/stereo/stereo.cpp
src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV)
# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

add_executable(stereo-inertial
src/stereo-inertial/stereo-inertial.cpp
src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

# ==============================================================================================================================================
# ==============================================================================================================================================
# ==============================================================================================================================================

install(TARGETS mono rgbd realsense_d455 stereo stereo-inertial simulation_rgbd fake_rgbd  simulation_rgbd_evaluator DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    vocabulary/
  DESTINATION share/${PROJECT_NAME}/vocabulary
)

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

