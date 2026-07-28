cmake_minimum_required(VERSION 3.8)
project(m3t_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# M3T + its transitive dependencies (the imported target references these)
find_package(Eigen3 REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(M3T REQUIRED)

set(ROS_DEPS
  rclcpp sensor_msgs geometry_msgs tf2_ros tf2_eigen
  visualization_msgs cv_bridge image_transport)

add_executable(m3t_tracker_node src/tracker_node.cpp)
target_include_directories(m3t_tracker_node PRIVATE include)
ament_target_dependencies(m3t_tracker_node ${ROS_DEPS})
target_link_libraries(m3t_tracker_node m3t)

install(TARGETS m3t_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
