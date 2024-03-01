cmake_minimum_required(VERSION 3.8)
project(uuv_cpc_sensor)

# ## Compile as C++11, supported in ROS Kinetic and newer
# set(CMAKE_CXX_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuv_plume_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
# Hint from: https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/CMakeLists.txt
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)


# ## Build libraries
# add_library(uuv_cpc_ros_sensor SHARED
#   src/CPCSensor.cc
# )
# ament_target_dependencies(uuv_cpc_ros_sensor
#   rclcpp
#   std_msgs
#   nav_msgs
#   sensor_msgs
#   geometry_msgs
#   uuv_plume_msgs
# )

## Build executables
add_executable(uuv_cpc_ros_sensor_node
  src/CPCSensorNode.cc
)
ament_target_dependencies(uuv_cpc_ros_sensor_node
  rclcpp
  rcpputils
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_msgs
  uuv_plume_msgs
  GeographicLib
)
# target_link_libraries(uuv_cpc_ros_sensor_node
#   uuv_cpc_ros_sensor
# )

## Install targets
install(TARGETS
  # uuv_cpc_ros_sensor
  uuv_cpc_ros_sensor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".~" EXCLUDE
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  uuv_plume_msgs
)
ament_package()



