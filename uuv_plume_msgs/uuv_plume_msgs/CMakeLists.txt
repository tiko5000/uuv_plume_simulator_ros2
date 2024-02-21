cmake_minimum_required(VERSION 3.8)
project(uuv_plume_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ParticleConcentration.msg"
    "msg/Salinity.msg"
    "srv/CreatePassiveScalarTurbulentPlume.srv"
    "srv/CreateSpheroidPlume.srv"
    "srv/SetPlumeLimits.srv"
    "srv/SetPlumeConfiguration.srv"
    "srv/GetPlumeConfiguration.srv"
    "srv/DeletePlume.srv"
    "srv/SetPlumeSourcePosition.srv"
    "srv/GetPlumeSourcePosition.srv"
    "srv/GetNumParticles.srv"
    "srv/StorePlumeState.srv"
    "srv/LoadPlumeParticles.srv"
    "srv/SetCurrentDirection.srv"
    "srv/SetCurrentModel.srv"
    "srv/GetCurrentModel.srv"
    "srv/SetCurrentVelocity.srv"
    DEPENDENCIES geometry_msgs std_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()