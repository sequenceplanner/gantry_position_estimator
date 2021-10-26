cmake_minimum_required(VERSION 3.5)
project(gantry_position_estimator)

find_package(ament_cmake REQUIRED)

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(sensor_msgs
          std_msgs
          std_srvs
          geometry_msgs
          tf2_msgs
          rcl
          rcl_action
          rmw_fastrtps_cpp       # (needed to build with RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
          FastRTPS               # (needed to build with RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
         )

# install binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros/colcon
ament_package()
