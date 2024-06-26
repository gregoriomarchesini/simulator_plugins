cmake_minimum_required(VERSION 3.8)
project(simulator_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils)          
find_package(simulator_msgs REQUIRED)  


# directories used by rclcpp package
include_directories(include ${rclcpp_INCLUDE_DIRS})


add_library(cw_model_plugin 
            SHARED src/relative_acceleration_cw_plugin.cpp)


target_include_directories(cw_model_plugin  PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)


# specify the dependancies
ament_target_dependencies(cw_model_plugin
                          rclcpp
                          gazebo_ros
                          gazebo
                          geometry_msgs
                          gazebo_msgs
                          simulator_msgs)


ament_export_libraries(cw_model_plugin)


# This command makes sure that the gazebo plug in is available in the lib and bin folder in the build directory
install(
    TARGETS cw_model_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
