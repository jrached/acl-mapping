cmake_minimum_required(VERSION 3.14.4)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(global_mapper_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE FALSE)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# find system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fla_utils REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(global_mapper REQUIRED)
find_package(fla_interfaces REQUIRED)
find_package(snapstack_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanningGrids.msg"
  DEPENDENCIES std_msgs 
 )

ament_export_dependencies(rosidl_default_runtime)

# global_mapper_ros wrapper library
add_library(global_mapper_ros
  src/global_mapper_ros/global_mapper_ros.cc
  )

target_include_directories(global_mapper_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(global_mapper_ros global_mapper PCL)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS global_mapper_ros EXPORT export_global_mapper_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  )
ament_export_targets(export_global_mapper_ros)

# global_mapper_node
add_executable(global_mapper_node src/global_mapper_node.cc)
ament_target_dependencies(global_mapper_node rclcpp global_mapper_ros)

# install node
install(TARGETS global_mapper_node
  DESTINATION lib/${project_name}
  )

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