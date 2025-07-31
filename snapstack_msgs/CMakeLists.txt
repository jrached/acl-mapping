cmake_minimum_required(VERSION 3.14.4)
set(CMAKE_CXX_STANDARD 17)
project(snapstack_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED) # not sure if this is actually necessary, i dont see any services in the repo...

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttitudeCommand.msg"
  "msg/CommAge.msg"
  "msg/ControlLog.msg"
  "msg/Goal.msg"
  "msg/IMU.msg"
  "msg/Motors.msg"
  "msg/QuadFlightMode.msg"
  "msg/SMCData.msg"
  "msg/State.msg"
  "msg/TimeFilter.msg"
  "msg/VioFilterState.msg"
  DEPENDENCIES std_msgs nav_msgs std_srvs builtin_interfaces geometry_msgs 
 )

ament_export_dependencies(rosidl_default_runtime)

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
