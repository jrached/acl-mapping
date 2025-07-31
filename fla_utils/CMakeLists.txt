cmake_minimum_required(VERSION 3.14.4)
set(CMAKE_CXX_STANDARD 11)
project(fla_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(fla_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# catkin_package(INCLUDE_DIRS include
# )

# include_directories(include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   ${OpenCV_INCLUDE_DIRS}
#   )

# target_include_directories(Eigen3 PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# target_include_directories(OpenCV PUBLIC
#    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
ament_target_dependencies(laserscan_to_pointcloud rclcpp sensor_msgs fla_interfaces laser_geometry)


# add_executable(quat_to_rpy src/quat_to_rpy.cpp)
# add_dependencies(quat_to_rpy mavros_msgs_generate_messages_cpp)
# target_link_libraries(quat_to_rpy ${catkin_LIBRARIES})

install(TARGETS 
  laserscan_to_pointcloud
  DESTINATION lib/${PROJECT_NAME}
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