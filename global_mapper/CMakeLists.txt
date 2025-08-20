cmake_minimum_required(VERSION 3.14.4)
project(global_mapper VERSION 0.0.0 LANGUAGES CXX)

find_package(ament_cmake      REQUIRED)
find_package(Eigen3           REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common)

# 1) Tell ament/cmake where our public headers live:
ament_export_include_directories(include)

#-------------------------------------------------------------------------------
# occupancy_grid
#-------------------------------------------------------------------------------
add_library(occupancy_grid SHARED src/occupancy_grid/occupancy_grid.cc)
target_include_directories(occupancy_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(occupancy_grid PUBLIC ${PCL_LIBRARIES})
install(
  TARGETS occupancy_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#-------------------------------------------------------------------------------
# distance_grid
#-------------------------------------------------------------------------------
add_library(distance_grid SHARED src/distance_grid/distance_grid.cc)
target_include_directories(distance_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(distance_grid PUBLIC occupancy_grid)
install(
  TARGETS distance_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#-------------------------------------------------------------------------------
# cost_grid
#-------------------------------------------------------------------------------
add_library(cost_grid SHARED src/cost_grid/cost_grid.cc)
target_include_directories(cost_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cost_grid PUBLIC distance_grid occupancy_grid)
install(
  TARGETS cost_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#-------------------------------------------------------------------------------
# temporal_grid
#-------------------------------------------------------------------------------
add_library(temporal_grid SHARED src/temporal_grid/temporal_grid.cc)
target_include_directories(temporal_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(temporal_grid PUBLIC ${PCL_LIBRARIES})
install(
  TARGETS temporal_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#-------------------------------------------------------------------------------
# global_mapper (top‐level)
#-------------------------------------------------------------------------------
add_library(global_mapper SHARED src/global_mapper/global_mapper.cc)
target_include_directories(global_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(global_mapper PUBLIC
  cost_grid
  ${PCL_LIBRARIES}
)
install(
  TARGETS global_mapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# 2) install all of our headers with one shot:
install(
  DIRECTORY include/
  DESTINATION include
)

# 3) export so downstream `find_package(global_mapper)` works:
ament_export_libraries(occupancy_grid
                       distance_grid
                       cost_grid
                       temporal_grid
                       global_mapper)
ament_export_dependencies(PCL Eigen3)
ament_package()
