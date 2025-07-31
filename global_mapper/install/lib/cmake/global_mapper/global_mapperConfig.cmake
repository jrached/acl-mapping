# Config file for the module example
# It defines the following variables:
# ${PROJECT_NAME}_INCLUDE_DIR  - Location of header files
# ${PROJECT_NAME}_INCLUDE_DIRS - All include directories needed to use ${PROJECT_NAME}
# ${PROJECT_NAME}_LIBRARY      - ${PROJECT_NAME} library
# ${PROJECT_NAME}_LIBRARIES    - ${PROJECT_NAME} library and all dependent libraries
# ${PROJECT_NAME}_DEFINITIONS  - Compiler definitions as semicolon separated list

find_library(global_mapper_LIBRARY global_mapper
  PATHS /home/kkondo/code/acl_mapping_ws/src/acl-mapping/global_mapper/install/lib
  NO_DEFAULT_PATH
  )

set(global_mapper_LIBRARIES ${global_mapper_LIBRARY}
  ${PCL_LIBRARIES}
  )

find_path(global_mapper_INCLUDE_DIR global_mapper/global_mapper.h
  PATHS /home/kkondo/code/acl_mapping_ws/src/acl-mapping/global_mapper/install/include
  NO_DEFAULT_PATH
  )

set(global_mapper_INCLUDE_DIRS ${global_mapper_INCLUDE_DIR}
  src/cost_grid
  src/distance_grid
  src/voxel_grid
  src/occupancy_grid
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  )

set(global_mapper_DEFINITIONS "-std=c++11")
