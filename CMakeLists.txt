cmake_minimum_required(VERSION 3.28)
project(pairs_uav_testing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  pairs_lib
  pairs_msgs
  tf2_ros
  backward_ros
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

add_library(PairsUavTesting_PairsUavTesting)
add_library(pairs_uav_testing::pairs_uav_testing ALIAS PairsUavTesting_PairsUavTesting)
set_target_properties(PairsUavTesting_PairsUavTesting
  PROPERTIES
    EXPORT_NAME pairs_uav_testing
    OUTPUT_NAME pairs_uav_testing
)

target_sources(PairsUavTesting_PairsUavTesting
  PRIVATE
    "src/test_generic.cpp"
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/pairs_uav_testing/test_generic.h"
)

target_link_libraries(PairsUavTesting_PairsUavTesting
  PUBLIC
    rclcpp::rclcpp
    pairs_lib::pairs_lib
    tf2_ros::tf2_ros
    ${pairs_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  if(NOT DEFINED ENV{IN_NIX_SHELL})
    message(WARNING "Compiling lint tests")
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
    find_package(ament_cmake_xmllint REQUIRED)
    ament_xmllint()
  endif()

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS PairsUavTesting_PairsUavTesting
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  PairsUavTesting_PairsUavTesting
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
