cmake_minimum_required(VERSION 3.16)
project(bxi_octo_global_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# External OctoPlanner3D source tree used for PCD-to-OctoMap conversion and
# 3D global planning. It defaults to the vendored third_party copy.
set(OCTO_PLANNER3D_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/OctoPlanner3D-ROS2"
  CACHE PATH "Path to the OctoPlanner3D-ROS2 third_party tree")

if(NOT EXISTS "${OCTO_PLANNER3D_ROOT}/planner/include/global_planner.h")
  message(FATAL_ERROR "OCTO_PLANNER3D_ROOT does not point to OctoPlanner3D-ROS2: ${OCTO_PLANNER3D_ROOT}")
endif()

# Strip the original build machine's RUNPATH from private copies of the
# bundled OctoMap libraries before linking and installing them.
set(PORTABLE_OCTOMAP_LIBRARY
  "${CMAKE_CURRENT_BINARY_DIR}/portable_lib/liboctomap.so.1.10")
set(PORTABLE_OCTOMATH_LIBRARY
  "${CMAKE_CURRENT_BINARY_DIR}/portable_lib/liboctomath.so.1.10")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/portable_lib")
configure_file(
  "${OCTO_PLANNER3D_ROOT}/lib/liboctomap.so.1.10"
  "${PORTABLE_OCTOMAP_LIBRARY}" COPYONLY)
configure_file(
  "${OCTO_PLANNER3D_ROOT}/lib/liboctomath.so.1.10"
  "${PORTABLE_OCTOMATH_LIBRARY}" COPYONLY)
file(RPATH_REMOVE FILE "${PORTABLE_OCTOMAP_LIBRARY}")
file(RPATH_REMOVE FILE "${PORTABLE_OCTOMATH_LIBRARY}")

# OctoPlanner3D ships prebuilt OctoMap shared libraries in its lib directory.
add_library(octomap SHARED IMPORTED GLOBAL)
set_target_properties(octomap PROPERTIES
  IMPORTED_LOCATION "${PORTABLE_OCTOMAP_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${OCTO_PLANNER3D_ROOT}/octomap/include"
)

add_library(octomath SHARED IMPORTED GLOBAL)
set_target_properties(octomath PROPERTIES
  IMPORTED_LOCATION "${PORTABLE_OCTOMATH_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${OCTO_PLANNER3D_ROOT}/octomap/include"
)

# Build the third-party converter/planner sources into a local static target so
# the ROS2 node can call them directly.
add_library(octo_planner3d_core STATIC
  "${OCTO_PLANNER3D_ROOT}/octomap/src/pcd2octomap_converter.cpp"
  "${OCTO_PLANNER3D_ROOT}/planner/src/global_planner.cpp"
)
target_include_directories(octo_planner3d_core
  PUBLIC
    "${OCTO_PLANNER3D_ROOT}/octomap/include"
    "${OCTO_PLANNER3D_ROOT}/planner/include"
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(octo_planner3d_core
  PUBLIC
    octomap
    octomath
    ${PCL_LIBRARIES}
)
target_compile_definitions(octo_planner3d_core PUBLIC ${PCL_DEFINITIONS})

add_executable(octo_global_planner_node src/octo_global_planner_node.cpp)
target_include_directories(octo_global_planner_node
  PRIVATE
    "${OCTO_PLANNER3D_ROOT}/octomap/include"
    "${OCTO_PLANNER3D_ROOT}/planner/include"
)
target_link_libraries(octo_global_planner_node octo_planner3d_core)
ament_target_dependencies(
  octo_global_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)
set_target_properties(octo_global_planner_node PROPERTIES
  BUILD_RPATH "${OCTO_PLANNER3D_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN/.."
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(TARGETS
  octo_global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  "${PORTABLE_OCTOMAP_LIBRARY}"
  "${PORTABLE_OCTOMATH_LIBRARY}"
  DESTINATION lib
)

ament_package()
