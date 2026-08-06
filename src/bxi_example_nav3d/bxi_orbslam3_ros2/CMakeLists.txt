cmake_minimum_required(VERSION 3.8)
project(bxi_orbslam3_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_library(SYSTEM_CURL_LIBRARY NAMES curl libcurl.so.4
  PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH REQUIRED)

# The bundled ORB-SLAM3 binary was originally built in another workspace and
# may contain that workspace in its RUNPATH. Work on a private build copy so
# installed artifacts never retain the original machine's home directory.
set(PORTABLE_ORB_SLAM3_LIBRARY
  "${CMAKE_CURRENT_BINARY_DIR}/portable_lib/libORB_SLAM3.so")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/portable_lib")
configure_file("${ORB_SLAM3_LIBRARY}" "${PORTABLE_ORB_SLAM3_LIBRARY}" COPYONLY)
file(RPATH_REMOVE FILE "${PORTABLE_ORB_SLAM3_LIBRARY}")

add_executable(rgbd_inertial
  src/rgbd-inertial/rgbd-inertial.cpp
  src/rgbd-inertial/rgbd-inertial-node.cpp
)

add_executable(sensor_qos_relay src/sensor-qos-relay.cpp)
ament_target_dependencies(sensor_qos_relay rclcpp sensor_msgs)
target_include_directories(rgbd_inertial PRIVATE
  include
  src/rgbd-inertial
  ${OpenCV_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)
ament_target_dependencies(rgbd_inertial
  cv_bridge geometry_msgs nav_msgs rclcpp rtabmap_msgs sensor_msgs tf2_ros
)
target_link_libraries(rgbd_inertial
  ${OpenCV_LIBS}
  -Wl,--no-as-needed
  ${PORTABLE_ORB_SLAM3_LIBRARY}
  ${DBoW2_LIBRARY}
  ${g2o_LIBRARY}
  -Wl,--as-needed
  ${Pangolin_LIBRARIES}
  ${SYSTEM_CURL_LIBRARY}
)

# Keep the installed adapter independent of the source-tree location. The
# vendored ORB-SLAM3 libraries are installed beside the workspace libraries,
# and the executable resolves them relative to its own install location.
set_target_properties(rgbd_inertial PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(TARGETS rgbd_inertial sensor_qos_relay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES
  "${PORTABLE_ORB_SLAM3_LIBRARY}"
  "${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so"
  "${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so"
  DESTINATION lib
)
install(FILES
  "${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt"
  DESTINATION share/${PROJECT_NAME}/Vocabulary
)

ament_package()
