cmake_minimum_required(VERSION 3.8)
project(bxi_example_cpp_line_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_library(SYSTEM_CURL_LIBRARY
  NAMES curl
  PATHS /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)

add_executable(cpp_line_detector src/cpp_line_detector.cpp)
target_compile_features(cpp_line_detector PUBLIC cxx_std_17)
target_include_directories(cpp_line_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cpp_line_detector ${OpenCV_LIBS})
if(SYSTEM_CURL_LIBRARY)
  target_link_libraries(cpp_line_detector ${SYSTEM_CURL_LIBRARY})
endif()
ament_target_dependencies(
  cpp_line_detector
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(cpp_line_debug_viewer src/cpp_line_debug_viewer.cpp)
target_compile_features(cpp_line_debug_viewer PUBLIC cxx_std_17)
target_include_directories(cpp_line_debug_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cpp_line_debug_viewer ${OpenCV_LIBS})
if(SYSTEM_CURL_LIBRARY)
  target_link_libraries(cpp_line_debug_viewer ${SYSTEM_CURL_LIBRARY})
endif()
ament_target_dependencies(
  cpp_line_debug_viewer
  rclcpp
  sensor_msgs
)

install(TARGETS
  cpp_line_detector
  cpp_line_debug_viewer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
