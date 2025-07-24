cmake_minimum_required(VERSION 3.5)
project(px2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (ROS 2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)


# ONNX Runtime
find_path(ONNX_RUNTIME_SESSION_INCLUDE_DIRS onnxruntime_cxx_api.h HINTS /usr/local/include/onnxruntime/core/session/)
find_library(ONNX_RUNTIME_LIB onnxruntime HINTS /usr/local/include/onnxruntime)

# **Include Directories**
include_directories(
  ${CMAKE_SOURCE_DIR}/include  
)

# Project ==> Node
add_executable(${PROJECT_NAME} 
  src/inference.cpp 
  src/yolo_inference.cpp
)

# Ensure dependencies are linked correctly
target_include_directories(${PROJECT_NAME} PRIVATE 
  ${CMAKE_SOURCE_DIR}/include
  ${ONNX_RUNTIME_SESSION_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  ${ONNX_RUNTIME_LIB} 
  ${OpenCV_LIBS}
  ament_index_cpp::ament_index_cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  ament_index_cpp
  sensor_msgs
  cv_bridge
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# dataset and weights
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(
  DIRECTORY weights/
  DESTINATION share/${PROJECT_NAME}/weights
)
ament_package()

