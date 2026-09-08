cmake_minimum_required(VERSION 3.16)

# WSL installations commonly have CUDA under /usr/local/cuda while nvcc is
# not added to non-interactive shells. Select the already-installed compiler
# before project() enables the CUDA language.
if(NOT DEFINED CMAKE_CUDA_COMPILER)
  foreach(CUDA_NVCC
      /usr/local/cuda/bin/nvcc
      /usr/local/cuda-11.8/bin/nvcc
      /usr/local/cuda-12/bin/nvcc)
    if(EXISTS "${CUDA_NVCC}")
      set(CMAKE_CUDA_COMPILER "${CUDA_NVCC}" CACHE FILEPATH "CUDA compiler")
      break()
    endif()
  endforeach()
endif()

project(graspgen_perception LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS
    $ENV{TENSORRT_ROOT}/include
    /usr/include/x86_64-linux-gnu
    /usr/local/TensorRT/include
    /usr/local/TensorRT-8.6.1.6/include
  REQUIRED)
find_library(NVINFER_LIBRARY nvinfer
  HINTS
    $ENV{TENSORRT_ROOT}/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
    /usr/local/TensorRT-8.6.1.6/lib
  REQUIRED)

find_library(NVINFER_PLUGIN_LIBRARY nvinfer_plugin
  HINTS
    $ENV{TENSORRT_ROOT}/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
    /usr/local/TensorRT-8.6.1.6/lib
  REQUIRED)

include_directories(
  include/graspgen_perception
  ${TENSORRT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(graspgen_yolo_plugins SHARED include/graspgen_perception/yololayer.cu)
target_compile_definitions(graspgen_yolo_plugins PRIVATE API_EXPORTS)
target_link_libraries(graspgen_yolo_plugins ${NVINFER_LIBRARY} ${NVINFER_PLUGIN_LIBRARY} CUDA::cudart)

set(INFERENCE_SOURCES
  src/block.cpp
  src/calibrator.cpp
  src/model.cpp
  src/postprocess.cpp
  src/postprocess.cu
  src/preprocess.cu
)

add_executable(yolov8_seg_node src/yolov8_seg.cpp ${INFERENCE_SOURCES})
target_compile_definitions(yolov8_seg_node PRIVATE API_EXPORTS)
ament_target_dependencies(yolov8_seg_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs std_msgs
  cv_bridge image_transport message_filters pcl_conversions)
target_link_libraries(yolov8_seg_node
  graspgen_yolo_plugins
  ${NVINFER_LIBRARY}
  ${NVINFER_PLUGIN_LIBRARY}
  CUDA::cudart
  dl
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS yolov8_seg_node graspgen_yolo_plugins
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
