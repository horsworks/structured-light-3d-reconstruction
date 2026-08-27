cmake_minimum_required(VERSION 3.25)

project(structured_light_3d_reconstruction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 CONFIG REQUIRED)

find_package(
  OpenCV
  REQUIRED
  COMPONENTS
    core
    imgproc
    imgcodecs
    calib3d
    features2d
)

find_package(
  PCL
  CONFIG
  REQUIRED
  COMPONENTS
    common
    io
    filters
)

if(MSVC)
  add_compile_options(
    /W4
    /permissive-
    /utf-8
  )
else()
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

add_library(
  sl3d
  STATIC
    src/fringe.cpp
    src/phase.cpp
    src/phase_unwrap.cpp
    src/camera_calibration.cpp
    src/projector_calibration.cpp
    src/triangulation.cpp
    src/point_cloud_process.cpp
)

target_include_directories(
  sl3d
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
  sl3d
  PUBLIC
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

add_executable(
  sl3d_info
  apps/sl3d_info_app.cpp
)

target_include_directories(
  sl3d_info
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  sl3d_info
  PRIVATE
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

add_executable(
  generate_fringe
  apps/generate_fringe_app.cpp
)

target_link_libraries(
  generate_fringe
  PRIVATE
    sl3d
)

add_executable(
  calibrate_camera
  apps/calibrate_camera_app.cpp
)

target_link_libraries(
  calibrate_camera
  PRIVATE
    sl3d
)

add_executable(
  calibrate_projector
  apps/calibrate_projector_app.cpp
)

target_link_libraries(
  calibrate_projector
  PRIVATE
    sl3d
)

add_executable(
  reconstruct
  apps/reconstruct_app.cpp
)

target_link_libraries(
  reconstruct
  PRIVATE
    sl3d
)

add_executable(
  process_point_cloud
  apps/process_point_cloud_app.cpp
)

target_link_libraries(
  process_point_cloud
  PRIVATE
    sl3d
)

include(CTest)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()