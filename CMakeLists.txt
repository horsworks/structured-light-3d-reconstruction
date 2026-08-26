cmake_minimum_required(VERSION 3.25)

project(
    structured_light_3d_reconstruction
    VERSION 0.1.0
    DESCRIPTION "C++ implementation of a structured-light 3D reconstruction pipeline"
    LANGUAGES CXX
)

# ============================================================
# C++ standard
# ============================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# ============================================================
# Dependencies
# ============================================================

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)


# ============================================================
# Compiler warnings
# ============================================================

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


# ============================================================
# Applications
# ============================================================

add_executable(
    sl3d_info
    apps/sl3d_info.cpp
)

target_link_libraries(
    sl3d_info
    PRIVATE
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

target_include_directories(
    sl3d_info
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)


# ============================================================
# Tests
# ============================================================

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()