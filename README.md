# Structured Light 3D Reconstruction

基于 C++17 实现的条纹结构光三维重建项目。

本项目主要用于个人系统复盘相移结构光三维重建中的算法与工程实现。

> Status: Work in Progress

## Planned Pipeline

```text
Sinusoidal Fringe Generation
        ↓
N-step Phase Shifting
        ↓
Wrapped Phase
        ↓
Multi-frequency Phase Unwrapping
        ↓
Absolute Phase
        ↓
Camera Calibration
        ↓
Projector Calibration
        ↓
Camera-Projector Triangulation
        ↓
3D Point Cloud

Planned Modules
Sinusoidal fringe generation
N-step phase shifting
Multi-frequency phase unwrapping
Camera calibration
Projector calibration
Structured-light triangulation
Point-cloud generation
Reconstruction accuracy evaluation
Tech Stack
C++17
CMake
vcpkg
OpenCV
Eigen

PCL will be introduced later for point-cloud processing.

Build

Requirements:
C++17 compiler
CMake >= 3.25
Ninja
vcpkg

Set the environment variable:
VCPKG_ROOT=<path-to-vcpkg>

Configure:
cmake --preset debug

Build:
cmake --build --preset debug

Test:
ctest --preset debug

Background
The original algorithms were developed and validated mainly in MATLAB
during my graduate research. This repository refactors the main
structured-light pipeline into a modular C++ implementation.