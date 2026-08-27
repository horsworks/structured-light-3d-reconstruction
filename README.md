# Structured Light 3D Reconstruction

基于 C++17 实现的 DLP 相移结构光三维重建项目，包含相位计算、相机/投影仪标定、三角测量以及基础点云处理。

## 项目流程

```text
正弦条纹生成
    ↓
N 步相移
    ↓
包裹相位
    ↓
三频外差展开
    ↓
绝对相位
    ↓
相机标定
    ↓
投影仪标定
    ↓
相机—投影仪外参
    ↓
单方向三角测量
    ↓
三维点云
    ↓
点云后处理
```

## 已实现功能

- 正弦相移条纹生成
- N 步相移包裹相位计算
- 调制度计算与有效区域筛选
- 三频外差相位展开
- 相机标定
- DLP 投影仪标定
- 相机—投影仪相对位姿标定
- 单方向结构光三维重建
- PLY / PCD 点云导入导出
- ASCII / Binary 点云格式
- PassThrough 滤波
- Statistical Outlier Removal
- VoxelGrid 降采样
- 核心模块单元测试

## 项目结构

```text
structured-light-3d-reconstruction/
├── apps/
│   ├── generate_fringe_app.cpp
│   ├── calibrate_camera_app.cpp
│   ├── calibrate_projector_app.cpp
│   ├── reconstruct_app.cpp
│   └── process_point_cloud_app.cpp
│
├── config/
│   ├── camera_calibration.yaml
│   ├── projector_calibration.yaml
│   ├── reconstruction.yaml
│   └── point_cloud_processing.yaml
│
├── include/structured_light/
├── src/
├── tests/
├── docs/
├── CMakeLists.txt
├── CMakePresets.json
└── vcpkg.json
```

## 开发环境

- C++17
- CMake >= 3.25
- Ninja
- MSVC
- vcpkg Manifest Mode
- OpenCV
- Eigen
- PCL

## 构建

Debug：

```powershell
cmake --preset debug
cmake --build --preset debug
```

Release：

```powershell
cmake --preset release
cmake --build --preset release
```

运行测试：

```powershell
ctest --preset debug
```

## 主要程序

### 相机标定

```powershell
.\build\debug\calibrate_camera.exe `
  config\camera_calibration.yaml
```

### 投影仪标定

```powershell
.\build\debug\calibrate_projector.exe `
  config\projector_calibration.yaml
```

### 三维重建

```powershell
.\build\debug\reconstruct.exe `
  config\reconstruction.yaml
```

### 点云处理

```powershell
.\build\debug\process_point_cloud.exe `
  config\point_cloud_processing.yaml
```

## 当前限制

- 当前相位展开实现为特定三频外差方案
- 实物重建采用单方向条纹