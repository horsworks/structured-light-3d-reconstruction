# Structured Light 3D Reconstruction

基于 C++17 的相移结构光三维重建项目。

本项目主要用于个人重新梳理结构光三维重建中的算法流程、数据接口与工程实现。

## 核心流程

```text
条纹生成
    ↓
相移图像
    ↓
包裹相位
    ↓
多频展开
    ↓
绝对相位
    ↓
相机 / 投影仪标定
    ↓
三角测量
    ↓
三维点云
    ↓
精度评价
```

## 功能模块

| 模块 | 说明 |
| --- | --- |
| 条纹生成 | 生成水平/垂直正弦条纹，支持周期、相移步数等参数设置 |
| 相位计算 | N 步相移法，计算包裹相位、调制度及有效区域 |
| 多频展开 | 基于多频外差恢复绝对相位 |
| 相机标定 | 相机内参、畸变参数及外参标定 |
| 投影仪标定 | 基于绝对相位建立投影仪坐标对应并完成标定 |
| 三角测量 | 根据相机与投影仪几何关系恢复三维坐标 |
| 点云处理 | 点云生成、导出及基础预处理 |
| 精度评价 | 重投影误差、平面/球拟合及 RMSE 等指标 |

## 项目结构

```text
structured-light-3d-reconstruction/
├── CMakeLists.txt
├── CMakePresets.json
├── vcpkg.json
├── include/structured_light/   # 公开头文件
├── src/                        # 算法实现
├── apps/                       # 可执行程序与示例
├── tests/                      # 测试与数值验证
├── config/                     # 配置文件
├── data/sample/                # 小型测试数据
└── docs/                       # 补充说明
```

部分目录会随着对应模块的实现逐步加入。

## 开发环境

- **语言**：C++17
- **构建**：CMake >= 3.25 + Ninja
- **包管理**：vcpkg（Manifest Mode）
- **编译器**：MSVC（Windows），后续使用 GCC 验证 Linux / WSL 构建
- **核心依赖**：OpenCV（图像处理/标定）、Eigen（线性代数/几何）、PCL（点云处理）

当前第三方依赖通过 `vcpkg.json` 管理。

## 构建

| 配置 | Configure | Build |
| --- | --- | --- |
| Debug | `cmake --preset debug` | `cmake --build --preset debug` |
| Release | `cmake --preset release` | `cmake --build --preset release` |

运行测试：

```powershell
ctest --preset debug
```

## License

暂未设置开源许可证。