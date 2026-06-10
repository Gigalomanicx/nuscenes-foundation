# rosbag2nuscenes

将 **ROS 2 rosbag2** 目录中的里程计、相机、（可选）激光雷达等消息，导出为与 **[nuScenes](https://www.nuscenes.org/)** 兼容的目录结构与元数据（`v1.0-*` JSON、`samples/`、`sweeps/` 等），便于用官方或 fork 的 **nuScenes dev-kit** 与下游占用/检测代码读取。

本仓库在 [linklab-uva/rosbag2nuscenes](https://github.com/linklab-uva/rosbag2nuscenes) 基础上演进：支持自定义消息（`msg_interfaces`）、多相机 batch 切分、Docker 一键构建运行等。

rosbag2nuscenes/
├── main.cpp                          # 入口：4 个命令行参数
├── CMakeLists.txt                    # 构建配置
├── include/rosbag2nuscenes/
│   ├── Bag2Scenes.hpp                # 主转换类（核心）
│   ├── MessageConverter.hpp          # ROS 消息反序列化 & 类型转换
│   ├── MessageTypes.hpp              # 自定义消息结构体定义
│   └── SensorDataWriter.hpp          # 多线程传感器数据写入器
├── src/
│   ├── Bag2Scenes.cpp                # 主转换逻辑实现（~1000 行）
│   ├── MessageConverter.cpp          # 消息转换实现
│   ├── SensorDataWriter.cpp          # 数据写入实现
│   └── pugixml.cpp                   # XML 解析库
├── msg_interfaces/                   # 自定义 ROS 2 消息包（Hcinspvatzcb 等）
├── urdf/                             # 车辆/传感器外参 URDF 文件
├── params/                           # YAML 配置文件（bjut, mit, hylight_xf）
├── docker/                           # Docker 构建 & 运行脚本
├── indicators/                       # Git submodule - 进度条库
├── export_data.py                    # Python 辅助脚本（按场景打 tar）
└── nuscenes_tutorial.ipynb           # 教程 Notebook
---

## 功能概要

- 从 bag 读取 **odometry**（`nav_msgs/Odometry` 或自定义 `Hcinspvatzcb` 等，见代码与参数配置）。
- 按 YAML 配置写入 **相机**（及可选 **LiDAR**）到 `sample` / `sample_data` / `ego_pose` 等表；标注相关 JSON 中为占位项，以保持与 dev-kit 结构兼容。
- **时间戳**：与 nuScenes 一致，导出为 **微秒整型**；**`sample_data.filename`** 为相对数据集根的路径。
- **Sample 划分**（`BAG_INFO.SAMPLE_STRATEGY`）：
  - `batch_cameras`：按播放顺序为每路相机各取一帧，凑齐全部相机后形成一个 sample（适合未硬同步的多相机 bag）。
  - `time_window`：按 `SAMPLE_INTERVAL` 时间窗 + 每窗每通道至多一条（适合已同步数据）。

---

## 仓库中「必须保留」的内容

| 路径 | 说明 |
|------|------|
| `CMakeLists.txt` | 工程与依赖 |
| `main.cpp` | 入口：`rosbag2_dir param_yaml output_dir num_workers` |
| `src/*.cpp` | 核心实现（含 `pugixml.cpp`） |
| `include/rosbag2nuscenes/`、`include/pugixml*.hpp` | 头文件 |
| `msg_interfaces/` | ROS 2 消息包，需先 **colcon build**，CMake `find_package(msg_interfaces)` |
| `indicators/` | Git **子模块**（进度条，头文件库），克隆时需 `git submodule update --init --recursive` |
| `urdf/` | 参数里 `BAG_INFO.URDF` 引用的车辆/传感器外参；至少保留你实际使用的 `.urdf` |
| `params/` | 至少一份可用 YAML；若使用内联标定 + `sensing_params/` 下 txt，需一并保留 |
| `docker/Dockerfile`、`docker/entrypoint.sh` | 镜像内编译 `msg_interfaces` + 本工程并运行 |
| `.gitmodules` | 声明 `indicators`（及可选 `nuscenes-devkit`）子模块 |

---

## 可选 / 辅助（可删不影响 C++ 转换）

| 路径 | 说明 |
|------|------|
| `docker/run.sh` | 宿主机一键 `docker build` + `docker run`；可删，改用自行命令 |
| `docker/build.sh` | 简单封装 `docker build`，可选 |
| `Bag2Scenes_origin.cpp` | 根目录备份/旧版参考，**不参与** `src/*.cpp` 编译，可删 |
| `export_data.py` | 依赖已安装的 `nuscenes`，按场景打 tar，非转换核心 |
| `nuscenes_tutorial.ipynb` | 教程笔记本 |
| `nuscenes-devkit/` | 子模块；未初始化则不存在。用于 Python 侧浏览数据，**不是** C++ 编译依赖 |

---

## 建议不要提交 / 可直接删除的生成物

| 路径 | 说明 |
|------|------|
| `build/` | 本地 CMake 构建目录，应删除或加入 `.gitignore` |
| `docker/install/`、`docker/log/` | 若在仓库根执行过 colcon 且产物落在此，属**误提交**的构建残留，可删；**Dockerfile 在镜像内会重新编译**，不依赖这两目录 |
| 转换输出目录下的 `v1.0-*`、`samples/`、`sweeps/`、`*.json`（数据） | 由 `.gitignore` 忽略；磁盘上可随时删 |

---

## 依赖（本地编译）

- 已 source 的 **ROS 2**（推荐 **Humble**，与 Dockerfile 一致）
- **C++17** 编译器
- **ament_cmake**、`rclcpp`、`rosbag2_cpp`、`sensor_msgs`、`nav_msgs`、`cv_bridge`、`pcl_conversions`、`pcl_ros`、`Eigen3`、`yaml-cpp`、`OpenCV`
- 先编译 **`msg_interfaces`** 再编译本仓库，保证 `find_package(msg_interfaces)` 成功

Docker 镜像内已安装 ROS Humble 与常用库，并在构建阶段完成 `msg_interfaces` + `rosbag2nuscenes`。

---

## 本地构建示例

```bash
# 若尚未拉子模块
git submodule update --init --recursive

# 终端 A：编译 msg_interfaces（路径按你的 colcon 工作区调整）
cd /path/to/colcon_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select msg_interfaces
source install/setup.bash

# 终端 B：编译 rosbag2nuscenes
cd /path/to/rosbag2nuscenes
mkdir -p build && cd build
cmake ..
make -j
```

运行（4 个参数缺一不可）：

```bash
./rosbag2nuscenes /path/to/rosbag2_dir /path/to/config.yaml /path/to/output_dir 4
```

---

## Docker 构建与运行

构建镜像（**上下文为仓库根目录**，以便复制 `msg_interfaces` 与源码）：

```bash
cd /path/to/rosbag2nuscenes
docker build -f docker/Dockerfile -t rosbag2nuscenes .
```

运行：将 **bag 目录**、**参数文件**、**输出目录** 挂载进容器，并传入与本地相同的四个参数。可参考 `docker/run.sh`：按需修改其中的 `ROSBAG_DIR`、`PARAM_FILE`、`OUTPUT_DIR`、`NUM_WORKERS`，在宿主机先 `mkdir -p` 输出目录并保证权限，再执行脚本。

容器内入口为 `docker/entrypoint.sh`，会 source ROS 与 `msg_interfaces` 的安装空间后执行 `build/rosbag2nuscenes`。

---

## 参数文件

- 示例：`params/bjut.yaml`、`params/mit.yaml`、`params/hylight_xf.yaml`。
- 必填字段包括 `BAG_INFO`（团队名、描述、里程计 topic、`URDF` 路径、`SAMPLE_STRATEGY` 等）和 `SENSOR_INFO`（各传感器 topic、frame、相机标定等）。
- `URDF` 一般为相对于**当前工作目录**或工程内路径，请与 `urdf/` 中文件一致。

---

## nuScenes dev-kit

官方 dev-kit 对部分传感器命名有硬编码；若自定义传感器名导致问题，可使用社区 fork（见上游文档）。`export_data.py` 用于在已安装 `nuscenes` Python 包的前提下按场景打包文件，**不是**转换程序的一部分。

---

## 已知说明

- 若 bag 中雷达/LiDAR 话题顺序或策略与实现不一致，可能出现 **sample 中仅有相机、无雷达行** 等情况，需结合 `SAMPLE_STRATEGY` 与实际 bag 检查。
- 自动 3D 标注不可用；标注相关 JSON 中为兼容 dev-kit 的占位内容。
