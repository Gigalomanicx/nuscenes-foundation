# nuscenes_foundation

[English](README.md)

将 ROS 2 rosbag2 目录转换为 [nuScenes](https://www.nuscenes.org/) 数据集格式（v1.0-* JSON 元数据、`samples/`、`sweeps/`、`ego_pose` 等），兼容官方 nuScenes devkit 及下游感知算法。

基于 [linklab-uva/rosbag2nuscenes](https://github.com/linklab-uva/rosbag2nuscenes) 开发，新增自定义 ROS 2 消息支持、多相机 batch 采样策略和 Docker 构建。

## 目录结构

```
nuscenes_foundation/
├── main.cpp                  # 入口（4 个命令行参数）
├── CMakeLists.txt            # CMake 构建配置（C++17）
├── include/rosbag2nuscenes/
│   ├── Bag2Scenes.hpp        # 核心转换类
│   ├── MessageConverter.hpp  # ROS 消息反序列化
│   ├── MessageTypes.hpp      # 传感器消息结构体定义
│   └── SensorDataWriter.hpp  # 多线程磁盘写入器
├── src/
│   ├── Bag2Scenes.cpp        # 主转换逻辑
│   ├── MessageConverter.cpp  # 消息类型转换
│   ├── SensorDataWriter.cpp  # 并行文件 I/O
│   └── pugixml.cpp           # XML 解析库（用于 URDF）
├── include/
│   ├── pugixml.hpp           # pugixml 头文件
│   └── pugiconfig.hpp
├── params/                   # YAML 配置文件
│   ├── bjut.yaml             # 示例：BJUT 数据
│   ├── mit.yaml              # 示例：MIT 数据
│   ├── hylight_xf.yaml       # 示例：Hylight XF 数据
│   └── sensing_params/       # 相机内参标定文件
├── urdf/                     # 车辆/传感器外参 URDF 文件
│   ├── bjut.urdf
│   ├── av21.urdf
│   └── hylight_xf.urdf
├── docker/                   # Docker 构建与运行
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── build.sh
│   └── run.sh
├── indicators/               # Git 子模块（终端进度条库）
├── export_data.py            # 按场景打包数据（需安装 nuscenes pip 包）
├── nuscenes_tutorial.ipynb   # Jupyter 教程 notebook
├── LICENSE                   # Apache 2.0
└── .gitmodules
```

## 使用方法

### 命令行参数

```
./rosbag2nuscenes <rosbag_dir> <param_yaml> <output_dir> <num_workers>
```

| 参数           | 说明                                     |
|----------------|------------------------------------------|
| `rosbag_dir`   | ROS 2 bag 目录路径                       |
| `param_yaml`   | YAML 配置文件路径                        |
| `output_dir`   | nuScenes 格式数据输出目录                |
| `num_workers`  | 并行 I/O 线程数（整数，>= 1）            |

### Sample 划分策略

通过 YAML 文件中的 `BAG_INFO.SAMPLE_STRATEGY` 配置：

- **`batch_cameras`**（默认）：按 bag 中消息顺序，每路相机各取一帧，凑齐全部相机后形成一个 sample。适用于未做时间同步的多相机 bag。
- **`time_window`**：按 `SAMPLE_INTERVAL`（秒）划分时间窗，每窗每通道至多取一条消息。适用于已时间同步的数据。

### 本地编译

```bash
# 1. 克隆（含子模块）
git clone --recurse-submodules <repo-url>
cd nuscenes_foundation

# 2. 先编译 msg_interfaces（调整路径到你的 colcon 工作区）
cd /path/to/colcon_ws
colcon build --packages-select msg_interfaces
source install/setup.bash

# 3. 编译转换程序
cd /path/to/nuscenes_foundation
mkdir build && cd build
cmake ..
make -j$(nproc)

# 4. 运行
./rosbag2nuscenes /path/to/bag /path/to/config.yaml /path/to/output 4
```

### Docker 运行

```bash
# 构建镜像
docker build -f docker/Dockerfile -t rosbag2nuscenes .

# 运行（也可使用 docker/run.sh 便捷脚本）
docker run --rm \
  --mount type=bind,src=/path/to/bag,target=/data/rosbag \
  --mount type=bind,src=/path/to/config.yaml,target=/params/config.yaml \
  --mount type=bind,src=/path/to/output,target=/output \
  rosbag2nuscenes \
  /data/rosbag /params/config.yaml /output 4
```

## 依赖

- **ROS 2** Humble（推荐）
- **C++17** 编译器
- ROS 2 包：`rclcpp`、`rosbag2_cpp`、`sensor_msgs`、`nav_msgs`、`cv_bridge`、`pcl_conversions`、`pcl_ros`、`tf2_eigen`
- 系统库：`OpenCV`、`PCL`、`Eigen3`、`yaml-cpp`、`nlohmann-json`

## 许可证

[Apache License 2.0](LICENSE)
