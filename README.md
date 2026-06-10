# nuscenes_foundation

[中文文档](README_CN.md)

Convert ROS 2 rosbag2 directories to [nuScenes](https://www.nuscenes.org/) dataset format (v1.0-* JSON metadata, `samples/`, `sweeps/`, `ego_pose`, etc.), compatible with the official nuScenes devkit and downstream perception pipelines.

Based on [linklab-uva/rosbag2nuscenes](https://github.com/linklab-uva/rosbag2nuscenes), with support for custom ROS 2 messages, multi-camera batch sampling, and Docker builds.

## Project Structure

```
nuscenes_foundation/
├── main.cpp                  # Entry point (4 CLI arguments)
├── CMakeLists.txt            # CMake build config (C++17)
├── include/rosbag2nuscenes/
│   ├── Bag2Scenes.hpp        # Core converter class
│   ├── MessageConverter.hpp  # ROS message deserialization
│   ├── MessageTypes.hpp      # Sensor message struct definitions
│   └── SensorDataWriter.hpp  # Multi-threaded disk writer
├── src/
│   ├── Bag2Scenes.cpp        # Main conversion logic
│   ├── MessageConverter.cpp  # Message type conversion
│   ├── SensorDataWriter.cpp  # Parallel file I/O
│   └── pugixml.cpp           # XML parser (URDF)
├── include/
│   ├── pugixml.hpp           # pugixml headers
│   └── pugiconfig.hpp
├── params/                   # YAML config files
│   ├── bjut.yaml             # Example: BJUT campus dataset
│   ├── mit.yaml              # Example: MIT dataset
│   ├── hylight_xf.yaml       # Example: Hylight XF dataset
│   └── sensing_params/       # Camera intrinsic calibration files
├── urdf/                     # Vehicle/sensor extrinsic URDF files
│   ├── bjut.urdf
│   ├── av21.urdf
│   └── hylight_xf.urdf
├── docker/                   # Docker build & run
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── build.sh
│   └── run.sh
├── indicators/               # Git submodule (terminal progress bars)
├── export_data.py            # Pack sensor data by scene (requires nuscenes pip)
├── nuscenes_tutorial.ipynb   # Jupyter tutorial notebook
├── LICENSE                   # Apache 2.0
└── .gitmodules
```

## Usage

### Arguments

```
./rosbag2nuscenes <rosbag_dir> <param_yaml> <output_dir> <num_workers>
```

| Argument      | Description                                      |
|---------------|--------------------------------------------------|
| `rosbag_dir`  | Path to the ROS 2 bag directory                  |
| `param_yaml`  | Path to the YAML configuration file              |
| `output_dir`  | Output directory for nuScenes-format data        |
| `num_workers` | Number of parallel I/O threads (integer >= 1)    |

### Sample Strategies

Configured via `BAG_INFO.SAMPLE_STRATEGY` in the YAML file:

- **`batch_cameras`** (default): Collects one frame from each camera in bag order, forms a sample once all cameras are present. Best for unsynchronized bags.
- **`time_window`**: Divides time by `SAMPLE_INTERVAL` seconds, takes at most one message per channel per window. Best for synchronized data.

### Local Build

```bash
# 1. Clone with submodules
git clone --recurse-submodules <repo-url>
cd nuscenes_foundation

# 2. Build msg_interfaces first (adjust to your colcon workspace)
cd /path/to/colcon_ws
colcon build --packages-select msg_interfaces
source install/setup.bash

# 3. Build the converter
cd /path/to/nuscenes_foundation
mkdir build && cd build
cmake ..
make -j$(nproc)

# 4. Run
./rosbag2nuscenes /path/to/bag /path/to/config.yaml /path/to/output 4
```

### Docker

```bash
# Build image
docker build -f docker/Dockerfile -t rosbag2nuscenes .

# Run (see docker/run.sh for a convenience wrapper)
docker run --rm \
  --mount type=bind,src=/path/to/bag,target=/data/rosbag \
  --mount type=bind,src=/path/to/config.yaml,target=/params/config.yaml \
  --mount type=bind,src=/path/to/output,target=/output \
  rosbag2nuscenes \
  /data/rosbag /params/config.yaml /output 4
```

## Dependencies

- **ROS 2** Humble (recommended)
- **C++17** compiler
- ROS 2 packages: `rclcpp`, `rosbag2_cpp`, `sensor_msgs`, `nav_msgs`, `cv_bridge`, `pcl_conversions`, `pcl_ros`, `tf2_eigen`
- System: `OpenCV`, `PCL`, `Eigen3`, `yaml-cpp`, `nlohmann-json`

## License

[Apache License 2.0](LICENSE)
