# rosbag2 → nuScenes 格式转换：ROS2 Humble + 自定义 msg + C++ 转换程序
# 构建上下文应为 docker/ 目录（含上层 msg_interfaces 的 COPY 路径需与 docker build 一致）
FROM osrf/ros:humble-desktop

# 系统依赖：JSON、PCL/ROS 桥、locale、常用工具
RUN apt update && \
    apt install -y --no-install-recommends \
    nlohmann-json3-dev \
    ros-humble-pcl-ros \
    locales \
    software-properties-common \
    git

# 时区与 UTF-8 区域设置（避免日志/路径编码问题）
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ---------- 先编译 ROS2 包 msg_interfaces（主工程依赖其消息定义）----------
RUN mkdir -p /colcon_ws/src

WORKDIR /colcon_ws/src

COPY msg_interfaces /colcon_ws/src/msg_interfaces

RUN rm -rf build install log

WORKDIR /colcon_ws

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select msg_interfaces

# ---------- 编译 rosbag2nuscenes 本体 ----------
COPY . /rosbag2nuscenes

RUN rm -rf build log

WORKDIR /rosbag2nuscenes/build

RUN . /opt/ros/humble/setup.sh && \
    . /colcon_ws/install/setup.sh && \
    cmake .. && \
    make

# 运行时入口：run.sh 挂载数据后由 entrypoint 调用可执行文件
WORKDIR /rosbag2nuscenes/docker

ENTRYPOINT ["./entrypoint.sh"]
