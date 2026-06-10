#ifndef BAG2SCENES_HPP
#define BAG2SCENES_HPP

/**
 * @file Bag2Scenes.hpp
 * @brief ROS2 Bag 转 nuScenes 数据集格式的主类头文件
 *
 * 本文件定义了 Bag2Scenes 类的接口，负责将 ROS2 bag 文件转换为 nuScenes 数据集格式。
 * 支持的传感器类型包括：相机、激光雷达、雷达、IMU 和 odometry。
 */

#include <string>
#include <set>
#include <algorithm>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <cstdint>
#include "MessageTypes.hpp"
#include "MessageConverter.hpp"
#include "SensorDataWriter.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rcpputils/asserts.hpp>
#include "yaml-cpp/yaml.h"
#include <nlohmann/json.hpp>
#include "pugixml.hpp"
#include <indicators/cursor_control.hpp>
#include <indicators/block_progress_bar.hpp>
#include <indicators/multi_progress.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>

namespace fs = std::filesystem;

/**
 * @class Bag2Scenes
 * @brief ROS2 Bag 转 nuScenes 格式的主转换类
 *
 * 负责解析参数文件、读取 bag 数据、转换传感器数据并写入 nuScenes 格式。
 *
 * 使用流程：
 * 1. 构造函数：解析 YAML 参数文件，获取 bag 元数据
 * 2. writeScene()：执行转换，生成 nuScenes 所需的所有 JSON 文件
 */
class Bag2Scenes {
    public:
        /**
         * @brief 构造函数
         * @param rosbag_dir rosbag 文件目录路径
         * @param param_file YAML 参数文件路径
         * @param output_dir 输出目录路径
         * @param num_workers 并行写入线程数
         */
        Bag2Scenes(const fs::path rosbag_dir, const fs::path param_file, const fs::path output_dir, int num_workers);

        /**
         * @brief 执行完整的场景转换
         *
         * 主要步骤：
         * 1. 写入日志文件 (log.json)
         * 2. 写入地图信息 (map.json)
         * 3. 写入传感器标定数据 (calibrated_sensor.json, sensor.json)
         * 4. 写入样本数据 (sample_data.json, sample.json)
         * 5. 写入 ego pose 数据 (ego_pose.json)
         * 6. 写入场景信息 (scene.json)
         */
        void writeScene();

    private:

        /**
         * @brief 生成随机 token 字符串
         * @return 32 字符的十六进制随机字符串
         */
        std::string generateToken();

        /**
         * @brief 解析字符串为浮点数向量
         * @param str 空格分隔的浮点数字符串 (如 "1.0 2.0 3.0")
         * @return 浮点数向量
         */
        std::vector<float> splitString(std::string str);

        /**
         * @brief 集齐一轮相机后写入 sample / sample_data，并清空 pending
         *
         * 用于未做时间同步的 bag：按顺序凑满 YAML 中声明的全部相机后再提交一个 sample，
         * 保证每个 sample 恰好对应每路相机各一帧（可选再绑定本窗口内最后一帧雷达）。
         */
        void flushPendingCameraBatch(nlohmann::json& previous_data, SensorDataWriter& data_writer);

        /** time_window 策略：老板版逻辑，按 SAMPLE_INTERVAL 划分并每通道每窗至多一条 key 进 samples */
        bool is_key_frame(std::string channel, unsigned long timestamp);

        /** nuScenes 元数据：timestamp 为微秒（uint64）；内部 ROS 仍为纳秒 */
        static uint64_t timestampNsToUs(unsigned long timestamp_ns);

        /** 相对数据集根目录的路径，文件名内时间戳为微秒（与 nuScenes 一致） */
        std::string sensorDataRelativePath(std::string channel, unsigned long timestamp_ns);

        /** 磁盘写入用的绝对路径（output_dir_ + 相对路径） */
        fs::path getFilename(std::string channel, unsigned long timestamp);

        /**
         * @brief 根据时间戳获取最近的 ego pose
         * @param timestamp 目标时间戳
         * @return 对应 ego_pose.json 中的 token
         */
        std::string getClosestEgoPose(unsigned long timestamp);

        /**
         * @brief 写入日志文件
         * @return 生成 log.json 的 token
         */
        std::string writeLog();

        /**
         * @brief 写入地图信息文件 (map.json)
         * @param log_token 关联的 log token
         */
        void writeMap(std::string log_token);

        /**
         * @brief 写入样本记录
         * @return sample token
         */
        std::string writeSample();

        /**
         * @brief 写入传感器数据 (样本和 sweep)
         * @param previous_data 用于追加的 JSON 数组引用
         *
         * 处理流程：
         * 1. 预注册内联校准的相机传感器
         * 2. 遍历 bag 中所有消息
         * 3. 根据消息类型转换并写入文件
         * 4. 更新 sample_data.json
         */
        void writeSampleData(nlohmann::json& previous_data);

        /**
         * @brief 写入 Ego 位姿数据
         * @param previous_poses 用于追加的 JSON 数组引用
         *
         * 从 odometry 话题读取车辆位姿，转换为 nuScenes ego_pose 格式。
         * 使用独立线程处理，与 writeSampleData 并行执行。
         */
        void writeEgoPose(nlohmann::json& previous_poses);

        /**
         * @brief 写入传感器标定信息
         * @param frame 传感器 frame ID
         * @param camera_intrinsic 相机内参矩阵 (3x3)
         * @param distortion_params 畸变参数 [k1, k2, p1, p2, k3]
         *
         * 从 URDF 文件读取传感器外参（位置和旋转），
         * 结合内参生成 calibrated_sensor.json 条目。
         */
        void writeCalibratedSensor(std::string frame, std::vector<std::vector<float>> camera_intrinsic, std::vector<float> distortion_params);

        /**
         * @brief 写入分类体系文件
         *
         * 生成 nuScenes 所需的：
         * - attribute.json
         * - category.json
         * - visibility.json
         */
        void writeTaxonomyFiles();

        /**
         * @brief 写入传感器定义
         * @param channel 传感器通道名称
         * @return sensor token
         */
        std::string writeSensor(std::string channel);

        // ========== 成员变量 ==========

        int num_workers_;                                      /**< 并行写入线程数 */
        fs::path output_dir_;                                   /**< 输出目录路径 */
        rosbag2_storage::StorageOptions storage_options_;       /**< rosbag 存储选项 */
        rosbag2_cpp::ConverterOptions converter_options_;       /**< 转换器选项 */
        rosbag2_storage::BagMetadata bag_data_;                /**< bag 文件元数据 */
        std::string bag_dir_;                                  /**< bag 目录名 */

        std::vector<std::string> lidar_topics_;               /**< 激光雷达 topic 列表 */
        std::vector<std::string> camera_topics_;              /**< 相机 topic 列表 */

        /** 相机校准配置（可选择内联参数或 ROS topic） */
        std::vector<std::string> camera_calib_topics_;         /**< 相机校准 topic 列表（"inline" 表示内联） */
        std::map<std::string, bool> camera_calib_inline_;      /**< 各相机的校准方式：true=内联，false=ROS topic */
        std::map<std::string, std::vector<std::vector<float>>> camera_calib_intrinsic_;  /**< 内联内参矩阵 (frame -> 3x3 matrix) */
        std::map<std::string, std::vector<float>> camera_calib_distortion_;              /**< 内联畸变参数 [k1, k2, p1, p2, k3] */

        std::vector<std::string> topics_of_interest_;          /**< 需要处理的 topic 列表 */

        std::vector<std::pair<unsigned long, std::string>> ego_pose_queue_;  /**< Ego 位姿队列 (timestamp, token) */
        std::map<std::string, std::string> topic_to_type_;    /**< Topic 到消息类型的映射 */
        std::map<std::string, unsigned long> last_timestamp_received_;  /**< 各 topic 最后接收时间戳 */

        /** YAML 中相机 FRAME 的顺序，一轮凑满即为一个 sample（不要求多相机时间对齐） */
        std::vector<std::string> camera_frame_order_;
        /** 当前正在凑的一轮相机：frame_id -> 图像（凑满后一次性写出） */
        std::unordered_map<std::string, std::unique_ptr<CameraMessageT>> pending_cameras_;
        /** 当前轮次内看到的最后一帧雷达，随相机 batch 一起写入同一 sample */
        std::unique_ptr<LidarMessageT> staged_lidar_;

        /** true=batch_cameras（默认，未同步 bag）；false=time_window（已同步数据，老板版间隔逻辑） */
        bool use_batch_camera_strategy_ = true;
        /** time_window 时与老板版一致：参考时间步进与判窗，单位秒，对应 YAML SAMPLE_INTERVAL */
        double sample_interval_sec_ = 0.5;
        /** 仅 time_window：上一采样参考时间（纳秒） */
        unsigned long previous_sampled_timestamp_ = 0;
        /** 仅 time_window：当前时间窗内已出现过的 channel */
        std::unordered_set<std::string> sensors_sampled_;

        int nbr_samples_;                                      /**< 样本总数 */
        bool ego_pose_done_;                                   /**< Ego pose 处理完成标志 */

        nlohmann::json samples_;                               /**< 样本数据缓存 */
        std::string previous_sample_token_;                    /**< 上一个样本 token */
        std::string next_sample_token_;                       /**< 下一个样本 token */
        std::string current_sample_token_;                    /**< 当前样本 token */
        std::string scene_token_;                             /**< 场景 token */

        std::mutex ego_pose_mutex_;                            /**< Ego 位姿处理互斥锁 */
        std::condition_variable ego_pose_ready_;               /**< Ego 位姿就绪条件变量 */
        unsigned long waiting_timestamp_;                      /**< 等待处理的位姿时间戳 */

        YAML::Node frame_info_;                               /**< Frame 信息缓存 */
        YAML::Node param_yaml_;                               /**< 参数文件解析结果 */

        /** 进度条显示 */
        indicators::BlockProgressBar odometry_bar_;            /**< Odometry 处理进度条 */
        indicators::BlockProgressBar sensor_data_bar_;         /**< 传感器数据处理进度条 */
        indicators::MultiProgress<indicators::BlockProgressBar, 2> progress_bars_;  /**< 多进度条管理器 */
};


#endif
