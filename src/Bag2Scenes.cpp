#include "rosbag2nuscenes/Bag2Scenes.hpp"
#include <iostream>

/**
 * 构造函数：初始化Bag2Scenes转换器
 *
 * 主要初始化步骤：
 * 1. 读取YAML参数文件
 * 2. 打开rosbag并读取元数据
 * 3. 解析传感器配置（相机、激光雷达）
 * 4. 初始化采样相关状态变量
 *
 * @param rosbag_dir rosbag文件所在目录
 * @param param_file YAML配置文件路径
 * @param output_dir 输出目录路径
 * @param num_workers 数据写入线程数
 */
Bag2Scenes::Bag2Scenes(const fs::path rosbag_dir, const fs::path param_file, const fs::path output_dir, int num_workers) :
odometry_bar_{
        indicators::option::BarWidth{80},
        indicators::option::ForegroundColor{indicators::Color::white},
        indicators::option::PrefixText{"Parsing Ego Odometry "},
        indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}
},
sensor_data_bar_{
    indicators::option::BarWidth{80},
    indicators::option::ForegroundColor{indicators::Color::white},
    indicators::option::PrefixText{"Parsing Sensor Data  "},
    indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}
},
progress_bars_(sensor_data_bar_, odometry_bar_) {
    srand(static_cast<unsigned>(time(nullptr)));
    // Read Parameter File
    try {
        param_yaml_ = YAML::LoadFile(param_file.string());
    } catch (const std::exception& e) {
        std::cerr << "Error reading " << param_file << ": " << e.what() << std::endl;
        exit(1);
    }

    // 检查必要的YAML节点是否存在
    if (!param_yaml_["BAG_INFO"]) {
        std::cerr << "Missing BAG_INFO section in YAML file" << std::endl;
        exit(1);
    }
    
    if (!param_yaml_["SENSOR_INFO"]) {
        std::cerr << "Missing SENSOR_INFO section in YAML file" << std::endl;
        exit(1);
    }
    
    // Read odometry topic
    std::string odom_topic;
    try {
        odom_topic = param_yaml_["BAG_INFO"]["ODOM_TOPIC"].as<std::string>();
    } catch (const YAML::BadConversion& e) {
        std::cerr << "Error converting ODOM_TOPIC to string. Make sure it's a valid string." << std::endl;
        exit(1);
    }
    // ******

    // Storage Options
    storage_options_.uri = rosbag_dir;
    storage_options_.storage_id = "sqlite3";
    // Converter Options
    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";
    // Open Bag and Store Metadata
    rosbag2_cpp::readers::SequentialReader reader;
    try {
        reader.open(storage_options_, converter_options_);
    } catch (const std::exception& e) {
        std::cerr << "Error opening rosbag: " << e.what() << std::endl;
        exit(1);
    }
    std::vector<rosbag2_storage::TopicMetadata> topic_types = reader.get_all_topics_and_types();
    bag_data_ = reader.get_metadata();
    for (const auto& itr : topic_types) {
        std::cout << itr.name << std::endl;
        topic_to_type_.insert({itr.name, itr.type});
    }
    for (YAML::Node::iterator itr = param_yaml_["SENSOR_INFO"].begin(); itr != param_yaml_["SENSOR_INFO"].end(); itr++) {
        std::string sensor_name = itr->first.as<std::string>();
        std::string modality = sensor_name.substr(0, sensor_name.find("_"));
        std::transform(modality.begin(), modality.end(), modality.begin(), ::tolower);
        std::string topic = itr->second["TOPIC"].as<std::string>();
        frame_info_[itr->second["FRAME"].as<std::string>()]["previous_timestamp"] = 0;
        frame_info_[itr->second["FRAME"].as<std::string>()]["previous_token"] = "";
        frame_info_[itr->second["FRAME"].as<std::string>()]["next_token"] = generateToken();
        frame_info_[itr->second["FRAME"].as<std::string>()]["name"] = sensor_name;
        frame_info_[itr->second["FRAME"].as<std::string>()]["sensor_token"] = generateToken();
        frame_info_[itr->second["FRAME"].as<std::string>()]["modality"] = modality;
        if (topic != "null") {
            topics_of_interest_.push_back(topic);
            if (modality == "lidar") {
                lidar_topics_.push_back(topic);
            } else if (modality == "camera") {
                camera_topics_.push_back(topic);
                std::string frame = itr->second["FRAME"].as<std::string>();
                camera_frame_order_.push_back(frame);

                if (itr->second["CALIB"].IsScalar()) {
                    std::string calib_topic = itr->second["CALIB"].as<std::string>();
                    camera_calib_topics_.push_back(calib_topic);
                    camera_calib_inline_[frame] = false;
                    camera_calib_intrinsic_[frame] = std::vector<std::vector<float>>();
                    camera_calib_distortion_[frame] = std::vector<float>();
                    if (calib_topic != "null") {
                        topics_of_interest_.push_back(calib_topic);
                    }
                } else {
                    camera_calib_topics_.push_back("inline");
                    camera_calib_inline_[frame] = true;

                    YAML::Node calib_node = itr->second["CALIB"];
                    float fx = calib_node["fx"].as<float>();
                    float fy = calib_node["fy"].as<float>();
                    float cx = calib_node["cx"].as<float>();
                    float cy = calib_node["cy"].as<float>();

                    std::vector<std::vector<float>> intrinsic = {
                        {fx, 0.0f, cx},
                        {0.0f, fy, cy},
                        {0.0f, 0.0f, 1.0f}
                    };
                    camera_calib_intrinsic_[frame] = intrinsic;

                    float k1 = calib_node["k1"] ? calib_node["k1"].as<float>() : 0.0f;
                    float k2 = calib_node["k2"] ? calib_node["k2"].as<float>() : 0.0f;
                    float p1 = calib_node["p1"] ? calib_node["p1"].as<float>() : 0.0f;
                    float p2 = calib_node["p2"] ? calib_node["p2"].as<float>() : 0.0f;
                    float k3 = calib_node["k3"] ? calib_node["k3"].as<float>() : 0.0f;
                    camera_calib_distortion_[frame] = {k1, k2, p1, p2, k3};

                    std::cout << "  " << frame << " inline calib: fx=" << fx << " fy=" << fy
                              << " cx=" << cx << " cy=" << cy << std::endl;
                }
            } else {
                std::cerr << "Invalid sensor " << sensor_name << " in " << param_file
                          << ". Ensure sensor is of type LIDAR or CAMERA and is named [SENSOR TYPE]_[SENSOR LOCATION]" << std::endl;
                exit(1);
            }
        }
    }
    reader.close();
    bag_dir_ = rosbag_dir.parent_path().filename();
    indicators::show_console_cursor(false);
    if (camera_frame_order_.empty()) {
        std::cerr << "SENSOR_INFO 中未声明任何相机，无法生成 nuScenes sample。" << std::endl;
        exit(1);
    }
    use_batch_camera_strategy_ = true;
    sample_interval_sec_ = 0.5;
    if (param_yaml_["BAG_INFO"]["SAMPLE_INTERVAL"]) {
        sample_interval_sec_ = param_yaml_["BAG_INFO"]["SAMPLE_INTERVAL"].as<double>();
    }
    if (param_yaml_["BAG_INFO"]["SAMPLE_STRATEGY"]) {
        std::string strat = param_yaml_["BAG_INFO"]["SAMPLE_STRATEGY"].as<std::string>();
        std::transform(strat.begin(), strat.end(), strat.begin(), ::tolower);
        if (strat == "time_window" || strat == "synced" || strat == "boss") {
            use_batch_camera_strategy_ = false;
        } else if (strat != "batch_cameras" && strat != "batch" && strat != "async") {
            std::cerr << "BAG_INFO.SAMPLE_STRATEGY 非法，请使用 batch_cameras 或 time_window" << std::endl;
            exit(1);
        }
    }
    if (use_batch_camera_strategy_) {
        std::cout << "SAMPLE_STRATEGY=batch_cameras：每 sample 凑齐 " << camera_frame_order_.size()
                  << " 路相机（未时间同步 bag）。" << std::endl;
    } else {
        previous_sampled_timestamp_ = bag_data_.starting_time.time_since_epoch().count();
        std::cout << "SAMPLE_STRATEGY=time_window：间隔约 " << sample_interval_sec_
                  << " s，按老板版逻辑划分 sample（已时间同步推荐）。" << std::endl;
    }
    previous_sample_token_ = "";
    next_sample_token_ = generateToken();
    nbr_samples_ = 0;
    waiting_timestamp_ = 0;
    num_workers_ = num_workers;
    output_dir_ = output_dir;
    ego_pose_done_ = false;
    std::cout << "\nInitialization complete." << std::endl;
}

/**
 * writeScene - 主场景写入函数
 *
 * 负责协调所有数据的写入流程：
 * 1. 创建输出目录结构
 * 2. 写入日志信息
 * 3. 启动两个并行线程处理里程计和传感器数据
 * 4. 等待线程完成后写入最终的JSON文件
 *
 * 注意：使用线程并行处理是因为里程计数据（EgoPose）和传感器数据的写入顺序
 * 需要保持与bag中消息相同的顺序，这样可以确保时间戳的准确性
 */
void Bag2Scenes::writeScene() {
    MessageConverter message_converter;
    scene_token_ = generateToken();
    if (!fs::exists(output_dir_ / "v1.0-mini/")) {
        fs::create_directory(output_dir_ / "v1.0-mini");
        fs::create_directory(output_dir_ / "samples");
        fs::create_directory(output_dir_ / "sweeps");
    }
    std::string log_token = writeLog();
    nlohmann::json scene;
    scene["token"] = scene_token_;
    scene["log_token"] = log_token;
    scene["first_sample_token"] = next_sample_token_;
    std::unordered_set<std::string> calibrated_sensors;
    nlohmann::json ego_poses;
    nlohmann::json sample_data;
    nlohmann::json scenes;
    if (fs::exists(output_dir_ / "v1.0-mini/ego_pose.json")) {
        std::ifstream ego_poses_in(output_dir_ / "v1.0-mini/ego_pose.json");
        ego_poses = nlohmann::json::parse(ego_poses_in);
        ego_poses_in.close();
    }
    if (fs::exists(output_dir_ / "v1.0-mini/sample.json")) {
        std::ifstream sample_in(output_dir_ / "v1.0-mini/sample.json");
        samples_ = nlohmann::json::parse(sample_in);
        sample_in.close();
    }
    if (fs::exists(output_dir_ / "v1.0-mini/sample_data.json")) {
        std::ifstream sample_data_in(output_dir_ / "v1.0-mini/sample_data.json");
        sample_data = nlohmann::json::parse(sample_data_in);
        sample_data_in.close();
    }
    if (fs::exists(output_dir_ / "v1.0-mini/scene.json")) {
        std::ifstream scene_in(output_dir_ / "v1.0-mini/scene.json");
        scenes = nlohmann::json::parse(scene_in);
        scene_in.close();
    }
    // Write odometry data
    std::thread ego_pose_thread(&Bag2Scenes::writeEgoPose, this, std::ref(ego_poses));
    // Write sensor data
    std::thread sensor_data_thread(&Bag2Scenes::writeSampleData, this, std::ref(sample_data));
    ego_pose_thread.join();
    sensor_data_thread.join();
    std::cout << "Writing Files..." << std::endl;
    std::ofstream ego_poses_out(output_dir_ / "v1.0-mini/ego_pose.json");
    ego_poses_out << ego_poses.dump(4) << std::endl;
    ego_poses_out.close();
    std::ofstream sample_data_out(output_dir_ / "v1.0-mini/sample_data.json");
    sample_data_out << sample_data.dump(4) << std::endl;
    sample_data_out.close();
    if (!samples_.empty()) {
        samples_.back()["next"] = "";
    }
    std::ofstream sample_out(output_dir_ / "v1.0-mini/sample.json");
    sample_out << samples_.dump(4) << std::endl;
    sample_out.close();
    scene["nbr_samples"] = nbr_samples_;
    scene["last_sample_token"] = current_sample_token_;
    scene["name"] = bag_dir_;
    scene["description"] = param_yaml_["BAG_INFO"]["DESCRIPTION"].as<std::string>();
    scenes.push_back(scene);
    std::ofstream scene_out(output_dir_ / "v1.0-mini/scene.json");
    scene_out << scenes.dump(4) << std::endl;
    scene_out.close();
    writeTaxonomyFiles();
    indicators::show_console_cursor(true);
    std::cout << "Done." << std::endl;
}

/**
 * writeLog - 写入日志信息
 *
 * 创建或追加log.json文件，记录本次数据转换的日志信息
 * 包括：bag文件名、日期、采集地点等
 *
 * @return 新创建的log token
 */
std::string Bag2Scenes::writeLog() {
    std::string log_token = generateToken();
    nlohmann::json logs;
    if (fs::exists(output_dir_ / "v1.0-mini/log.json")) {
        std::ifstream log_in(output_dir_ / "v1.0-mini/log.json");
        logs = nlohmann::json::parse(log_in);
        log_in.close();
    }
    nlohmann::json new_log;
    new_log["token"] = log_token;
    new_log["logfile"] = bag_dir_;
    new_log["vehicle"] = param_yaml_["BAG_INFO"]["TEAM"].as<std::string>();
    std::stringstream ss;
    const long time = static_cast<long>(std::floor(bag_data_.starting_time.time_since_epoch().count() * 1e-9));
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d");
    new_log["date_captured"] = ss.str();
    new_log["location"] = param_yaml_["BAG_INFO"]["TRACK"].as<std::string>();
    logs.push_back(new_log);
    std::ofstream log_out(output_dir_ / "v1.0-mini/log.json");
    log_out << logs.dump(4) << std::endl;
    log_out.close();
    writeMap(log_token);
    return log_token;
}

/**
 * writeMap - 写入地图信息
 *
 * 创建或更新map.json文件，记录日志与地图的关联关系
 * 如果已存在相同category的地图，则追加log_token；否则创建新地图记录
 *
 * @param log_token 日志token，用于关联地图和日志
 */
void Bag2Scenes::writeMap(std::string log_token) {
    nlohmann::json maps;
    if (fs::exists(output_dir_ / "v1.0-mini/map.json")) {
        std::ifstream map_in(output_dir_ / "v1.0-mini/map.json");
        maps = nlohmann::json::parse(map_in);
        map_in.close();
        for (nlohmann::json& map : maps) {
            if (map["category"] == param_yaml_["BAG_INFO"]["TRACK"].as<std::string>()) {
                map["log_tokens"].push_back(log_token);
                std::ofstream map_out(output_dir_ / "v1.0-mini/map.json");
                map_out << maps.dump(4) << std::endl;
                map_out.close();
                return;
            }
        }
    }
    nlohmann::json new_map;
    new_map["token"] = generateToken();
    new_map["log_tokens"] = std::vector<std::string>{log_token};
    new_map["category"] = param_yaml_["BAG_INFO"]["TRACK"].as<std::string>();
    new_map["filename"] = "";
    maps.push_back(new_map);
    std::ofstream map_out(output_dir_ / "v1.0-mini/map.json");
    map_out << maps.dump(4) << std::endl;
    map_out.close();
    return;
}

std::string Bag2Scenes::writeSample() {
    return std::string();
}

/**
 * writeSampleData - 写入传感器数据（相机图像和激光雷达点云）
 *
 * 核心处理流程：
 * 1. 遍历 bag 中传感器消息（顺序读取）
 * 2. 相机：按 frame 去重凑满 YAML 中声明的全部相机后，再 flush 为一个 sample（不要求多相机时间对齐）
 * 3. 雷达：同一轮内保留最后一帧，随相机 batch 一并写入
 * 4. 反序列化、写文件、构建 sample_data.json
 *
 * 注意：此函数与writeEgoPose并行运行，通过互斥锁同步
 *
 * @param previous_data 输出参数，用于存储sample_data记录的JSON数组引用
 */
void Bag2Scenes::writeSampleData(nlohmann::json& previous_data) {
    std::unique_lock<std::mutex> lck(ego_pose_mutex_);
    int sensor_data_msgs = 0;
    for (rosbag2_storage::TopicInformation topic_data : bag_data_.topics_with_message_count) {
        if (std::find(topics_of_interest_.begin(), topics_of_interest_.end(), topic_data.topic_metadata.name) != topics_of_interest_.end()) {
            sensor_data_msgs += topic_data.message_count;
        }
    }
    sensor_data_bar_.set_option(indicators::option::MaxProgress{sensor_data_msgs});
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options_, converter_options_);
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = std::vector<std::string> {topics_of_interest_};
    reader.set_filter(storage_filter);
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer = factory.load_deserializer("cdr");
    MessageConverter message_converter;
    SensorDataWriter data_writer(num_workers_);
    std::unordered_set<std::string> calibrated_sensors;

    for (const auto& calib_pair : camera_calib_inline_) {
        if (calib_pair.second) {
            std::string frame = calib_pair.first;
            writeCalibratedSensor(frame, camera_calib_intrinsic_[frame], camera_calib_distortion_[frame]);
            calibrated_sensors.insert("inline_" + frame);
        }
    }

    int expected_calib_topics = 0;
    for (const auto& calib_topic : camera_calib_topics_) {
        if (calib_topic != "inline") {
            expected_calib_topics++;
        }
    }
    lck.unlock();
    for (int i = 0; i < sensor_data_msgs; i++) {
        if (reader.has_next()) {
            auto serialized_message = reader.read_next();
            auto message_wrapper = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            std::string msg_type = topic_to_type_[serialized_message->topic_name];
            message_converter.getROSMsg(msg_type, message_wrapper);
            auto library = rosbag2_cpp::get_typesupport_library(msg_type, "rosidl_typesupport_cpp");
            auto type_support = rosbag2_cpp::get_typesupport_handle(msg_type, "rosidl_typesupport_cpp", library);
            cdr_deserializer->deserialize(serialized_message, type_support, message_wrapper);
            sensor_data_bar_.set_option(indicators::option::PostfixText{
                std::to_string(i+1) + "/" + std::to_string(sensor_data_msgs)
            });
            progress_bars_.tick<0>();
            if (calibrated_sensors.size() == lidar_topics_.size() + camera_calib_topics_.size() && msg_type == "sensor_msgs/msg/CameraInfo") {
                continue;
            }
            if (use_batch_camera_strategy_) {
                if (msg_type == "sensor_msgs/msg/CompressedImage" || msg_type == "sensor_msgs/msg/Image") {
                    CameraMessageT* camera_message = message_converter.getCameraMessage();
                    std::string fid = camera_message->frame_id;
                    if (!fid.empty() && fid[0] == '/') {
                        fid = fid.substr(1);
                    }
                    if (std::find(camera_frame_order_.begin(), camera_frame_order_.end(), fid) == camera_frame_order_.end()) {
                        delete camera_message;
                        continue;
                    }
                    if (pending_cameras_.count(fid)) {
                        delete camera_message;
                        continue;
                    }
                    auto cam_copy = std::make_unique<CameraMessageT>();
                    cam_copy->timestamp = camera_message->timestamp;
                    cam_copy->frame_id = fid;
                    cam_copy->image = camera_message->image.clone();
                    delete camera_message;
                    pending_cameras_[fid] = std::move(cam_copy);
                    if (pending_cameras_.size() == camera_frame_order_.size()) {
                        flushPendingCameraBatch(previous_data, data_writer);
                    }
                    continue;
                }
                if (msg_type == "sensor_msgs/msg/PointCloud2") {
                    LidarMessageT* lidar_message = message_converter.getLidarMessage();
                    if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                        writeCalibratedSensor(lidar_message->frame_id, std::vector<std::vector<float>>(), std::vector<float>());
                        calibrated_sensors.insert({serialized_message->topic_name});
                    }
                    auto lid_copy = std::make_unique<LidarMessageT>();
                    lid_copy->timestamp = lidar_message->timestamp;
                    lid_copy->frame_id = lidar_message->frame_id;
                    lid_copy->cloud = lidar_message->cloud;
                    delete lidar_message;
                    staged_lidar_ = std::move(lid_copy);
                    continue;
                }
                if (msg_type == "sensor_msgs/msg/CameraInfo") {
                    CameraCalibrationT camera_calibration = message_converter.getCameraCalibration();
                    if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                        writeCalibratedSensor(camera_calibration.frame_id, camera_calibration.intrinsic, std::vector<float>());
                        calibrated_sensors.insert({serialized_message->topic_name});
                    }
                    continue;
                }
                std::cerr << "Message filtering broken... Check your param file." << std::endl;
                exit(1);
            } else {
            nlohmann::json sample_data;
            fs::path filename;
            std::string rel_path;
            if (msg_type == "sensor_msgs/msg/CompressedImage" || msg_type == "sensor_msgs/msg/Image") {
                CameraMessageT* camera_message = message_converter.getCameraMessage();
                std::string fid = camera_message->frame_id;
                if (!fid.empty() && fid[0] == '/') {
                    fid = fid.substr(1);
                }
                if (!frame_info_[fid] || !frame_info_[fid]["name"]) {
                    delete camera_message;
                    continue;
                }
                rel_path = sensorDataRelativePath(fid, camera_message->timestamp);
                filename = getFilename(fid, camera_message->timestamp);
                sample_data["token"] = frame_info_[fid]["next_token"].as<std::string>();
                sample_data["calibrated_sensor_token"] = frame_info_[fid]["sensor_token"].as<std::string>();
                sample_data["ego_pose_token"] = getClosestEgoPose(camera_message->timestamp);
                sample_data["height"] = camera_message->image.rows;
                sample_data["width"] = camera_message->image.cols;
                sample_data["timestamp"] = timestampNsToUs(camera_message->timestamp);
                sample_data["prev"] = frame_info_[fid]["previous_token"].as<std::string>();
                frame_info_[fid]["previous_token"] = frame_info_[fid]["next_token"].as<std::string>();
                frame_info_[fid]["next_token"] = generateToken();
                sample_data["next"] = frame_info_[fid]["next_token"].as<std::string>();
                data_writer.writeSensorData(camera_message, filename);
            } else if (msg_type == "sensor_msgs/msg/PointCloud2") {
                LidarMessageT* lidar_message = message_converter.getLidarMessage();
                std::string lid_fid = lidar_message->frame_id;
                if (!lid_fid.empty() && lid_fid[0] == '/') {
                    lid_fid = lid_fid.substr(1);
                }
                if (!frame_info_[lid_fid] || !frame_info_[lid_fid]["name"]) {
                    delete lidar_message;
                    continue;
                }
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(lidar_message->frame_id, std::vector<std::vector<float>>(), std::vector<float>());
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
                rel_path = sensorDataRelativePath(lid_fid, lidar_message->timestamp);
                filename = getFilename(lid_fid, lidar_message->timestamp);
                sample_data["token"] = frame_info_[lid_fid]["next_token"].as<std::string>();
                sample_data["calibrated_sensor_token"] = frame_info_[lid_fid]["sensor_token"].as<std::string>();
                sample_data["ego_pose_token"] = getClosestEgoPose(lidar_message->timestamp);
                sample_data["height"] = 0;
                sample_data["width"] = 0;
                sample_data["timestamp"] = timestampNsToUs(lidar_message->timestamp);
                sample_data["prev"] = frame_info_[lid_fid]["previous_token"].as<std::string>();
                frame_info_[lid_fid]["previous_token"] = frame_info_[lid_fid]["next_token"].as<std::string>();
                frame_info_[lid_fid]["next_token"] = generateToken();
                sample_data["next"] = frame_info_[lid_fid]["next_token"].as<std::string>();
                data_writer.writeSensorData(lidar_message, filename);
            } else if (msg_type == "sensor_msgs/msg/CameraInfo") {
                CameraCalibrationT camera_calibration = message_converter.getCameraCalibration();
                if (calibrated_sensors.find(serialized_message->topic_name) == calibrated_sensors.end()) {
                    writeCalibratedSensor(camera_calibration.frame_id, camera_calibration.intrinsic, std::vector<float>());
                    calibrated_sensors.insert({serialized_message->topic_name});
                }
                continue;
            } else {
                std::cerr << "Message filtering broken... Check your param file." << std::endl;
                exit(1);
            }
            sample_data["sample_token"] = current_sample_token_;
            sample_data["filename"] = rel_path;
            {
                const size_t dot = rel_path.rfind('.');
                sample_data["fileformat"] = (dot != std::string::npos) ? rel_path.substr(dot + 1) : "";
            }
            sample_data["is_key_frame"] = (bool)(rel_path.find("samples") != std::string::npos);
            previous_data.push_back(sample_data);
            }
        }
    }
    if (use_batch_camera_strategy_) {
        if (!pending_cameras_.empty()) {
            std::cerr << "Warning: bag 结束时未凑满一轮相机 ("
                      << pending_cameras_.size() << "/" << camera_frame_order_.size()
                      << ")，已丢弃末尾不完整数据。" << std::endl;
            pending_cameras_.clear();
        }
        staged_lidar_.reset();
    }
    data_writer.close();
}

void Bag2Scenes::flushPendingCameraBatch(nlohmann::json& previous_data, SensorDataWriter& data_writer) {
    current_sample_token_ = next_sample_token_;
    next_sample_token_ = generateToken();

    unsigned long sum_ts_ns = 0;
    for (const auto& frame : camera_frame_order_) {
        auto it = pending_cameras_.find(frame);
        if (it == pending_cameras_.end()) {
            std::cerr << "flushPendingCameraBatch: missing frame " << frame << std::endl;
            pending_cameras_.clear();
            return;
        }
        sum_ts_ns += it->second->timestamp;
    }
    const uint64_t mean_ts_us =
        timestampNsToUs(static_cast<unsigned long>(sum_ts_ns / camera_frame_order_.size()));

    nlohmann::json sample;
    sample["token"] = current_sample_token_;
    sample["timestamp"] = mean_ts_us;
    sample["scene_token"] = scene_token_;
    sample["prev"] = previous_sample_token_;
    sample["next"] = next_sample_token_;
    samples_.push_back(sample);
    previous_sample_token_ = current_sample_token_;
    nbr_samples_++;

    auto append_sensor_row = [&](SensorMessageT* msg, const fs::path& path, const std::string& rel_path, int height, int width) {
        nlohmann::json sample_data;
        std::string ch = msg->frame_id;
        if (!ch.empty() && ch[0] == '/') {
            ch = ch.substr(1);
        }
        sample_data["token"] = frame_info_[ch]["next_token"].as<std::string>();
        sample_data["calibrated_sensor_token"] = frame_info_[ch]["sensor_token"].as<std::string>();
        sample_data["ego_pose_token"] = getClosestEgoPose(msg->timestamp);
        sample_data["height"] = height;
        sample_data["width"] = width;
        sample_data["timestamp"] = timestampNsToUs(msg->timestamp);
        sample_data["prev"] = frame_info_[ch]["previous_token"].as<std::string>();
        frame_info_[ch]["previous_token"] = frame_info_[ch]["next_token"].as<std::string>();
        frame_info_[ch]["next_token"] = generateToken();
        sample_data["next"] = frame_info_[ch]["next_token"].as<std::string>();
        sample_data["sample_token"] = current_sample_token_;
        sample_data["filename"] = rel_path;
        const size_t dot = rel_path.rfind('.');
        sample_data["fileformat"] = (dot != std::string::npos) ? rel_path.substr(dot + 1) : "";
        sample_data["is_key_frame"] = true;
        previous_data.push_back(sample_data);
        // SensorDataWriter 工作线程在写完文件后会 delete 该指针（与 getCameraMessage 堆分配约定一致）
        data_writer.writeSensorData(msg, path);
    };

    for (const auto& frame : camera_frame_order_) {
        CameraMessageT* cam = pending_cameras_[frame].release();
        std::string rel = sensorDataRelativePath(cam->frame_id, cam->timestamp);
        fs::path filename = getFilename(cam->frame_id, cam->timestamp);
        append_sensor_row(cam, filename, rel, cam->image.rows, cam->image.cols);
    }
    pending_cameras_.clear();

    if (staged_lidar_) {
        LidarMessageT* lid = staged_lidar_.release();
        std::string rel = sensorDataRelativePath(lid->frame_id, lid->timestamp);
        fs::path filename = getFilename(lid->frame_id, lid->timestamp);
        append_sensor_row(lid, filename, rel, 0, 0);
    }
}

bool Bag2Scenes::is_key_frame(std::string channel, unsigned long timestamp) {
    if (!channel.empty() && channel[0] == '/') {
        channel = channel.substr(1);
    }
    unsigned long interval_ns = static_cast<unsigned long>(sample_interval_sec_ * 1e9);
    const unsigned long bag_start_ns = static_cast<unsigned long>(bag_data_.starting_time.time_since_epoch().count());
    if (previous_sampled_timestamp_ == bag_start_ns
        || static_cast<long>(timestamp) - static_cast<long>(previous_sampled_timestamp_) > static_cast<long>(interval_ns)) {
        previous_sampled_timestamp_ += interval_ns;
        nbr_samples_++;
        nlohmann::json sample;
        current_sample_token_ = next_sample_token_;
        sample["token"] = current_sample_token_;
        sample["timestamp"] = timestampNsToUs(previous_sampled_timestamp_);
        sample["scene_token"] = scene_token_;
        sample["prev"] = previous_sample_token_;
        previous_sample_token_ = next_sample_token_;
        next_sample_token_ = generateToken();
        sample["next"] = next_sample_token_;
        samples_.push_back(sample);
        sensors_sampled_.clear();
        sensors_sampled_.insert(channel);
        return true;
    }
    if (sensors_sampled_.find(channel) == sensors_sampled_.end()) {
        sensors_sampled_.insert(channel);
        return true;
    }
    return false;
}

/**
 * writeEgoPose - 写入里程计（EgoPose）数据
 *
 * 负责读取bag中的里程计消息并转换为nuScenes格式的ego_pose.json
 * ego_pose表示车辆在世界坐标系中的位姿（位置和朝向）
 *
 * 处理流程：
 * 1. 读取指定话题的里程计消息
 * 2. 将里程计数据（Odometry）转换为变换矩阵
 * 3. 计算相对于首帧的相对位姿
 * 4. 存储为ego_pose记录
 *
 * 注意：此函数与writeSampleData并行运行，通过互斥锁和条件变量同步
 *
 * @param previous_poses 输出参数，用于存储ego_pose记录的JSON数组引用
 */
void Bag2Scenes::writeEgoPose(nlohmann::json& previous_poses) {
    std::unique_lock<std::mutex> lck(ego_pose_mutex_);
    std::string odometry_topic = param_yaml_["BAG_INFO"]["ODOM_TOPIC"].as<std::string>();
    int odometry_msgs = 0;
    for (rosbag2_storage::TopicInformation topic_data : bag_data_.topics_with_message_count) {
        if (topic_data.topic_metadata.name == odometry_topic) {
            odometry_msgs = topic_data.message_count;
        }
    }
    odometry_bar_.set_option(indicators::option::MaxProgress{odometry_msgs});
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options_, converter_options_);
    rosbag2_storage::StorageFilter storage_filter{};
    storage_filter.topics = std::vector<std::string> {odometry_topic};
    reader.set_filter(storage_filter);
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer = factory.load_deserializer("cdr");
    lck.unlock();
    MessageConverter message_converter;
    if (param_yaml_["BAG_INFO"]["LAT_ORIGIN"] && param_yaml_["BAG_INFO"]["LON_ORIGIN"]) {
        double lat_origin = param_yaml_["BAG_INFO"]["LAT_ORIGIN"].as<double>();
        double lon_origin = param_yaml_["BAG_INFO"]["LON_ORIGIN"].as<double>();
        message_converter.setGeoOrigin(lat_origin, lon_origin);
    }
    for (int i = 0; i < odometry_msgs; i++) {
        if (reader.has_next()) {
            auto serialized_message = reader.read_next();
            auto message_wrapper = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            std::string msg_type = topic_to_type_[serialized_message->topic_name];
            message_converter.getROSMsg(msg_type, message_wrapper);
            auto library = rosbag2_cpp::get_typesupport_library(msg_type, "rosidl_typesupport_cpp");
            auto type_support = rosbag2_cpp::get_typesupport_handle(msg_type, "rosidl_typesupport_cpp", library);
            cdr_deserializer->deserialize(serialized_message, type_support, message_wrapper);
            OdometryMessageT odometry_message;
            if (msg_type == "msg_interfaces/msg/Hcinspvatzcb") {
                odometry_message = message_converter.getHcinspvatzcbMessage();
            } else {
                odometry_message = message_converter.getOdometryMessage();
            }
            nlohmann::json new_pose;
            new_pose["token"] = generateToken();
            new_pose["timestamp"] = timestampNsToUs(odometry_message.timestamp);
            new_pose["rotation"] = odometry_message.orientation;
            new_pose["translation"] = odometry_message.position;
            previous_poses.push_back(new_pose);
            if (new_pose["token"].is_string()) {
                std::string token_str = new_pose["token"];
                auto pose_pair = std::make_pair(odometry_message.timestamp, token_str);
                std::unique_lock<std::mutex> lck(ego_pose_mutex_);
                ego_pose_queue_.push_back(pose_pair);
                if (waiting_timestamp_ && odometry_message.timestamp > waiting_timestamp_) {
                    waiting_timestamp_ = 0;
                    ego_pose_ready_.notify_one();
                }
            }
        }
        odometry_bar_.set_option(indicators::option::PostfixText{
            std::to_string(i+1) + "/" + std::to_string(odometry_msgs)
        });
        progress_bars_.tick<1>();
    }
    ego_pose_done_ = true;
}

void Bag2Scenes::writeCalibratedSensor(std::string frame_id, std::vector<std::vector<float>> camera_intrinsic, std::vector<float> distortion_params) {
    nlohmann::json calibrated_sensors;
    if (fs::exists(output_dir_ / "v1.0-mini/calibrated_sensor.json")) {
        std::ifstream calibrated_sensor_in(output_dir_ / "v1.0-mini/calibrated_sensor.json");
        calibrated_sensors = nlohmann::json::parse(calibrated_sensor_in);
        calibrated_sensor_in.close();
    }
    fs::path urdf_file = fs::path("..") / param_yaml_["BAG_INFO"]["URDF"].as<std::string>();
    pugi::xml_document urdf;
    if (!urdf.load_file(urdf_file.c_str())) {
        std::cerr << "Error loading URDF file" << std::endl;
        exit(1);
    }
    pugi::xml_node joint = urdf.child("robot").find_child_by_attribute("type", "fixed");
    while (joint) {
        std::string channel = joint.child("child").attribute("link").value();
        if (channel == frame_id) {
            std::string translation = joint.child("origin").attribute("xyz").value();
            std::string rotation = joint.child("origin").attribute("rpy").value();
            nlohmann::json calibrated_sensor;
            calibrated_sensor["token"] = frame_info_[channel]["sensor_token"].as<std::string>();
            calibrated_sensor["sensor_token"] = writeSensor(channel);
            calibrated_sensor["translation"] = splitString(translation);
            std::vector<float> euler_angles = splitString(rotation);
            if (!euler_angles.size()) {
                euler_angles = {0.0, 0.0, 0.0};
            }
            Eigen::Quaternionf quat;
            if (frame_info_[channel]["modality"].as<std::string>() == "camera") {
                quat = Eigen::AngleAxisf(euler_angles[2] - 0.5 * M_PI, Eigen::Vector3f::UnitZ())
                     * Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY())
                     * Eigen::AngleAxisf(euler_angles[0] - 0.5 * M_PI, Eigen::Vector3f::UnitX());

            } else {
                quat = Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ())
                     * Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY())
                     * Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX());
            }
            std::vector<float> wxyz = {quat.w(), quat.x(), quat.y(), quat.z()};
            calibrated_sensor["rotation"] = wxyz;
            calibrated_sensor["camera_intrinsic"] = camera_intrinsic;
            if (!distortion_params.empty()) {
                calibrated_sensor["distortion_parameters"] = distortion_params;
            }
            calibrated_sensors.push_back(calibrated_sensor);
            std::ofstream calibrated_sensor_out(output_dir_ / "v1.0-mini/calibrated_sensor.json");
            calibrated_sensor_out << calibrated_sensors.dump(4) << std::endl;
            calibrated_sensor_out.close();
        }
        joint = joint.next_sibling();
    }
}

std::string Bag2Scenes::writeSensor(std::string channel) {
    nlohmann::json sensors;
    std::string sensor_token;
    if (!fs::exists(output_dir_ / "v1.0-mini/sensor.json")) {
        nlohmann::json sensor;
        sensor_token = generateToken();
        sensor["token"] = sensor_token;
        std::string directory = frame_info_[channel]["name"].as<std::string>();
        sensor["channel"] = directory;
        sensor["modality"] = frame_info_[channel]["modality"].as<std::string>();
        fs::create_directory(output_dir_ / fs::path("samples") / directory);
        fs::create_directory(output_dir_ / fs::path("sweeps") / directory);
        sensors.push_back(sensor);
        std::ofstream sensor_out(output_dir_ / "v1.0-mini/sensor.json");
        sensor_out << sensors.dump(4) << std::endl;
        sensor_out.close();
    } else {
        std::ifstream sensor_in(output_dir_ / "v1.0-mini/sensor.json");
        std::string directory = frame_info_[channel]["name"].as<std::string>();
        sensors = nlohmann::json::parse(sensor_in);
        sensor_in.close();
        for (nlohmann::json sensor : sensors) {
            if (sensor["channel"] == directory) return sensor["token"];
        }
        nlohmann::json sensor;
        sensor_token = generateToken();
        sensor["token"] = sensor_token;
        sensor["channel"] = directory;
        sensor["modality"] = frame_info_[channel]["modality"].as<std::string>();
        fs::create_directory(output_dir_ / fs::path("samples") / directory);
        fs::create_directory(output_dir_ / fs::path("sweeps") / directory);
        sensors.push_back(sensor);
        std::ofstream sensor_out(output_dir_ / "v1.0-mini/sensor.json");
        sensor_out << sensors.dump(4) << std::endl;
        sensor_out.close();
    }
    return sensor_token;
}

void Bag2Scenes::writeTaxonomyFiles() {
    nlohmann::json category;
    nlohmann::json attribute;
    nlohmann::json visibility;
    nlohmann::json instance;
    nlohmann::json annotation;
    annotation["token"] = generateToken();
    if (!fs::exists(output_dir_ / "v1.0-mini/category.json")) {
        category["token"] = generateToken();
        category["name"] = "";
        category["description"] = "";
        nlohmann::json categories;
        categories.push_back(category);
        std::ofstream category_out(output_dir_ / "v1.0-mini/category.json");
        category_out << categories.dump(4) << std::endl;
        category_out.close();
    }
    if (!fs::exists(output_dir_ / "v1.0-mini/attribute.json")) {
        attribute["token"] = generateToken();
        attribute["name"] = "";
        attribute["description"] = "";
        nlohmann::json attributes;
        attributes.push_back(attribute);
        std::ofstream attribute_out(output_dir_ / "v1.0-mini/attribute.json");
        attribute_out << attributes.dump(4) << std::endl;
        attribute_out.close();
    }
    if (!fs::exists(output_dir_ / "v1.0-mini/visibility.json")) {
        visibility["token"] = "1";
        visibility["description"] = "";
        visibility["level"] = "";
        nlohmann::json visibilities;
        visibilities.push_back(visibility);
        std::ofstream visibility_out(output_dir_ / "v1.0-mini/visibility.json");
        visibility_out << visibilities.dump(4) << std::endl;
        visibility_out.close();
    }
    if (!fs::exists(output_dir_ / "v1.0-mini/instance.json")) {
        instance["token"] = generateToken();
        instance["category_token"] = category["token"];
        instance["nbr_annotations"] = 0;
        instance["first_annotation_token"] = annotation["token"];
        instance["last_annotation_token"] = annotation["token"];
        nlohmann::json instances;
        instances.push_back(instance);
        std::ofstream instance_out(output_dir_ / "v1.0-mini/instance.json");
        instance_out << instances.dump(4) << std::endl;
        instance_out.close();
    }
    if (!fs::exists(output_dir_ / "v1.0-mini/sample_annotation.json")) {
        annotation["sample_token"] = current_sample_token_;
        annotation["instance_token"] = instance["token"];
        annotation["visibility_token"] = visibility["token"];
        annotation["attribute_tokens"] = std::vector<std::string> {attribute["token"]};
        annotation["translation"] = std::vector<float> {0.0, 0.0, 0.0};
        annotation["size"] = std::vector<float> {0.0, 0.0, 0.0};
        annotation["rotation"] = std::vector<float> {1.0, 0.0, 0.0, 0.0};
        annotation["prev"] = "";
        annotation["next"] = "";
        annotation["num_lidar_pts"] = 0;
        annotation["num_radar_pts"] = 0;
        nlohmann::json annotations;
        annotations.push_back(annotation);
        std::ofstream annotation_out(output_dir_ / "v1.0-mini/sample_annotation.json");
        annotation_out << annotations.dump(4) << std::endl;
        annotation_out.close();
    }
}

/**
 * generateToken - 生成32位十六进制随机token
 *
 * 用于唯一标识nuScenes中的各种实体（sample, sensor, pose等）
 * 生成一个32字符的十六进制字符串
 *
 * @return 32位十六进制token字符串
 */
std::string Bag2Scenes::generateToken() {
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_int_distribution<int> dist(0, 15);
    static const char hex_chars[] = "0123456789abcdef";
    char token[33];
    for (int i = 0; i < 32; i++) {
        token[i] = hex_chars[dist(gen)];
    }
    token[32] = '\0';
    return std::string(token);
}

/**
 * splitString - 字符串分割函数
 *
 * 将以空格分隔的字符串分割为浮点数向量
 * 例如："0.5 1.2 3.4" -> [0.5, 1.2, 3.4]
 *
 * @param str 输入字符串
 * @return 浮点数向量
 */
std::vector<float> Bag2Scenes::splitString(std::string str) {
    std::string s;
    std::stringstream ss(str);
    std::vector<float> v;
    while (getline(ss, s, ' ')) {
        v.push_back(std::stof(s));
    }
    return v;
}

uint64_t Bag2Scenes::timestampNsToUs(unsigned long timestamp_ns) {
    return static_cast<uint64_t>(static_cast<unsigned long long>(timestamp_ns) / 1000ULL);
}

std::string Bag2Scenes::sensorDataRelativePath(std::string channel, unsigned long timestamp_ns) {
    if (!channel.empty() && channel[0] == '/') {
        channel = channel.substr(1);
    }
    std::string directory = frame_info_[channel]["name"].as<std::string>();
    std::string modality = directory.substr(0, directory.find("_"));
    std::transform(modality.begin(), modality.end(), modality.begin(), ::tolower);
    std::string extension;
    std::string base_dir;
    if (modality == "lidar") {
        extension = ".pcd.bin";
    } else if (modality == "camera") {
        extension = ".jpg";
    }
    if (use_batch_camera_strategy_) {
        base_dir = "samples";
    } else {
        base_dir = is_key_frame(channel, timestamp_ns) ? "samples" : "sweeps";
    }
    const unsigned long long ts_us = static_cast<unsigned long long>(timestamp_ns) / 1000ULL;
    char buf[4096];
    std::snprintf(buf, sizeof(buf), "%s/%s/%s__%s__%llu%s",
        base_dir.c_str(), directory.c_str(), bag_dir_.c_str(), directory.c_str(), ts_us, extension.c_str());
    return std::string(buf);
}

/** 磁盘写入路径；JSON 中 filename 使用 sensorDataRelativePath（相对数据集根，与 nuScenes 一致） */
fs::path Bag2Scenes::getFilename(std::string channel, unsigned long timestamp) {
    const std::string rel = sensorDataRelativePath(channel, timestamp);
    return fs::path(output_dir_.string() + "/" + rel);
}

/**
 * getClosestEgoPose - 根据时间戳获取最近的EgoPose token
 *
 * 由于传感器数据和里程计数据在不同的线程中并行处理，
 * 这个函数用于在线程间同步并找到与给定传感器时间戳最接近的里程计位姿
 *
 * 处理逻辑：
 * 1. 如果ego_pose_queue_为空（里程计数据尚未准备好），等待
 * 2. 在ego_pose_queue_中找到与给定timestamp最接近的ego_pose
 * 3. 返回该ego_pose的token
 *
 * 注意：这是一个线程安全的函数，使用互斥锁和条件变量进行同步
 *
 * @param timestamp 传感器消息时间戳（纳秒）
 * @return 与给定时间戳最接近的ego_pose的token
 */
std::string Bag2Scenes::getClosestEgoPose(unsigned long timestamp) {
    std::unique_lock<std::mutex> lck(ego_pose_mutex_);
    while (ego_pose_queue_.size() == 0) {
        waiting_timestamp_ = timestamp;
        ego_pose_ready_.wait(lck);
    }
    while (std::get<0>(ego_pose_queue_.back()) < timestamp && !ego_pose_done_) {
        waiting_timestamp_ = timestamp;
        ego_pose_ready_.wait(lck);
    }
    std::string previous_token;
    unsigned long previous_time_difference = INT64_MAX;
    for (unsigned long i = 0; i < ego_pose_queue_.size(); i++) {
        if (std::get<0>(ego_pose_queue_[i]) >= timestamp) {
            std::string return_token;
            if (std::get<0>(ego_pose_queue_[i]) - timestamp < previous_time_difference) {
                return_token = std::get<1>(ego_pose_queue_[i]);
            } else {
                return_token = previous_token;
            }
            ego_pose_queue_.erase(ego_pose_queue_.cbegin(), ego_pose_queue_.cbegin() + i);
            ego_pose_mutex_.unlock();
            return return_token;
        }
        previous_time_difference = timestamp - std::get<0>(ego_pose_queue_[i]);
        previous_token = std::get<1>(ego_pose_queue_[i]);
    }
    return std::get<1>(ego_pose_queue_.back());
}
