#include "rosbag2nuscenes/MessageConverter.hpp"
#include <iostream>
#include <cstdlib>


MessageConverter::MessageConverter() {
    lat_origin_ = 39.0;
    lon_origin_ = 116.0;
}

void MessageConverter::setGeoOrigin(double lat_origin, double lon_origin) {
    lat_origin_ = lat_origin;
    lon_origin_ = lon_origin;
}

LidarMessageT* MessageConverter::getLidarMessage() {
    LidarMessageT* lidar_msg = new LidarMessageT();
    lidar_msg->frame_id = lidar_ros_msg_.header.frame_id;
    lidar_msg->timestamp = static_cast<unsigned long>(lidar_ros_msg_.header.stamp.sec) * 1000000000UL + static_cast<unsigned long>(lidar_ros_msg_.header.stamp.nanosec);
    pcl::fromROSMsg(lidar_ros_msg_, lidar_msg->cloud);
    return lidar_msg;
}

CameraMessageT* MessageConverter::getCameraMessage() {
    CameraMessageT* camera_msg = new CameraMessageT();
    if (!camera_ros_msg_.header.frame_id.empty()) {
        camera_msg->frame_id = camera_ros_msg_.header.frame_id;
        camera_msg->timestamp = static_cast<unsigned long>(camera_ros_msg_.header.stamp.sec) * 1000000000UL + static_cast<unsigned long>(camera_ros_msg_.header.stamp.nanosec);
        camera_msg->image = cv_bridge::toCvCopy(camera_ros_msg_, "bgr8")->image;
    } else if (!image_ros_msg_.header.frame_id.empty()) {
        camera_msg->frame_id = image_ros_msg_.header.frame_id;
        camera_msg->timestamp = static_cast<unsigned long>(image_ros_msg_.header.stamp.sec) * 1000000000UL + static_cast<unsigned long>(image_ros_msg_.header.stamp.nanosec);
        camera_msg->image = cv_bridge::toCvCopy(image_ros_msg_, "bgr8")->image;
    }
    return camera_msg;
}

CameraCalibrationT MessageConverter::getCameraCalibration() {
    CameraCalibrationT calibration_msg;
    calibration_msg.frame_id = camera_info_ros_msg_.header.frame_id;
    for (int i = 0; i < 3; i++) {
        std::vector<float> intrinsic_row;
        for (int j = 0; j < 3; j++) {
            intrinsic_row.push_back(camera_info_ros_msg_.k[3*i + j]);
        }
        calibration_msg.intrinsic.push_back(intrinsic_row);
    }
    return calibration_msg;
}

OdometryMessageT MessageConverter::getOdometryMessage() {
    OdometryMessageT odometry_msg;
    odometry_msg.timestamp = static_cast<unsigned long>(odometry_ros_msg_.header.stamp.sec) * 1000000000UL + static_cast<unsigned long>(odometry_ros_msg_.header.stamp.nanosec);
    odometry_msg.position = { odometry_ros_msg_.pose.pose.position.x, odometry_ros_msg_.pose.pose.position.y, odometry_ros_msg_.pose.pose.position.z };
    odometry_msg.orientation = { odometry_ros_msg_.pose.pose.orientation.w, odometry_ros_msg_.pose.pose.orientation.x, odometry_ros_msg_.pose.pose.orientation.y, odometry_ros_msg_.pose.pose.orientation.z };
    return odometry_msg;
}

OdometryMessageT MessageConverter::getHcinspvatzcbMessage() {
    OdometryMessageT odometry_msg;
    odometry_msg.timestamp = static_cast<unsigned long>(hcinspvatzcb_ros_msg_.header.stamp.sec) * 1000000000UL + static_cast<unsigned long>(hcinspvatzcb_ros_msg_.header.stamp.nanosec);

    double lat_deg = hcinspvatzcb_ros_msg_.latitude;
    double lon_deg = hcinspvatzcb_ros_msg_.longitude;
    double alt_m = hcinspvatzcb_ros_msg_.altitude;
    
    // 检查经纬度和高度值是否有效
    if (std::isnan(lat_deg) || std::isinf(lat_deg)) lat_deg = lat_origin_;
    if (std::isnan(lon_deg) || std::isinf(lon_deg)) lon_deg = lon_origin_;
    if (std::isnan(alt_m) || std::isinf(alt_m)) alt_m = 0.0;

    double x_m = (lon_deg - lon_origin_) * 111320.0 * cos(lat_deg * M_PI / 180.0);
    double y_m = (lat_deg - lat_origin_) * 110540.0;
    double z_m = alt_m;
    
    // 检查计算结果是否有效
    if (std::isnan(x_m) || std::isinf(x_m)) x_m = 0.0;
    if (std::isnan(y_m) || std::isinf(y_m)) y_m = 0.0;
    if (std::isnan(z_m) || std::isinf(z_m)) z_m = 0.0;

    tf2::Quaternion quat;
    // 添加错误检查，确保角度值有效
    double roll = hcinspvatzcb_ros_msg_.roll;
    double pitch = hcinspvatzcb_ros_msg_.pitch;
    double yaw = hcinspvatzcb_ros_msg_.yaw;
    
    // 检查角度值是否有效
    if (std::isnan(roll) || std::isinf(roll)) roll = 0.0;
    if (std::isnan(pitch) || std::isinf(pitch)) pitch = 0.0;
    if (std::isnan(yaw) || std::isinf(yaw)) yaw = 0.0;
    
    quat.setRPY(
        roll * M_PI / 180.0,
        pitch * M_PI / 180.0,
        yaw * M_PI / 180.0
    );
    
    // 检查四元数是否有效
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    
    if (std::isnan(w) || std::isinf(w)) w = 1.0;
    if (std::isnan(x) || std::isinf(x)) x = 0.0;
    if (std::isnan(y) || std::isinf(y)) y = 0.0;
    if (std::isnan(z) || std::isinf(z)) z = 0.0;
    
    // 重新归一化四元数
    double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (norm == 0) {
        w = 1.0;
        x = y = z = 0.0;
    } else {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    odometry_msg.position = { x_m, y_m, z_m };
    odometry_msg.orientation = { w, x, y, z };
    return odometry_msg;
}

void MessageConverter::getROSMsg(std::string type, std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper) {
    if (type == "sensor_msgs/msg/CompressedImage") {
        message_wrapper->message = &camera_ros_msg_;
    } else if (type == "sensor_msgs/msg/Image") {
        message_wrapper->message = &image_ros_msg_;
    } else if (type == "sensor_msgs/msg/PointCloud2") {
        message_wrapper->message = &lidar_ros_msg_;
    } else if (type == "nav_msgs/msg/Odometry") {
        message_wrapper->message = &odometry_ros_msg_;
    } else if (type == "sensor_msgs/msg/CameraInfo") {
        message_wrapper->message = &camera_info_ros_msg_;
    } else if (type == "msg_interfaces/msg/Hcinspvatzcb") {
        message_wrapper->message = &hcinspvatzcb_ros_msg_;
    } else {
        std::cerr << "Message type unknown, cannot deserialize." << std::endl;
        exit(1);
    }
}
