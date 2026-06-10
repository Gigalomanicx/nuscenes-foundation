#ifndef MESSAGE_CONVERTER_HPP
#define MESSAGE_CONVERTER_HPP

#define PCL_NO_PRECOMPILE

#include <cstring>
#include <cmath>
#include "MessageTypes.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "msg_interfaces/msg/hcinspvatzcb.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>



class MessageConverter {
    public:
        MessageConverter();

        LidarMessageT* getLidarMessage();

        CameraMessageT* getCameraMessage();

        CameraCalibrationT getCameraCalibration();

        OdometryMessageT getOdometryMessage();

        OdometryMessageT getHcinspvatzcbMessage();

        void setGeoOrigin(double lat_origin, double lon_origin);

        void getROSMsg(std::string type, std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper);

    private:
        sensor_msgs::msg::PointCloud2 lidar_ros_msg_;
        sensor_msgs::msg::CompressedImage camera_ros_msg_;
        sensor_msgs::msg::Image image_ros_msg_;
        sensor_msgs::msg::CameraInfo camera_info_ros_msg_;
        nav_msgs::msg::Odometry odometry_ros_msg_;
        msg_interfaces::msg::Hcinspvatzcb hcinspvatzcb_ros_msg_;
        double lat_origin_;
        double lon_origin_;



};


#endif