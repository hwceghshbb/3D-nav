#ifndef __RGBD_INERTIAL_NODE_HPP__
#define __RGBD_INERTIAL_NODE_HPP__

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class RgbdInertialNode : public rclcpp::Node
{
public:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

    explicit RgbdInertialNode(ORB_SLAM3::System* pSLAM);
    ~RgbdInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabRgb(const ImageMsg::SharedPtr msg);
    void GrabDepth(const ImageMsg::SharedPtr msg);
    void GrabCameraInfo(const CameraInfoMsg::SharedPtr msg);
    void SyncWithImu();
    void PublishRgbd(const ImageMsg::SharedPtr &rgb_msg,
                     const ImageMsg::SharedPtr &depth_msg);
    void PublishMapPoints(const rclcpp::Time &stamp);
    void PublishPoseAndOdom(const Sophus::SE3f &Tcw,
                            const rclcpp::Time &stamp,
                            const std::string &source_frame_id);

    ORB_SLAM3::System* m_SLAM;

    rclcpp::Subscription<ImageMsg>::SharedPtr rgb_sub_;
    rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
    rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::deque<ImageMsg::SharedPtr> rgb_buf_;
    std::deque<ImageMsg::SharedPtr> depth_buf_;
    std::queue<ImuMsg::SharedPtr> imu_buf_;
    ImuMsg::SharedPtr last_imu_msg_;
    CameraInfoMsg::SharedPtr last_camera_info_msg_;
    std::mutex rgb_mutex_;
    std::mutex depth_mutex_;
    std::mutex imu_mutex_;
    std::mutex camera_info_mutex_;

    std::thread sync_thread_;
    std::atomic<bool> running_{true};
    size_t frame_count_ = 0;
    bool use_logical_time_ = false;
    bool use_imu_ = true;
    bool allow_unsynced_rgbd_ = false;
    double max_time_diff_ = 0.03;
    size_t image_buffer_size_ = 6;
    double max_input_lag_ = 0.10;
    double nominal_frame_dt_ = 0.1;
    std::string imu_axis_mode_ = "none";
    bool use_arrival_time_for_unsynced_ = true;
    double imu_init_stable_sec_ = 2.0;
    bool publish_pose_ = true;
    bool publish_tf_ = false;
    std::string map_frame_id_ = "map";
    std::string odom_frame_id_ = "orbslam3_odom";
    std::string child_frame_id_ = "orbslam3_camera";
    bool output_base_pose_ = true;
    Eigen::Vector3f base_from_camera_translation_{0.0628f, 0.0175f, 0.2515f};
    Eigen::Quaternionf base_from_camera_rotation_{0.5f, -0.5f, 0.5f, -0.5f};
    bool time_base_initialized_ = false;
    double time_base_ = 0.0;
    double last_raw_frame_time_ = 0.0;
    double last_orb_frame_time_ = 0.0;
    bool have_last_frame_time_ = false;
    std::chrono::steady_clock::time_point fps_window_start_;
    size_t fps_window_frames_ = 0;
    bool imu_initialized_reported_ = false;
    bool imu_initialized_seen_ = false;
    std::chrono::steady_clock::time_point imu_initialized_since_;
};

#endif
