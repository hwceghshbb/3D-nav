#include "rgbd-inertial-node.hpp"

#include <algorithm>
#include <chrono>

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

namespace
{
cv::Point3f ConvertImuVector(const geometry_msgs::msg::Vector3 &v, const std::string &axis_mode)
{
    if (axis_mode == "ros_base_to_orb_camera")
        return cv::Point3f(-v.y, -v.z, v.x);
    return cv::Point3f(v.x, v.y, v.z);
}

Eigen::Matrix3f OrbWorldToRosMapRotation()
{
    Eigen::Matrix3f R;
    R << 0.0f, 0.0f, 1.0f,
        -1.0f, 0.0f, 0.0f,
         0.0f,-1.0f, 0.0f;
    return R;
}
}

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    auto image_qos = rclcpp::SensorDataQoS();
    auto imu_qos = rclcpp::SensorDataQoS();

    use_logical_time_ = this->declare_parameter<bool>("use_logical_time", false);
    use_imu_ = this->declare_parameter<bool>("use_imu", true);
    allow_unsynced_rgbd_ = this->declare_parameter<bool>("allow_unsynced_rgbd", false);
    max_time_diff_ = this->declare_parameter<double>("max_time_diff", 0.03);
    image_buffer_size_ = static_cast<size_t>(std::max<int64_t>(
        2, this->declare_parameter<int>("image_buffer_size", 6)));
    max_input_lag_ = std::max(
        0.0, this->declare_parameter<double>("max_input_lag", 0.10));
    nominal_frame_dt_ = this->declare_parameter<double>("nominal_frame_dt", 0.1);
    imu_axis_mode_ = this->declare_parameter<std::string>("imu_axis_mode", "none");
    use_arrival_time_for_unsynced_ = this->declare_parameter<bool>("use_arrival_time_for_unsynced", true);
    imu_init_stable_sec_ = this->declare_parameter<double>("imu_init_stable_sec", 2.0);
    publish_pose_ = this->declare_parameter<bool>("publish_pose", true);
    publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
    map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "orbslam3_odom");
    child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "orbslam3_camera");
    output_base_pose_ = this->declare_parameter<bool>("output_base_pose", true);
    const auto base_translation = this->declare_parameter<std::vector<double>>(
        "base_from_camera_translation", {0.0628, 0.0175, 0.2515});
    const auto base_quaternion = this->declare_parameter<std::vector<double>>(
        "base_from_camera_quaternion", {-0.5, 0.5, -0.5, 0.5});
    if (base_translation.size() != 3 || base_quaternion.size() != 4)
        throw std::runtime_error("base_from_camera pose must contain 3 translation and 4 XYZW quaternion values");
    base_from_camera_translation_ = Eigen::Vector3f(
        base_translation[0], base_translation[1], base_translation[2]);
    base_from_camera_rotation_ = Eigen::Quaternionf(
        base_quaternion[3], base_quaternion[0], base_quaternion[1], base_quaternion[2]);
    base_from_camera_rotation_.normalize();
    fps_window_start_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(),
                "ORB-SLAM3 %s params: use_logical_time=%s allow_unsynced_rgbd=%s max_time_diff=%.3f image_buffer_size=%zu max_input_lag=%.3f nominal_frame_dt=%.3f imu_axis_mode=%s use_arrival_time_for_unsynced=%s imu_init_stable_sec=%.1f publish_pose=%s publish_tf=%s map_frame_id=%s odom_frame_id=%s child_frame_id=%s",
                use_imu_ ? "RGB-D-Inertial" : "RGB-D",
                use_logical_time_ ? "true" : "false",
                allow_unsynced_rgbd_ ? "true" : "false",
                max_time_diff_, image_buffer_size_, max_input_lag_, nominal_frame_dt_, imu_axis_mode_.c_str(),
                use_arrival_time_for_unsynced_ ? "true" : "false",
                imu_init_stable_sec_,
                publish_pose_ ? "true" : "false",
                publish_tf_ ? "true" : "false",
                map_frame_id_.c_str(), odom_frame_id_.c_str(),
                child_frame_id_.c_str());

    rgb_sub_ = this->create_subscription<ImageMsg>(
        "camera/rgb", image_qos, std::bind(&RgbdInertialNode::GrabRgb, this, _1));
    depth_sub_ = this->create_subscription<ImageMsg>(
        "camera/depth", image_qos, std::bind(&RgbdInertialNode::GrabDepth, this, _1));
    camera_info_sub_ = this->create_subscription<CameraInfoMsg>(
        "camera/camera_info", image_qos,
        std::bind(&RgbdInertialNode::GrabCameraInfo, this, _1));
    if (use_imu_)
        imu_sub_ = this->create_subscription<ImuMsg>(
            "camera/imu", imu_qos, std::bind(&RgbdInertialNode::GrabImu, this, _1));

    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "orbslam3/map_points", rclcpp::QoS(10).reliable());
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "orbslam3/pose", rclcpp::QoS(10).reliable());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "orbslam3/odom", rclcpp::QoS(10).reliable());
    rgbd_pub_ = this->create_publisher<rtabmap_msgs::msg::RGBDImage>(
        "orbslam3/rgbd_image", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sync_thread_ = std::thread(&RgbdInertialNode::SyncWithImu, this);
}

RgbdInertialNode::~RgbdInertialNode()
{
    running_ = false;
    if (sync_thread_.joinable())
        sync_thread_.join();

    m_SLAM->Shutdown();

    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectoryEuRoC.txt");
    m_SLAM->SaveTrajectoryEuRoC("FrameTrajectoryEuRoC.txt");
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    last_imu_msg_ = msg;
    imu_buf_.push(msg);
    while (imu_buf_.size() > 1000)
        imu_buf_.pop();
}

void RgbdInertialNode::GrabRgb(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    rgb_buf_.push_back(msg);
    while (rgb_buf_.size() > image_buffer_size_)
        rgb_buf_.pop_front();
}

void RgbdInertialNode::GrabDepth(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_buf_.push_back(msg);
    while (depth_buf_.size() > image_buffer_size_)
        depth_buf_.pop_front();
}

void RgbdInertialNode::GrabCameraInfo(const CameraInfoMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    last_camera_info_msg_ = msg;
}

void RgbdInertialNode::SyncWithImu()
{
    while (running_)
    {
        ImageMsg::SharedPtr rgb_msg;
        ImageMsg::SharedPtr depth_msg;
        double rgb_time = 0.0;
        double depth_time = 0.0;
        bool wait_for_input = false;

        {
            std::lock_guard<std::mutex> rgb_lock(rgb_mutex_);
            std::lock_guard<std::mutex> depth_lock(depth_mutex_);
            std::lock_guard<std::mutex> imu_lock(imu_mutex_);
            const bool cached_imu_available = allow_unsynced_rgbd_ &&
                use_arrival_time_for_unsynced_ && static_cast<bool>(last_imu_msg_);
            const bool imu_available = !use_imu_ || !imu_buf_.empty() || cached_imu_available;

            if (rgb_buf_.empty() || depth_buf_.empty() || !imu_available)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Waiting for buffers: rgb=%zu depth=%zu imu=%zu cached_imu=%s",
                                     rgb_buf_.size(), depth_buf_.size(), imu_buf_.size(),
                                     cached_imu_available ? "true" : "false");
                wait_for_input = true;
            }
            else
            {
                rgb_time = Utility::StampToSec(rgb_buf_.front()->header.stamp);
                depth_time = Utility::StampToSec(depth_buf_.front()->header.stamp);

                if (allow_unsynced_rgbd_)
                {
                    rgb_msg = rgb_buf_.back();
                    depth_msg = depth_buf_.back();
                    const double rgb_header_time = Utility::StampToSec(rgb_msg->header.stamp);
                    const double depth_header_time = Utility::StampToSec(depth_msg->header.stamp);
                    rgb_time = rgb_header_time;
                    depth_time = depth_header_time;
                    if (use_arrival_time_for_unsynced_)
                    {
                        rgb_time = this->now().seconds();
                        depth_time = rgb_time;
                    }

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Pairing latest RGB-D frames: rgb_header=%.6f depth_header=%.6f header_diff=%.6f tracking_time_source=%s",
                                         rgb_header_time, depth_header_time, std::abs(rgb_header_time - depth_header_time),
                                         use_arrival_time_for_unsynced_ ? "arrival" : "header");
                    while (!rgb_buf_.empty())
                        rgb_buf_.pop_front();
                    while (!depth_buf_.empty())
                        depth_buf_.pop_front();
                }
                else
                {
                    size_t dropped_rgb = 0;
                    size_t dropped_depth = 0;
                    bool pair_found = false;

                    if (max_input_lag_ > 0.0)
                    {
                        const double newest_common_time = std::min(
                            Utility::StampToSec(rgb_buf_.back()->header.stamp),
                            Utility::StampToSec(depth_buf_.back()->header.stamp));
                        const double oldest_allowed_time = newest_common_time - max_input_lag_;
                        while (!rgb_buf_.empty() &&
                               Utility::StampToSec(rgb_buf_.front()->header.stamp) < oldest_allowed_time)
                        {
                            rgb_buf_.pop_front();
                            ++dropped_rgb;
                        }
                        while (!depth_buf_.empty() &&
                               Utility::StampToSec(depth_buf_.front()->header.stamp) < oldest_allowed_time)
                        {
                            depth_buf_.pop_front();
                            ++dropped_depth;
                        }
                    }

                    while (!rgb_buf_.empty() && !depth_buf_.empty())
                    {
                        rgb_time = Utility::StampToSec(rgb_buf_.front()->header.stamp);
                        depth_time = Utility::StampToSec(depth_buf_.front()->header.stamp);
                        const double rgbd_diff = std::abs(rgb_time - depth_time);

                        if (rgbd_diff <= max_time_diff_)
                        {
                            pair_found = true;
                            break;
                        }

                        if (rgb_time < depth_time)
                        {
                            rgb_buf_.pop_front();
                            ++dropped_rgb;
                        }
                        else
                        {
                            depth_buf_.pop_front();
                            ++dropped_depth;
                        }
                    }

                    if (dropped_rgb > 0 || dropped_depth > 0)
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Dropped unsynced RGB-D frames while catching up: rgb=%zu depth=%zu last_diff=%.6f max=%.6f remaining_rgb=%zu remaining_depth=%zu",
                                             dropped_rgb, dropped_depth, std::abs(rgb_time - depth_time),
                                             max_time_diff_, rgb_buf_.size(), depth_buf_.size());
                    }

                    if (!pair_found)
                    {
                        wait_for_input = true;
                    }
                    else if (use_imu_ &&
                             rgb_time > Utility::StampToSec(imu_buf_.back()->header.stamp))
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Waiting for IMU to catch up: rgb=%.6f latest_imu=%.6f",
                                             rgb_time, Utility::StampToSec(imu_buf_.back()->header.stamp));
                        wait_for_input = true;
                    }
                    else
                    {
                        rgb_msg = rgb_buf_.front();
                        depth_msg = depth_buf_.front();
                        rgb_buf_.pop_front();
                        depth_buf_.pop_front();
                    }
                }
            }
        }

        if (wait_for_input)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (!time_base_initialized_)
        if (use_imu_)
        {
            time_base_ = rgb_time;
            time_base_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "RGB-D-Inertial time base initialized at raw stamp %.6f", time_base_);
        }

        cv_bridge::CvImageConstPtr cv_ptr_rgb;
        cv_bridge::CvImageConstPtr cv_ptr_depth;
        try
        {
            cv_ptr_rgb = cv_bridge::toCvShare(rgb_msg);
            cv_ptr_depth = cv_bridge::toCvShare(depth_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            continue;
        }

        std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
        const double orb_rgb_time = use_logical_time_
            ? (have_last_frame_time_ ? last_orb_frame_time_ + nominal_frame_dt_ : 0.0)
            : rgb_time - time_base_;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (allow_unsynced_rgbd_ && use_arrival_time_for_unsynced_)
            {
                std::vector<ImuMsg::SharedPtr> latest_imu;
                latest_imu.reserve(32);
                while (!imu_buf_.empty())
                {
                    latest_imu.push_back(imu_buf_.front());
                    imu_buf_.pop();
                }

                if (latest_imu.empty() && last_imu_msg_)
                    latest_imu.push_back(last_imu_msg_);

                if (have_last_frame_time_ && !latest_imu.empty())
                {
                    const size_t available_count = std::min<size_t>(latest_imu.size(), 20);
                    const size_t sample_count = std::max<size_t>(available_count, 2);
                    const size_t start_index = latest_imu.size() > available_count
                        ? latest_imu.size() - available_count
                        : 0;
                    const double interval = std::max(orb_rgb_time - last_orb_frame_time_, nominal_frame_dt_);
                    cv::Point3f last_acc(0.0f, 0.0f, 0.0f);
                    cv::Point3f last_gyr(0.0f, 0.0f, 0.0f);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Using %zu recent IMU samples for arrival-time RGB-D frame",
                                         available_count);
                    for (size_t i = 0; i < sample_count; ++i)
                    {
                        if (i < available_count)
                        {
                            const auto &imu_msg = latest_imu[start_index + i];
                            last_acc = ConvertImuVector(imu_msg->linear_acceleration, imu_axis_mode_);
                            last_gyr = ConvertImuVector(imu_msg->angular_velocity, imu_axis_mode_);
                        }
                        const double alpha = static_cast<double>(i + 1) / static_cast<double>(sample_count + 1);
                        const double orb_imu_time = last_orb_frame_time_ + alpha * interval;
                        imu_measurements.emplace_back(last_acc, last_gyr, orb_imu_time);
                    }
                }
            }
            else
            {
                const double min_imu_time = have_last_frame_time_ ? last_raw_frame_time_ : time_base_;
                const double raw_imu_span = rgb_time - min_imu_time;

                while (!imu_buf_.empty() && Utility::StampToSec(imu_buf_.front()->header.stamp) < min_imu_time)
                    imu_buf_.pop();

                while (!imu_buf_.empty() && Utility::StampToSec(imu_buf_.front()->header.stamp) <= rgb_time)
                {
                    const auto &imu_msg = imu_buf_.front();
                    const double imu_time = Utility::StampToSec(imu_msg->header.stamp);
                    const cv::Point3f acc = ConvertImuVector(imu_msg->linear_acceleration, imu_axis_mode_);
                    const cv::Point3f gyr = ConvertImuVector(imu_msg->angular_velocity, imu_axis_mode_);
                    double orb_imu_time = imu_time - time_base_;
                    if (use_logical_time_ && have_last_frame_time_ && raw_imu_span > 1e-6)
                    {
                        const double alpha = (imu_time - min_imu_time) / raw_imu_span;
                        orb_imu_time = last_orb_frame_time_ + alpha * nominal_frame_dt_;
                    }
                    imu_measurements.emplace_back(acc, gyr, orb_imu_time);
                    imu_buf_.pop();
                }
            }
        }

        if (use_imu_ && !have_last_frame_time_)
        {
            last_raw_frame_time_ = rgb_time;
            last_orb_frame_time_ = 0.0;
            have_last_frame_time_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Skipping first RGB-D-Inertial frame while collecting IMU history");
            continue;
        }

        if (use_imu_ && (orb_rgb_time <= 0.0 || imu_measurements.size() < 2))
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for enough IMU samples before RGB-D-Inertial tracking");
            continue;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Tracking ORB-SLAM3 %s frame at %.6f with %zu IMU samples",
                             use_imu_ ? "RGB-D-Inertial" : "RGB-D",
                             orb_rgb_time, imu_measurements.size());

        const auto track_start = std::chrono::steady_clock::now();
        const Sophus::SE3f Tcw = m_SLAM->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image,
                                                   orb_rgb_time, imu_measurements);
        const auto track_end = std::chrono::steady_clock::now();
        const double track_ms = std::chrono::duration<double, std::milli>(track_end - track_start).count();
        last_raw_frame_time_ = rgb_time;
        last_orb_frame_time_ = orb_rgb_time;

        frame_count_++;
        fps_window_frames_++;
        const bool imu_initialized_raw = !use_imu_ || m_SLAM->IsImuInitialized();
        bool imu_initialized_stable = !use_imu_;

        if (!use_imu_)
        {
            imu_initialized_stable = true;
        }
        else if (imu_initialized_raw)
        {
            if (!imu_initialized_seen_)
            {
                imu_initialized_seen_ = true;
                imu_initialized_since_ = track_end;
                RCLCPP_INFO(this->get_logger(),
                            "ORB-SLAM3 IMU initialization detected. Waiting %.1f s for it to stay stable before publishing map points.",
                            imu_init_stable_sec_);
            }

            const double stable_sec =
                std::chrono::duration<double>(track_end - imu_initialized_since_).count();
            imu_initialized_stable = stable_sec >= imu_init_stable_sec_;
        }
        else
        {
            imu_initialized_seen_ = false;
            imu_initialized_reported_ = false;
        }

        if (use_imu_ && !imu_initialized_stable)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Waiting for stable ORB-SLAM3 IMU initialization. Keep slow translation and small rotations; map points are not published yet.");
        }
        else if (use_imu_ && !imu_initialized_reported_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "ORB-SLAM3 IMU initialization stayed stable. Publishing map points and continuing normal RGB-D-Inertial tracking.");
            imu_initialized_reported_ = true;
        }

        const double fps_window_sec = std::chrono::duration<double>(track_end - fps_window_start_).count();
        if (fps_window_sec >= 2.0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "ORB-SLAM3 %s throughput: %.2f FPS, last TrackRGBD %.1f ms, imu_initialized=%s imu_stable=%s",
                        use_imu_ ? "RGB-D-Inertial" : "RGB-D",
                        static_cast<double>(fps_window_frames_) / fps_window_sec,
                        track_ms,
                        imu_initialized_raw ? "true" : "false",
                        imu_initialized_stable ? "true" : "false");
            fps_window_start_ = track_end;
            fps_window_frames_ = 0;
        }

        if (imu_initialized_stable && frame_count_ % 10 == 0)
            PublishMapPoints(rgb_msg->header.stamp);

        if (imu_initialized_stable && !m_SLAM->isLost())
        {
            PublishPoseAndOdom(Tcw, rgb_msg->header.stamp, rgb_msg->header.frame_id);
            PublishRgbd(rgb_msg, depth_msg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RgbdInertialNode::PublishRgbd(const ImageMsg::SharedPtr &rgb_msg,
                                   const ImageMsg::SharedPtr &depth_msg)
{
    CameraInfoMsg::SharedPtr camera_info;
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        camera_info = last_camera_info_msg_;
    }
    if (!camera_info)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Cannot publish synchronized RGB-D: camera info is unavailable");
        return;
    }

    rtabmap_msgs::msg::RGBDImage message;
    message.header = rgb_msg->header;
    message.rgb_camera_info = *camera_info;
    message.rgb_camera_info.header = rgb_msg->header;
    message.depth_camera_info = message.rgb_camera_info;
    message.rgb = *rgb_msg;
    message.depth = *depth_msg;
    message.depth.header = rgb_msg->header;
    rgbd_pub_->publish(message);
}

void RgbdInertialNode::PublishPoseAndOdom(const Sophus::SE3f &Tcw,
                                          const rclcpp::Time &stamp,
                                          const std::string &source_frame_id)
{
    if (!publish_pose_ && !publish_tf_)
        return;

    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Matrix3f C = OrbWorldToRosMapRotation();
    Eigen::Matrix3f R_ros = C * Twc.rotationMatrix();
    Eigen::Vector3f t_ros = C * Twc.translation();
    if (output_base_pose_)
    {
        R_ros = R_ros * base_from_camera_rotation_.toRotationMatrix().transpose();
        t_ros = t_ros - R_ros * base_from_camera_translation_;
    }
    Eigen::Quaternionf q_ros(R_ros);
    q_ros.normalize();

    const std::string child_frame =
        child_frame_id_.empty()
            ? (source_frame_id.empty() ? "orbslam3_camera" : source_frame_id)
            : child_frame_id_;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose.position.x = t_ros.x();
    pose_msg.pose.position.y = t_ros.y();
    pose_msg.pose.position.z = t_ros.z();
    pose_msg.pose.orientation.x = q_ros.x();
    pose_msg.pose.orientation.y = q_ros.y();
    pose_msg.pose.orientation.z = q_ros.z();
    pose_msg.pose.orientation.w = q_ros.w();

    if (publish_pose_)
    {
        pose_pub_->publish(pose_msg);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = child_frame;
        odom_msg.pose.pose = pose_msg.pose;
        odom_pub_->publish(odom_msg);
    }

    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = pose_msg.header;
        tf_msg.child_frame_id = child_frame;
        tf_msg.transform.translation.x = pose_msg.pose.position.x;
        tf_msg.transform.translation.y = pose_msg.pose.position.y;
        tf_msg.transform.translation.z = pose_msg.pose.position.z;
        tf_msg.transform.rotation = pose_msg.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

void RgbdInertialNode::PublishMapPoints(const rclcpp::Time &stamp)
{
    std::vector<ORB_SLAM3::MapPoint*> map_points = m_SLAM->GetAllMapPoints();

    std::vector<Eigen::Vector3f> points;
    points.reserve(map_points.size());
    for (ORB_SLAM3::MapPoint* map_point : map_points)
    {
        if (!map_point || map_point->isBad())
            continue;
        points.push_back(map_point->GetWorldPos());
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "map";
    cloud.height = 1;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const Eigen::Vector3f &point : points)
    {
        *iter_x = point.z();
        *iter_y = -point.x();
        *iter_z = -point.y();
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    map_points_pub_->publish(cloud);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Published %zu ORB-SLAM3 map points", points.size());
}
