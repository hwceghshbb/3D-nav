#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

class SensorQosRelay : public rclcpp::Node
{
public:
  SensorQosRelay()
  : Node("sensor_qos_relay")
  {
    const auto depth_input = declare_parameter<std::string>(
      "depth_input", "/hardware/body_depth_camera/depth/image_rect_raw");
    const auto depth_info_input = declare_parameter<std::string>(
      "depth_info_input", "/hardware/body_depth_camera/depth/camera_info");
    const auto color_info_input = declare_parameter<std::string>(
      "color_info_input", "/hardware/body_depth_camera/color/camera_info");
    const auto depth_output = declare_parameter<std::string>(
      "depth_output", "/nav/depth_registration/raw_depth_reliable");
    const auto depth_info_output = declare_parameter<std::string>(
      "depth_info_output", "/nav/depth_registration/depth_info_reliable");
    const auto color_info_output = declare_parameter<std::string>(
      "color_info_output", "/nav/depth_registration/color_info_reliable");

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_output, output_qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      depth_info_output, output_qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      color_info_output, output_qos);

    auto input_qos = rclcpp::SensorDataQoS().keep_last(10);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_input, input_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        depth_pub_->publish(*message);
        ++depth_count_;
      });
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_input, input_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        depth_info_pub_->publish(*message);
        ++depth_info_count_;
      });
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_input, input_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        color_info_pub_->publish(*message);
        ++color_info_count_;
      });

    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(2), [this]() {
        RCLCPP_INFO(
          get_logger(),
          "QoS relay totals: depth=%lu depth_info=%lu color_info=%lu",
          depth_count_, depth_info_count_, color_info_count_);
      });

    RCLCPP_INFO(
      get_logger(),
      "Relaying camera inputs from BEST_EFFORT to RELIABLE for depth registration");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  uint64_t depth_count_{0};
  uint64_t depth_info_count_{0};
  uint64_t color_info_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorQosRelay>());
  rclcpp::shutdown();
  return 0;
}
