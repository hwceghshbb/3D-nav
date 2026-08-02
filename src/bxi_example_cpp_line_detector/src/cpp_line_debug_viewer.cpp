#include <cstdint>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CppLineDebugViewer : public rclcpp::Node
{
public:
  CppLineDebugViewer()
  : Node("cpp_line_debug_viewer")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/simulation/front_line_camera/cpp_line_debug");
    window_name_ = declare_parameter<std::string>("window_name", "cpp_line_debug");
    window_width_ = declare_parameter<int>("window_width", 640);
    window_height_ = declare_parameter<int>("window_height", 360);

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, window_width_, window_height_);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CppLineDebugViewer::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "showing C++ line debug image: %s", image_topic_.c_str());
  }

  ~CppLineDebugViewer() override
  {
    cv::destroyWindow(window_name_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat bgr;
    if (!rosImageToBgr(*msg, bgr)) {
      return;
    }

    cv::imshow(window_name_, bgr);
    const int key = cv::waitKey(1) & 0xFF;
    if (key == 27 || key == 'q') {
      rclcpp::shutdown();
    }
  }

  bool rosImageToBgr(const sensor_msgs::msg::Image & msg, cv::Mat & bgr)
  {
    const int channels = imageChannels(msg.encoding);
    if (channels == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "unsupported image encoding: %s",
        msg.encoding.c_str());
      return false;
    }

    const int expected_step = static_cast<int>(msg.width) * channels;
    if (static_cast<int>(msg.step) < expected_step ||
      msg.data.size() < static_cast<size_t>(msg.height * msg.step))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid image layout");
      return false;
    }

    cv::Mat raw(static_cast<int>(msg.height), static_cast<int>(msg.step), CV_8UC1, const_cast<uint8_t *>(msg.data.data()));
    cv::Mat packed = raw.colRange(0, expected_step).clone();

    if (msg.encoding == "mono8") {
      cv::Mat mono(static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC1, packed.data);
      cv::cvtColor(mono.clone(), bgr, cv::COLOR_GRAY2BGR);
      return true;
    }

    cv::Mat color(static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC(channels), packed.data);
    if (msg.encoding == "rgb8") {
      cv::cvtColor(color.clone(), bgr, cv::COLOR_RGB2BGR);
    } else if (msg.encoding == "bgr8") {
      bgr = color.clone();
    } else if (msg.encoding == "rgba8") {
      cv::cvtColor(color.clone(), bgr, cv::COLOR_RGBA2BGR);
    } else if (msg.encoding == "bgra8") {
      cv::cvtColor(color.clone(), bgr, cv::COLOR_BGRA2BGR);
    }
    return true;
  }

  int imageChannels(const std::string & encoding) const
  {
    if (encoding == "mono8") {
      return 1;
    }
    if (encoding == "rgb8" || encoding == "bgr8") {
      return 3;
    }
    if (encoding == "rgba8" || encoding == "bgra8") {
      return 4;
    }
    return 0;
  }

  std::string image_topic_;
  std::string window_name_;
  int window_width_{640};
  int window_height_{360};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppLineDebugViewer>());
  rclcpp::shutdown();
  return 0;
}
