#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{

struct Segment
{
  double center_x{};
  int start_x{};
  int end_x{};
};

struct FitResult
{
  bool valid{false};
  cv::Vec3d coeff{0.0, 0.0, 0.0};  // x = a*y^2 + b*y + c
};

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double rateLimitedEma(double previous, double raw, double alpha, double max_step)
{
  const double delta = clamp(raw - previous, -max_step, max_step);
  return (1.0 - alpha) * previous + alpha * (previous + delta);
}

}  // namespace

class CppLineDetector : public rclcpp::Node
{
public:
  CppLineDetector()
  : Node("cpp_line_detector")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/simulation/front_line_camera/image_raw");
    line_offset_topic_ = declare_parameter<std::string>("line_offset_topic", "/simulation/front_line_camera/line_offset");
    line_state_topic_ = declare_parameter<std::string>("line_state_topic", "/simulation/front_line_camera/line_state");
    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/simulation/front_line_camera/cpp_line_debug");
    enable_debug_image_ = declare_parameter<bool>("enable_debug_image", true);
    threshold_value_ = declare_parameter<int>("threshold_value", 155);
    canny_low_ = declare_parameter<double>("canny_low", 60.0);
    canny_high_ = declare_parameter<double>("canny_high", 150.0);
    roi_top_ratio_ = declare_parameter<double>("roi_top_ratio", 0.50);
    roi_bottom_ratio_ = declare_parameter<double>("roi_bottom_ratio", 0.96);
    scan_band_height_ = declare_parameter<int>("scan_band_height", 7);
    scan_stride_ = declare_parameter<int>("scan_stride", 7);
    min_rows_ = declare_parameter<int>("min_rows", 4);
    smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.18);

    line_offset_pub_ = create_publisher<std_msgs::msg::Float32>(line_offset_topic_, 10);
    line_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(line_state_topic_, 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 1);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CppLineDetector::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "cpp line detector: %s -> %s",
      image_topic_.c_str(),
      line_state_topic_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat bgr;
    if (!rosImageToBgr(*msg, bgr)) {
      return;
    }

    const int width = bgr.cols;
    const int height = bgr.rows;
    if (width <= 0 || height <= 0) {
      return;
    }
    image_height_ = height;

    if (!initialized_) {
      last_lane_width_ = width * 0.42;
      last_lane_center_ = width * 0.5;
      initialized_ = true;
    }

    const int roi_top = static_cast<int>(clamp(height * roi_top_ratio_, 0.0, height - 1.0));
    const int roi_bottom = static_cast<int>(clamp(height * roi_bottom_ratio_, roi_top + 1.0, height - 1.0));
    const double image_center_x = width * 0.5;

    cv::Mat gray = toGray(bgr);                              // 1. 灰度化
    cv::Mat denoised = denoise(gray);                        // 2. 降噪
    cv::Mat binary = thresholdImage(denoised, roi_top, roi_bottom);  // 3. 阈值分割
    cv::Mat edges = detectEdges(denoised, roi_top, roi_bottom);      // 4. 边缘检测
    cv::Mat line_mask = buildLineMask(binary, edges, roi_top, roi_bottom);

    std::vector<cv::Point2d> left_points;
    std::vector<cv::Point2d> right_points;
    std::vector<cv::Point2d> center_points;
    double measured_lane_width = last_lane_width_;
    const double confidence = scanLane(
      line_mask,
      roi_top,
      roi_bottom,
      width,
      image_center_x,
      left_points,
      right_points,
      center_points,
      measured_lane_width);

    FitResult center_fit;
    if (static_cast<int>(center_points.size()) >= min_rows_) {
      center_fit = fitQuadratic(center_points);              // 5. 二次曲线拟合
    }

    double raw_offset = last_offset_;
    double raw_heading = 0.0;
    if (center_fit.valid) {
      last_lane_width_ = 0.90 * last_lane_width_ + 0.10 * measured_lane_width;
      const double near_y = roi_bottom - 4.0;
      const double raw_center = evalPoly(center_fit.coeff, near_y);
      last_lane_center_ = rateLimitedEma(last_lane_center_, raw_center, smoothing_alpha_, width * 0.06);

      raw_offset = computeOffset(center_fit.coeff, roi_top, roi_bottom, image_center_x);
      last_center_coeff_ = center_fit.coeff;
      has_last_center_coeff_ = true;
      lost_frames_ = 0;
    } else {
      lost_frames_++;
      if (has_last_center_coeff_ && lost_frames_ <= 5) {
        raw_offset = computeOffset(last_center_coeff_, roi_top, roi_bottom, image_center_x);
      } else {
        raw_offset = last_offset_ * 0.96;
      }
    }

    const double alpha = confidence > 0.0 ? (0.08 + 0.18 * confidence) : 0.08;
    last_offset_ = rateLimitedEma(last_offset_, raw_offset, alpha, 0.08);
    last_heading_ = raw_heading;
    last_control_ = last_offset_;

    publishState(confidence);
    if (enable_debug_image_) {
      const cv::Vec3d debug_coeff = center_fit.valid ? center_fit.coeff : last_center_coeff_;
      const bool has_debug_coeff = center_fit.valid || has_last_center_coeff_;
      cv::Mat debug = makeDebugImage(
        bgr,
        binary,
        edges,
        line_mask,
        roi_top,
        roi_bottom,
        left_points,
        right_points,
        center_points,
        debug_coeff,
        has_debug_coeff,
        confidence);
      publishDebugImage(debug, msg->header.stamp);
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

  cv::Mat toGray(const cv::Mat & bgr) const
  {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }

  cv::Mat denoise(const cv::Mat & gray) const
  {
    cv::Mat denoised;
    cv::GaussianBlur(gray, denoised, cv::Size(5, 5), 0.0);
    return denoised;
  }

  cv::Mat thresholdImage(const cv::Mat & denoised, int roi_top, int roi_bottom) const
  {
    cv::Mat binary;
    cv::threshold(denoised, binary, threshold_value_, 255, cv::THRESH_BINARY);
    applyRectRoi(binary, roi_top, roi_bottom);
    return binary;
  }

  cv::Mat detectEdges(const cv::Mat & denoised, int roi_top, int roi_bottom) const
  {
    cv::Mat edges;
    cv::Canny(denoised, edges, canny_low_, canny_high_);
    applyRectRoi(edges, roi_top, roi_bottom);
    return edges;
  }

  cv::Mat buildLineMask(const cv::Mat & binary, const cv::Mat & edges, int roi_top, int roi_bottom) const
  {
    cv::Mat edge_dilated;
    cv::dilate(edges, edge_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    cv::Mat mask;
    cv::bitwise_or(binary, edge_dilated, mask);
    applyRectRoi(mask, roi_top, roi_bottom);

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));
    cv::morphologyEx(
      mask,
      mask,
      cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 7)));
    return mask;
  }

  void applyRectRoi(cv::Mat & image, int roi_top, int roi_bottom) const
  {
    image.rowRange(0, std::max(0, roi_top)).setTo(0);
    if (roi_bottom + 1 < image.rows) {
      image.rowRange(roi_bottom + 1, image.rows).setTo(0);
    }
  }

  double scanLane(
    const cv::Mat & mask,
    int roi_top,
    int roi_bottom,
    int width,
    double image_center_x,
    std::vector<cv::Point2d> & left_points,
    std::vector<cv::Point2d> & right_points,
    std::vector<cv::Point2d> & center_points,
    double & measured_lane_width)
  {
    const int band_height = std::max(3, scan_band_height_);
    const int stride = std::max(3, scan_stride_);
    const int max_line_width = std::max(12, static_cast<int>(width * 0.075));
    const double min_lane_width = width * 0.12;
    const double max_lane_width = width * 0.70;
    double expected_center = 0.75 * last_lane_center_ + 0.25 * image_center_x;
    double expected_width = clamp(last_lane_width_, min_lane_width, max_lane_width);
    std::vector<double> lane_widths;

    for (int y0 = roi_bottom - band_height; y0 > roi_top; y0 -= stride) {
      const int y1 = std::min(y0 + band_height, roi_bottom);
      const int y = (y0 + y1 - 1) / 2;
      std::vector<int> projection(width, 0);
      for (int row = y0; row < y1; ++row) {
        const uint8_t * ptr = mask.ptr<uint8_t>(row);
        for (int x = 0; x < width; ++x) {
          if (ptr[x] > 0) {
            projection[x]++;
          }
        }
      }

      std::vector<int> active;
      const int threshold = std::max(2, static_cast<int>(band_height * 0.25));
      for (int x = 0; x < width; ++x) {
        if (projection[x] >= threshold) {
          active.push_back(x);
        }
      }

      const auto segments = extractSegments(active, max_line_width);
      const Segment * left = nearestSegment(segments, expected_center, true);
      const Segment * right = nearestSegment(segments, expected_center, false);

      bool row_valid = false;
      double row_center = expected_center;
      double lane_width = expected_width;
      if (left != nullptr && right != nullptr) {
        lane_width = right->center_x - left->center_x;
        if (lane_width >= min_lane_width && lane_width <= max_lane_width) {
          row_center = 0.5 * (left->center_x + right->center_x);
          left_points.emplace_back(left->center_x, y);
          right_points.emplace_back(right->center_x, y);
          lane_widths.push_back(lane_width);
          row_valid = true;
        }
      } else if (left != nullptr) {
        row_center = left->center_x + expected_width * 0.5;
        left_points.emplace_back(left->center_x, y);
        row_valid = true;
      } else if (right != nullptr) {
        row_center = right->center_x - expected_width * 0.5;
        right_points.emplace_back(right->center_x, y);
        row_valid = true;
      }

      if (!row_valid) {
        continue;
      }
      if (!center_points.empty() && std::abs(row_center - expected_center) > width * 0.12) {
        continue;
      }

      center_points.emplace_back(clamp(row_center, 0.0, width - 1.0), y);
      expected_center = 0.70 * expected_center + 0.30 * row_center;
      expected_width = 0.85 * expected_width + 0.15 * lane_width;
    }

    if (!lane_widths.empty()) {
      std::nth_element(lane_widths.begin(), lane_widths.begin() + lane_widths.size() / 2, lane_widths.end());
      measured_lane_width = lane_widths[lane_widths.size() / 2];
    }

    if (static_cast<int>(center_points.size()) < min_rows_) {
      return 0.0;
    }

    const double row_score = clamp(static_cast<double>(center_points.size()) / 18.0, 0.0, 1.0);
    const double continuity = computeContinuity(center_points, width);
    return clamp(0.60 * row_score + 0.40 * continuity, 0.0, 1.0);
  }

  std::vector<Segment> extractSegments(const std::vector<int> & active, int max_line_width) const
  {
    std::vector<Segment> segments;
    if (active.empty()) {
      return segments;
    }

    int start = active.front();
    int prev = active.front();
    for (size_t i = 1; i <= active.size(); ++i) {
      const bool finished = i == active.size() || active[i] > prev + 1;
      if (finished) {
        const int end = prev;
        const int width = end - start + 1;
        if (width >= 2 && width <= max_line_width) {
          segments.push_back({0.5 * (start + end), start, end});
        }
        if (i < active.size()) {
          start = active[i];
        }
      }
      if (i < active.size()) {
        prev = active[i];
      }
    }
    return segments;
  }

  const Segment * nearestSegment(const std::vector<Segment> & segments, double center_x, bool left_side) const
  {
    const Segment * best = nullptr;
    for (const auto & segment : segments) {
      if (left_side && segment.center_x >= center_x) {
        continue;
      }
      if (!left_side && segment.center_x <= center_x) {
        continue;
      }
      if (best == nullptr ||
        std::abs(segment.center_x - center_x) < std::abs(best->center_x - center_x))
      {
        best = &segment;
      }
    }
    return best;
  }

  FitResult fitQuadratic(const std::vector<cv::Point2d> & points) const
  {
    FitResult result;
    if (points.size() < 3) {
      return result;
    }

    cv::Mat A(static_cast<int>(points.size()), 3, CV_64F);
    cv::Mat b(static_cast<int>(points.size()), 1, CV_64F);
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
      const double y = points[i].y;
      A.at<double>(i, 0) = y * y;
      A.at<double>(i, 1) = y;
      A.at<double>(i, 2) = 1.0;
      b.at<double>(i, 0) = points[i].x;
    }

    cv::Mat coeff;
    if (!cv::solve(A, b, coeff, cv::DECOMP_SVD)) {
      return result;
    }

    result.valid = true;
    result.coeff = cv::Vec3d(coeff.at<double>(0, 0), coeff.at<double>(1, 0), coeff.at<double>(2, 0));
    return result;
  }

  double evalPoly(const cv::Vec3d & coeff, double y) const
  {
    return coeff[0] * y * y + coeff[1] * y + coeff[2];
  }

  double derivativePoly(const cv::Vec3d & coeff, double y) const
  {
    return 2.0 * coeff[0] * y + coeff[1];
  }

  double computeOffset(const cv::Vec3d & coeff, int roi_top, int roi_bottom, double image_center_x) const
  {
    double weighted_sum = 0.0;
    double weight_total = 0.0;
    for (int i = 0; i < 20; ++i) {
      const double t = static_cast<double>(i) / 19.0;
      const double y = roi_top + t * (roi_bottom - roi_top);
      const double weight = 0.30 + t;
      weighted_sum += weight * ((evalPoly(coeff, y) - image_center_x) / std::max(image_center_x, 1.0));
      weight_total += weight;
    }
    return clamp(weighted_sum / std::max(weight_total, 1.0e-6), -1.0, 1.0);
  }

  double computeHeading(const cv::Vec3d & coeff) const
  {
    const double y = image_height_ * 0.72;
    const double slope = derivativePoly(coeff, y);
    return clamp(std::atan(slope) / (M_PI * 0.25), -1.0, 1.0);
  }

  double computeContinuity(const std::vector<cv::Point2d> & points, int width) const
  {
    if (points.size() < 4) {
      return 1.0;
    }
    std::vector<double> diffs;
    diffs.reserve(points.size() - 1);
    for (size_t i = 1; i < points.size(); ++i) {
      diffs.push_back(points[i].x - points[i - 1].x);
    }
    const double mean = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
    double variance = 0.0;
    for (double diff : diffs) {
      variance += (diff - mean) * (diff - mean);
    }
    variance /= std::max<size_t>(diffs.size(), 1);
    const double stddev = std::sqrt(variance);
    return clamp(1.0 - stddev / std::max(width * 0.07, 1.0), 0.0, 1.0);
  }

  void publishState(double confidence)
  {
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = static_cast<float>(last_offset_);
    line_offset_pub_->publish(offset_msg);

    std_msgs::msg::Float32MultiArray state_msg;
    state_msg.data = {
      static_cast<float>(last_offset_),
      static_cast<float>(last_heading_),
      static_cast<float>(confidence),
      static_cast<float>(last_control_)};
    line_state_pub_->publish(state_msg);
  }

  cv::Mat makeDebugImage(
    const cv::Mat & bgr,
    const cv::Mat & binary,
    const cv::Mat & edges,
    const cv::Mat & line_mask,
    int roi_top,
    int roi_bottom,
    const std::vector<cv::Point2d> & left_points,
    const std::vector<cv::Point2d> & right_points,
    const std::vector<cv::Point2d> & center_points,
    const cv::Vec3d & center_coeff,
    bool has_center_coeff,
    double confidence) const
  {
    cv::Mat overlay = bgr.clone();
    cv::rectangle(
      overlay,
      cv::Point(0, roi_top),
      cv::Point(overlay.cols - 1, roi_bottom),
      cv::Scalar(50, 180, 50),
      1);

    cv::Mat mask_color = cv::Mat::zeros(bgr.size(), CV_8UC3);
    mask_color.setTo(cv::Scalar(0, 80, 0), line_mask > 0);
    mask_color.setTo(cv::Scalar(180, 80, 0), edges > 0);
    mask_color.setTo(cv::Scalar(220, 220, 220), binary > 0);
    cv::addWeighted(overlay, 0.78, mask_color, 0.45, 0.0, overlay);

    drawPoints(overlay, left_points, cv::Scalar(255, 150, 0));
    drawPoints(overlay, right_points, cv::Scalar(0, 220, 255));
    drawPoints(overlay, center_points, cv::Scalar(0, 255, 0));

    if (has_center_coeff) {
      std::vector<cv::Point> curve;
      for (int y = roi_top; y <= roi_bottom; y += 4) {
        const int x = static_cast<int>(std::round(evalPoly(center_coeff, y)));
        if (x >= 0 && x < overlay.cols) {
          curve.emplace_back(x, y);
        }
      }
      if (curve.size() >= 2) {
        std::vector<std::vector<cv::Point>> curves{curve};
        cv::polylines(overlay, curves, false, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      }
    }

    const int center_x = overlay.cols / 2;
    cv::line(overlay, cv::Point(center_x, 0), cv::Point(center_x, overlay.rows - 1), cv::Scalar(80, 80, 255), 1);
    cv::circle(overlay, cv::Point(static_cast<int>(std::round(last_lane_center_)), static_cast<int>(overlay.rows * 0.74)), 5, cv::Scalar(80, 255, 80), -1);

    const std::string status =
      "cpp ctrl=" + formatNumber(last_control_) +
      " offset=" + formatNumber(last_offset_) +
      " head=" + formatNumber(last_heading_) +
      " conf=" + formatNumber(confidence);
    cv::putText(overlay, status, cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(overlay, status, cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    return overlay;
  }

  void drawPoints(cv::Mat & image, const std::vector<cv::Point2d> & points, const cv::Scalar & color) const
  {
    const int step = std::max(1, static_cast<int>(points.size()) / 30);
    for (size_t i = 0; i < points.size(); i += step) {
      cv::circle(
        image,
        cv::Point(static_cast<int>(std::round(points[i].x)), static_cast<int>(std::round(points[i].y))),
        3,
        color,
        -1,
        cv::LINE_AA);
    }
  }

  std::string formatNumber(double value) const
  {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%+.3f", value);
    return std::string(buffer);
  }

  void publishDebugImage(const cv::Mat & bgr, const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "cpp_line_debug";
    msg.height = static_cast<uint32_t>(bgr.rows);
    msg.width = static_cast<uint32_t>(bgr.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = 0;
    msg.step = static_cast<uint32_t>(bgr.cols * 3);
    msg.data.resize(static_cast<size_t>(msg.step * msg.height));
    if (bgr.isContinuous()) {
      std::copy(bgr.data, bgr.data + msg.data.size(), msg.data.begin());
    } else {
      for (int row = 0; row < bgr.rows; ++row) {
        const auto * src = bgr.ptr<uint8_t>(row);
        auto * dst = msg.data.data() + static_cast<size_t>(row) * msg.step;
        std::copy(src, src + msg.step, dst);
      }
    }
    debug_image_pub_->publish(msg);
  }

  std::string image_topic_;
  std::string line_offset_topic_;
  std::string line_state_topic_;
  std::string debug_image_topic_;
  bool enable_debug_image_{true};
  int threshold_value_{155};
  double canny_low_{60.0};
  double canny_high_{150.0};
  double roi_top_ratio_{0.50};
  double roi_bottom_ratio_{0.96};
  int scan_band_height_{7};
  int scan_stride_{7};
  int min_rows_{4};
  double smoothing_alpha_{0.18};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr line_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  bool initialized_{false};
  double last_lane_width_{260.0};
  double last_lane_center_{320.0};
  double last_offset_{0.0};
  double last_heading_{0.0};
  double last_control_{0.0};
  int lost_frames_{0};
  bool has_last_center_coeff_{false};
  cv::Vec3d last_center_coeff_{0.0, 0.0, 0.0};
  int image_height_{360};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppLineDetector>());
  rclcpp::shutdown();
  return 0;
}
