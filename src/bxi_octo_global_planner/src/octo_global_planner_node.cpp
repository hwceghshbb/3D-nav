#include "global_planner.h"
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace

class BxiOctoGlobalPlannerNode : public rclcpp::Node
{
public:
  BxiOctoGlobalPlannerNode()
  : Node("bxi_octo_global_planner")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const std::string input_pcd = declare_parameter<std::string>(
      "input_pcd",
      "/home/hwc/code/elf-nav/third_party/OctoPlanner3D-ROS2/octomap/pcd_files/building2_9.pcd");
    const std::string output_bt = declare_parameter<std::string>("output_bt", "/tmp/bxi_octo_global_map.bt");
    const double cloud_scale = declare_parameter<double>("cloud_scale", 1.0);
    const double octomap_resolution = declare_parameter<double>("octomap_resolution", 0.20);
    const double robot_radius = declare_parameter<double>("robot_radius", 0.18);
    const bool require_ground_support = declare_parameter<bool>("require_ground_support", true);
    const bool strict_direct_ground_support = declare_parameter<bool>("strict_direct_ground_support", false);
    const int ground_support_xy_radius_cells = declare_parameter<int>("ground_support_xy_radius_cells", 1);
    const int ground_support_depth_cells = declare_parameter<int>("ground_support_depth_cells", 1);
    const int snap_search_radius_cells = declare_parameter<int>("snap_search_radius_cells", 20);
    const int max_iterations = declare_parameter<int>("max_iterations", 1000000);
    const bool enable_preblocked_costmap = declare_parameter<bool>("enable_preblocked_costmap", true);
    const int preblocked_costmap_radius_cells = declare_parameter<int>("preblocked_costmap_radius_cells", 3);
    const double preblocked_costmap_weight = declare_parameter<double>("preblocked_costmap_weight", 1.0);
    const double vertical_search_padding_below = declare_parameter<double>("vertical_search_padding_below", 1.0);
    const double vertical_search_padding_above = declare_parameter<double>("vertical_search_padding_above", 0.6);
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/simulation/base_footprint/pose");
    const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    const std::string path_topic = declare_parameter<std::string>("path_topic", "/initial_path");
    const std::string debug_path_topic = declare_parameter<std::string>("debug_path_topic", "/octo_global_path");
    const std::string map_topic = declare_parameter<std::string>("map_marker_topic", "/octo_occupied_map");
    const std::string start_marker_topic = declare_parameter<std::string>("start_marker_topic", "/octo_start_marker");
    const std::string goal_marker_topic = declare_parameter<std::string>("goal_marker_topic", "/octo_goal_marker");
    start_z_offset_ = declare_parameter<double>("start_z_offset", 0.30);
    goal_z_offset_ = declare_parameter<double>("goal_z_offset", 0.30);
    path_z_offset_ = declare_parameter<double>("path_z_offset", 0.80);
    map_alpha_ = declare_parameter<double>("map_alpha", 0.72);
    publish_map_ = declare_parameter<bool>("publish_map", true);
    map_publish_period_ = declare_parameter<double>("map_publish_period", 2.0);

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, latched_qos);
    debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(debug_path_topic, latched_qos);
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(map_topic, latched_qos);
    start_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(start_marker_topic, latched_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(goal_marker_topic, latched_qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&BxiOctoGlobalPlannerNode::onOdom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic,
      rclcpp::QoS(10),
      std::bind(&BxiOctoGlobalPlannerNode::onGoal, this, std::placeholders::_1));

    converter_ = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
    converter_->setInputPcdFile(input_pcd);
    converter_->setOutputBtFile(output_bt);
    converter_->setInputScale(cloud_scale);
    converter_->setResolution(octomap_resolution);
    planner_ = std::make_shared<global_planner::GlobalPlanner>();
    planner_->setRobotRadius(robot_radius);
    planner_->setRequireGroundSupport(require_ground_support);
    planner_->setGroundSupportParams(
      strict_direct_ground_support,
      ground_support_xy_radius_cells,
      ground_support_depth_cells);
    planner_->setSnapSearchRadiusCells(snap_search_radius_cells);
    planner_->setMaxIterations(max_iterations);
    planner_->setPreblockedCostmapEnabled(enable_preblocked_costmap);
    planner_->setPreblockedCostmapParams(preblocked_costmap_radius_cells, preblocked_costmap_weight);
    planner_->setVerticalSearchPadding(vertical_search_padding_below, vertical_search_padding_above);

    RCLCPP_INFO(
      get_logger(),
      "Building OctoMap from %s (cloud_scale=%.6f, resolution=%.3f, robot_radius=%.3f)",
      input_pcd.c_str(),
      cloud_scale,
      octomap_resolution,
      robot_radius);
    if (!converter_->convert()) {
      RCLCPP_ERROR(get_logger(), "Failed to build OctoMap; planner will wait but cannot plan.");
      return;
    }

    octree_ = converter_->getOctomap();
    planner_->setOctomap(octree_);
    map_ready_ = true;

    if (publish_map_) {
      publishMap();
      map_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(0.1, map_publish_period_))),
        std::bind(&BxiOctoGlobalPlannerNode::publishMap, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Octo global planner ready: odom=%s goal=%s path=%s",
      odom_topic.c_str(),
      goal_topic.c_str(),
      path_topic.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = msg;
    planIfReady();
  }

  void planIfReady()
  {
    if (!map_ready_ || !planner_) {
      RCLCPP_WARN(get_logger(), "OctoMap is not ready; ignoring goal for now.");
      return;
    }
    if (!odom_) {
      RCLCPP_WARN(get_logger(), "No odometry yet; goal cached.");
      return;
    }
    if (!goal_) {
      return;
    }

    global_planner::PointPose start;
    start.x = odom_->pose.pose.position.x;
    start.y = odom_->pose.pose.position.y;
    start.z = odom_->pose.pose.position.z + start_z_offset_;

    global_planner::PointPose goal;
    goal.x = goal_->pose.position.x;
    goal.y = goal_->pose.position.y;
    goal.z = goal_->pose.position.z + goal_z_offset_;

    publishPoseMarker(start, "octo_start", makeColor(0.10F, 0.90F, 0.20F, 1.0F), start_marker_pub_);
    publishPoseMarker(goal, "octo_goal", makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);

    RCLCPP_INFO(
      get_logger(),
      "Planning Octo 3D path: start=[%.2f %.2f %.2f] goal=[%.2f %.2f %.2f]",
      start.x,
      start.y,
      start.z,
      goal.x,
      goal.y,
      goal.z);

    planner_->makePlan(start, goal);

    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "OctoPlanner3D returned an empty path.");
      publishPath(path);
      return;
    }

    publishPath(path);
    RCLCPP_INFO(get_logger(), "Published Octo global path with %zu poses.", path.size());
  }

  void publishPath(const std::vector<global_planner::PointPose> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(point.x, point.y, point.z + path_z_offset_);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
    debug_path_pub_->publish(msg);
  }

  void publishMap()
  {
    if (!octree_ || !map_pub_) {
      return;
    }

    const float alpha = static_cast<float>(std::clamp(map_alpha_, 0.05, 1.0));
    std::unordered_map<double, visualization_msgs::msg::Marker> markers_by_size;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }

      const double size = it.getSize();
      auto marker_it = markers_by_size.find(size);
      if (marker_it == markers_by_size.end()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "octo_occupied_voxels";
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color = makeColor(0.45F, 0.22F, 0.06F, alpha);
        marker_it = markers_by_size.emplace(size, std::move(marker)).first;
      }
      marker_it->second.points.push_back(makePoint(it.getX(), it.getY(), it.getZ()));
    }

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cleanup;
    cleanup.header.frame_id = frame_id_;
    cleanup.header.stamp = now();
    cleanup.ns = "octo_occupied_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    int id = 0;
    for (auto & entry : markers_by_size) {
      auto & marker = entry.second;
      marker.header.stamp = now();
      marker.id = id++;
      array.markers.push_back(marker);
    }

    map_pub_->publish(array);
  }

  void publishPoseMarker(
    const global_planner::PointPose & pose,
    const std::string & ns,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(pose.x, pose.y, pose.z);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color = color;
    publisher->publish(marker);
  }

  std::string frame_id_;
  double start_z_offset_ = 0.30;
  double goal_z_offset_ = 0.30;
  double path_z_offset_ = 0.80;
  double map_alpha_ = 0.72;
  double map_publish_period_ = 2.0;
  bool publish_map_ = true;
  bool map_ready_ = false;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BxiOctoGlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
