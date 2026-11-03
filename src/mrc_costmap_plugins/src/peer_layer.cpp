#include "mrc_costmap_plugins/peer_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mrc_costmap_plugins
{

void PeerLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PeerLayer: failed to lock costmap node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("peer_poses")));
  declareParameter("peer_radius", rclcpp::ParameterValue(0.22));
  declareParameter("timeout", rclcpp::ParameterValue(1.0));
  declareParameter("self_clearance", rclcpp::ParameterValue(0.30));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic_);
  node->get_parameter(getFullName("peer_radius"), peer_radius_);
  node->get_parameter(getFullName("timeout"), timeout_);
  node->get_parameter(getFullName("self_clearance"), self_clearance_);
  node->get_parameter(getFullName("transform_tolerance"), transform_tolerance_);

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    topic_, rclcpp::QoS(1).best_effort(),
    std::bind(&PeerLayer::peersCallback, this, std::placeholders::_1));
  current_ = true;
  RCLCPP_INFO(
    logger_, "PeerLayer '%s' listening on '%s' (peer_radius=%.2f m)",
    name_.c_str(), sub_->get_topic_name(), peer_radius_);
}

void PeerLayer::peersCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = msg;
}

void PeerLayer::touchDisc(
  double x, double y, double * min_x, double * min_y, double * max_x, double * max_y) const
{
  *min_x = std::min(*min_x, x - peer_radius_);
  *min_y = std::min(*min_y, y - peer_radius_);
  *max_x = std::max(*max_x, x + peer_radius_);
  *max_y = std::max(*max_y, y + peer_radius_);
}

void PeerLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  prev_peers_.swap(peers_);
  peers_.clear();

  geometry_msgs::msg::PoseArray::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = latest_;
  }

  if (enabled_ && msg) {
    const double age = (clock_->now() - rclcpp::Time(msg->header.stamp, clock_->get_clock_type()))
      .seconds();
    if (age > timeout_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "PeerLayer: peer poses are %.1fs old, ignoring", age);
    } else {
      const std::string global = layered_costmap_->getGlobalFrameID();
      try {
        geometry_msgs::msg::TransformStamped tf_msg = tf_->lookupTransform(
          global, msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance_));
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        for (const auto & p : msg->poses) {
          const tf2::Vector3 v = tf * tf2::Vector3(p.position.x, p.position.y, 0.0);
          peers_.emplace_back(v.x(), v.y());
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000, "PeerLayer: cannot transform %s -> %s: %s",
          msg->header.frame_id.c_str(), global.c_str(), ex.what());
      }
    }
  }

  // Expand over old *and* new positions so vacated cells get re-composited.
  for (const auto & p : prev_peers_) {touchDisc(p.first, p.second, min_x, min_y, max_x, max_y);}
  for (const auto & p : peers_) {touchDisc(p.first, p.second, min_x, min_y, max_x, max_y);}
}

void PeerLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || peers_.empty()) {
    return;
  }
  const double res = master_grid.getResolution();
  const double r2 = peer_radius_ * peer_radius_;
  const double self2 = self_clearance_ * self_clearance_;
  for (const auto & p : peers_) {
    int ci0, cj0, ci1, cj1;
    master_grid.worldToMapEnforceBounds(p.first - peer_radius_, p.second - peer_radius_, ci0, cj0);
    master_grid.worldToMapEnforceBounds(p.first + peer_radius_, p.second + peer_radius_, ci1, cj1);
    ci0 = std::max(ci0, min_i);
    cj0 = std::max(cj0, min_j);
    ci1 = std::min(ci1, max_i - 1);
    cj1 = std::min(cj1, max_j - 1);
    for (int j = cj0; j <= cj1; ++j) {
      for (int i = ci0; i <= ci1; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        const double dx = wx - p.first, dy = wy - p.second;
        if (dx * dx + dy * dy > r2 + 0.25 * res * res) {
          continue;
        }
        // Never paint over our own footprint: a lethal start cell makes the
        // planner fail instantly. The collision monitor handles that case.
        const double sx = wx - robot_x_, sy = wy - robot_y_;
        if (sx * sx + sy * sy < self2) {
          continue;
        }
        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void PeerLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  peers_.clear();
  prev_peers_.clear();
  current_ = true;
}

}  // namespace mrc_costmap_plugins

PLUGINLIB_EXPORT_CLASS(mrc_costmap_plugins::PeerLayer, nav2_costmap_2d::Layer)
