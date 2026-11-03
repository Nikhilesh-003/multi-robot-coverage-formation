// PeerLayer: marks the footprint of every *other* robot in the fleet as LETHAL.
//
// Why: a TurtleBot3's lidar plane sits above the other robots' bodies, so a peer
// only shows up as its small lidar dome. This layer stamps a full-size disc at
// each peer pose (published by mrc_fleet/fleet_monitor) so that the normal
// inflation layer keeps robots apart. Nothing is stored between cycles: the
// layer redraws peers every update and expands the bounds over the previous
// positions, so a robot that drives away never leaves a stale obstacle behind.
#ifndef MRC_COSTMAP_PLUGINS__PEER_LAYER_HPP_
#define MRC_COSTMAP_PLUGINS__PEER_LAYER_HPP_

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mrc_costmap_plugins
{

class PeerLayer : public nav2_costmap_2d::Layer
{
public:
  PeerLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void peersCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  void touchDisc(
    double x, double y, double * min_x, double * min_y, double * max_x,
    double * max_y) const;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr latest_;

  std::vector<std::pair<double, double>> peers_;       // in costmap global frame
  std::vector<std::pair<double, double>> prev_peers_;
  double robot_x_{0.0}, robot_y_{0.0};

  std::string topic_;
  double peer_radius_{0.22};
  double timeout_{1.0};
  double self_clearance_{0.30};
  double transform_tolerance_{0.5};
  rclcpp::Logger logger_{rclcpp::get_logger("PeerLayer")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace mrc_costmap_plugins

#endif  // MRC_COSTMAP_PLUGINS__PEER_LAYER_HPP_
