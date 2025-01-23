#ifndef HACKA_KAIO__HACKA_KAIO_NODE_HPP
#define HACKA_KAIO__HACKA_KAIO_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hacka_kaio
{
class hacka_kaio_node : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit hacka_kaio_node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~hacka_kaio_node() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  void getParameters();
  void configPubSub();
  void configTimers();
  void configSrvCli();

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_goto_;
  rclcpp::TimerBase::SharedPtr                                               tmr_pub_goto_;
  void                                                                       tmrPubGoto();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_have_goal_;
  void                                                  subHaveGoal(const std_msgs::msg::Bool &msg);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_node_;
  void                                               srvStartNode(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_tackoff_;
  void                                              cltTackoff();

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_land_;
  void                                              cltLand();

  bool is_active_;
  bool _have_goal_;
  bool _start_;

  double _rate_tmr_pub_goto_;

  int _actions_;
  int _waypoints_qty_points_;

  std::vector<double> _waypoints_points_;
};
}  // namespace hacka_kaio

#endif
