#include "hacka_kaio/hacka_kaio_node.hpp"

namespace hacka_kaio
{
/* hacka_kaio_node() //{ */
hacka_kaio_node::hacka_kaio_node(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("calculator_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.tmr_pub_goto", rclcpp::ParameterValue(0.5));
  declare_parameter("waypoints.qty_points", rclcpp::ParameterValue(4));
  declare_parameter<std::vector<double>>("waypoints.points", std::vector<double>{2.0, 0.0, 2.0,
                                                                                 2.0, 2.0, 2.0,
                                                                                 0.0, 2.0, 2.0,
                                                                                 0.0, 0.0, 2.0});


  is_active_ = false;
  _have_goal_ = false;
  _start_ = false;

  _actions_ = 0;
}
//}

/* ~hacka_kaio_node() //{ */
hacka_kaio_node::~hacka_kaio_node() {
}
//}

/* on_configure() //{ */
CallbackReturn hacka_kaio_node::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configSrvCli();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn hacka_kaio_node::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_goto_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn hacka_kaio_node::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_goto_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn hacka_kaio_node::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  sub_have_goal_.reset();

  pub_goto_.reset();

  tmr_pub_goto_.reset();

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn hacka_kaio_node::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void hacka_kaio_node::getParameters() {
  get_parameter("rate.tmr_pub_goto", _rate_tmr_pub_goto_);
  get_parameter("waypoints.qty_points", _waypoints_qty_points_);
  get_parameter("waypoints.points", _waypoints_points_);
}
//}

/* configPubSub() //{ */
void hacka_kaio_node::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  // Pubs and Subs for topics
  sub_have_goal_ = create_subscription<std_msgs::msg::Bool>("uav1/have_goal", 1, std::bind(&hacka_kaio_node::subHaveGoal, this, std::placeholders::_1));

  pub_goto_ = create_publisher<geometry_msgs::msg::Pose>("uav1/goto", 1);
}
//}

/* configTimers() //{ */
void hacka_kaio_node::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_pub_goto_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_tmr_pub_goto_),
                                    std::bind(&hacka_kaio_node::tmrPubGoto, this), nullptr);
}
//}

/* configSrvCli() //{ */
void hacka_kaio_node::configSrvCli() {
  RCLCPP_INFO(get_logger(), "initSrvCli");

  srv_start_node_ = create_service<std_srvs::srv::Trigger>("start_state_machine", std::bind(
                                                           &hacka_kaio_node::srvStartNode, this, 
                                                           std::placeholders::_1, std::placeholders::_2));

  clt_tackoff_ = create_client<std_srvs::srv::Trigger>("uav1/takeoff");
  clt_land_    = create_client<std_srvs::srv::Trigger>("uav1/land");
}
//}

/* subHaveGoal() //{ */
void hacka_kaio_node::subHaveGoal(const std_msgs::msg::Bool &msg) {
  if (!is_active_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_have_goal_);
  _have_goal_ = msg.data;

  //RCLCPP_INFO(this->get_logger(), "have_goal: %d", msg.data);
}
//}

/* tmrPubGoto() //{ */
void hacka_kaio_node::tmrPubGoto() {
  if (!is_active_) {
    return;
  }

  bool have_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_have_goal_);
    have_goal = _have_goal_;
  }

  if(_start_){
    if(!have_goal){
        if(!_actions_){
            cltTackoff();

            _actions_ = 1;
        }

        else if(_actions_ <= _waypoints_qty_points_){
            geometry_msgs::msg::Pose msg;

            msg.position.x = _waypoints_points_[3*(_actions_-1)];
            msg.position.y = _waypoints_points_[3*(_actions_-1)+1];
            msg.position.z = _waypoints_points_[3*(_actions_-1)+2];

            pub_goto_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published goto: %f, %f, %f", msg.position.x, msg.position.y, msg.position.z);

            _actions_++;
        }

        else{
            cltLand();
            _start_ = false;
        }
    }
  }
}
//}

/* srvStartNode() //{ */
void hacka_kaio_node::srvStartNode([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>  response) {
  if (!is_active_) {
    return;
  }

  response->success = true;

  _start_ = true;

  response->message = "Node started";

  //RCLCPP_INFO(this->get_logger(), "start_node: %d", response->success);
}
//}

/* cltTackoff() //{ */
void hacka_kaio_node::cltTackoff() {
  if (!is_active_) {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future_result = clt_tackoff_->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        //RCLCPP_INFO(this->get_logger(), "Takeoff successful");
      } else {
        //RCLCPP_WARN(this->get_logger(), "Takeoff failed");
      }
    });

  //RCLCPP_INFO(this->get_logger(), "Request sent to takeoff");
}
//}

/* cltLand() //{ */
void hacka_kaio_node::cltLand() {
  if (!is_active_) {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future_result = clt_land_->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        //RCLCPP_INFO(this->get_logger(), "Land successful");
      } else {
        //RCLCPP_WARN(this->get_logger(), "Land failed");
      }
    });

  //RCLCPP_INFO(this->get_logger(), "Request sent to land");
}
//}

}  // namespace hacka_kaio_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hacka_kaio::hacka_kaio_node)
