#include "arm_ctrl/arm_ctrl.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace arm_ctrl
{

  ArmCtrl::ArmCtrl(const rclcpp::NodeOptions &options)
      : Node("arm_ctrl", options)
  {
    // コンストラクタの実装
    RCLCPP_INFO(this->get_logger(), "ArmCtrl node has been created.");

    // Joyサブスクライバー
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&ArmCtrl::joy_callback, this, std::placeholders::_1));

    // r_posサブスクライバー
    r_pos_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "r_pos", 10,
        std::bind(&ArmCtrl::r_pos_callback, this, std::placeholders::_1));

    // arm_cmdサービスクライアント
    arm_cmd_client_ = this->create_client<std_srvs::srv::Trigger>("arm_cmd");

    // arm_targetパブリッシャー
    arm_target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_target", 10);
  }

  void ArmCtrl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Joyメッセージの処理
  }

  void ArmCtrl::r_pos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // r_posメッセージの処理
  }

  ArmCtrl::~ArmCtrl()
  {
    // デストラクタの実装
    RCLCPP_INFO(this->get_logger(), "ArmCtrl node has been destroyed.");
  }

} // namespace arm_ctrl

RCLCPP_COMPONENTS_REGISTER_NODE(arm_ctrl::ArmCtrl)