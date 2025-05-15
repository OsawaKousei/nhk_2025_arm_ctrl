#ifndef ARM_CTRL__ARM_CTRL_HPP_
#define ARM_CTRL__ARM_CTRL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp> // 仮のサービス型。int型サービスは独自定義が必要

#include "arm_ctrl/visibility_control.h"

namespace arm_ctrl
{

  class ArmCtrl : public rclcpp::Node
  {
  public:
    // マルチプラットフォーム対応のためのマクロ
    TUTORIAL_PUBLIC

    explicit ArmCtrl(const rclcpp::NodeOptions &options);

    virtual ~ArmCtrl();

  private:
    // Joyサブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // arm_cmdサービスクライアント（int型サービス。仮にstd_srvs::srv::SetBoolを使っています。独自サービスなら型を修正してください）
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_cmd_client_;

    // arm_targetパブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_target_pub_;
  };

} // namespace arm_ctrl

#endif // ARM_CTRL__ARM_CTRL_HPP_