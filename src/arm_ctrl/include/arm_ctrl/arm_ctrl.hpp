#ifndef ARM_CTRL__ARM_CTRL_HPP_
#define ARM_CTRL__ARM_CTRL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "arm_ctrl/visibility_control.h"
#include "arm_ctrl/arm_ctrl_logic.hpp"

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

    // r_posサブスクライバー
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr r_pos_sub_;
    void r_pos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // arm_cmdサービスクライアント
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_cmd_client_;

    // arm_targetパブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_target_pub_;

    // arm_ctrl_logic
    ArmCtrlLogic arm_ctrl_logic_;

    // サービスコール中フラグ
    bool service_calling_ = false;
    void arm_cmd_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  };

} // namespace arm_ctrl

#endif // ARM_CTRL__ARM_CTRL_HPP_