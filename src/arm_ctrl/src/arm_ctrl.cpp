#include "arm_ctrl/arm_ctrl.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>

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
    r_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "r_pos", 10,
        std::bind(&ArmCtrl::r_pos_callback, this, std::placeholders::_1));

    // arm_cmdサービスクライアント
    arm_cmd_client_ = this->create_client<std_srvs::srv::Trigger>("arm_cmd");
    service_calling_ = false;

    // arm_targetパブリッシャー
    arm_target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("arm_target", 10);
  }

  // 指定のボタンが押されたら、アームのシーケンス処理を開始する
  void ArmCtrl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // ボタン1と3が同時押しされているかチェック
    if (msg->buttons.size() > 3 && msg->buttons[2] && msg->buttons[3])
    {
      // サービスリクエストが進行中でない場合
      if (!service_calling_ && arm_cmd_client_->service_is_ready())
      {
        RCLCPP_INFO(this->get_logger(), "Button 1 and 3 are pressed simultaneously and no service request is in progress. Sending service request...");
        // 非同期でサービスリクエストを送信
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        service_calling_ = true;
        auto future = arm_cmd_client_->async_send_request(
            request,
            std::bind(&ArmCtrl::arm_cmd_response_callback, this, std::placeholders::_1));
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Service request is already in progress or service is not ready.");
      }
    }
  }

  // アームのシーケンス処理が完了したら、呼び出し待機状態を解除する
  void ArmCtrl::arm_cmd_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    service_calling_ = false;
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Service response received: success=%d, message=%s", response->success, response->message.c_str());
  }

  // r_posからarm_targetを計算してarm_targetトピックにパブリッシュする
  void ArmCtrl::r_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received r_pos data from r_pos topic");
    // std_msgs::msg::Float32MultiArray → std::vector<float>
    std::vector<double> r_pos_vec = {
        msg->x,
        msg->y};

    // float型に変換
    std::vector<float> r_pos_vec_float(r_pos_vec.begin(), r_pos_vec.end());
    // 計算
    std::vector<float> arm_target_vec = arm_ctrl_logic_.calc_arm_target(r_pos_vec_float);

    geometry_msgs::msg::Point arm_target_msg;
    arm_target_msg.x = arm_target_vec[0];
    arm_target_msg.y = arm_target_vec[1];
    arm_target_msg.z = 0.0;
    arm_target_pub_->publish(arm_target_msg);
    RCLCPP_INFO(this->get_logger(), "Published arm_target data");
  }

  ArmCtrl::~ArmCtrl()
  {
    // デストラクタの実装
    RCLCPP_INFO(this->get_logger(), "ArmCtrl node has been destroyed.");
  }

} // namespace arm_ctrl

RCLCPP_COMPONENTS_REGISTER_NODE(arm_ctrl::ArmCtrl)