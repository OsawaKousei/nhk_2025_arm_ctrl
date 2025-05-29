#include "arm_ctrl/arm_ctrl.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

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

    // target_valueサブスクライバー
    target_value_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/target_value", 10,
        std::bind(&ArmCtrl::target_value_callback, this, std::placeholders::_1));

    // arm_cmdサービスクライアント
    rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    arm_cmd_client_ = this->create_client<std_srvs::srv::Trigger>("arm_cmd", qos_profile);

    // arm_targetパブリッシャー
    arm_target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("arm_target", 10);

    this->arm_target_msg.x = this->arm_target_angle_init_value_;
    this->arm_target_msg.y = this->arm_target_speed_value_;
    this->arm_target_msg.z = 0.0;
  }

  // 指定のボタンが押されたら、アームのシーケンス処理を開始する
  void ArmCtrl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    bool current_button4_state = msg->buttons[4];
    bool current_button5_state = msg->buttons[5];
    // ボタン4が押されているかチェック
    if (current_button4_state && !prev_button4_state_)
    {
      RCLCPP_INFO(this->get_logger(), "Publishing reset message...");
      this->arm_target_msg.z = this->arm_reset_cmd_value_; // リセットコマンドの値を設定
      arm_target_pub_->publish(this->arm_target_msg);
    }

    // ボタン5が押されているかチェック
    if (current_button5_state && !prev_button5_state_)
    {
      RCLCPP_INFO(this->get_logger(), "Sending service request...");
      // 非同期でサービスリクエストを送信
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = arm_cmd_client_->async_send_request(
          request,
          std::bind(&ArmCtrl::arm_cmd_response_callback, this, std::placeholders::_1));
    }

    // 現在のボタンの状態を保存
    prev_button4_state_ = current_button4_state;
    prev_button5_state_ = current_button5_state;
  }

  // アームのシーケンス処理が完了したら、呼び出し待機状態を解除する
  void ArmCtrl::arm_cmd_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
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

    this->arm_target_msg.x = arm_target_vec[0];
    this->arm_target_msg.y = arm_target_vec[1];
    this->arm_target_msg.z = 0.0;
    arm_target_pub_->publish(this->arm_target_msg);
    RCLCPP_INFO(this->get_logger(), "Published arm_target data");
  }

  // target_valueトピックからのデータを受信して表示するコールバック
  void ArmCtrl::target_value_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received target_value: %f", msg->data);
    float diff = msg->data - this->robot_distance_init_value_;
    float adjusted_value = diff * this->robot_distance_coefficient_;
    this->arm_target_msg.x = this->arm_target_angle_init_value_;
    this->arm_target_msg.y = this->arm_target_speed_value_ + adjusted_value;
    this->arm_target_msg.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Adjusted arm_target speed value: %f", this->arm_target_msg.y);
    // arm_targetをパブリッシュ
    arm_target_pub_->publish(this->arm_target_msg);
    RCLCPP_INFO(this->get_logger(), "Published adjusted arm_target data");
  }

  ArmCtrl::~ArmCtrl()
  {
    // デストラクタの実装
    RCLCPP_INFO(this->get_logger(), "ArmCtrl node has been destroyed.");
  }
} // namespace arm_ctrl

RCLCPP_COMPONENTS_REGISTER_NODE(arm_ctrl::ArmCtrl)