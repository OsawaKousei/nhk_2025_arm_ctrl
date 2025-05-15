#include "arm_ctrl/arm_ctrl.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace arm_ctrl
{

  ArmCtrl::ArmCtrl(const rclcpp::NodeOptions &options)
      : Node("arm_ctrl", options)
  {
    // コンストラクタの実装
    RCLCPP_INFO(this->get_logger(), "ArmCtrl node has been created.");
  }

  ArmCtrl::~ArmCtrl()
  {
    // デストラクタの実装
    RCLCPP_INFO(this->get_logger(), "ArmCtrl node has been destroyed.");
  }

} // namespace arm_ctrl

RCLCPP_COMPONENTS_REGISTER_NODE(arm_ctrl::ArmCtrl)