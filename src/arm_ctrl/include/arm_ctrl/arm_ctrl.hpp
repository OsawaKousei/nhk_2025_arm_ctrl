#ifndef ARM_CTRL__ARM_CTRL_HPP_
#define ARM_CTRL__ARM_CTRL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
  };

} // namespace arm_ctrl

#endif // ARM_CTRL__ARM_CTRL_HPP_