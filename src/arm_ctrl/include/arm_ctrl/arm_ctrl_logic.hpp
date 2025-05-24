#ifndef ARM_CTRL__ARM_CTRL_LOGIC_HPP_
#define ARM_CTRL__ARM_CTRL_LOGIC_HPP_

#include <vector>

namespace arm_ctrl
{
  class ArmCtrlLogic
  {
  public:
    ArmCtrlLogic() = default;
    ~ArmCtrlLogic() = default;

    // r_posからarm_targetを計算
    std::vector<float> calc_arm_target(const std::vector<float> &r_pos);
  };
} // namespace arm_ctrl

#endif // ARM_CTRL__ARM_CTRL_LOGIC_HPP_
