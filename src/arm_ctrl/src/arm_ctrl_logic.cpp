#include "arm_ctrl/arm_ctrl_logic.hpp"

namespace arm_ctrl
{
  std::vector<float> ArmCtrlLogic::calc_arm_target(const std::vector<float> &r_pos)
  {
    // ここではr_posをそのまま返す（必要に応じて計算ロジックを追加）
    return r_pos;
  }
}
