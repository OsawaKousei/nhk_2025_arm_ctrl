#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <arm_ctrl/arm_ctrl.hpp>

int main(int argc, char *argv[])
{
  // 標準出力stdoutのバッファリングを無効化する
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto arm_ctrl_node = std::make_shared<arm_ctrl::ArmCtrl>(rclcpp::NodeOptions());
  exec.add_node(arm_ctrl_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}