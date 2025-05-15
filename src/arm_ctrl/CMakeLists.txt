cmake_minimum_required(VERSION 3.8)
project(arm_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#ライブラリの生成
ament_auto_add_library(arm_ctrl SHARED
  src/arm_ctrl.cpp
  src/arm_ctrl_logic.cpp
)
rclcpp_components_register_nodes(arm_ctrl "arm_ctrl::arm_ctrl")

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 実行ファイルのビルド
ament_auto_add_executable(
  arm_ctrl_node src/arm_ctrl_node.cpp
)

#ライブラリのビルド
ament_auto_package(INSTALL_TO_SHARE)