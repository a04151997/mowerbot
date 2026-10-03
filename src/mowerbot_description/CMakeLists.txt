cmake_minimum_required(VERSION 3.8)
project(mowerbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 車輛幾何的單一來源載入器 (階段 38)：launch、manager、bridge、測試都 import 它
ament_python_install_package(${PROJECT_NAME})

# 幾何閘門 (階段 38)：實機 / 模擬 launch 的最前面，allow_provisional 檢查不通過就讓 launch 失敗
install(PROGRAMS scripts/geometry_guard DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY
    urdf
    launch
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
