cmake_minimum_required(VERSION 3.8)
project(mowerbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 尋找必要的依賴
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 宣告你要編譯哪些檔案
set(msg_files
  "msg/MowerStatus.msg"
  "msg/MotorStatus.msg"
)

set(srv_files
  "srv/SetDriveMode.srv"
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

#產生接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs
)
ament_package()
