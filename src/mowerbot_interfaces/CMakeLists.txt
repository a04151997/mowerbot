cmake_minimum_required(VERSION 3.8)
project(mowerbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找必要的依賴
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # <== 新增：為了支援 Path 格式

# 宣告你要編譯哪些 Message 檔案
set(msg_files
  "msg/MowerStatus.msg"
  "msg/MotorStatus.msg"
  "msg/ObstaclePolygons.msg" # <== 階段 11：作業區內部障礙物的輪廓集合
)

# 宣告你要編譯哪些 Service 檔案
set(srv_files
  "srv/SetDriveMode.srv"
  "srv/GenerateCoveragePath.srv" # <== 新增：我們剛剛寫的新服務
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 產生介面程式碼
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs nav_msgs # <== 新增：加入 nav_msgs 作為依賴
)

ament_package()