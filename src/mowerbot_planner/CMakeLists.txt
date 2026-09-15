cmake_minimum_required(VERSION 3.8)
project(mowerbot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 尋找必要的 ROS 2 依賴套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mowerbot_interfaces REQUIRED)

# 2. 尋找 Fields2Cover 函式庫 (【關鍵修正】：首字母大寫)
find_package(Fields2Cover REQUIRED)

# 3. 定義要編譯的 C++ 節點
add_executable(f2c_server src/f2c_server.cpp)
target_include_directories(f2c_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 4. 連結所有需要的函式庫
ament_target_dependencies(f2c_server
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  mowerbot_interfaces
)
# 【關鍵修正】：連結目標名稱也要大寫
target_link_libraries(f2c_server Fields2Cover::Fields2Cover)

# 5. 設定安裝路徑
install(TARGETS
  f2c_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()