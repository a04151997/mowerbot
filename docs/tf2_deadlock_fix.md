# tf2_ros Buffer::waitForTransform 死鎖與修正 overlay（2026-10-05，新小電腦排錯）

## 症狀
中途 controller_server 一直報 `Transform data too old when converting from map to odom`，
Transform time 停在某個固定時間，其他程序（mower_manager、獨立 tf2 listener、rosbag）收到的 map->odom 一直正常。
隨機發生（sim 54 / 155 / 188 / 631 / 823 s），fastrtps 與 cyclonedds 都會發生，不是 DDS、也不是 slam 換手時序。

## 根因
gdb 取證（`sudo gdb -p <controller_server pid> -batch -ex "thread apply all bt"`）：兩個執行緒互等（AB-BA）：
* costmap 掃描 callback：`tf2_ros::MessageFilter -> Buffer::waitForTransform`，持有 `timer_to_request_map_mutex_`，
  再呼叫 `BufferCore::addTransformableRequest` 要 `transformable_requests_mutex_`。
* TF 監聽執行緒：`setTransform -> testTransformableRequests` 持有 `transformable_requests_mutex_`，
  callback 要 `timer_to_request_map_mutex_`。
TF 監聽執行緒永久卡死 -> 該程序的 tf buffer 不再更新。tf2_ros 0.25.23（apt）與 humble 分支 0.25.24 都有此問題。

## 修正
`tools/tf2_deadlock_fix/tf2_buffer_deadlock.patch`：只改 `tf2_ros/src/buffer.cpp`（不改 header / ABI）。
addTransformableRequest 不再在 timer mutex 底下呼叫；callback 若早於 timer 登記就先暫存、登記後補送。

## 建置（使用者層級，不需 sudo、不動系統套件）
    mkdir -p ~/tf2_fix_ws/src && cd ~/tf2_fix_ws/src
    git clone -b humble https://github.com/ros2/geometry2 && cd geometry2 && git apply ~/mowerbot/tools/tf2_deadlock_fix/tf2_buffer_deadlock.patch
    cd ~/tf2_fix_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select tf2_ros --cmake-args -DBUILD_TESTING=OFF
`run_demo.sh` 會自動 source `~/tf2_fix_ws/install/setup.bash`（可用 TF2_FIX_WS 改路徑）。
驗證載入：`grep libtf2_ros /proc/$(pgrep -f 'controller_server.*--ros-args'|head -1)/maps` 應指向 ~/tf2_fix_ws。

## 對真實流程的意義
這是 Nav2 + tf2 在 Humble 的共通問題，實機（上位機）同樣會遇到。實機部署必須帶同樣的修正（或等上游修掉），
不是只有模擬測試腳本需要。
