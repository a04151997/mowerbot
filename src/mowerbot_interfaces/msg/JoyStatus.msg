# 手把的連線與按鍵狀態 (階段 19 新增)。
#
# 由 mower_teleop 以 5 Hz 發布到 /joy_status。
#
# 【為什麼是 teleop 發，不是另外寫一個節點】
# button_deadman / button_stop 的 index 是從 joystick.yaml 讀進來的參數，
# 只有 teleop 知道。第二個節點要自己再抄一份 index，
# 兩邊遲早會不同步 —— 而不同步的後果是「畫面說安全鈕沒按，實際按著」。

std_msgs/Header header

# 最近 0.5 秒內有收到 /joy 就算連線中。
# joy_node 的 autorepeat_rate 是 20 Hz，手把靜止時也會重複發最後狀態，
# 所以「沒有訊息」就等於「手把不在了」，可以當成可靠的心跳。
bool connected

# 目前 deadman (LB) 是否按住。失去連線時一律 false ——
# 寧可誤報「沒按住」，不可誤報「按住中」。
bool deadman_held

# 目前急停鍵是否按住。失去連線時同樣一律 false。
bool stop_pressed

# 距離上一筆 /joy 幾秒。從未收到過時是一個很大的值 (999.0)。
float32 last_msg_age
