# 車輛與作業狀態 (由 mower_manager 以 5 Hz 發布到 /mower_status)。
#
# 【硬體欄位目前沒有來源】
# battery_* / bumper_pressed / is_overheated 目前一律填 0 或 false：
# 這些值要等實車的 bridge_node 接上驅動板之後才讀得到
# (見 mowerbot_bridge/drivers/README.md 的 D 節)。
# 屆時要決定是「manager 訂閱 bridge_node 的遙測後合併發布」，
# 還是「把訊息拆成車輛狀態與作業狀態兩支」。
# 現在不要處理這件事 —— 在沒有硬體來源之前，
# 任何填進去的值都是假的，GUI 顯示假的電量比不顯示更糟。
#
# stop_active 與 mode 是真的：它們來自 manager 自己的模式狀態。

#電源
float32 battery_voltage #電池電壓
float32 battery_current #即時電流
float32 battery_percentage #電池電量

#安全系統
bool bumper_pressed #實體保險桿是否觸發
bool stop_active #緊急停止按鈕是否觸發
bool is_overheated #馬達或驅動是否過熱

#作業狀態
uint8 mode #0:移動建圖 1:導航割草 2:搖桿控制割草




