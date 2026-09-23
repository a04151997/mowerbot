# 覆蓋任務的即時狀態 (階段 18 新增)。
#
# 這些資料 mower_manager 內部本來就有 (階段 17 的跳過機制已經在記錄)，
# 只是沒有對外發布，所以 GUI 無從得知任務跑到哪裡。
# 由 mower_manager 以 5 Hz 發布到 /mission_status。

std_msgs/Header header

# --- 任務狀態 ---
uint8 STATE_IDLE = 0        # 待命：沒有任務在跑
uint8 STATE_PLANNING = 1    # 規劃中：已送出 F2C 請求，還沒拿到路徑
uint8 STATE_EXECUTING = 2   # 執行中：佇列正在逐段送給 Nav2
uint8 STATE_DONE = 3        # 已完成：佇列跑完 (可能有跳過的段落)
uint8 STATE_ABORTED = 4     # 已中止：急停、模式切換、規劃失敗、或連續失敗達門檻
uint8 state

# --- 進度 ---
# 段數是「佇列裡的任務數」，含 approach、周邊環繞、銜接段與割草線，
# 不是只有割草線 —— GUI 上的進度條要反映實際還要走多久。
uint16 total_segments
uint16 completed_segments
uint16 skipped_segments

# 目前正在執行的段落，例如 "割草線 5/13"、"approach"、"周邊環繞 2/4"。
# 沒有任務在跑時是空字串。
string current_label

# 已經跳過的段落 (階段 17 的行為：被臨時障礙物擋住就跳過繼續)。
# 格式 "割草線 5/13 (-4.00, -2.15)"，帶座標是為了實車能回頭補割。
string[] skipped_labels

# 任務層級的說明訊息，給 HMI 直接顯示用 (階段 23 新增)。
# 目前唯一的來源是「拒絕開始」：車子當下停的位置淨空不足以原地掉頭時，
# manager 不會啟動任務，而是把原因寫在這裡，例如
#   「無法開始：車輛目前位置 (-5.36, -4.57) 離邊界只有 0.49 m
#     (掉頭需要 0.58 m)，不足以原地掉頭。請先用手動模式(模式 0)
#     把車輛移到開闊處，再切回自動割草。」
# 沒有訊息時是空字串。state 之外還需要這個欄位，是因為「已中止」
# 有很多種原因，只看 state 使用者不知道該做什麼。
string message
