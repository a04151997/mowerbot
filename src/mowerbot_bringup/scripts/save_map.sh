#!/usr/bin/env bash
# ==========================================================================
# 一鍵存圖：把目前 slam_toolbox 建好的地圖同時存成兩種格式
#
#   1. serialize_map -> <name>.posegraph + <name>.data
#      給 slam_toolbox 的定位模式 (localization.launch.py) 讀
#   2. save_map      -> <name>.pgm + <name>.yaml
#      給 nav2 的 map_server 讀
#
# 兩種都要存：定位模式讀的是位姿圖，Nav2 的靜態圖層讀的是佔據網格圖，
# 少存一種就會有一邊起不來。
#
# 用法 (建圖模式要正在跑)：
#   ros2 run mowerbot_bringup save_map.sh            # 存成 mowerbot_map
#   ros2 run mowerbot_bringup save_map.sh my_field   # 自訂檔名
#   MAP_DIR=/tmp ros2 run mowerbot_bringup save_map.sh
# ==========================================================================
set -u

NAME="${1:-mowerbot_map}"

# 預設存到原始碼樹的 maps/，這樣重新 colcon build 之後會被安裝進 share/。
# 存到別處時用 MAP_DIR 覆寫。
if [ -z "${MAP_DIR:-}" ]; then
    SHARE_DIR="$(ros2 pkg prefix mowerbot_bringup 2>/dev/null)/share/mowerbot_bringup"
    # 從 install 目錄回推原始碼目錄；推不出來就退回 install 裡的 maps/
    SRC_DIR="$(cd "${SHARE_DIR}/../../../.." 2>/dev/null && pwd)/src/mowerbot_bringup/maps"
    if [ -d "${SRC_DIR}" ]; then
        MAP_DIR="${SRC_DIR}"
    else
        MAP_DIR="${SHARE_DIR}/maps"
    fi
fi

mkdir -p "${MAP_DIR}"
TARGET="${MAP_DIR}/${NAME}"

echo "存圖目標 (不含副檔名): ${TARGET}"
echo ""

rc=0

echo "[1/2] serialize_map -> ${NAME}.posegraph + ${NAME}.data"
if ! ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '${TARGET}'}"; then
    echo "  !! serialize_map 呼叫失敗，確認建圖模式是否正在跑" >&2
    rc=1
fi
echo ""

# /slam_toolbox/save_map 這個服務本身會間歇性回傳 result=255
# (RESULT_UNDEFINED_FAILURE)，而且與前面有沒有呼叫 serialize_map 無關。
# 實測：連續呼叫 15 次有 3 次失敗 (約 20%)，
# 其中「單獨呼叫 save_map」5 次就失敗了 2 次。
# 服務回傳失敗時不會產生 .pgm/.yaml，所以這裡重試最多 3 次，
# 每次都把結果印出來，讓這個不穩定性留在 log 裡看得見，而不是被默默吞掉。
#
# 2026-10-20：slam_toolbox 的 save_map 其實只是在裡面跑
#   ros2 run nav2_map_server map_saver_cli -f <名稱> --ros-args -p map_subscribe_transient_local:=true
# map_saver_cli 預設只等 /map 2 秒 (save_map_timeout)，等不到就失敗、回 255。
# CPU 忙的時候 (模擬：建圖 + F2C 一起跑) 3 次全失敗。所以前兩次照舊走 slam_toolbox，
# 第 3 次直接跑 map_saver_cli、等 10 秒；三種方式存出來的檔一樣 (同一支程式、同樣的預設門檻)。
echo "[2/2] save_map -> ${NAME}.pgm + ${NAME}.yaml"
saved=0
for attempt in 1 2 3; do
    echo "  第 ${attempt} 次嘗試..."
    if [ "${attempt}" -lt 3 ]; then
        out="$(ros2 service call /slam_toolbox/save_map \
                slam_toolbox/srv/SaveMap \
                "{name: {data: '${TARGET}'}}" 2>&1)"
        echo "${out}"
        echo "${out}" | grep -q "result=0" && saved=1
    else
        echo "  (直接跑 map_saver_cli，等 /map 最多 10 秒)"
        if timeout 30 ros2 run nav2_map_server map_saver_cli -f "${TARGET}" --ros-args \
                -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 2>&1; then
            saved=1
        fi
    fi
    if [ "${saved}" -eq 1 ]; then
        if [ "${attempt}" -gt 1 ]; then
            echo "  (注意：save_map 第 ${attempt} 次才成功，這個服務本身不穩定)"
        fi
        break
    fi
    echo "  !! save_map 回傳失敗 (常見為 result=255)，重試中..." >&2
    sleep 2
done
if [ "${saved}" -ne 1 ]; then
    echo "  !! save_map 連續 3 次都失敗，確認建圖模式是否正在跑" >&2
    rc=1
fi

# 【階段 29】未觀測的格子讀回來時必須還是「未知」，不能變成「自由」。
#
# save_map 內部用 nav2 的 map_saver 寫檔，.pgm 的灰階是
#   自由 254 / 佔據 0 / 未知 205
# 但 .yaml 寫的是 map_saver 的預設 free_thresh: 0.25。map_server 讀回來時
# 以 p = (255 - 灰階) / 255 判定，未知的 205 -> p = 0.196 < 0.25 -> **自由**。
# 實測（報告 29.1）：即時 /map 有 7347 格 -1，存檔 205 也是 7347 格，
# map_server 讀回來 -1 變成 0 格，全部變成可割的草地。
# 障礙物背面、沒掃到的角落因此被規劃成割草線 —— 箱子只剩被看到的兩面。
#
# 改成 0.10：三種灰階的 p 是 0.004 / 0.196 / 1.0，
# 0.10 讓 254 仍是自由、205 回到未知，兩邊都有大餘裕。
# 「沒看過的地方就不割」—— 與邊界腐蝕、headland 同一個保守方向。
if [ -f "${TARGET}.yaml" ]; then
    sed -i 's/^free_thresh: .*/free_thresh: 0.10/' "${TARGET}.yaml"
    echo "  free_thresh 改成 0.10（未觀測的格子讀回來仍是未知，見報告 29.1）"
fi
echo ""

# 禁區 (2026-10-18)：記下這張圖的 session_id (建圖那次開機產生的；用存好的地圖開機時沿用那張圖的)，
# 再產生一張給人畫禁區的圖 <名稱>_draw.png
# (用法見 mowerbot_action/keepout.py)。keepout_server 沒開時 (只跑 SLAM、舊版程式) 跳過，
# 不影響存圖本身，也不改 rc。
if [ -f "${TARGET}.yaml" ]; then
    if session="$(ros2 run mowerbot_action keepout_tool session 2>/dev/null)" && [ -n "${session}" ]; then
        echo "${session}" > "${TARGET}.session"
        echo "  地圖的 session：${session}（禁區、外框跟著它存；下次用這張圖啟動會載回來）"
        ros2 run mowerbot_action keepout_tool template "${TARGET}" \
            || echo "  !! 畫禁區用的圖沒有產生" >&2
    else
        echo "  (keepout_server 沒開，沒有產生畫禁區用的圖)"
    fi
    echo ""
fi

echo "產生的檔案:"
found=0
for ext in posegraph data pgm yaml; do
    f="${TARGET}.${ext}"
    if [ -f "${f}" ]; then
        printf '  %-12s %10s bytes  %s\n' ".${ext}" "$(stat -c%s "${f}")" "${f}"
        found=$((found + 1))
    else
        printf '  %-12s %10s\n' ".${ext}" "(沒有產生)"
        rc=1
    fi
done

if [ "${found}" -eq 4 ]; then
    echo ""
    echo "四個檔案都在。接下來可以用定位模式啟動："
    echo "  ros2 launch mowerbot_bringup localization.launch.py map_file_name:=${TARGET}"
fi

exit "${rc}"
