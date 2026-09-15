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

echo "[2/2] save_map -> ${NAME}.pgm + ${NAME}.yaml"
if ! ros2 service call /slam_toolbox/save_map \
        slam_toolbox/srv/SaveMap \
        "{name: {data: '${TARGET}'}}"; then
    echo "  !! save_map 呼叫失敗，確認建圖模式是否正在跑" >&2
    rc=1
fi
echo ""

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
