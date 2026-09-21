#!/usr/bin/env bash
# ==========================================================================
# mowerbot 一鍵啟動
#
# 把 gazebo / mower_control / navigation / rviz 四支 launch 收成一次啟動
# (demo.launch.py)，並且幫忙做好啟動前的準備與離開時的收尾：
#
#   1. 清掉上次閃退或 Ctrl+C 沒收乾淨的殘留行程 (會印出清掉了哪些)
#   2. export SVGA_VGPU10=0   —— VMware 的 mesa svga 驅動 OpenGL workaround
#   3. source ROS 與本 workspace 的環境
#   4. 啟動 demo.launch.py，所有參數原封不動透傳進去
#   5. Ctrl+C 時把整組子行程收乾淨才退出，不留孤兒節點污染下一次啟動
#
# 用法:
#   ./run_demo.sh                          無頭 Gazebo + RViz (預設)
#   ./run_demo.sh gui:=true                要看 Gazebo 的 3D 畫面
#   ./run_demo.sh nav:=false               只跑建圖，不啟動 Nav2
#   ./run_demo.sh world:=mow_field.world   換世界
#   ./run_demo.sh rviz:=false gui:=true    只看 Gazebo，不開 RViz
#
# 參數就是 demo.launch.py 的 launch argument，可以疊加。
# ==========================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash
WS_SETUP="${WS}/install/setup.bash"

# --------------------------------------------------------------------------
# 怎麼認定一個行程是「這個 workspace 留下來的殘留」
#
# 舊版是對每個節點名稱做 pgrep -f (比對整條命令列)，那會誤殺無關的行程 ——
# 命令列裡剛好出現 controller_server 字樣的編輯器、tail、甚至執行這支腳本的
# shell 自己都會被殺掉 (實際發生過三次)。
#
# 現在改成兩條規則：
#   (1) 本專案的節點：命令列有 --ros-args，而且命令列或環境變數裡出現
#       這個 workspace 的 install 路徑。
#       從這裡啟動的節點，命令列會帶 <WS>/install/... 的參數檔或執行檔路徑；
#       少數 (robot_state_publisher / joint_state_publisher) 的參數檔在 /tmp，
#       但它們的環境變數 AMENT_PREFIX_PATH 一定含有這個 install 路徑。
#       為了不誤殺「只是 source 過這個 workspace 的互動式 shell」，
#       走環境變數這條規則時額外要求命令列裡有 --ros-args (那是 ROS 節點的特徵)。
#   (2) gzserver / gzclient：名稱夠獨特，維持名稱比對 (pgrep -x 比對執行檔名)。
#
# 兩條規則都排除自己、自己的父行程、以及所有祖先行程。
# --------------------------------------------------------------------------
WS_INSTALL="${WS}/install"

# 名稱比對的行程 (執行檔名精確比對)
STALE_EXACT_NAMES=(gzserver gzclient)

# 自己與所有祖先的 PID，這些永遠不能殺
ancestor_pids() {
    local pid=$$ out=""
    while [ -n "${pid}" ] && [ "${pid}" != "0" ] && [ "${pid}" != "1" ]; do
        out="${out} ${pid}"
        pid="$(awk '{print $4}' "/proc/${pid}/stat" 2>/dev/null)"
    done
    echo "${out}"
}

# 屬於這個 workspace 的 ROS 節點行程
ws_pids() {
    local excl="$1" pid d
    for d in /proc/[0-9]*; do
        pid="${d#/proc/}"
        case " ${excl} " in *" ${pid} "*) continue ;; esac
        # 一律要求命令列有 --ros-args (ROS 節點的特徵)，
        # 這樣 tail/編輯器之類「命令列剛好提到 install 目錄」的行程不會中獎
        grep -qa -- "--ros-args" "${d}/cmdline" 2>/dev/null || continue
        # 命令列直接帶 install 路徑，或環境變數指向這個 workspace
        if grep -qa -- "${WS_INSTALL}" "${d}/cmdline" 2>/dev/null ||
                grep -qa -- "${WS_INSTALL}" "${d}/environ" 2>/dev/null; then
            echo "${pid}"
        fi
    done
}

# 名稱精確比對的行程
named_pids() {
    local excl="$1" name pid
    for name in "${STALE_EXACT_NAMES[@]}"; do
        for pid in $(pgrep -x -- "${name}" 2>/dev/null || true); do
            case " ${excl} " in *" ${pid} "*) continue ;; esac
            echo "${pid}"
        done
    done
}

stale_pids() {
    local excl
    excl="$(ancestor_pids)"
    { ws_pids "${excl}"; named_pids "${excl}"; } | sort -un
}

# --------------------------------------------------------------------------
# 清掉殘留行程。不靜默地殺：每一個都印出 PID 與完整指令列。
# --------------------------------------------------------------------------
kill_stale() {
    local found=0 pid

    for pid in $(stale_pids); do
        kill -0 "${pid}" 2>/dev/null || continue
        if [ "${found}" = 0 ]; then
            echo "--- 清掉上次留下的殘留行程 ---"
            found=1
        fi
        printf '  kill %-7s %s\n' "${pid}" "$(ps -o cmd= -p "${pid}" 2>/dev/null | cut -c1-100)"
        kill -TERM "${pid}" 2>/dev/null
    done

    if [ "${found}" = 0 ]; then
        echo "--- 沒有殘留行程，環境是乾淨的 ---"
        return
    fi

    # 給 1 秒好好收，收不掉的直接 KILL
    sleep 1
    for pid in $(stale_pids); do
        kill -0 "${pid}" 2>/dev/null || continue
        printf '  kill -9 %-7s %s\n' "${pid}" "$(ps -o cmd= -p "${pid}" 2>/dev/null | cut -c1-100)"
        kill -KILL "${pid}" 2>/dev/null
    done

    # gzserver 佔著的 port (11345) 要一點時間才釋放，太快重開會起不來
    echo "  等 2 秒讓 port 釋放..."
    sleep 2
}

# --------------------------------------------------------------------------
# 離開時的收尾：先請 ros2 launch 自己收 (SIGINT)，收不完再硬清一次。
# --------------------------------------------------------------------------
LAUNCH_PID=""
LAUNCH_PGID=""

shutdown() {
    trap - INT TERM EXIT
    echo ""
    echo "--- 收到中斷，正在關閉 ---"

    if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        # setsid 起的，ros2 launch 自成一個 process group，
        # 對整個 group 送訊號才能連帶收掉 gzserver 這種孫行程
        kill -INT "-${LAUNCH_PGID:-${LAUNCH_PID}}" 2>/dev/null
        for _ in $(seq 1 15); do
            kill -0 "${LAUNCH_PID}" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo "  ros2 launch 15 秒內沒收完，改送 SIGKILL"
            kill -KILL "-${LAUNCH_PGID:-${LAUNCH_PID}}" 2>/dev/null
            sleep 1
        fi
    fi

    # ros2 launch 收完之後 Gazebo 常常還有殘骸，再掃一次確保乾淨
    kill_stale
    echo "--- 已關閉 ---"
    exit 0
}

# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
echo "=========================================================="
echo " mowerbot 一鍵啟動   workspace: ${WS}"
echo "=========================================================="

kill_stale

# VMware 的 mesa svga 驅動在 OpenGL 3 路徑上有已知問題，
# 會讓 gzclient / RViz 的畫面壞掉甚至閃退，關掉 vgpu10 走舊路徑比較穩。
export SVGA_VGPU10=0
echo "--- SVGA_VGPU10=0 (VMware mesa svga 的 OpenGL workaround) ---"

if [ ! -f "${ROS_SETUP}" ]; then
    echo "找不到 ${ROS_SETUP}，無法繼續" >&2
    exit 2
fi
# ROS 的 setup.bash 會讀 AMENT_TRACE_SETUP_FILES 等沒有預先定義的變數，
# 在 set -u 底下 source 會直接中止，所以這一段暫時關掉 -u。
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
if [ -f "${WS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${WS_SETUP}"
    set -u
else
    set -u
    echo "找不到 ${WS_SETUP}，請先在 ${WS} 執行 colcon build" >&2
    exit 2
fi
echo "--- 已 source ROS 與 workspace 環境 ---"

trap shutdown INT TERM EXIT

echo "--- ros2 launch mowerbot_bringup demo.launch.py $* ---"
echo ""
# 自成一個 session/process group，離開時才能對整個 group 送訊號
setsid ros2 launch mowerbot_bringup demo.launch.py "$@" &
LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d ' ')"

wait "${LAUNCH_PID}"
shutdown
