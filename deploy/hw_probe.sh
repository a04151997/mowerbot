#!/usr/bin/env bash
# ==========================================================================
# mowerbot 現場硬體診斷（階段 34）
#
# ⚠️⚠️⚠️ 這支腳本只讀不寫 ⚠️⚠️⚠️
#   - 絕對不送出任何馬達控制指令，也不對任何序列埠寫入任何一個位元組。
#     序列埠一律用 O_RDONLY 開啟：檔案描述子本身就沒有寫入權限，
#     程式裡也沒有任何 write() 呼叫。
#   - 不改系統設定：不裝套件、不寫 udev 規則、不改群組、不重載任何服務。
#   - 唯一會碰到的「設定」是本機序列埠的 termios（鮑率、raw 模式，主機端的 UART
#     設定，不會送出任何資料）。
#   - 【ECHO】Linux 序列埠的預設 termios 開著 ECHO：收到的每個位元組會被核心
#     「原封不動從 TX 送回去」—— 等於把驅動板送來的封包回寫給驅動板。
#     所以開啟後第一件事就是關掉 ECHO 並清空兩個方向的佇列，
#     結束時也【不】把原本（開著 ECHO）的設定還原回去。
#     這是用假裝置實測抓到的：沒處理時每次會回送一整包（41 bytes）。
#   - 注意：Linux 核心在「開啟」序列埠時會拉高 DTR/RTS 兩條控制線
#     （任何程式開序列埠都一樣，包括 ROS 驅動）。有「一鍵下載電路」的 STM32 板
#     可能因此重置一次。這不是指令，但請在車子架高、或馬達電源關閉時執行。
#     每個序列埠只開一次（換鮑率不重開），把這個影響降到最少。
#
# 偵測內容：
#   1. 系統（主機、核心、CPU、記憶體、磁碟）
#   2. lsusb 完整列表
#   3. /dev/ttyUSB* /dev/ttyACM* 對應的 USB 裝置（VID:PID、廠商、序號）
#      + 常見 2D 光達的 VID:PID 比對
#   4. 每個序列埠用 9600 / 57600 / 115200 / 230400 各試讀 2 秒，印原始位元組（hex）
#      + 輔助判讀：輪趣下位機的 24 byte 上行封包、LD06/LD19 的封包
#   5. 手把 /dev/input/js*（名稱、軸數、按鍵數）
#   6. 網路介面與 IP
#   7. ROS 2 版本、workspace 有沒有編譯好
#   8. 摘要（放在最後，終端機最後一個畫面拍照就能看到重點）
#
# 用法：
#   bash deploy/hw_probe.sh
# 輸出：目前目錄的 hw_probe_<時間>.txt（目前目錄不能寫就改寫到家目錄），同時印在畫面上。
# 不需要 sudo。
# ==========================================================================
set -u

WS="${MOWERBOT_WS:-${HOME}/mowerbot}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$(pwd)"
if ! touch "${OUT_DIR}/.hw_probe_write_test" 2>/dev/null; then
    OUT_DIR="${HOME}"
else
    rm -f "${OUT_DIR}/.hw_probe_write_test"
fi
OUT="${OUT_DIR}/hw_probe_${STAMP}.txt"
READ_SECONDS="${HW_PROBE_READ_SECONDS:-2}"

exec > >(tee "${OUT}") 2>&1

SUMMARY=()
add_summary() { SUMMARY+=("$*"); }

sec() {
    echo ""
    echo "=== $* ==="
}

echo "############################################################"
echo "# mowerbot hw_probe  $(date '+%Y-%m-%d %H:%M:%S')"
echo "# 只讀不寫：不送任何馬達指令、不寫任何序列埠"
echo "############################################################"

# --------------------------------------------------------------------------
sec "1. 系統"
# --------------------------------------------------------------------------
echo "主機  : $(hostname)"
echo "使用者: ${USER}  群組: $(id -nG)"
echo "系統  : $(. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME}")"
echo "核心  : $(uname -r)"
echo "CPU   : $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ *//') x $(nproc)"
echo "--- 記憶體 ---"
free -h | sed 's/^/  /'
echo "--- 磁碟 ---"
df -h / "${HOME}" 2>/dev/null | awk '!seen[$0]++' | sed 's/^/  /'
ROOT_AVAIL_G=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
add_summary "磁碟 / 剩 ${ROOT_AVAIL_G} GB；記憶體 $(free -h | awk '/^Mem:/{print $2" 總量, "$7" 可用"}')"
if id -nG | tr ' ' '\n' | grep -qx dialout; then
    DIALOUT_OK=1
else
    DIALOUT_OK=0
    if id -nG "${USER}" | tr ' ' '\n' | grep -qx dialout; then
        echo "⚠️ ${USER} 已加入 dialout，但這個登入階段還沒生效（要登出再登入）"
    else
        echo "⚠️ ${USER} 不在 dialout 群組，序列埠可能讀不到（Permission denied）"
    fi
fi

# --------------------------------------------------------------------------
sec "2. lsusb"
# --------------------------------------------------------------------------
if command -v lsusb >/dev/null 2>&1; then
    lsusb
    add_summary "USB 裝置 $(lsusb | wc -l) 個（完整列表見第 2 節）"
else
    echo "(沒有 lsusb 指令，改讀 /sys/bus/usb/devices)"
    for d in /sys/bus/usb/devices/*; do
        [ -f "${d}/idVendor" ] || continue
        printf '%s %s:%s %s %s\n' "$(basename "${d}")" "$(cat "${d}/idVendor")" \
            "$(cat "${d}/idProduct")" "$(cat "${d}/manufacturer" 2>/dev/null)" \
            "$(cat "${d}/product" 2>/dev/null)"
    done
fi

# --------------------------------------------------------------------------
sec "3. 序列埠 ↔ USB 裝置"
# --------------------------------------------------------------------------
# VID:PID -> 可能是什麼。USB 轉序列晶片是通用零件，同一個 VID:PID 會出現在
# 很多產品上（輪趣的下位機、LD19、RPLidar 都用 CP2102），只能當「候選」，
# 要配合第 4 節的原始位元組一起判斷。
known_usb() {
    case "$1" in
        10c4:ea60) echo "CP210x 晶片｜候選：輪趣下位機(序號 0002)、LD06/LD19、RPLidar A1/A2/A3/S1/S2/C1、YDLidar X4" ;;
        1a86:7523) echo "CH340 晶片｜候選：舊款輪趣下位機、YDLidar X2/X3、部分 LD06 轉接板" ;;
        1a86:55d4) echo "CH9102 晶片｜候選：新款輪趣下位機(序號 0002)、部分 LD19 轉接板" ;;
        1a86:55d3|1a86:55d6) echo "CH343/CH9143 晶片｜候選：輪趣下位機、部分光達轉接板" ;;
        0403:6001|0403:6015) echo "FTDI 晶片｜通用 USB 轉序列" ;;
        067b:2303) echo "PL2303 晶片｜通用 USB 轉序列" ;;
        15d1:0000) echo "Hokuyo URG 系列光達（ttyACM）" ;;
        0483:5740) echo "STM32 虛擬序列埠（USB CDC）｜可能是驅動板直接走 USB" ;;
        2341:*|2a03:*) echo "Arduino" ;;
        *) echo "" ;;
    esac
}

TTYS=()
for t in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "${t}" ] && TTYS+=("${t}")
done

if [ "${#TTYS[@]}" -eq 0 ]; then
    echo "沒有 /dev/ttyUSB* 或 /dev/ttyACM*"
    add_summary "序列埠：無（驅動板 / 光達的 USB 都沒被認到）"
fi

declare -A TTY_DESC
for t in "${TTYS[@]}"; do
    props="$(udevadm info -q property -n "${t}" 2>/dev/null)"
    get() { echo "${props}" | awk -F= -v k="$1" '$1==k{print substr($0, length(k)+2)}'; }
    vid="$(get ID_VENDOR_ID)"; pid="$(get ID_MODEL_ID)"
    vendor="$(get ID_VENDOR_FROM_DATABASE)"; [ -n "${vendor}" ] || vendor="$(get ID_VENDOR)"
    model="$(get ID_MODEL_FROM_DATABASE)"; [ -n "${model}" ] || model="$(get ID_MODEL)"
    serial="$(get ID_SERIAL_SHORT)"
    drv="$(get ID_USB_DRIVER)"
    path="$(get ID_PATH)"
    byid="$(for l in /dev/serial/by-id/*; do [ "$(readlink -f "${l}")" = "${t}" ] && basename "${l}"; done 2>/dev/null)"
    links="$(for l in /dev/*; do [ -L "${l}" ] && [ "$(readlink -f "${l}")" = "${t}" ] && basename "${l}"; done 2>/dev/null | tr '\n' ' ')"
    hint="$(known_usb "${vid}:${pid}")"
    echo "${t}"
    echo "  VID:PID ${vid}:${pid}  驅動 ${drv}"
    echo "  廠商    ${vendor}"
    echo "  型號    ${model}"
    echo "  序號    ${serial:-(無)}"
    echo "  by-id   ${byid:-(無)}"
    echo "  by-path ${path:-(無)}"
    echo "  /dev 下指向它的別名: ${links:-(無)}"
    [ -n "${hint}" ] && echo "  比對    ${hint}"
    TTY_DESC["${t}"]="${vid}:${pid} ${model} 序號${serial:-無}"
done

# --------------------------------------------------------------------------
sec "4. 序列埠試讀（只讀，${READ_SECONDS} 秒 x 4 種鮑率）"
# --------------------------------------------------------------------------
echo "每個鮑率印前 96 個位元組（hex）與總位元組數。"
echo "輔助判讀（只是模式比對，最後以原始位元組為準）："
echo "  輪趣下位機：115200，24 byte 一包，7B 開頭 7D 結尾，第 23 byte = 前 22 byte XOR"
echo "  LD06/LD19 ：230400，47 byte 一包，54 2C 開頭，會自己一直送"
echo "  RPLidar   ：沒收到啟動指令前不送資料（本腳本不送），所以會讀到 0 byte"

PROBE_PY="$(cat <<'PYEOF'
import os, sys, termios, time, select

port = sys.argv[1]
secs = float(sys.argv[2])
BAUDS = [(9600, termios.B9600), (57600, termios.B57600),
         (115200, termios.B115200), (230400, termios.B230400)]


def wheeltec_frames(buf):
    """輪趣上行封包：24 bytes, [0]=0x7B, [23]=0x7D, [22]=XOR([0..21])"""
    n, first = 0, None
    for i in range(len(buf) - 23):
        if buf[i] != 0x7B or buf[i + 23] != 0x7D:
            continue
        x = 0
        for b in buf[i:i + 22]:
            x ^= b
        if x == buf[i + 22]:
            n += 1
            if first is None:
                first = buf[i:i + 24]
    return n, first


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


def ld_frames(buf):
    return sum(1 for i in range(len(buf) - 1) if buf[i] == 0x54 and buf[i + 1] == 0x2C)


try:
    # O_RDONLY：這個檔案描述子沒有寫入權限。O_NOCTTY：不要變成控制終端機。
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
except OSError as e:
    print('  開不起來：%s' % e)
    if e.errno == 13:
        print('  → 權限不足：把使用者加進 dialout 並重新登入')
    elif e.errno == 16:
        print('  → 被其他程式佔用（ROS 驅動、ModemManager？）')
    sys.exit(0)



def set_raw(fd, bconst):
    # raw 8N1：lflag=0 關掉 ECHO（不然收到的位元組會被核心從 TX 回送給裝置），
    # 不做任何字元轉換；CREAD 讓 UART 接收
    a = termios.tcgetattr(fd)
    a[0] = 0                                             # iflag
    a[1] = 0                                             # oflag
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    a[3] = 0                                             # lflag（ECHO 關）
    a[4] = bconst                                        # ispeed
    a[5] = bconst                                        # ospeed
    termios.tcsetattr(fd, termios.TCSANOW, a)
    # 輸出佇列也清掉：開啟到上一行之間若有被 ECHO 排進去的位元組，在送出前丟掉
    termios.tcflush(fd, termios.TCIOFLUSH)


# 開啟後的第一個動作：立刻關 ECHO
set_raw(fd, BAUDS[0][1])
results = []
try:
    for baud, bconst in BAUDS:
        set_raw(fd, bconst)
        buf = bytearray()
        t_end = time.monotonic() + secs
        while True:
            left = t_end - time.monotonic()
            if left <= 0:
                break
            r, _, _ = select.select([fd], [], [], min(left, 0.2))
            if r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if chunk:
                    buf.extend(chunk)
        head = buf[:96]
        print('  -- %6d baud: %d bytes (%.0f B/s)' % (baud, len(buf), len(buf) / secs))
        for i in range(0, len(head), 24):
            print('     ' + ' '.join('%02X' % b for b in head[i:i + 24]))
        nw, first = wheeltec_frames(buf)
        nl = ld_frames(buf)
        if nw:
            vx = s16(first[2], first[3]) / 1000.0
            vy = s16(first[4], first[5]) / 1000.0
            wz = s16(first[6], first[7]) / 1000.0
            volt = ((first[20] << 8) | first[21]) / 1000.0
            print('     ★ 輪趣格式封包 %d 包（BCC 正確）。第一包解讀：'
                  'Flag_Stop=%d vx=%.3f m/s vy=%.3f m/s wz=%.3f rad/s 電壓=%.2f V'
                  % (nw, first[1], vx, vy, wz, volt))
        if nl >= 3:
            print('     ★ 54 2C 標頭出現 %d 次（LD06/LD19 光達的封包格式）' % nl)
        results.append((baud, len(buf), nw, nl))
finally:
    # 不還原開啟前的 termios：那份設定開著 ECHO，還原到 close 之間收到的位元組會被回送
    os.close(fd)

best = max(results, key=lambda r: (r[2], r[3], r[1]))
tag = ''
if best[2]:
    tag = 'WHEELTEC'
elif best[3] >= 3:
    tag = 'LD06/LD19'
elif best[1] == 0:
    tag = 'SILENT'
print('RESULT %s %d %d %d %d' % (tag or 'UNKNOWN', best[0], best[1], best[2], best[3]))
PYEOF
)"

for t in "${TTYS[@]}"; do
    echo ""
    echo "[${t}] ${TTY_DESC[${t}]}"
    if ! command -v python3 >/dev/null 2>&1; then
        echo "  沒有 python3，無法試讀"
        continue
    fi
    res="$(python3 -c "${PROBE_PY}" "${t}" "${READ_SECONDS}" 2>&1)"
    echo "${res}" | grep -v '^RESULT '
    r="$(echo "${res}" | grep '^RESULT ' | tail -1)"
    if [ -n "${r}" ]; then
        read -r _ tag baud nbytes nw nl <<< "${r}"
        case "${tag}" in
            WHEELTEC)  add_summary "${t}: ★ 輪趣格式（${baud} baud，${nw} 包）— ${TTY_DESC[${t}]}" ;;
            LD06/LD19) add_summary "${t}: ★ LD06/LD19 光達格式（${baud} baud）— ${TTY_DESC[${t}]}" ;;
            SILENT)    add_summary "${t}: 4 種鮑率都沒收到資料（RPLidar 類？或板子沒上電）— ${TTY_DESC[${t}]}" ;;
            *)         add_summary "${t}: 有資料但不認得（最多 ${nbytes} bytes @ ${baud}），看第 4 節 hex — ${TTY_DESC[${t}]}" ;;
        esac
    else
        add_summary "${t}: 無法試讀（見第 4 節）— ${TTY_DESC[${t}]}"
    fi
done

# --------------------------------------------------------------------------
sec "5. 手把 /dev/input/js*"
# --------------------------------------------------------------------------
JS_PY="$(cat <<'PYEOF'
import fcntl, os, struct, sys, array
dev = sys.argv[1]
try:
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
except OSError as e:
    print('  %s 開不起來：%s' % (dev, e))
    sys.exit(0)
def ioc(nr, size):
    # _IOR('j', nr, size)
    return (2 << 30) | (size << 16) | (ord('j') << 8) | nr
buf = array.array('B', [0])
fcntl.ioctl(fd, ioc(0x11, 1), buf); axes = buf[0]
fcntl.ioctl(fd, ioc(0x12, 1), buf); buttons = buf[0]
name = array.array('B', [0] * 128)
fcntl.ioctl(fd, ioc(0x13, 128), name)
os.close(fd)
n = bytes(name).split(b'\0')[0].decode(errors='replace')
print('  %s: %s  軸 %d 個、按鍵 %d 個' % (dev, n, axes, buttons))
print('JS %s|%d|%d' % (n, axes, buttons))
PYEOF
)"
JS_FOUND=0
for j in /dev/input/js*; do
    [ -e "${j}" ] || continue
    JS_FOUND=1
    res="$(python3 -c "${JS_PY}" "${j}" 2>&1)"
    echo "${res}" | grep -v '^JS '
    r="$(echo "${res}" | grep '^JS ' | cut -c4-)"
    if [ -n "${r}" ]; then
        IFS='|' read -r jn ja jb <<< "${r}"
        add_summary "手把 ${j}: ${jn}（軸 ${ja}、鍵 ${jb}）"
    fi
done
if [ "${JS_FOUND}" = 0 ]; then
    echo "沒有 /dev/input/js*（手把沒插、或接收器沒配對）"
    add_summary "手把：無"
fi

# --------------------------------------------------------------------------
sec "6. 網路"
# --------------------------------------------------------------------------
ip -brief addr 2>/dev/null || ip addr
echo "預設路由: $(ip route show default 2>/dev/null | head -1)"
add_summary "IP: $(ip -brief -4 addr 2>/dev/null | awk '$1!="lo"{print $1"="$3}' | tr '\n' ' ')"

# --------------------------------------------------------------------------
sec "7. ROS 2 與 workspace"
# --------------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2   : Humble（/opt/ros/humble）"
    dpkg-query -W -f='  ${Package} ${Version}\n' ros-humble-desktop ros-humble-ros-base \
        ros-humble-navigation2 ros-humble-slam-toolbox ros-humble-fields2cover 2>/dev/null
    ROS_OK="Humble 已安裝"
else
    echo "ROS 2   : 沒有 /opt/ros/humble"
    ls -d /opt/ros/* 2>/dev/null | sed 's/^/  其他版本: /'
    ROS_OK="沒有 ROS 2 Humble"
fi
echo "workspace: ${WS}"
if [ -d "${WS}/.git" ]; then
    echo "  git    : $(git -C "${WS}" log -1 --format='%h %ad %<(40,trunc)%s' --date=short 2>/dev/null)"
    dirty="$(git -C "${WS}" status --porcelain 2>/dev/null | wc -l)"
    echo "  未 commit 的變更: ${dirty} 個檔案"
fi
if [ -f "${WS}/install/setup.bash" ]; then
    n_pkg=$(ls -d "${WS}"/install/mowerbot_* 2>/dev/null | wc -l)
    echo "  install/ 存在，mowerbot 套件 ${n_pkg} 個：$(ls -d "${WS}"/install/mowerbot_* 2>/dev/null | xargs -n1 basename | tr '\n' ' ')"
    WS_OK="已編譯（${n_pkg} 個 mowerbot 套件）"
else
    echo "  沒有 install/setup.bash → 還沒 colcon build"
    WS_OK="未編譯"
fi
add_summary "ROS: ${ROS_OK}；workspace ${WS}: ${WS_OK}"
for l in /dev/mowerbot_base /dev/mowerbot_lidar; do
    if [ -e "${l}" ]; then
        echo "  ${l} -> $(readlink -f "${l}")"
    else
        echo "  ${l} 不存在（udev 規則還沒裝或裝置沒插）"
    fi
done

# --------------------------------------------------------------------------
sec "8. 摘要"
# --------------------------------------------------------------------------
[ "${DIALOUT_OK}" = 1 ] || add_summary "⚠️ 這個登入階段沒有 dialout 權限，序列埠結果可能不完整"
for s in "${SUMMARY[@]}"; do
    echo "- ${s}"
done
echo ""
echo "報告檔：${OUT}"
