#!/usr/bin/env bash
# ==========================================================================
# mowerbot 新機器一鍵安裝（階段 34）
#
# 在一台全新的 Ubuntu 22.04 上，從隨身碟把整個開發環境裝起來：
#
#   1. 檢查系統版本（不是 Ubuntu 22.04 就拒絕）
#   2. 隨身碟上的 .deb 快取（有的話先放進 apt 快取，少下載）
#   3. 基本工具（curl / git / gnupg ...）
#   4. ROS 2 的 apt 來源（優先用隨身碟上的 ros2-apt-source .deb）
#   5. ROS 2 Humble desktop + colcon + rosdep
#   6. 專案執行時需要、但 package.xml 沒有宣告的套件
#   7. rosdep init / update
#   8. 從隨身碟的 git bundle 還原 workspace（預設 ~/mowerbot）
#   9. rosdep install（並 apt-mark hold 鎖住 ros-humble-fields2cover）
#  10. colcon build
#  11. 把 source 指令寫進 ~/.bashrc
#  12. 使用者加入 dialout 群組（序列埠權限）
#  13. 印出「下一步做什麼」
#
# 用法（在隨身碟的根目錄）：
#   bash setup_new_machine.sh
# 一定要用 bash 執行，不要 ./setup_new_machine.sh：Ubuntu 自動掛載 FAT / exFAT 隨身碟時
# 檔案沒有執行權限（udisks 的 showexec 只給 .exe/.com/.bat），./ 會 Permission denied（階段 35 實測）。
#
# 可以用環境變數改位置：
#   MOWERBOT_WS=~/mowerbot            workspace 要放哪裡
#   MOWERBOT_BUNDLE=<路徑>/mowerbot.bundle
#
# 可以重複執行：已經做完的步驟會印「已完成，跳過」，不會重裝。
# 任何一步失敗都會停下來，印出是哪一步、哪一行指令失敗，不會靜默繼續。
# 完整輸出同時寫進 ~/mowerbot_setup_<時間>.log。
#
# 需要網路：apt 與 rosdep update 一定要連網（隨身碟的 .deb 快取只能減少下載量）。
# 需要 sudo：開頭會問一次密碼，之後腳本自己維持 sudo 不過期。
# ==========================================================================
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${MOWERBOT_WS:-${HOME}/mowerbot}"
BUNDLE="${MOWERBOT_BUNDLE:-${SCRIPT_DIR}/mowerbot.bundle}"
DEB_CACHE_DIR="${SCRIPT_DIR}/debs"
ROS_APT_SOURCE_DEB="${SCRIPT_DIR}/ros2-apt-source_1.3.0.jammy_all.deb"
ROS_APT_SOURCE_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb"
BASHRC_MARK="# >>> mowerbot (setup_new_machine.sh) >>>"
BASHRC_MARK_END="# <<< mowerbot (setup_new_machine.sh) <<<"

export DEBIAN_FRONTEND=noninteractive
APT_GET=(sudo DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Use-Pty=0)

# ROS 2 Humble desktop + colcon + rosdep（ros-dev-tools 內含 colcon 與 rosdep）
ROS_PKGS=(ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep)

# 專案執行時會用到、但 package.xml 沒有宣告的套件（rosdep 不會幫忙裝）。
# 清單與 docs/hardware_bringup.md M0 第 2 步相同，另加 run_demo.sh 需要的 Gazebo。
#   navigation2 / nav2-bringup : navigation.launch.py 的 controller_server、lifecycle_manager、DWB
#   xacro                      : URDF 解析
#   gazebo-ros-pkgs            : run_demo.sh（模擬）
#   python3-serial             : 序列埠（驅動板、deploy/hw_probe.sh）
EXTRA_PKGS=(ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
            ros-humble-robot-state-publisher ros-humble-joint-state-publisher
            ros-humble-joy ros-humble-xacro ros-humble-tf2-tools
            ros-humble-gazebo-ros-pkgs
            python3-serial python3-opencv python3-pyqt5)

BASE_PKGS=(curl git gnupg ca-certificates lsb-release software-properties-common)

# 覆蓋率基準線（simulation_results.md 35 節）量的 Fields2Cover 版本。rosdep 裝完後 apt-mark hold 鎖住。
F2C_EXPECTED="2.1.0"

# --------------------------------------------------------------------------
# 輸出、計時、失敗處理
# --------------------------------------------------------------------------
LOG="${HOME}/mowerbot_setup_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "${LOG}") 2>&1

T_START=$(date +%s)
STEP_NO=0
STEP_TOTAL=13
CURRENT_STEP="(尚未開始)"
STEP_T0=${T_START}
SUDO_KEEPALIVE_PID=""

step() {
    STEP_NO=$((STEP_NO + 1))
    CURRENT_STEP="$1"
    STEP_T0=$(date +%s)
    echo ""
    echo "=================================================================="
    echo " [${STEP_NO}/${STEP_TOTAL}] $1"
    echo "=================================================================="
}

step_done() {
    echo "  ✔ 完成（本步 $(( $(date +%s) - STEP_T0 )) 秒）"
}

skip() {
    echo "  → 已完成，跳過：$*"
}

fail() {
    echo "" >&2
    echo "❌ 安裝中止。失敗的步驟：[${STEP_NO}/${STEP_TOTAL}] ${CURRENT_STEP}" >&2
    echo "   原因：$*" >&2
    echo "   完整輸出：${LOG}" >&2
    echo "   修好原因之後直接重跑這支腳本即可，已完成的步驟會自動跳過。" >&2
    exit 1
}

on_err() {
    local rc=$1 line=$2 cmd=$3
    fail "第 ${line} 行的指令失敗（回傳 ${rc}）：${cmd}"
}
trap 'on_err $? ${LINENO} "${BASH_COMMAND}"' ERR

cleanup() {
    if [ -n "${SUDO_KEEPALIVE_PID}" ]; then
        kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

pkgs_missing() {
    # 印出還沒裝的套件名稱（dpkg 狀態不是 "install ok installed" 的）
    local p
    for p in "$@"; do
        if [ "$(dpkg-query -W -f='${Status}' "${p}" 2>/dev/null)" != "install ok installed" ]; then
            echo "${p}"
        fi
    done
}

source_ros() {
    # ROS 的 setup.bash 會讀沒有定義的變數，在 set -u 底下 source 會中止
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
}

echo "=================================================================="
echo " mowerbot 新機器安裝   $(date '+%Y-%m-%d %H:%M:%S')"
echo "   隨身碟目錄：${SCRIPT_DIR}"
echo "   workspace ：${WS}"
echo "   log       ：${LOG}"
echo "=================================================================="

# --------------------------------------------------------------------------
step "檢查系統版本與執行身分"
# --------------------------------------------------------------------------
if [ "${EUID}" -eq 0 ]; then
    fail "請用一般使用者身分執行，不要用 sudo 或 root。" \
         "（~/.bashrc 與 dialout 群組要設在實際登入的那個帳號上；需要權限的地方腳本會自己叫 sudo。）"
fi
if [ ! -r /etc/os-release ]; then
    fail "讀不到 /etc/os-release，無法確認系統版本。"
fi
# shellcheck disable=SC1091
OS_ID="$(. /etc/os-release && echo "${ID:-}")"
OS_VER="$(. /etc/os-release && echo "${VERSION_ID:-}")"
OS_NAME="$(. /etc/os-release && echo "${PRETTY_NAME:-unknown}")"
echo "  系統：${OS_NAME}"
if [ "${OS_ID}" != "ubuntu" ] || [ "${OS_VER}" != "22.04" ]; then
    fail "這台是「${OS_NAME}」，不是 Ubuntu 22.04，拒絕安裝。" \
         "理由：ROS 2 Humble 的官方二進位套件只發布給 Ubuntu 22.04（jammy）；" \
         "20.04 沒有 Humble 套件，24.04 對應的是 ROS 2 Jazzy。" \
         "這個 workspace、Nav2 / slam_toolbox 的參數與 51 項自動化測試全部是在 22.04 + Humble 上驗證的，" \
         "換版本等於換一套沒驗證過的軟體。請重灌 Ubuntu 22.04 LTS。"
fi
echo "  使用者：${USER}"
echo "  需要 sudo 權限（裝套件、rosdep init、加 dialout 群組），請輸入密碼："
sudo -v || fail "sudo 驗證失敗。這個帳號需要 sudo 權限。"
# 裝 ROS 可能超過 sudo 預設的 15 分鐘有效期，背景每 60 秒續一次，避免中途又要密碼
( while true; do sudo -n -v 2>/dev/null || exit 0; sleep 60; kill -0 $$ 2>/dev/null || exit 0; done ) &
SUDO_KEEPALIVE_PID=$!
step_done

# --------------------------------------------------------------------------
step "隨身碟上的 .deb 快取"
# --------------------------------------------------------------------------
if compgen -G "${DEB_CACHE_DIR}/*.deb" > /dev/null; then
    N_ALL=$(find "${DEB_CACHE_DIR}" -maxdepth 1 -name '*.deb' | wc -l)
    SZ=$(du -sh "${DEB_CACHE_DIR}" | cut -f1)
    echo "  隨身碟上有 ${N_ALL} 個 .deb（${SZ}），複製進 /var/cache/apt/archives/（已存在的不覆蓋）"
    echo "  版本跟 apt 索引完全一致的才會被採用，不一致的 apt 會自己上網下載新版"
    sudo cp -n "${DEB_CACHE_DIR}"/*.deb /var/cache/apt/archives/ \
        || fail "複製 .deb 快取失敗（磁碟空間不夠？ df -h /var）"
    # 隨身碟自動掛載（FAT / exFAT）時檔案通常是 0600、屬於使用者；cp 會把 0600 帶過去，
    # 變成 root 擁有、只有 root 能讀。apt 抓檔時會降權成 _apt 使用者，讀不到就會
    # 「File not found ... (13: Permission denied)」（階段 35 在開發機上遇到同一個錯）。
    # 所以複製過去的一律改成 644，不管隨身碟用什麼權限掛載。
    sudo find /var/cache/apt/archives -maxdepth 1 -name '*.deb' ! -perm -004 -exec chmod 644 {} + \
        || fail "調整 /var/cache/apt/archives 裡 .deb 的權限失敗"
else
    skip "隨身碟上沒有 debs/ 資料夾，全部從網路下載"
fi
step_done

# --------------------------------------------------------------------------
step "基本工具（curl / git / gnupg ...）"
# --------------------------------------------------------------------------
mapfile -t MISSING < <(pkgs_missing "${BASE_PKGS[@]}")
if [ "${#MISSING[@]}" -eq 0 ]; then
    skip "${BASE_PKGS[*]} 都已安裝"
else
    echo "  要安裝：${MISSING[*]}"
    "${APT_GET[@]}" update || fail "apt-get update 失敗。檢查網路（ping 8.8.8.8）與 /etc/apt/sources.list。"
    "${APT_GET[@]}" install "${MISSING[@]}" || fail "安裝基本工具失敗：${MISSING[*]}"
fi
if grep -rhqE '^[^#].*\buniverse\b|^Components:.*\buniverse\b' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
    skip "apt 的 universe 來源已啟用"
else
    echo "  啟用 universe 來源（ROS 2 需要）"
    sudo add-apt-repository -y universe || fail "add-apt-repository universe 失敗"
fi
step_done

# --------------------------------------------------------------------------
step "ROS 2 的 apt 來源"
# --------------------------------------------------------------------------
if grep -rlqE 'packages\.ros\.org/ros2|/ros2/ubuntu' /etc/apt/sources.list.d/ 2>/dev/null; then
    skip "已經有 ROS 2 的 apt 來源：$(grep -rlE 'packages\.ros\.org/ros2|/ros2/ubuntu' /etc/apt/sources.list.d/ | tr '\n' ' ')"
else
    if [ -f "${ROS_APT_SOURCE_DEB}" ]; then
        echo "  使用隨身碟上的 $(basename "${ROS_APT_SOURCE_DEB}")"
        DEB="${ROS_APT_SOURCE_DEB}"
    else
        echo "  隨身碟上沒有 ros2-apt-source .deb，從 GitHub 下載"
        DEB="$(mktemp --suffix=.deb)"
        curl -fsSL -o "${DEB}" "${ROS_APT_SOURCE_URL}" \
            || fail "下載 ${ROS_APT_SOURCE_URL} 失敗。檢查網路，或把這個檔案放到隨身碟根目錄。"
    fi
    sudo dpkg -i "${DEB}" || fail "安裝 ros2-apt-source 失敗（sudo dpkg -i ${DEB}）"
fi
echo "  apt-get update（下載套件索引）"
"${APT_GET[@]}" update || fail "apt-get update 失敗。檢查網路，或 ROS 來源的 GPG 金鑰是否過期（訊息裡有 NO_PUBKEY / EXPKEYSIG）。"
step_done

# --------------------------------------------------------------------------
step "ROS 2 Humble desktop + colcon + rosdep"
# --------------------------------------------------------------------------
mapfile -t MISSING < <(pkgs_missing "${ROS_PKGS[@]}")
if [ "${#MISSING[@]}" -eq 0 ]; then
    skip "${ROS_PKGS[*]} 都已安裝"
else
    echo "  要安裝：${MISSING[*]}（第一次安裝約 2~3 GB 下載，視網路速度要數十分鐘）"
    "${APT_GET[@]}" install "${MISSING[@]}" \
        || fail "安裝 ROS 2 Humble 失敗。常見原因：網路中斷（重跑即可續傳）、磁碟空間不足（df -h /）。"
fi
[ -f /opt/ros/humble/setup.bash ] || fail "裝完之後找不到 /opt/ros/humble/setup.bash"
step_done

# --------------------------------------------------------------------------
step "專案需要但 package.xml 沒有宣告的套件"
# --------------------------------------------------------------------------
mapfile -t MISSING < <(pkgs_missing "${EXTRA_PKGS[@]}")
if [ "${#MISSING[@]}" -eq 0 ]; then
    skip "都已安裝"
else
    echo "  要安裝：${MISSING[*]}"
    "${APT_GET[@]}" install "${MISSING[@]}" || fail "安裝失敗：${MISSING[*]}"
fi
step_done

# --------------------------------------------------------------------------
step "rosdep init / update"
# --------------------------------------------------------------------------
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    skip "rosdep 已 init"
else
    sudo rosdep init || fail "sudo rosdep init 失敗（要連到 raw.githubusercontent.com）"
fi
if [ -d "${HOME}/.ros/rosdep/sources.cache" ]; then
    skip "rosdep 已 update（${HOME}/.ros/rosdep/sources.cache 存在）"
else
    rosdep update --rosdistro humble \
        || fail "rosdep update 失敗（要連到 raw.githubusercontent.com，實驗室網路擋 GitHub 的話會在這裡失敗）"
fi
step_done

# --------------------------------------------------------------------------
step "從 git bundle 還原 workspace"
# --------------------------------------------------------------------------
if [ -d "${WS}/.git" ]; then
    skip "${WS} 已經是 git repo，不動它（避免蓋掉現場的修改）"
    echo "    workspace 目前：$(git -C "${WS}" log -1 --format='%h %<(60,trunc)%s' 2>/dev/null)"
    if [ -f "${BUNDLE}" ]; then
        BUNDLE_HEAD="$(git bundle list-heads "${BUNDLE}" | awk '$2=="HEAD"{print $1}')"
        WS_HEAD="$(git -C "${WS}" rev-parse HEAD)"
        if [ -n "${BUNDLE_HEAD}" ] && [ "${BUNDLE_HEAD}" != "${WS_HEAD}" ]; then
            echo "    ⚠️ 隨身碟 bundle 的 HEAD 是 ${BUNDLE_HEAD:0:7}，跟 workspace（${WS_HEAD:0:7}）不同。"
            echo "       要更新的話手動：git -C ${WS} pull ${BUNDLE} main"
        fi
    fi
elif [ -e "${WS}" ]; then
    fail "${WS} 已存在但不是 git repo。請確認裡面的東西，搬走或改用 MOWERBOT_WS=<別的路徑> 重跑。"
else
    [ -f "${BUNDLE}" ] || fail "找不到 git bundle：${BUNDLE}（應該跟這支腳本放在同一個資料夾）"
    # 不用 git bundle verify：Ubuntu 22.04 的 git 2.34 在 repo 外面執行會直接失敗
    # （"need a repository to verify a bundle"）。改用 sha256 驗檔案，git clone 本身也會檢查物件。
    if [ -f "${BUNDLE}.sha256" ]; then
        echo "  驗證 bundle 的 sha256：${BUNDLE}"
        (cd "$(dirname "${BUNDLE}")" && sha256sum -c "$(basename "${BUNDLE}").sha256") \
            || fail "mowerbot.bundle 的 sha256 不符，隨身碟上的檔案損毀。重新用 deploy/make_usb.sh 做一次隨身碟。"
    else
        echo "  ⚠️ 沒有 ${BUNDLE}.sha256，跳過檔案校驗（git clone 仍會檢查物件完整性）"
    fi
    git clone "${BUNDLE}" "${WS}" || fail "git clone ${BUNDLE} 失敗（bundle 損毀或不完整）"
    echo "    還原為：$(git -C "${WS}" log -1 --format='%h %<(60,trunc)%s')"
fi
step_done

# --------------------------------------------------------------------------
step "rosdep install（package.xml 宣告的相依套件）"
# --------------------------------------------------------------------------
source_ros
# 不加 -r：任何一個 key 解析或安裝失敗就停下來，不要安靜地少裝東西
(cd "${WS}" && rosdep install --from-paths src --ignore-src -y --rosdistro humble) \
    || fail "rosdep install 失敗。上面的訊息會寫是哪一個 key 找不到或哪個套件裝不起來。"
# F2C 鎖版本（階段 35）：覆蓋率基準線是用 2.1.0 量的，之後 apt upgrade 不能把它悄悄升級
F2C_VER="$(dpkg-query -W -f='${Version}' ros-humble-fields2cover 2>/dev/null)" \
    || fail "rosdep install 之後找不到 ros-humble-fields2cover"
echo "  Fields2Cover：ros-humble-fields2cover ${F2C_VER}"
if apt-mark showhold | grep -qx ros-humble-fields2cover; then
    skip "ros-humble-fields2cover 已經 apt-mark hold"
else
    sudo apt-mark hold ros-humble-fields2cover || fail "apt-mark hold ros-humble-fields2cover 失敗"
fi
case "${F2C_VER}" in
    "${F2C_EXPECTED}"*) ;;
    *)  echo "  ⚠️⚠️ 裝到的 Fields2Cover 是 ${F2C_VER}，不是基準線用的 ${F2C_EXPECTED}。"
        echo "       規劃結果可能跟 simulation_results.md 35 節的基準線不同；已照樣鎖住這個版本。" ;;
esac
step_done

# --------------------------------------------------------------------------
step "colcon build"
# --------------------------------------------------------------------------
echo "  colcon 是增量編譯：已經編好、原始碼沒變的套件不會重編"
# 重新 source：上一步 rosdep 可能剛裝了 vendor 套件（例如 fields2cover 需要的 ortools_vendor），
# 它們的 lib 路徑要重新 source 才會進環境變數。沒有這一行，第一次安裝時
# mowerbot_planner 會連結失敗（libortools.so.9 not found），重跑才會好。
source_ros
(cd "${WS}" && colcon build --symlink-install) \
    || fail "colcon build 失敗。往上找第一個 'Failed' 或 'error:' 的套件；log 在 ${WS}/log/latest_build/"
[ -f "${WS}/install/setup.bash" ] || fail "編譯完找不到 ${WS}/install/setup.bash"
step_done

# --------------------------------------------------------------------------
step "把 source 指令寫進 ~/.bashrc"
# --------------------------------------------------------------------------
if grep -qF "${BASHRC_MARK}" "${HOME}/.bashrc" 2>/dev/null; then
    skip "~/.bashrc 已經有 mowerbot 區塊"
else
    {
        echo ""
        echo "${BASHRC_MARK}"
        echo "source /opt/ros/humble/setup.bash"
        echo "source ${WS}/install/setup.bash"
        echo "${BASHRC_MARK_END}"
    } >> "${HOME}/.bashrc"
    echo "  已加入 ~/.bashrc："
    echo "    source /opt/ros/humble/setup.bash"
    echo "    source ${WS}/install/setup.bash"
fi
step_done

# --------------------------------------------------------------------------
step "使用者加入 dialout 群組（序列埠權限）"
# --------------------------------------------------------------------------
NEED_RELOGIN=0
if id -nG "${USER}" | tr ' ' '\n' | grep -qx dialout; then
    skip "${USER} 已經在 dialout 群組"
    if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
        NEED_RELOGIN=1
    fi
else
    sudo usermod -aG dialout "${USER}" || fail "sudo usermod -aG dialout ${USER} 失敗"
    echo "  已加入 dialout。要登出再登入才會生效。"
    NEED_RELOGIN=1
fi
step_done

# --------------------------------------------------------------------------
step "完成，下一步做什麼"
# --------------------------------------------------------------------------
T_TOTAL=$(( $(date +%s) - T_START ))
echo ""
echo "  ✅ 安裝完成，總共 $((T_TOTAL / 60)) 分 $((T_TOTAL % 60)) 秒。log：${LOG}"
echo ""
if [ "${NEED_RELOGIN}" = 1 ]; then
    echo "  1. 【先登出再登入】（或重開機），dialout 群組才會生效，否則開序列埠會 Permission denied。"
else
    echo "  1. （dialout 已生效，不用重新登入）"
fi
echo "  2. 開一個新的終端機（讓 ~/.bashrc 生效），先跑模擬確認環境正常："
echo "       cd ${WS} && ./run_demo.sh"
echo "     看到 RViz 與 HMI 視窗、地圖開始長出來就是正常。Ctrl+C 結束。"
echo "  3. 接上車子的 USB 之後，跑現場診斷（只讀不寫，不會送任何馬達指令）："
echo "       bash ${WS}/deploy/hw_probe.sh"
echo "     產生 hw_probe_<時間>.txt，整份拍照或傳回來。"
echo "  4. 量測表單：${WS}/docs/field_checklist.md（印出來帶去）"
echo "  5. 實機上線步驟：${WS}/docs/hardware_bringup.md（M0 已由這支腳本完成，從 M1 開始）"
