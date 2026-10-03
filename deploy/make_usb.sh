#!/usr/bin/env bash
# ==========================================================================
# 產生隨身碟內容（階段 34）
#
# 用法：
#   deploy/make_usb.sh <隨身碟掛載路徑>            例：deploy/make_usb.sh /media/$USER/MOWERBOT
#   DEBS_FROM=<資料夾> deploy/make_usb.sh <路徑>    另外把 .deb 快取放進 debs/
#
# .deb 快取的固定位置：~/mowerbot_deb_cache/debs（開發機上，1742 個、約 1015 MB，不進版控）
#   DEBS_FROM=~/mowerbot_deb_cache/debs deploy/make_usb.sh /media/$USER/MOWERBOT
# 怎麼來的、多久該重收一次，見 ~/mowerbot_deb_cache/README.txt。
#
# 放進去的東西：
#   setup_new_machine.sh / hw_probe.sh       安裝與現場診斷（從 deploy/ 複製）
#   mowerbot.bundle (+ .sha256)              整個 git repo（git bundle --all，含完整歷史）
#   ros2-apt-source_1.3.0.jammy_all.deb      ROS 2 apt 來源與 GPG 金鑰（4.5 KB）
#   field_checklist.md / hardware_bringup.md 印出來帶去 / 現場照著做
#   debs/                                    （選用）apt 的 .deb 快取
#
# bundle 只包含已經 commit 的東西：工作目錄有未 commit 的修改時拒絕執行，
# 免得隨身碟上的版本跟你以為的不一樣。
# ==========================================================================
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-}"
ROS_APT_SOURCE_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb"

[ -n "${DEST}" ] || { echo "用法：$0 <隨身碟掛載路徑>" >&2; exit 2; }
mkdir -p "${DEST}"

if [ -n "$(git -C "${REPO}" status --porcelain)" ]; then
    echo "❌ ${REPO} 有未 commit 的修改，bundle 不會包含它們。先 commit 再做隨身碟：" >&2
    git -C "${REPO}" status --short >&2
    exit 1
fi

echo "--- git bundle（HEAD = $(git -C "${REPO}" log -1 --format='%h %<(50,trunc)%s')）"
git -C "${REPO}" bundle create "${DEST}/mowerbot.bundle" --all
git -C "${REPO}" bundle verify "${DEST}/mowerbot.bundle" > /dev/null
(cd "${DEST}" && sha256sum mowerbot.bundle > mowerbot.bundle.sha256)

echo "--- 腳本與文件"
install -m 755 "${REPO}/deploy/setup_new_machine.sh" "${DEST}/setup_new_machine.sh"
install -m 755 "${REPO}/deploy/hw_probe.sh" "${DEST}/hw_probe.sh"
install -m 644 "${REPO}/docs/field_checklist.md" "${DEST}/field_checklist.md"
install -m 644 "${REPO}/docs/hardware_bringup.md" "${DEST}/hardware_bringup.md"

if [ ! -f "${DEST}/ros2-apt-source_1.3.0.jammy_all.deb" ]; then
    echo "--- 下載 ros2-apt-source .deb"
    curl -fsSL -o "${DEST}/ros2-apt-source_1.3.0.jammy_all.deb" "${ROS_APT_SOURCE_URL}"
fi

if [ -n "${DEBS_FROM:-}" ]; then
    echo "--- .deb 快取：${DEBS_FROM} -> ${DEST}/debs/"
    mkdir -p "${DEST}/debs"
    cp -n "${DEBS_FROM}"/*.deb "${DEST}/debs/"
fi

echo ""
echo "隨身碟內容（${DEST}）："
(cd "${DEST}" && find . -maxdepth 1 -mindepth 1 -printf '%f\n' | sort | while read -r f; do
    printf '  %-40s %s\n' "${f}" "$(du -sh "${f}" | cut -f1)"
done)
echo "  合計 $(du -sh "${DEST}" | cut -f1)"
