# mowerbot 專案規則

## 回報語言

所有回報一律用繁體中文（程式碼與 log 原文除外）。

## 凍結條件（在拿到實機原地旋轉角速度之前）

在拿到實機的原地旋轉角速度量測結果之前，不得更動：

- URDF（`src/mowerbot_description/urdf/` 下所有 xacro，包含前輪摩擦與輪子位置）
- headland（`headland_width`，`mower_control.launch.py` 與 `f2c_server.cpp` 的預設值）
- 車輛幾何（`src/mowerbot_description/config/vehicle.yaml`，所有項目維持「暫定值」）
- 任何跟掉頭有關的邏輯（掉頭淨空檢查、原地旋轉、approach 的 goal checker 等）

理由：階段 30 的實驗顯示，模擬裡能不能原地掉頭完全取決於前後輪摩擦的比例
（見 `docs/simulation_results.md` 30.4 節），在沒有實機數字之前調這些只是在調猜測值。
實機量測步驟見 `docs/hardware_bringup.md`。
