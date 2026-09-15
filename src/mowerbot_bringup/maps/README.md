# maps/

存放建圖結果。實際的地圖檔不進版控 (見 workspace 根目錄的 .gitignore)，
因為它們是每次建圖的產物，而且是二進位檔。

一次建圖會產生兩組檔案，兩組都要存：

| 檔案                     | 產生服務                       | 用途                                   |
|--------------------------|--------------------------------|----------------------------------------|
| `mowerbot_map.pgm` + `.yaml` | `/slam_toolbox/save_map`       | 給 nav2 的 `map_server` 讀的佔據網格圖 |
| `mowerbot_map.posegraph` + `.data` | `/slam_toolbox/serialize_map` | 給 slam_toolbox 定位模式讀的位姿圖     |

## 一鍵存圖

建圖模式跑著的時候，執行：

```bash
ros2 run mowerbot_bringup save_map.sh              # 存成預設檔名 mowerbot_map
ros2 run mowerbot_bringup save_map.sh my_field     # 自訂檔名
```

或直接呼叫服務 (兩個都要做)：

```bash
# 1. 序列化位姿圖 (給 slam_toolbox 定位模式)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '<workspace>/src/mowerbot_bringup/maps/mowerbot_map'}"

# 2. 存佔據網格圖 (給 nav2 map_server)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '<workspace>/src/mowerbot_bringup/maps/mowerbot_map'}}"
```

兩個服務的檔名都**不要**帶副檔名，slam_toolbox 會自己補上。

存完之後用定位模式啟動：

```bash
ros2 launch mowerbot_bringup localization.launch.py
```
