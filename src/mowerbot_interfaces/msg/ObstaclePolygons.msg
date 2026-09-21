# 作業區內部障礙物的輪廓集合 (階段 11 新增)。
#
# /f2c_boundary 用的 geometry_msgs/PolygonStamped 一次只能帶一個多邊形，
# 而內部障礙物可能有好幾個，所以另外定義這個型別，
# 由 map_to_boundary 發布到 /f2c_obstacles，mower_manager 訂閱後轉給 F2C。

std_msgs/Header header

# 每個多邊形是一個障礙物的輪廓，座標與 header.frame_id 一致 (map)
geometry_msgs/Polygon[] polygons
