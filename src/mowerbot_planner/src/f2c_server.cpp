#include <rclcpp/rclcpp.hpp>
#include <mowerbot_interfaces/srv/generate_coverage_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <utility>
#include <vector>

// 引入 Fields2Cover 核心函式庫
#include <fields2cover.h>

using GenerateCoveragePath = mowerbot_interfaces::srv::GenerateCoveragePath;
using std::placeholders::_1;
using std::placeholders::_2;

class F2CServer : public rclcpp::Node {
public:
    F2CServer() : Node("f2c_server") {
        // 地頭 (headland) 寬度，單位公尺。
        //
        // 為什麼需要：沒有地頭時，F2C 的第 1 條割草線距離邊界只有半個刀盤寬
        // (0.25 m)，但車體半寬是 0.34 m (輪外緣 +-0.34)。在開闊地測試看不出問題，
        // 但真實草地的邊界是牆或圍籬時，第 1 條與最後 1 條割草線會落在 costmap
        // 的膨脹層裡 (inflation_radius 0.45)，車子根本進不去。
        // 而且 mower_manager 的跑道 (lead-in) 會往割草線起點的後方延伸，
        // 也需要這塊空間，否則跑道會跑到邊界外。
        //
        // 預設 0.5 m 與 mower_manager 的 lead_in_length 預設值一致，
        // 這樣跑道剛好落在地頭裡而不會超出邊界。
        // 設 0 時停用地頭，維持加入這個功能之前的行為。
        this->declare_parameter<double>("headland_width", 0.5);

        // 周邊環繞 (perimeter pass)：在弓字形割草線之前，先沿著作業區
        // (mainland) 的邊緣繞一圈。
        //
        // 為什麼需要：階段 8 的未覆蓋面積分析發現，作業區 9.09% 的未覆蓋面積有
        // 99.5% 集中在單一區塊，位置固定在最外側那幾條割草線旁邊 —— 成因是
        // 掉頭時的外凸弧，最靠邊那條割草線的外側沒有鄰居可以補。
        // 補一圈沿邊界的環繞就是標準解法 (報告 7.1 節的後續工作)。
        //
        // 這一圈走在 mainland 再往內縮 tool_width/2 的位置，也就是
        // 距離真實邊界 headland_width + tool_width/2。用 0.5 m 地頭與
        // 0.3 m 線距算是 0.65 m，大於 costmap 的 inflation_radius (0.45 m)，
        // 車子進得去。直接貼著真實邊界繞是不行的：車體半寬 0.34 m。
        //
        // 設 false 時完全不產生這一圈，回到加入這個功能之前的行為。
        this->declare_parameter<bool>("perimeter_pass", true);

        srv_ = this->create_service<GenerateCoveragePath>(
            "generate_coverage_path",
            std::bind(&F2CServer::handle_request, this, _1, _2)
        );
        RCLCPP_INFO(this->get_logger(),
            "🚀 F2C 割草線伺服器已啟動，地頭寬度 %.2f m，周邊環繞 %s，等待邊界輸入...",
            this->get_parameter("headland_width").as_double(),
            this->get_parameter("perimeter_pass").as_bool() ? "開啟" : "關閉");
    }

private:
    rclcpp::Service<GenerateCoveragePath>::SharedPtr srv_;

    void handle_request(const std::shared_ptr<GenerateCoveragePath::Request> request,
                        std::shared_ptr<GenerateCoveragePath::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "📥 收到路徑規劃請求！刀盤寬度: %.2f m", request->tool_width);

        if (request->boundary.points.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "邊界頂點數量不足 (需大於等於3)！");
            response->success = false;
            return;
        }

        // ==========================================
        // 1. 轉換 ROS 2 多邊形至 F2C 格式
        // ==========================================
        f2c::types::LinearRing ring;
        for (const auto& p : request->boundary.points) {
            ring.addPoint(p.x, p.y);
        }
        
        auto start_p = request->boundary.points.front();
        auto end_p = request->boundary.points.back();
        if (start_p.x != end_p.x || start_p.y != end_p.y) {
            ring.addPoint(start_p.x, start_p.y);
        }

        f2c::types::Cell cell;
        cell.addRing(ring);

        // ---- 作業區內部的障礙物：當成 Cell 的內環 (hole) 挖掉 ----
        // 階段 11 之前這裡只有外環，落在外環內部的障礙物在規劃階段完全不存在，
        // F2C 的割草線會直接穿過去，Nav2 跟到那一段撞上 costmap 的膨脹層，
        // 該條割草線 ABORTED、整個任務中斷 (報告 7.10 節)。
        // 加成內環之後 F2C 的 swath generator 與 headland 都會自動避開。
        size_t n_holes = 0;
        for (const auto& obs : request->obstacles) {
            if (obs.points.size() < 3) {
                RCLCPP_WARN(this->get_logger(),
                    "⚠️ 略過一個只有 %zu 個頂點的障礙物輪廓 (至少要 3 個)。",
                    obs.points.size());
                continue;
            }
            f2c::types::LinearRing hole;
            for (const auto& p : obs.points) {
                hole.addPoint(p.x, p.y);
            }
            const auto& h0 = obs.points.front();
            const auto& hN = obs.points.back();
            if (h0.x != hN.x || h0.y != hN.y) {
                hole.addPoint(h0.x, h0.y);
            }
            cell.addRing(hole);
            ++n_holes;
        }

        f2c::types::Cells field(cell);

        try {
            // ==========================================
            // 2. 產生地頭 (headland)，在內縮後的區域上算割草線
            // ==========================================
            const double headland_width =
                this->get_parameter("headland_width").as_double();
            const double field_area = field.area();

            f2c::types::Cells mainland = field;
            if (headland_width > 0.0) {
                f2c::hg::ConstHL hl;
                // generateHeadlands 回傳的是「扣掉地頭之後的本田 (mainland)」，
                // 也就是真正要來回割草的區域。
                mainland = hl.generateHeadlands(field, headland_width);

                if (mainland.size() == 0 || mainland.area() <= 0.0) {
                    RCLCPP_ERROR(this->get_logger(),
                        "地頭寬度 %.2f m 之後已經沒有可作業面積了 "
                        "(原始面積 %.2f m^2)，請縮小地頭或擴大邊界。",
                        headland_width, field_area);
                    response->success = false;
                    return;
                }
            }
            const double mainland_area = mainland.area();

            if (n_holes > 0) {
                RCLCPP_INFO(this->get_logger(),
                    "🕳️ 內部障礙物 %zu 個已挖成內環，作業面積扣掉障礙物後 %.2f m^2。",
                    n_holes, field_area);
            }

            RCLCPP_INFO(this->get_logger(),
                "🌾 地頭寬度 %.2f m：原始面積 %.2f m^2 -> 內縮後作業面積 %.2f m^2 "
                "(佔 %.1f%%)",
                headland_width, field_area, mainland_area,
                field_area > 0.0 ? 100.0 * mainland_area / field_area : 0.0);

            // ==========================================
            // 2.5 周邊環繞 (perimeter pass) 的環
            // ==========================================
            // generateHeadlandSwaths(field, w, 1, true) 回傳的就是
            // field.buffer(-w * 0.5)，也就是距離 field 邊緣 w/2 的那一圈。
            // 這裡的 field 已經是扣掉地頭的 mainland，所以這一圈落在
            // 作業區邊緣往內 tool_width/2 的位置，與最外側那條割草線重疊，
            // 剛好補掉掉頭外凸弧留下的那一塊。
            std::vector<std::pair<double, double>> perimeter_pts;
            const bool perimeter_pass =
                this->get_parameter("perimeter_pass").as_bool();
            if (perimeter_pass && request->tool_width > 0.0) {
                f2c::hg::ConstHL hl_ring;
                auto rings = hl_ring.generateHeadlandSwaths(
                    mainland, request->tool_width, 1, true);
                if (!rings.empty() && rings[0].size() > 0) {
                    const auto ring = rings[0].getCell(0).getExteriorRing();
                    for (size_t i = 0; i < ring.size(); ++i) {
                        perimeter_pts.emplace_back(ring.getX(i), ring.getY(i));
                    }
                }
                if (perimeter_pts.size() < 4) {
                    RCLCPP_WARN(this->get_logger(),
                        "⚠️ 周邊環繞產生不出可用的環 (只有 %zu 個頂點)，"
                        "這次只送弓字形割草線。作業區可能太小。",
                        perimeter_pts.size());
                    perimeter_pts.clear();
                }
            }

            // ==========================================
            // 3. 呼叫 Swath Generator (在內縮後的 mainland 上)
            // ==========================================
            f2c::sg::BruteForce sg;
            // 回傳的型態是 SwathsByCells (多區塊的割草線集合)
            auto swaths_by_cells = sg.generateSwaths(0.0, request->tool_width, mainland);

            // ==========================================
            // 4. 手動組裝 Boustrophedon (弓字型) 導航航點
            // ==========================================
            nav_msgs::msg::Path ros_path;
            ros_path.header.stamp = this->now();
            ros_path.header.frame_id = "map"; 

            // ---- 4a. 周邊環繞的航點，放在整條路徑的最前面 ----
            // 航點間距與割草線一致 (0.1 m)，否則 N2 的平均間距判定會被拉高。
            // 每一段都不含終點 (下一段的起點就是它)，最後再補回起點把圈收起來：
            // 「最後一個航點的座標與第 0 個完全相同」就是 mower_manager
            // 用來辨識「這一段是環繞、不是割草線」的依據，不需要改 srv 介面。
            const double kPerimeterSpacing = 0.1;
            size_t n_perimeter_clipped = 0;
            for (size_t i = 0; i + 1 < perimeter_pts.size(); ++i) {
                const double x1 = perimeter_pts[i].first;
                const double y1 = perimeter_pts[i].second;
                const double x2 = perimeter_pts[i + 1].first;
                const double y2 = perimeter_pts[i + 1].second;
                const double dx = x2 - x1, dy = y2 - y1;
                const double seg_len = std::hypot(dx, dy);
                if (seg_len < 1e-9) {
                    continue;
                }
                tf2::Quaternion q;
                q.setRPY(0, 0, std::atan2(dy, dx));
                const int n_seg = std::max(
                    1, static_cast<int>(std::round(seg_len / kPerimeterSpacing)));
                for (int k = 0; k < n_seg; ++k) {
                    const double ratio = static_cast<double>(k) / static_cast<double>(n_seg);
                    const double px = x1 + dx * ratio;
                    const double py = y1 + dy * ratio;
                    // 這一圈是從 mainland 的「外環」算出來的，它不知道內部障礙物
                    // 在哪裡。障礙物靠近作業區邊緣時，環繞會直接從它旁邊掃過去 ——
                    // 實測車子中心離箱子只剩 0.23 m (車體半寬 0.34 m，已經擦到)。
                    // 所以逐個航點檢查：只留下真的落在 mainland 裡面的點
                    // (mainland 已經把障礙物挖成內環)，被擋住的那一段留一個缺口，
                    // mower_manager 會在缺口處把環繞切成兩段分別執行。
                    // 頂點層級的檢查不夠，環只有四個角點，長邊會整條漏掉。
                    if (n_holes > 0) {
                        const f2c::types::Point probe(px, py);
                        if (!mainland.isPointIn(probe) &&
                                !mainland.isPointInBorder(probe)) {
                            ++n_perimeter_clipped;
                            continue;
                        }
                    }
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = ros_path.header;
                    pose.pose.position.x = px;
                    pose.pose.position.y = py;
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                    ros_path.poses.push_back(pose);
                }
            }
            if (!ros_path.poses.empty()) {
                // 收尾航點：座標取第 0 個航點 (完全相同的 double)，
                // 朝向沿用最後一段的方向。
                geometry_msgs::msg::PoseStamped closing = ros_path.poses.back();
                closing.pose.position = ros_path.poses.front().pose.position;
                ros_path.poses.push_back(closing);
            }
            const size_t n_perimeter_poses = ros_path.poses.size();
            if (n_perimeter_clipped > 0) {
                RCLCPP_INFO(this->get_logger(),
                    "🔄 周邊環繞有 %zu 個航點被內部障礙物擋掉，那一段留缺口 "
                    "(環繞本身不會繞行障礙物，見報告 11.5 節)。",
                    n_perimeter_clipped);
            }

            // 確保有生成成功的割草線
            // 階段 11 之前這裡寫死 swaths_by_cells[0] (「我們只有一塊草地」)。
            // 挖掉內部障礙物之後，作業區有可能被切成不只一塊，
            // 只取第 0 塊會整塊草地沒割到，所以改成每一塊都走一遍。
            for (size_t c = 0; c < swaths_by_cells.size(); ++c) {
                auto swaths = swaths_by_cells[c];

                bool reverse = false;
                for (size_t i = 0; i < swaths.size(); ++i) {
                    // 這裡拿出的才是真正單一條的 f2c::types::Swath
                    auto swath = swaths[i]; 
                    
                    f2c::types::Point p1 = reverse ? swath.endPoint() : swath.startPoint();
                    f2c::types::Point p2 = reverse ? swath.startPoint() : swath.endPoint();

                    double yaw = std::atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);

                    // 【航點內插】只送起點與終點的話，相鄰航點可能相距 10 公尺以上，
                    // 任何區域控制器 (DWB / RPP) 都追不動。這裡在起點與終點之間
                    // 以 0.1 公尺為間距線性補點。
                    // 所有內插點共用同一個 orientation，也就是這條 swath 的行進方向，
                    // 不要給單位四元數，否則控制器會以為車頭要一直朝向 +x。
                    const double waypoint_spacing = 0.1;
                    const double dx = p2.getX() - p1.getX();
                    const double dy = p2.getY() - p1.getY();
                    const double swath_length = std::hypot(dx, dy);
                    const int num_segments = std::max(
                        1, static_cast<int>(std::round(swath_length / waypoint_spacing)));

                    for (int k = 0; k <= num_segments; ++k) {
                        const double ratio = static_cast<double>(k) / static_cast<double>(num_segments);

                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = ros_path.header;
                        pose.pose.position.x = p1.getX() + dx * ratio;
                        pose.pose.position.y = p1.getY() + dy * ratio;
                        pose.pose.orientation.x = q.x();
                        pose.pose.orientation.y = q.y();
                        pose.pose.orientation.z = q.z();
                        pose.pose.orientation.w = q.w();
                        ros_path.poses.push_back(pose);
                    }

                    reverse = !reverse; 
                }
            }

            response->coverage_path = ros_path;
            response->success = true;
            
            size_t n_swaths = 0;
            for (size_t c = 0; c < swaths_by_cells.size(); ++c) {
                n_swaths += swaths_by_cells[c].size();
            }
            if (n_perimeter_poses > 0) {
                double perimeter_len = 0.0;
                for (size_t i = 0; i + 1 < n_perimeter_poses; ++i) {
                    perimeter_len += std::hypot(
                        ros_path.poses[i + 1].pose.position.x - ros_path.poses[i].pose.position.x,
                        ros_path.poses[i + 1].pose.position.y - ros_path.poses[i].pose.position.y);
                }
                RCLCPP_INFO(this->get_logger(),
                    "🔄 周邊環繞：%zu 個航點，全長 %.2f m，"
                    "走在作業區邊緣往內 %.2f m 處 (距真實邊界 %.2f m)。",
                    n_perimeter_poses, perimeter_len, request->tool_width / 2.0,
                    headland_width + request->tool_width / 2.0);
            } else {
                RCLCPP_INFO(this->get_logger(), "🔄 周邊環繞：未產生 (功能關閉或作業區太小)。");
            }
            RCLCPP_INFO(this->get_logger(),
                "🎉 割草線計算成功！割草線 %zu 條，共生成 %zu 個 Nav2 關鍵航點。",
                n_swaths, ros_path.poses.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Fields2Cover 運算失敗: %s", e.what());
            response->success = false;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<F2CServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
