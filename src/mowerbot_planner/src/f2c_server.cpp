#include <rclcpp/rclcpp.hpp>
#include <mowerbot_interfaces/srv/generate_coverage_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

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

        srv_ = this->create_service<GenerateCoveragePath>(
            "generate_coverage_path",
            std::bind(&F2CServer::handle_request, this, _1, _2)
        );
        RCLCPP_INFO(this->get_logger(),
            "🚀 F2C 割草線伺服器已啟動，地頭寬度 %.2f m，等待邊界輸入...",
            this->get_parameter("headland_width").as_double());
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

            RCLCPP_INFO(this->get_logger(),
                "🌾 地頭寬度 %.2f m：原始面積 %.2f m^2 -> 內縮後作業面積 %.2f m^2 "
                "(佔 %.1f%%)",
                headland_width, field_area, mainland_area,
                field_area > 0.0 ? 100.0 * mainland_area / field_area : 0.0);

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

            // 確保有生成成功的割草線
            if (swaths_by_cells.size() > 0) {
                // 【關鍵修正】：我們只有一塊草地，所以剝開第一層，取出真正的 Swaths 群組
                auto swaths = swaths_by_cells[0]; 
                
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
            
            const size_t n_swaths =
                (swaths_by_cells.size() > 0) ? swaths_by_cells[0].size() : 0;
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
