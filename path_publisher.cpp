#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node {
public:
  PathPublisher() : Node("path_publisher") {
    // transient_local so late subscribers (RViz) still see the path
    auto qos = rclcpp::QoS(1).transient_local();
    pub_ = create_publisher<nav_msgs::msg::Path>("plan", qos);

    path_.header.frame_id = "map";

    // Square centered near turtlesim’s start (~5.5,5.5)
    const double cx = 5.5, cy = 5.5, side = 4.0;
    // Dört köşe tanımlanır
    std::vector<std::pair<double,double>> corners = {
      {cx - side/2, cy - side/2}, // Aşağı Sol (3.5, 3.5)
      {cx + side/2, cy - side/2}, // Aşağı Sağ (7.5, 3.5)
      {cx + side/2, cy + side/2}, // Yukarı Sağ (7.5, 7.5)
      {cx - side/2, cy + side/2}, // Yukarı Sol (3.5, 7.5)
      {cx - side/2, cy - side/2}  // Döngüyü kapatmak için tekrar Aşağı Sol
    };

    const int pts_per_edge = 40;  // densify edges
    for (size_t i = 0; i + 1 < corners.size(); ++i) {
      auto [x0,y0] = corners[i];
      auto [x1,y1] = corners[i+1];
      for (int k = 0; k <= pts_per_edge; ++k) {
        double t = double(k)/pts_per_edge;
        double x = x0 + (x1 - x0)*t;
        double y = y0 + (y1 - y0)*t;

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.pose.position.x = x;
        ps.pose.position.y = y;

        // Yönelim (yaw): Bir sonraki noktaya doğru sabit kalır
        double yaw = std::atan2(y1 - y0, x1 - x0);
        tf2::Quaternion q; q.setRPY(0,0,yaw);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();

        path_.poses.push_back(ps);
      }
    }

    // republish once per second (handy for RViz/late subscribers)
    timer_ = create_wall_timer(1s, [this](){
      path_.header.stamp = now();
      for (auto &ps : path_.poses) ps.header.stamp = path_.header.stamp;
      pub_->publish(path_);
    });

    RCLCPP_INFO(get_logger(), "Publishing sharp square Path on 'plan'.");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}

