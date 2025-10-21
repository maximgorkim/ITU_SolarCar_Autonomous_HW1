#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("twist_demo");
  auto pub  = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  auto timer = node->create_wall_timer(200ms, [pub](){
    geometry_msgs::msg::Twist t; t.linear.x = 1.0; t.angular.z = 0.5; // yavaş ileri + dön
    pub->publish(t);
  });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
