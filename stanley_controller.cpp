#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>

using std::placeholders::_1;

// Clamps angle to [-pi, pi]
static double wrap_to_pi(double a) {
while (a > M_PI) a -= 2 * M_PI;
while (a < -M_PI) a += 2 * M_PI;
return a;
}

// Finds curvature given three points (x0,y0), (x1,y1), (x2,y2)
static double getCurvature(double x0, double y0, double x1, double y1, double x2, double y2) {
    // Finds the radius of the circle passing through points P0, P1, P2
    // Radius R = abc / (2 * Area)

    // Lengths of sides of triangle
    double a_sq = std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
    double b_sq = std::pow(x0 - x2, 2) + std::pow(y0 - y2, 2);
    double c_sq = std::pow(x0 - x1, 2) + std::pow(y0 - y1, 2);

    double a = std::sqrt(a_sq);
    double b = std::sqrt(b_sq);
    double c = std::sqrt(c_sq);

    // Calculate area using Heron's formula
    double s = (a + b + c) / 2.0;
    double area_sq = s * (s - a) * (s - b) * (s - c);
    double area = std::sqrt(std::max(0.0, area_sq));

    // Curvature
    // kappa = 4 * Area / (a * b * c)
    double kappa = 0.0;
    if (area > 1e-6) {
        kappa = (4.0 * area) / (a * b * c);
    }

    // Determine the sign of curvature using cross product
    double cross_product = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1);
    
    return (cross_product >= 0.0) ? kappa : -kappa;
}


class StanleyController : public rclcpp::Node {
public:
StanleyController() : Node("stanley_controller") {
// Parameters
this->declare_parameter("k", 1.8);
this->declare_parameter("v_max", 0.5);
this->declare_parameter("v_min", 0.001);
this->declare_parameter("k_omega_gain", 1.2);
// NEW FEEDFORWARD PARAMETERS
this->declare_parameter("L_WHEELBASE", 0.1);    // Turtlesim wheelbase length
this->declare_parameter("L_LOOKAHEAD", 0.4);    // Lookahead distance (L_ff)
this->declare_parameter("k_ff", 1.0);           // Curvature feedforward gain

this->get_parameter("k", k_);
this->get_parameter("v_max", v_max_);
this->get_parameter("v_min", v_min_);
this->get_parameter("k_omega_gain", k_omega_gain_);
this->get_parameter("L_WHEELBASE", L_WHEELBASE_);
this->get_parameter("L_LOOKAHEAD", L_LOOKAHEAD_);
this->get_parameter("k_ff", k_ff_);


// Subscriber and Publisher
path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
"plan", 10, std::bind(&StanleyController::onPath, this, _1));
pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
"/turtle1/pose", 10, std::bind(&StanleyController::onPose, this, _1));
cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);

traj_.header.frame_id = "map";

RCLCPP_INFO(this->get_logger(), "Stanley running (k=%.2f, V_max=%.2f, V_min=%.2f, k_omega_gain=%.2f).", k_, v_max_, v_min_, k_omega_gain_);
RCLCPP_INFO(this->get_logger(), "FF active (L_WHEELBASE=%.2f, L_LOOKAHEAD=%.2f, k_ff=%.2f).", L_WHEELBASE_, L_LOOKAHEAD_, k_ff_);
}

private:
void onPath(const nav_msgs::msg::Path &p) { path_ = p; }

void onPose(const turtlesim::msg::Pose &pose) {
if (path_.poses.size() < 3) return; // At least 3 points are required for curvature

double rx = pose.x;
double ry = pose.y;
double ryaw = pose.theta;

// 1. Find Nearest Path Segment
double min_dist_sq = std::numeric_limits<double>::infinity();
size_t best_i = 0; // Closest segment index (P0)
double t_nearest = 0.0; // Parameter t representing the nearest point [0, 1]
double total_error_ = 0.0; 

for (size_t i = 0; i + 1 < path_.poses.size(); ++i) {
    const auto &p0 = path_.poses[i].pose.position;
    const auto &p1 = path_.poses[i + 1].pose.position;
    double vx = rx - p0.x;
    double vy = ry - p0.y;
    double ux = p1.x - p0.x;
    double uy = p1.y - p0.y;
    double len_sq = ux * ux + uy * uy;
    double p = 0.0;
    if (len_sq > 1e-6) {
        p = (vx * ux + vy * uy) / len_sq;
    }

    double t = std::min(1.0, std::max(0.0, p));
    double nearest_x = p0.x + t * ux;
    double nearest_y = p0.y + t * uy;

    double dist_sq = std::pow(rx - nearest_x, 2) + std::pow(ry - nearest_y, 2);
    if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_i = i;
        t_nearest = t;
    }
}
// Position of the closest segment
const auto &p0 = path_.poses[best_i].pose.position;
const auto &p1 = path_.poses[best_i + 1].pose.position;
double ux = p1.x - p0.x;
double uy = p1.y - p0.y;
double seg_len = std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));

// Segment yaw angle
double seg_yaw = std::atan2(uy, ux);

// 2. Cross Track Error (e_lat) Calculation
double vx = rx - p0.x;
double vy = ry - p0.y;
double cross = ux * vy - uy * vx;
double e_lat = (cross >= 0.0) ? std::sqrt(min_dist_sq) : -std::sqrt(min_dist_sq);
// ADD: accumulate and print total lateral error
total_error_ += std::abs(e_lat);
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"Total Lateral Error: %.4f", total_error_);


// 3. Heading Error (theta_e) Calculation
double theta_e = wrap_to_pi(seg_yaw - ryaw);

// ******************************************************
// NEW 4. FEEDFORWARD (FORWARD FEED) CALCULATION STEP
// ******************************************************
double kappa_ff = 0.0;
size_t ff_idx = best_i; // Lookahead point index

// Find the segment ahead by the lookahead distance
double remaining_len = (1.0 - t_nearest) * seg_len;
double lookahead_remaining = L_LOOKAHEAD_ - remaining_len;

// Lookahead distance continues until it falls on a point ahead on the path_
while (lookahead_remaining > 1e-6 && ff_idx + 1 < path_.poses.size() - 1) {
    ff_idx++;
    const auto &p_curr = path_.poses[ff_idx].pose.position;
    const auto &p_next = path_.poses[ff_idx + 1].pose.position;
    double next_seg_len = std::sqrt(std::pow(p_next.x - p_curr.x, 2) + std::pow(p_next.y - p_curr.y, 2));
    if (lookahead_remaining > next_seg_len) {
        lookahead_remaining -= next_seg_len;
    } else {
        // Lookahead point falls within this segment
        // For now, we will just use the start of the segment (ff_idx)
        break;
    }
}

// Calculate curvature around ff_idx point
// Use ff_idx, ff_idx+1, ff_idx+2 points
if (ff_idx + 2 < path_.poses.size()) {
    const auto &P0 = path_.poses[ff_idx].pose.position;
    const auto &P1 = path_.poses[ff_idx + 1].pose.position;
    const auto &P2 = path_.poses[ff_idx + 2].pose.position;
    
    // Calculate curvature using three points
    // kappa = 1 / R
    // where R is the radius of the circle passing through P0, P1, P2
    // Implement getCurvature function to compute this

    kappa_ff = getCurvature(P0.x, P0.y, P1.x, P1.y, P2.x, P2.y);
    
    // Limit maximum curvature to avoid extreme steering
    // For turtlesim, we can set a reasonable max curvature
    const double MAX_KAPPA = 50.0; 
    kappa_ff = std::min(MAX_KAPPA, std::max(-MAX_KAPPA, kappa_ff));
}


// 5. Stanley Controller Mechanism with Feedforward Term
double eps = 1e-3;

// Feedback term: lateral error and heading error
double delta_feedback = theta_e + std::atan2(-k_ * e_lat, v_max_ + eps);

// Feedforward term: based on kappa
// Kinematic model: tan(delta_ff) = L * kappa_ff
double delta_feedforward = std::atan(L_WHEELBASE_ * kappa_ff * k_ff_); // k_ff is a tuning gain

double delta = delta_feedback + delta_feedforward;

// Maximum steering angle limitation (e.g. 60 degrees)
const double delta_limit = M_PI / 3.0;
delta = std::min(delta_limit, std::max(-delta_limit, delta));


// 6. Adaptive Linear Velocity (V_cmd) Calculation (Speed Reduction LOGIC MAINTAINED)
// Normalized steering intensity (0: straight, 1: max turn)
double delta_normalized = std::abs(delta) / delta_limit;
// Speed reduction proportional to steering intensity
double speed_reduction = (v_max_ - v_min_) * delta_normalized;
double v_cmd = v_max_ - speed_reduction;
// Keep speed between V_min and V_max
v_cmd = std::min(v_max_, std::max(v_min_, v_cmd));


// 7. Turtlesim Commands Generation
geometry_msgs::msg::Twist cmd;

cmd.linear.x = v_cmd; // Adaptive Linear Velocity
// Angular Velocity (Omega)
cmd.angular.z = k_omega_gain_ * delta;
// Limit Angular Velocity
double max_omega = 2.8;
cmd.angular.z = std::min(max_omega, std::max(-max_omega, cmd.angular.z));

cmd_pub_->publish(cmd);
// 8. Trajectory Publishing for Visualization
geometry_msgs::msg::PoseStamped ps;
ps.header.frame_id = "map";
ps.header.stamp = this->now();
ps.pose.position.x = rx;
ps.pose.position.y = ry;

traj_.poses.push_back(ps);
traj_.header.stamp = ps.header.stamp;
traj_pub_->publish(traj_);

// Debug Log
RCLCPP_DEBUG(this->get_logger(), "e_lat: %.3f, theta_e: %.3f, delta_fb: %.3f, delta_ff: %.3f, kappa_ff: %.2f, V_cmd: %.2f", 
e_lat, theta_e, delta_feedback, delta_feedforward, kappa_ff, v_cmd);
}

// ROS objects
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;

// Path storage
nav_msgs::msg::Path path_;
nav_msgs::msg::Path traj_;

// Parameters
double k_{1.8}, v_max_{0.5}, v_min_{0.01}, k_omega_gain_{1.2}; 
double L_WHEELBASE_{0.1}, L_LOOKAHEAD_{0.4}, k_ff_{1.0};
};

int main(int argc, char** argv) {
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<StanleyController>());
rclcpp::shutdown();
return 0;
}