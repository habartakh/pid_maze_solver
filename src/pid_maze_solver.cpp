#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct WayPoint {
  double dx;   // change in the x-coordinate of the robot's position
  double dy;   // change in the y-coordinate of the robot's position
  double dphi; // change in the orientation angle of the robot

  WayPoint(double a = 0.0, double b = 0.0, double c = 0.0)
      : dx(a), dy(b), dphi(c) {}
};

class PIDMazeSolver : public rclcpp::Node {
public:
  PIDMazeSolver() : Node("pid_maze_solver") {

    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom_callback_group_;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&PIDMazeSolver::odom_callback, this, _1), options1);

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    twist_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    twist_timer = this->create_wall_timer(
        100ms, std::bind(&PIDMazeSolver::control_loop, this),
        timer_callback_group_);

    waypoints_traj_init();

    RCLCPP_INFO(this->get_logger(), "Initialized PID Maze Solver node");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Compute distance travelled in X and Y axis
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    dx = current_x - old_x;
    dy = current_y - old_y;

    distance_travelled_x += dx; // positive if x > 0
    distance_travelled_y += dy; // negative if y < 0

    RCLCPP_INFO(this->get_logger(), "distance_travelled_x = %f ",
                distance_travelled_x);
    RCLCPP_INFO(this->get_logger(), "distance_travelled_y = %f ",
                distance_travelled_y);

    old_x = current_x;
    old_y = current_y;

    // Then, compute robot current yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);
    RCLCPP_INFO(this->get_logger(),
                "Received Odometry - current_yaw: %f radians", current_yaw);
  }

  // Add all the waypoints the robot is going throughout the maze
  void waypoints_traj_init() {

    waypoints_traj.push_back(WayPoint(+0.244, +0.000, +0.000)); // 1
    waypoints_traj.push_back(WayPoint(+0.266, -0.134, -0.785)); // -PI/4
    waypoints_traj.push_back(WayPoint(+0.000, -1.256, -0.785)); // 3
    waypoints_traj.push_back(WayPoint(+0.303, +0.000, +1.571)); // 4
    waypoints_traj.push_back(WayPoint(+0.000, +0.467, +1.571)); // 5

    waypoints_traj.push_back(WayPoint(+0.421, +0.000, +0.000)); // 6
    waypoints_traj.push_back(WayPoint(+0.000, +0.615, +0.000)); // 7
    waypoints_traj.push_back(WayPoint(+0.497, +0.000, +0.000)); // 8
    waypoints_traj.push_back(WayPoint(+0.000, +0.802, +0.000)); // 9

    waypoints_traj.push_back(WayPoint(-0.450, +0.000, +1.571)); // 10

    waypoints_traj.push_back(WayPoint(+0.000, -0.301, +0.000)); // 11
    waypoints_traj.push_back(WayPoint(-0.480, +0.000, +0.000)); // 12

    waypoints_traj.push_back(WayPoint(-0.433, +0.229, -0.785)); // 13
    waypoints_traj.push_back(WayPoint(-0.374, +0.000, +0.785)); // 14
    waypoints_traj.push_back(WayPoint(+0.000, +0.000, +3.141)); // Final step
  }

  // Move the robot according to the desired trajectory
  void control_loop() {}

  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_pub->publish(stop_msg);
  }

  // Variable declarations
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  // Parameters to move the robot
  geometry_msgs::msg::Twist twist_cmd;

  // Parameters used to compute the yaw
  double current_yaw = 0.0;

  // Parameters used to compute the distance travelled
  double old_x = 0.0;
  double old_y = 0.0;
  double current_x = 0.0;
  double current_y = 0.0;
  double dx = 0.0; // distance between two odom messages
  double dy = 0.0;
  double distance_travelled_x = 0.0;
  double distance_travelled_y = 0.0;

  // Waypoints the robot is passing by throughout the maze
  std::vector<WayPoint> waypoints_traj;
  long unsigned int traj_index = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<PIDMazeSolver> pid_maze_solver =
      std::make_shared<PIDMazeSolver>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pid_maze_solver);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}