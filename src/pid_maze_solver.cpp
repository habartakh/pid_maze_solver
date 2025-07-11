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

// Going through the maze consists of a sequence of turn / move etc..
// The adequate PID Controller is used for each state
enum RobotState {
  MOVE, // Move along X, Y axes to reach the waypoint
  TURN, // Turn towards the next waypoint
  STOP
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

    robot_state = STOP;

    RCLCPP_INFO(this->get_logger(), "Initialized PID Maze Solver node");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Compute robot current yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);
    // RCLCPP_INFO(this->get_logger(),
    //             "Received Odometry - current_yaw: %f radians", current_yaw);

    // Compute distance travelled in X and Y axis
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
  }

  // Add all the waypoints the robot is going throughout the maze
  void waypoints_traj_init() {

    waypoints_traj.push_back(WayPoint(+0.353, +0.000, +0.000)); // 1
    waypoints_traj.push_back(WayPoint(+0.150, +0.000, -0.785)); // 2
    waypoints_traj.push_back(WayPoint(+1.590, +0.000, -0.785)); // 3
    waypoints_traj.push_back(WayPoint(+0.453, +0.000, +1.571)); // 4
    waypoints_traj.push_back(WayPoint(-0.467, +0.000, +1.571)); // 5

    // waypoints_traj.push_back(WayPoint(+0.421, +0.000, +0.000)); // 6
    // waypoints_traj.push_back(WayPoint(+0.000, +0.615, +0.000)); // 7
    // waypoints_traj.push_back(WayPoint(+0.497, +0.000, +0.000)); // 8
    // waypoints_traj.push_back(WayPoint(+0.000, +0.802, +0.000)); // 9

    // waypoints_traj.push_back(WayPoint(-0.450, +0.000, +1.571)); // 10

    // waypoints_traj.push_back(WayPoint(+0.000, -0.301, +0.000)); // 11
    // waypoints_traj.push_back(WayPoint(-0.480, +0.000, +0.000)); // 12

    // waypoints_traj.push_back(WayPoint(-0.433, +0.229, -0.785)); // 13
    // waypoints_traj.push_back(WayPoint(-0.374, +0.000, +0.785)); // 14
    // waypoints_traj.push_back(WayPoint(+0.000, +0.000, +3.141)); // Final step
  }

  // Move the robot according to the desired trajectory
  void control_loop() {

    // If the robot finished the maze, stop it then exit the node
    if (traj_index >= waypoints_traj.size()) {
      stop_robot();
      RCLCPP_INFO_ONCE(this->get_logger(), "Completed the trajectory! ");
      rclcpp::shutdown();
      return;
    }

    target = waypoints_traj[traj_index];

    if (target.dphi != 0.000) {

      if (robot_state != MOVE) {
        robot_state = TURN;
      }

    } else {
      if (target.dx != 0.000 || target.dy != 0.000) {
        robot_state = MOVE; // After turn sequence ends, start move sequence
      }
    }

    if (robot_state == TURN) {
      // RCLCPP_INFO(this->get_logger(), "ROBOT STATE == TURN ");
      turn_controller();
    }

    if (robot_state == MOVE) {
      // RCLCPP_INFO(this->get_logger(), "ROBOT STATE == MOVE ");
      distance_controller();
    }

    if (robot_state == STOP) {
      // RCLCPP_INFO(this->get_logger(), "ROBOT STATE == STOP ");
      stop_robot();
    }
  }

  void turn_controller() {

    // Compute the target yaw ONLY ONCE per Waypoint
    if (!set_target_yaw) {
      // Compute the angle left to the target
      target_yaw = normalize_angle(current_yaw + target.dphi);
      set_target_yaw = true;
    }

    // Compute the angle left to the target
    double error_yaw = target_yaw - current_yaw;

    // RCLCPP_INFO(this->get_logger(), "error_yaw : %f", error_yaw);

    // If the robot reached the target waypoint
    if (std::abs(error_yaw) < 0.02) {
      RCLCPP_INFO(this->get_logger(), "Turned towards waypoint number : %lu",
                  traj_index + 1);

      //  reset the yaw error computed to reach the waypoint
      prev_error_yaw = 0.0;
      set_target_yaw = false; // Reset target yaw for next waypoint

      if (target.dx != 0.000 || target.dy != 0.000) {
        robot_state = MOVE; // After turn sequence ends, start move sequence
      }

      else {
        traj_index++; // Go to the next waypoint
      }

      stop_robot(); // Stop the robot for 20 * 0.1 = 2 seconds
      rclcpp::sleep_for(std::chrono::seconds(2));
      return;
    }

    // Calculate delta time
    rclcpp::Time now = this->now();
    double dt = prev_time_yaw.nanoseconds() == 0
                    ? 0.05
                    : (now - prev_time_yaw).seconds();
    prev_time_yaw = now;

    // Integral terms
    integral_yaw += error_yaw * dt;

    // Derivative terms
    double derivative_yaw = (error_yaw - prev_error_yaw) / dt;

    // PID control law
    angular_speed =
        kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw;

    // Make sure the robot stays within max speed bounds
    angular_speed =
        std::clamp(angular_speed, -max_angular_speed, +max_angular_speed);

    // RCLCPP_INFO(this->get_logger(), "angular_speed = %f ", angular_speed);

    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = angular_speed;

    twist_pub->publish(twist_cmd);

    prev_error_yaw = error_yaw;
  }

  void distance_controller() {

    // Compute the yaw at the start of the movement
    if (!move_initialized) {
      yaw_start = current_yaw;
      x_start = current_x;
      y_start = current_y;
      move_initialized = true;
    }

    //! IMPORTANT Step: Transform the global dx and dy
    // into the robot local frame used for the waypoints values
    // Use yaw at the start of the movement. Otherwise, every time the robot
    // rotates slightly, the local frame moves too resulting in dx dy errors
    double goal_x =
        x_start + target.dx * cos(yaw_start) - target.dy * sin(yaw_start);
    double goal_y =
        y_start + target.dx * sin(yaw_start) + target.dy * cos(yaw_start);

    // Compute the distance left to the target
    double error_x = goal_x - current_x;
    double error_y = goal_y - current_y;

    // Important: when dx or dy = 0 during lateral movements,
    // the corresponding error still accumulates noise, resulting in
    // non null velocities that need to be corrected
    if (std::abs(target.dy) < 1e-3)
      error_y = 0.0;
    if (std::abs(target.dx) < 1e-3)
      error_x = 0.0;

    double distance = std::hypot(error_x, error_y);

    RCLCPP_INFO(this->get_logger(), "error_x is : %f", error_x);
    // RCLCPP_INFO(this->get_logger(), "error_y is : %f", error_y);
    // RCLCPP_INFO(this->get_logger(), "Distance is : %f", distance);

    // If the robot reached the target waypoint
    if (distance < 0.01) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint number : %lu",
                  traj_index + 1);

      // reset the integral controller
      integral_x = 0.0;
      integral_y = 0.0;

      move_initialized = false; // Reset for next move phase

      traj_index++;       // Go to the next waypoint
      robot_state = TURN; // Reset robot state

      // Stop the robot for 20 * 0.1 = 2 seconds
      stop_robot();
      rclcpp::sleep_for(std::chrono::seconds(2));
      return;
    }

    // Calculate delta time
    rclcpp::Time now = this->now();
    double dt = prev_time_distance.nanoseconds() == 0
                    ? 0.05
                    : (now - prev_time_distance).seconds();
    prev_time_distance = now;

    // Integral terms
    integral_x += error_x * dt;
    integral_y += error_y * dt;

    // Derivative terms
    double derivative_x = (error_x - prev_error_x) / dt;
    double derivative_y = (error_y - prev_error_y) / dt;

    // PID control law
    double vx = kp_distance * error_x + ki_distance * integral_x +
                kd_distance * derivative_x;
    double vy = kp_distance * error_y + ki_distance * integral_y +
                kd_distance * derivative_y;

    // Make sure the robot stays within max speed bounds
    vx = std::clamp(vx, -max_linear_speed, +max_linear_speed);
    vy = std::clamp(vy, -max_linear_speed, +max_linear_speed);

    // RCLCPP_INFO(this->get_logger(), "vx = %f ", vx);
    // RCLCPP_INFO(this->get_logger(), "vy = %f ", vy);

    twist_cmd.linear.x = vx;
    twist_cmd.linear.y = vy;
    twist_cmd.angular.z = 0.0;

    twist_pub->publish(twist_cmd);

    prev_error_x = error_x;
    prev_error_y = error_y;
  }

  // Send zero velocitites to stop the robot
  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_pub->publish(stop_msg);
  }

  // normalize angles to range [-pi, pi]
  double normalize_angle(double angle) {
    // RCLCPP_INFO(this->get_logger(), "Inside normalize_angle function ");
    while (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
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

  // Parameters used to compute the distance to the waypoint
  double current_x = 0.0;
  double current_y = 0.0;
  double x_start = 0.0;
  double y_start = 0.0;

  // Waypoints the robot is passing by throughout the maze
  std::vector<WayPoint> waypoints_traj;
  long unsigned int traj_index = 0;
  WayPoint target; // at each step of the trajectory

  // PID Turn Controller Parameters
  double angular_speed = 0.0;
  double kp_yaw = 2.5;        // Proportional Gain
  double ki_yaw = 0.0;        // Integral Gain
  double kd_yaw = 0.5;        // Derivative Gain
  double integral_yaw = 0.0;  // Integral terms of the PID controller
  rclcpp::Time prev_time_yaw; // instant t-1
  double prev_error_yaw = 0.0;
  double target_yaw = 0.0;
  bool set_target_yaw = false;
  double max_angular_speed =
      3.14; // source: https://husarion.com/manuals/rosbot-xl/

  // PID Distance Controller Parameters
  double kp_distance = 3.5;  // Proportional Gain
  double ki_distance = 0.05; // Integral Gain
  double kd_distance = 2.5;  // Derivative Gain
  double integral_x = 0.0;   // Integral terms of the PID controller
  double integral_y = 0.0;
  rclcpp::Time prev_time_distance; // instant t-1
  double prev_error_x = 0.0;
  double prev_error_y = 0.0;
  double max_linear_speed =
      0.8; // source: https://husarion.com/manuals/rosbot-xl/

  // Robot State Parameters
  RobotState robot_state;

  // Transform global dx and dy in local robot frame
  double yaw_start = 0.0;
  bool move_initialized = false;
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