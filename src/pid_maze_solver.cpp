#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h" // include the yaml library
#include <filesystem>      // Include the filesystem library

using std::placeholders::_1;
using namespace std::chrono_literals;

struct WayPoint {
  double dx;      // change in the x-coordinate of the robot's position
  double dy;      // change in the y-coordinate of the robot's position
  double dphi;    // change in the orientation angle of the robot
  bool holonomic; // the robot ativates holonomic mode or not

  WayPoint(double a = 0.0, double b = 0.0, double c = 0.0, bool holo = false)
      : dx(a), dy(b), dphi(c), holonomic(holo) {}
};

// Going through the maze consists of a sequence of turn / move etc..
// The adequate PID Controller is used for each state
enum RobotState {
  MOVE,         // Move along X, Y axes to reach the waypoint
  TURN,         // Turn towards the next waypoint
  CORRECT_TILT, // Correct the tilt generated after movement
  STOP
};

class PIDMazeSolver : public rclcpp::Node {
public:
  PIDMazeSolver(int scene_number)
      : Node("pid_maze_solver"), scene_number_(scene_number) {

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

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PIDMazeSolver::scan_callback, this, _1));

    readWaypointsYAML();

    robot_state = TURN;

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

    odom_received = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    max_range = msg->range_max;
    min_range = msg->range_min;
    front_distance = msg->ranges[0];
    right_distance = msg->ranges[579];
    left_distance = msg->ranges[179];
    back_distance = msg->ranges[359];
  }

  // Correct the trajectory of the robot when moving if it detects any obstacle
  void compute_obstacle_distances() {

    min_distance_threshold = 0.22;

    // Make sure that the distance is valid (!= inf)
    obstacle_front =
        ((front_distance < max_range && front_distance > min_range) &&
         front_distance < min_distance_threshold);
  }

  void readWaypointsYAML() {

    // Get the package's share directory and append the YAML file path
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory(
            "pid_maze_solver"); // Replace with your package name

    std::string waypoint_file_name = "";

    switch (scene_number_) {
    case 1: // Simulation
      waypoint_file_name = "waypoints_sim.yaml";
      break;

    case 2: // CyberWorld
      waypoint_file_name = "waypoints_real.yaml";
      max_linear_speed = 0.22; // Reduce speeds for real robot to avoid damage
      max_angular_speed = 0.2;
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }

    RCLCPP_INFO(this->get_logger(), "Waypoint file loaded: %s",
                waypoint_file_name.c_str());

    std::string yaml_file_path =
        package_share_directory + "/waypoints/" + waypoint_file_name;

    // Read points from the YAML file
    try {
      YAML::Node config = YAML::LoadFile(yaml_file_path);

      if (config["waypoints"]) {
        // Do whatever needed to generate an array of waypoint to use after
        for (const auto &node : config["waypoints"]) {
          if (!node.IsMap()) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid waypoint format, skipping...");
            continue;
          }

          double dx = node["dx"].as<double>();
          double dy = node["dy"].as<double>();
          double dphi = node["dphi"].as<double>();
          bool holonomic = node["holonomic"].as<bool>();

          waypoints_traj.emplace_back(dx, dy, dphi, holonomic);
        }
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s",
                   e.what());
    }
  }

  // Move the robot according to the desired trajectory
  void control_loop() {

    // Wait till receiving a valid odometry message
    if (!odom_received) {
      RCLCPP_WARN(this->get_logger(),
                  "No odometry message received yet, waiting... ");

      return;
    }

    // If the robot finished the maze, stop it then exit the node
    if (traj_index >= waypoints_traj.size()) {
      stop_robot();
      RCLCPP_INFO_ONCE(this->get_logger(), "Completed the trajectory! ");
      rclcpp::shutdown();
      return;
    }

    // Always compute the distance left to the front wall
    compute_obstacle_distances();

    target = waypoints_traj[traj_index];

    if (robot_state == TURN) {
      turn_controller(target.dphi);
    }

    if (robot_state == MOVE) {
      distance_controller();
    }

    if (robot_state == CORRECT_TILT) {
      correct_tilt();
    }

    if (robot_state == STOP) {
      stop_robot();
      return;
    }
  }

  // Apply a controller to turn robot by "targeted_dphi" radians
  void turn_controller(double targeted_dphi) {
    // Compute the target yaw ONLY ONCE per Waypoint
    if (!set_target_yaw) {
      // Compute the angle left to the target
      target_yaw = normalize_angle(current_yaw + targeted_dphi);
      set_target_yaw = true;
      integral_yaw = 0.0;
    }

    // Compute the angle left to the target
    double error_yaw = normalize_angle(target_yaw - current_yaw);

    // RCLCPP_INFO(this->get_logger(), "error_yaw : %f", error_yaw);

    // If the robot reached the target waypoint
    if (std::abs(error_yaw) < 0.02) {

      set_target_yaw = false; // Reset target yaw for next waypoint
      yaw_pre_move = current_yaw;

      if (robot_state == CORRECT_TILT) { // If the tilt was corrected
        traj_index++;                    // Go to the next waypoint
        robot_state = TURN;
        RCLCPP_INFO(this->get_logger(), "Successfully corrected robot tilt");

      }

      else {
        robot_state = MOVE; // After turn sequence ends, start move sequence
        RCLCPP_INFO(this->get_logger(), "Turned towards waypoint number : %lu",
                    traj_index + 1);
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

    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = angular_speed;

    twist_pub->publish(twist_cmd);

    prev_error_yaw = error_yaw;
  }

  void distance_controller() {

    // Save x, y, yaw at the start of the movement
    if (!move_initialized) {
      move_initialized = true;

      x_start = current_x;
      y_start = current_y;

      // Reset PID integrals and previous errors
      integral_distance = 0.0;
      prev_error_distance = 0.0;

      obstacle_front = false;
    }

    // Compute the actual distance travelled along x/y axes
    traveled_dx = current_x - x_start;
    traveled_dy = current_y - y_start;

    // Compute the distance left to the target
    double error_x = target.dx - traveled_dx;
    double error_y = target.dy - traveled_dy;
    // RCLCPP_INFO(this->get_logger(), "error_x is : %f", error_x);
    // RCLCPP_INFO(this->get_logger(), "error_y is : %f", error_y);

    // Important: when dx or dy = 0 during lateral movements,
    // the corresponding error still accumulates noise, resulting in
    // non null velocities that need to be corrected
    if (std::abs(target.dy) < 1e-3)
      error_y = 0.0;
    if (std::abs(target.dx) < 1e-3)
      error_x = 0.0;

    double error_distance = std::hypot(error_x, error_y);

    // RCLCPP_INFO(this->get_logger(), "error_distance is : %f",
    // error_distance);
    //  RCLCPP_INFO(this->get_logger(), "error_y is : %f", error_y);

    // If the robot reached the target waypoint
    if (error_distance < 0.01 || (error_distance < 0.1 && obstacle_front)) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint number : %lu",
                  traj_index + 1);

      yaw_post_move = current_yaw;

      move_initialized = false; // Reset for next move phase
      traveled_dx = 0.0;
      traveled_dy = 0.0;

      robot_state =
          CORRECT_TILT; // Correct the tilt before going to the next waypoint

      stop_robot(); // Stop the robot for 20 * 0.1 = 2 seconds
      rclcpp::sleep_for(std::chrono::seconds(2));
      return;
    }

    // Calculate delta time
    rclcpp::Time now = this->now();
    double dt = prev_time_distance.nanoseconds() == 0
                    ? 0.05
                    : (now - prev_time_distance).seconds();
    // dt = std::max(dt, 1e-3);

    prev_time_distance = now;

    // Integral terms
    integral_distance += error_distance * dt;

    // Derivative terms
    double derivative_distance = (error_distance - prev_error_distance) / dt;

    // PID control law
    double linear_vel = kp_distance * error_distance +
                        ki_distance * integral_distance +
                        kd_distance * derivative_distance;

    // Make sure the robot stays within max speed bounds
    linear_vel = std::clamp(linear_vel, -max_linear_speed, +max_linear_speed);

    // Set the direction for holonomic movement (Right/Left)
    // Right and left directions are different between simulation and real
    if (scene_number_ == 2) {
      direction = 1.0;
      if (target.dx < 0 || target.dy < 0) {
        direction = -1.0;
      }
    }

    // Depending on whether the robot is going to move along its x ar y axis,
    // we set the correct linear velocity
    if (!target.holonomic) {
      twist_cmd.linear.x = linear_vel;
      twist_cmd.linear.y = 0.0;
    } else {
      twist_cmd.linear.x = 0.0;
      twist_cmd.linear.y = direction * linear_vel;
    }
    twist_cmd.angular.z = -0.015;

    twist_pub->publish(twist_cmd);

    prev_error_distance = error_distance;
  }

  // Send zero velocitites to stop the robot
  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_pub->publish(stop_msg);
  }

  void correct_tilt() {
    double tilt = normalize_angle(yaw_pre_move - yaw_post_move);
    RCLCPP_DEBUG(this->get_logger(), "Correcting Tilt : %f ", tilt);
    turn_controller(tilt);
  }

  // normalize angles to range [-pi, pi]
  double normalize_angle(double angle) {
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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  // Parameters to move the robot
  geometry_msgs::msg::Twist twist_cmd;

  // Parameters used to compute the yaw
  double current_yaw = 0.0;
  bool odom_received = false;

  // Parameters used to compute the distance to the waypoint
  double old_x = 0.0;
  double old_y = 0.0;
  double current_x = 0.0;
  double current_y = 0.0;
  double x_start = 0.0;
  double y_start = 0.0;
  double traveled_dx = 0.0;
  double traveled_dy = 0.0;

  // Waypoints the robot is passing by throughout the maze
  std::vector<WayPoint> waypoints_traj;
  long unsigned int traj_index = 0;
  WayPoint target; // at each step of the trajectory

  // PID Turn Controller Parameters
  double angular_speed = 0.0;
  double kp_yaw = 1.5;        // Proportional Gain
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
  double kp_distance = 0.35;      // Proportional Gain
  double ki_distance = 0.005;     // Integral Gain
  double kd_distance = 0.15;      // Derivative Gain
  double integral_distance = 0.0; // Integral terms of the PID controller
  double integral_y = 0.0;
  rclcpp::Time prev_time_distance; // instant t-1
  double prev_error_distance = 0.0;
  double prev_error_y = 0.0;
  double max_linear_speed =
      0.8;                 // source: https://husarion.com/manuals/rosbot-xl/
  double direction = -1.0; // for holonomic mode

  // Robot State Parameters
  RobotState robot_state;

  bool move_initialized = false;

  // Distinguish between simulation and real scenarios
  int scene_number_; // 1 : Sim; 2: Real

  // Parameters for laserScan
  double max_range = 0.0;
  double min_range = 0.0;
  double left_distance = 0.0;
  double right_distance = 0.0;
  double front_distance = 0.0;
  double back_distance = 0.0;
  double min_distance_threshold = 0.2;
  bool obstacle_front = false;
  bool obstacle_left = false;
  bool obstacle_right = false;
  bool obstacle_back = false;

  // Parameters for correcting tilt
  double yaw_pre_move = 0.0;
  double yaw_post_move = 0.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  std::shared_ptr<PIDMazeSolver> pid_maze_solver =
      std::make_shared<PIDMazeSolver>(scene_number);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pid_maze_solver);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}