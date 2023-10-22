#include "odometry_estimation_node.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <functional>
#include <numeric>

using std::placeholders::_1;
using namespace std::chrono_literals;

// Constructor
OdometryEstimator::OdometryEstimator() : Node("odometry_publisher")
{
  // init vehicle model
  vehicle_model_ = VehicleModel::createConcreteVehicleModel("DifferentialDrive");

  // create subscribers
  right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
      "feedback/motor_right/rpm", 10,
      std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
  left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
      "feedback/motor_left/rpm", 10,
      std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

  right_wheel_subscriber_com = this->create_subscription<std_msgs::msg::Float64>(
      "commands/motor_right/speed", 10,
      std::bind(&OdometryEstimator::handleRightWheelCom, this, std::placeholders::_1));
  left_wheel_subscriber_com = this->create_subscription<std_msgs::msg::Float64>(
      "commands/motor_left/speed", 10,
      std::bind(&OdometryEstimator::handleLeftWheelCom, this, std::placeholders::_1));

  // create publisher and timer
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  publisher_base = this->create_publisher<nav_msgs::msg::Odometry>("base_odom", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&OdometryEstimator::publish, this));
}

void OdometryEstimator::handleRightWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_right)
{
  rpms_right_.push_back(rpm_right->data);
}

void OdometryEstimator::handleLeftWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_left)
{
  rpms_left_.push_back(rpm_left->data);
}

void OdometryEstimator::handleRightWheelCom(const std_msgs::msg::Float64::SharedPtr rpm_right)
{
  int rpm_right_ = (int)rpm_right->data;
  rpms_right_com.push_back(rpm_right_);
}

void OdometryEstimator::handleLeftWheelCom(const std_msgs::msg::Float64::SharedPtr rpm_left)
{
  int rpm_left_ = (int)rpm_left->data;
  rpms_left_com.push_back(rpm_left_);
}

void OdometryEstimator::publish()
{
  // calculate passed time since last publish
  TimePoint current_time = Clock::now();
  std::chrono::duration<double> dt = current_time - previous_time_;
  // calculate average of received rpm signals
  int rpm_left_avg = std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
  int rpm_right_avg = std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();
  int rpm_left_avg_com = std::accumulate(rpms_left_com.begin(), rpms_left_com.end(), 0.0) / rpms_left_com.size();
  int rpm_right_avg_com = std::accumulate(rpms_right_com.begin(), rpms_right_com.end(), 0.0) / rpms_right_com.size();
  rpm_left_avg = rpm_left_avg / 7 / 4; //divided by 7 gives actual rpm from erpm, divide by 4 gives rpm of wheels
  rpm_right_avg = rpm_right_avg / 7 / 4;
  rpm_left_avg_com = rpm_left_avg_com / 7 / 4;
  rpm_right_avg_com = rpm_right_avg_com / 7 / 4;
  rpms_left_.clear();
  rpms_right_.clear();
  rpms_left_com.clear();
  rpms_right_com.clear();
  // calculate new state based on input
  VehicleState new_state =
      vehicle_model_->calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt.count());
  VehicleState new_state_com =
      vehicle_model_->calculateNextState(rpm_left_avg_com, rpm_right_avg_com, state_com, dt.count());
  // create quaternion from yaw angle
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, new_state.yaw);
  // fill message and publish
  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "odom";
  message.pose.pose.position.x = new_state.x;
  message.pose.pose.position.y = new_state.y;
  message.pose.pose.orientation.x = quat.x();
  message.pose.pose.orientation.y = quat.y();
  message.pose.pose.orientation.z = quat.z();
  message.pose.pose.orientation.w = quat.w();

  tf2::Quaternion quat_com;
  quat_com.setRPY(0.0, 0.0, new_state_com.yaw);
  auto message_com = nav_msgs::msg::Odometry();
  message_com.header.stamp = this->get_clock()->now();
  message_com.header.frame_id = "base_link";
  message_com.pose.pose.position.x = new_state_com.x;
  message_com.pose.pose.position.y = new_state_com.y;
  message_com.pose.pose.orientation.x = quat_com.x();
  message_com.pose.pose.orientation.y = quat_com.y();
  message_com.pose.pose.orientation.z = quat_com.z();
  message_com.pose.pose.orientation.w = quat_com.w();
  publisher_->publish(message);
  publisher_base->publish(message_com);
  // update internal state
  state_ = new_state;
  state_com = new_state_com;
  previous_time_ = current_time;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryEstimator>());
  rclcpp::shutdown();
  return 0;
}