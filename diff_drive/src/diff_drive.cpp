// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DiffDrive : public rclcpp::Node
{

float wheel_base = std::stof(declare_parameter<std::string>("wheel_base", ""));
float wheel_radius = std::stof(declare_parameter<std::string>("wheel_radius", ""));

public:
  DiffDrive()
  : Node("diff_drive")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DiffDrive::diff_callback,this, _1));
    publisher_left = this->create_publisher<std_msgs::msg::Float64>("/commands/motor_left/speed", 100);
    publisher_right = this->create_publisher<std_msgs::msg::Float64>("/commands/motor_right/speed", 100);
  }

private:
  void diff_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float vel = msg->linear.x;
    float ang = msg->angular.z;

    float left_rpm  = (vel - 0.5f*ang*wheel_base)/((2 * M_PI) / 60 * wheel_radius);
    float right_rpm = (vel + 0.5f*ang*wheel_base)/((2 * M_PI) / 60 * wheel_radius);

    auto message_left = std_msgs::msg::Float64();
    message_left.data = left_rpm;
    auto message_right = std_msgs::msg::Float64();
    message_right.data = right_rpm;
    publisher_left->publish(message_left);
    publisher_right->publish(message_right);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDrive>());
  rclcpp::shutdown();
  return 0;
}
