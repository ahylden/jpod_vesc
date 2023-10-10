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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  DiffDrive()
  : Node("diff_drive"), count_(0)
  {
    std::string motor_side = declare_parameter<std::string>("motor_side", "");

    subscription_ = this->create_subscription<geometry_msgs::Twist>("cmd_vel", 10, std::bind(&DiffDrive::diff_callback,this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("/commands/motor_"+motor_side+"/speed", 10);
  }

private:
  void diff_callback(const geometry_msgs::Twist::SharedPtr msg)
  {
    int vel = msg->linear.x;
    auto message = std_msgs::msg::Float64();
    message.data = std::to_string(vel);
    publisher_->publish(message);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
