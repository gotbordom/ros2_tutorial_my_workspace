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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_interface_package_cpp/msg/num.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // For this stage of the tutorials I am now using a message object I created instead of the std msgs
    // auto message = std_msgs::msg::String();
    // Funny note here, I initally created that package with the _cpp assuming that I would only build
    // the interfaces for cpp OR python. Turns out by default it builds both... so that is a misleading
    // name I created.
    auto message = my_interface_package_cpp::msg::Num();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // seeing as my custom message Num only contains a number object no data object. So we can update
    // this as well.
    // To confirm this run 'ros2 interface show std_msgs/msg/String' then run
    // 'ros2 interface show my_interface_package_cpp/msg/Num'
    message.num = this->count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '" << std::to_string(message.num).str() << "'");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<my_interface_package_cpp::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
