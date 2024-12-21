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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_interface_package_cpp/msg/num.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // Similar to the producer code this is getting updated to now use a custom msg interface.
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_ = this->create_subscription<my_interface_package_cpp::msg::Num>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  // Another similar spot where we need to make sure the callback uses the right message object
  // void topic_callback(const std_msgs::msg::String & msg) const
  void topic_callback(const my_interface_package_cpp::msg::Num & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '" << std::to_string(msg.num).str() << "'");
  }
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<my_interface_package_cpp::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
