/******************************************************************************
MIT License

Copyright (c) 2022 Sharmitha Ganesan M.Eng (Robotics)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
********************************************************************************
*/

/**
 * @file walker.cpp
 * @author sganesa3@terpmail.umd.edu
 * @brief walker algorithm to publish cmd velocity based on depth image
 * @version 0.1
 * @date 2022-12-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono> // NOLINT
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals; // NOLINT

typedef enum {
  FORWARD,
  STOP,
  TURN,
} StateType;
/**
 * @brief class Chitti with walker node
 *
 */
class Chitti : public rclcpp::Node {
 public:
  Chitti() : Node("my_walker"), state_(STOP) {
    // creates publisher to publish walker/cmd topic
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("walker/cmd", 10);
    rclcpp::QoS depth_qos(10);
    depth_qos.keep_last(10);
    depth_qos.best_effort();
    depth_qos.durability_volatile();

    // creates subscriber to get /walker/depth topic
    auto subTopicName = "walker/depth";
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subTopicName, depth_qos,
        std::bind(&Chitti::subscribe_callback, this, _1));
    // create a 10Hz timer for processing
    auto processCallback = std::bind(&Chitti::process_callback, this);
    timer_ = this->create_wall_timer(1000ms, processCallback);
  }

 private:
  /**
   * @brief assign msg to lastImg
   *
   * @param msg
   */
  void subscribe_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    lastImg_ = msg;
  }
  /**
   * @brief publish cmd velocity
   *
   */
  void process_callback() {
    // Do nothing until the first data read
    if (lastImg_->header.stamp.sec == 0)
      return;

    // Create the message to publish (initialized to all 0)
    auto message = geometry_msgs::msg::Twist();

    // state machine (Mealy -- output on transition)
    switch (state_) {
    case FORWARD:
      if (hasObstacle()) {  // check transition
        state_ = STOP;
        publisher_->publish(message);
        RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
      }
      break;
    case STOP:
      if (hasObstacle()) {  // check transition
        state_ = TURN;
        message.angular.z = 0.7;
        publisher_->publish(message);
        RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
      } else {
        state_ = FORWARD;
        message.linear.x = 0.5;
        publisher_->publish(message);
        RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
      }
      break;
    case TURN:
      if (!hasObstacle()) {  // check transition
        state_ = FORWARD;
        message.linear.x = 0.5;
        publisher_->publish(message);
        RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
      }
      break;
    }
  }

  bool hasObstacle() {
    unsigned char *dataPtr = lastImg_->data.data();
    float *floatData = (float *)dataPtr; // NOLINT

    int idx;
    for (unsigned int row = 0; row < lastImg_->height - 40; row++)
      for (unsigned int col = 0; col < lastImg_->width; col++) {
        idx = (row * lastImg_->width) + col;
        if (floatData[idx] < 1.0) {
          RCLCPP_INFO(this->get_logger(),
                      "row=%d, col=%d, floatData[idx] = %.2f", row, col,
                      floatData[idx]);
          return true;
        }
      }

    return false;
  }

  ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr lastImg_;
  StateType state_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Chitti>());
  rclcpp::shutdown();
  return 0;
}
