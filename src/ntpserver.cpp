#include <chrono>
#include <memory>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <pthread.h> 
#include <sched.h>
#include <pthread.h> 
#include <sched.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "newtest/srv/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RpiTalker : public rclcpp::Node
{
public:
  RpiTalker()
  : Node("RpiTalker"), count_(0)
  {
    ntpserver = this->create_service<newtest::srv::Time>("NTP", std::bind(&RpiTalker::timer_synchronize, this,_1,std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to Time Synchronize.");

  }

private:
  void timer_synchronize(const std::shared_ptr<newtest::srv::Time::Request> request,
  std::shared_ptr<newtest::srv::Time::Response> response)
  {
    auto stamp1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    std::cout<< "request t1: " << request->t1 << std::endl;
    auto stamp2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    response->t2 = request->t1 + std::to_string(stamp1) + "," + std::to_string(stamp2)+ ",";
   
  }
  rclcpp::Service<newtest::srv::Time>::SharedPtr ntpserver;
  size_t count_;
};
//-----------------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpiTalker>());
    rclcpp::shutdown();
    return 0;
}
