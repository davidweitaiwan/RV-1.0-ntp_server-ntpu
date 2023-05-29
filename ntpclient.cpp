#include <chrono>
#include <memory>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <pthread.h> 
#include <sched.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "newtest/srv/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

static const std::vector<std::string> rspdata (const std::string rspstr, const std::string comma2)
{
  std::vector<std::string> result;
  std::string::size_type start,end;
  start = 0;
  end = rspstr.find(comma2);
  int add = 0;
  while (add < 3 && (end != std::string::npos))
  {
    result.push_back(rspstr.substr(start, end-start));
    start =  end + comma2.size();
    end = rspstr.find(comma2, start);
    add ++;
  }
  return result;
}

class Listener : public rclcpp::Node
{
public:
  Listener(): Node("PCListener"), count_(0)
  {
    ntpclient = this->create_client<newtest::srv::Time>("NTP");
    timer_synchronize();

    PeriodData.open("Data.csv");
    PeriodData << "ID,SendTime,RecvTime,Size,offset\n";
    
    timer_client = this->create_wall_timer(
      30s, std::bind(&Listener::timer_synchronize, this));
    
  }

  double offset;
  void timer_synchronize()
  {
    
    if (!ntpclient->wait_for_service(10s))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
    }
     
    auto request = std::make_shared<newtest::srv::Time::Request>();
    auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    request->t1 = std::to_string(stamp) + ",";
    auto result = ntpclient->async_send_request(request, std::bind(&Listener::response_callback,this,_1));
    
    std::cout << "------------------------------------------------- " << std::endl;
  }

  void response_callback(rclcpp::Client<newtest::srv::Time>::SharedFuture future)
  {
      auto status = future.wait_for(1s);
      if (status == std::future_status::ready) 
      {
      auto stamp2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      std::string comma2 = ",";
      std::vector<std::string> rsp = rspdata(future.get()->t2,comma2);
      std::cout << "client t1: " << rsp.at(0).c_str()<< std::endl;
      std::cout << "service t2: " << rsp.at(1).c_str()<< std::endl;
      std::cout << "service t3: " << rsp.at(2).c_str()<< std::endl;
      std::cout << "client t4: " << std::to_string(stamp2)<< std::endl;
      offset = ((std::stold(rsp.at(1))-std::stold(rsp.at(0))) + (std::stold(rsp.at(2))-stamp2))/2;
      std::cout << "offset: " <<std::to_string(offset)<< std::endl;

      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ");
      }
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<newtest::srv::Time>::SharedPtr ntpclient;
  rclcpp::TimerBase::SharedPtr timer_client;
  std::ofstream PeriodData; 
  size_t count_;
};
//-----------------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
