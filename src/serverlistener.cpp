#include <chrono>
#include <memory>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "newtest/srv/time.hpp"
#include <unistd.h>
#include <stdlib.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

static const std::vector<std::string> recvdata (const std::string str, const std::string comma)
{
  std::vector<std::string> result;
  std::string::size_type start,end;
  start = 0;
  end = str.find(comma);
  int add = 0;
  while (add < 3 && (end != std::string::npos))
  {
    result.push_back(str.substr(start, end-start));
    start =  end + comma.size();
    end = str.find(comma, start);
    add ++;
  }
  return result;
}


class Listener : public rclcpp::Node
{
public:
  Listener(const rclcpp::QoS qos)
  : Node("PCListener"), count_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "Rpi", qos, std::bind(&Listener::topic_callback, this, _1));
    ntpserver = this->create_service<newtest::srv::Time>("NTP", std::bind(&Listener::timer_synchronize, this,_1,std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to Time Synchronize.");

    PeriodData.open("Data.csv");
    PeriodData << "ID,SendTime,RecvTime,Size,Time-offset\n";
  }

private:

  void timer_synchronize(const std::shared_ptr<newtest::srv::Time::Request> request,
    std::shared_ptr<newtest::srv::Time::Response> response)
  {
    auto stamp1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(); 
    std::cout << "request t1 : " << request->t1 << std::endl;
    auto stamp2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    response->t2 = request->t1 + std::to_string(stamp1) + "," + std::to_string(stamp2)+ ",";
   
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    std::string comma = ",";
    std::vector<std::string> recv = recvdata(msg->data,comma);
    RCLCPP_INFO(this->get_logger(), "sendID: ""%s" ", sendtime: " "%s" ", sendMSGsize: " "%d" ", recvNum: " "%d",recv.at(0).c_str(),recv.at(1).c_str(),msg->data.size(),1+count_++);
    auto zero = recv.at(2).c_str();
    if ( atoi(zero) != 0)
    {
      PeriodData << recv.at(0).c_str() << "," << recv.at(1).c_str() << ","<< stamp << "," << msg->data.length() << "," << recv.at(2).c_str() <<std::endl;
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<newtest::srv::Time>::SharedPtr ntpserver;
  std::ofstream PeriodData; //write to publish time on txt file 
  size_t count_;
};

//-------------------------------------------------------------------------main---------------------------------------------------------------------------------

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ); 
  std::cout << "version " __DATE__ " " __TIME__ "\n";
  std::cout << "-----EXPERIMENCE SETTING-----" << std::endl;

  //Priority
  int ret;
  pthread_t this_thread = pthread_self();
  struct sched_param params;
  const char *OPTION_PRIORITY = "-p";
  const char *OPTION_PRIORITY_POLICY_FIFO = "-f";
  const char *OPTION_PRIORITY_POLICY_RR = "-RR";

  //Priority value
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY)) //read command keyword
  {
    char *priority_value = rcutils_cli_get_option(argv, argv + argc, OPTION_PRIORITY); //read the value after the command keyward
    params.sched_priority = std::stoi(priority_value);
  }else{
    params.sched_priority = 0;
  }
  std::cout << "Set thread realtime priority value = " << params.sched_priority << std::endl;

  //Priority FIFO & RR & OTHER policy
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY_POLICY_FIFO)) //read command keyword
  {
    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    std::cout << "Priority policy = <SCHED_FIFO>" << std::endl;
  }else if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY_POLICY_RR)){
    ret = pthread_setschedparam(this_thread, SCHED_RR, &params);
    std::cout << "Priority policy = <SCHED_RR>" << std::endl;
  }else{
    ret = pthread_setschedparam(this_thread, SCHED_OTHER, &params);
    std::cout << "Priority policy = <SCHED_OTHER>" << std::endl;
  }
  std::cout << "ret value : " << ret << std::endl;
  if (ret !=0 ) {
    std::cout << "Unsuccessful in setting thread realtime" << std::endl;\
  }


  
  //QoS
  const char *OPTION_QOS_RELIABILITY = "-r";
  const char *OPTION_QOS_HISTORY = "-all";
  const char *OPTION_QOS_DEPTH = "-d";
  int qos_depth = 0;
  rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rmw_qos_history_policy_t qos_history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;



  // judge reliable or best_effort
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_QOS_RELIABILITY))
  {
    //default best_effort
    qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    std::cout << "using qos_reliability :" <<"reliable"<< std::endl;
  }else{
    qos_reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    std::cout << "using qos_reliability :" << "best_effort" << std::endl;
  }



  // judge history is keep_all or kepp_last
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_QOS_HISTORY))
  {
    //default keeplast
    qos_history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    std::cout << "using qos_history :"<< "keep_all" << std::endl;
  }else{
    qos_history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    std::cout << "using qos_history :"<< "keep_last" << std::endl;
  }




  // judge depth value
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_QOS_DEPTH))
  {
    qos_depth = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_QOS_DEPTH));
    
  }else{
    qos_depth = 1;
  }
  std::cout << "using qos_depth: "<< qos_depth << std::endl;

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization( //QosInitialization : using history and depth at same time
      qos_history,
      qos_depth
    )
  );



  qos.reliability(qos_reliability);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>(qos));
  rclcpp::shutdown();
  return 0;
}