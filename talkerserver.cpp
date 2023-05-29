#include <chrono>
#include <memory>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
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
  RpiTalker(std::chrono::seconds ex_time, const rclcpp::QoS qos, std::chrono::microseconds rate, int size)
  : Node("RpiTalker"), count_(0)
  {
    msg_rate = rate;
    msg_size = size;
    msg_payload = (char *)malloc(msg_size * sizeof(char) + 1);  //malloc : configuration memory block,generate an array of a size to transfer
    for (int i = 0; i < msg_size; i++)
    {
      msg_payload[i] = '0';  //fill array
    }
    msg_payload[msg_size] = '\0';
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("Rpi", qos);
    ntpserver = this->create_service<newtest::srv::Time>("NTP", std::bind(&RpiTalker::timer_synchronize, this,_1,std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to Time Synchronize.");

    timer_ = this->create_wall_timer(
      msg_rate, std::bind(&RpiTalker::timer_callback, this));

    timer_stop_ = this->create_wall_timer(
      ex_time, std::bind(&RpiTalker::timer_stop_callback, this));
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

  void timer_callback()
  {
    auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(); //get system high_resolution_clock
    auto message = std_msgs::msg::String();
    
    std::string my_message_header_str =  std::to_string(count_++) + ","+ std::to_string(stamp) + ",";
    if (my_message_header_str.length() >=50){
      printf("message header too large\n");
      exit(0);
    }
    
    memcpy(msg_payload, my_message_header_str.c_str(), my_message_header_str.length()); // Overwrite part of data in the msg_payload file
    //message.data = std::to_string(count_++) + ","+ std::to_string(stamp) + "," + msg_payload;
    message.data = msg_payload;
    //RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
    //outfile<< count_ << "," << std::to_string(stamp) << std::endl;
    
    publisher_->publish(message);
  }
  void timer_stop_callback(){
    std::cout << "Send all message number : " << std::to_string(count_++) << std::endl;
    exit(0);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_stop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::chrono::microseconds msg_rate;
  int msg_size;
  size_t count_;
  char *msg_payload = NULL;
  rclcpp::Service<newtest::srv::Time>::SharedPtr ntpserver;
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

  //Priority FIFO policy
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


  const char *OPTION_EXPERIMENCE_TIME = "-t";
  std::chrono::seconds ex_time;
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_EXPERIMENCE_TIME)) //read command keyword
  {
    char *start = rcutils_cli_get_option(argv, argv + argc, OPTION_EXPERIMENCE_TIME); //read the value after the command keyward
    int value = std::stoi(start);
    std::chrono::seconds my_time(value);
    ex_time = my_time;
  }else{
    ex_time = 10s;
  }
  std::cout << "experience time : "<< ex_time.count() << std::endl;
  
  //QoS
  const char *OPTION_QOS_RELIABILITY = "-r";
  const char *OPTION_QOS_HISTORY = "-all";
  const char *OPTION_QOS_DEPTH = "-d";
  int qos_depth = 0;
  rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rmw_qos_history_policy_t qos_history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  const char *OPTION_MSG_RATE = "-mr";
  const char *OPTION_MSG_SIZE = "-ms";
  std::chrono::microseconds msg_rate = 1000us;
  size_t msg_size = 1000;

  // judge reliable or best_effort
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_QOS_RELIABILITY))
  {
    //default best_effort
    qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    std::cout << "using qos_reliability : " <<"reliable"<< std::endl;
  }else{
    qos_reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    std::cout << "using qos_reliability : " << "best_effort" << std::endl;
  }

  // judge history is keep_all or kepp_last
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_QOS_HISTORY))
  {
    //default keeplast
    qos_history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    std::cout << "using qos_history : "<< "keep_all" << std::endl;
  }else{
    qos_history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    std::cout << "using qos_history : "<< "keep_last" << std::endl;
  }

  // judge depth value
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_QOS_DEPTH))
  {
    qos_depth = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_QOS_DEPTH));
    
  }else{
    qos_depth = 1;
  }
  std::cout << "using qos_depth : "<< qos_depth << std::endl;

  // judge msg rate
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_MSG_RATE))
  {
    char *start = rcutils_cli_get_option(argv, argv + argc, OPTION_MSG_RATE);
    int value = std::stoi(start);
    std::chrono::microseconds time_interval(value);
    msg_rate = time_interval;
  }else{
    msg_rate = 1000us;
  }
  std::cout << "MSG rate(us) : " << msg_rate.count() << std::endl;

  // msg size
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_MSG_SIZE))
  {
    msg_size = std::stoi(rcutils_cli_get_option(argv, argv + argc, OPTION_MSG_SIZE));
    if (msg_size < 50){
      msg_size = 50;
      std::cout << "error : MSG size(byte) must be larger than 50, setting default value)" << msg_size << std::endl;
    }
    
  }else{
    msg_size = 1000;
  }
  std::cout << "MSG size(byte) : " << msg_size << std::endl;
  std::cout << "cmd end" << std::endl;

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization( //QosInitialization : using history and depth at same time
      qos_history,
      qos_depth
    )
  );
  
  qos.reliability(qos_reliability);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RpiTalker>(ex_time, qos, msg_rate, msg_size));
  rclcpp::shutdown();
  return 0;
}