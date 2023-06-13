#include <chrono>
#include <memory>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "newtest/srv/time.hpp"
#include <unistd.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

static const std::vector<std::string> recvdata (const std::string str, const std::string comma)
{
  std::vector<std::string> result;
  std::string::size_type start,end;
  start = 0;
  end = str.find(comma);
  int add = 0;
  while (add < 2 && (end != std::string::npos))
  {
    result.push_back(str.substr(start, end-start));
    start =  end + comma.size();
    end = str.find(comma, start);
    add ++;
  }
  return result;
}

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
  Listener(const rclcpp::QoS qos)
  : Node("PCListener"), count_(0)
  {
    ntpclient = this->create_client<newtest::srv::Time>("NTP");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "Rpi", qos, std::bind(&Listener::topic_callback, this, _1));
    
    PeriodData.open("Data.csv");
    PeriodData << "ID,SendTime,RecvTime,Size,offset\n";
    sleep(2);
    timer_synchronize();
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
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    std::string comma = ",";
    std::vector<std::string> recv = recvdata(msg->data,comma);
    //RCLCPP_INFO(this->get_logger(), "sendID: ""%s" ", sendtime: " "%s" ", sendMSGsize: " "%d" ", recvNum: " "%d",recv.at(0).c_str(),recv.at(1).c_str(),msg->data.size(),1+count_++);

    //std::cout << "offset: " <<std::to_string(offset)<< std::endl;
    PeriodData << recv.at(0).c_str() << "," << recv.at(1).c_str() << ","<< stamp << "," << msg->data.length() <<","<< std::to_string(offset) <<std::endl;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_client;
  std::ofstream PeriodData; //write to publish time on txt file 
  rclcpp::Client<newtest::srv::Time>::SharedPtr ntpclient;
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
