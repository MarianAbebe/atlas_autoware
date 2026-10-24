// Reproduce the pinned polling helper's middleware/ROS clock-domain mismatch.
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <stdexcept>
using namespace std::chrono_literals;
void require(bool value,const char * text){if(!value)throw std::runtime_error(text);}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);int status=0;
  try {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides({rclcpp::Parameter("use_sim_time",true)});
    options.arguments({"--ros-args","-r","/clock:=/atlas_polling/clock"});
    auto consumer=std::make_shared<rclcpp::Node>("polling_clock_consumer","/atlas_timing",options);
    auto sender=std::make_shared<rclcpp::Node>("polling_clock_sender","/atlas_timing");
    autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> poll(
      consumer.get(),"/atlas_timing/polling_odom");
    auto pub=sender->create_publisher<nav_msgs::msg::Odometry>("polling_odom",1);
    auto clock=sender->create_publisher<rosgraph_msgs::msg::Clock>("/atlas_polling/clock",10);
    rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(consumer);exec.add_node(sender);
    auto wait=[&](auto predicate) {
      const auto until=std::chrono::steady_clock::now()+3s;
      while(std::chrono::steady_clock::now()<until){exec.spin_some();if(predicate())return true;std::this_thread::sleep_for(1ms);}
      return false;
    };
    require(wait([&]{return pub->get_subscription_count() && clock->get_subscription_count();}),"discovery failed");
    rosgraph_msgs::msg::Clock time;time.clock.sec=1;clock->publish(time);
    require(wait([&]{return consumer->now().nanoseconds()==1000000000LL;}),"clock activation failed");
    nav_msgs::msg::Odometry message;message.header.stamp.sec=1;pub->publish(message);
    nav_msgs::msg::Odometry::ConstSharedPtr received;
    require(wait([&]{received=poll.take_data();return bool(received);}),"poll take failed");
    auto metadata=poll.last_taken_data_timestamp();require(metadata.has_value(),"metadata unavailable");
    time.clock.sec=2;clock->publish(time);
    require(wait([&]{return consumer->now().nanoseconds()==2000000000LL;}),"second clock step failed");
    const double header_age=(consumer->now()-rclcpp::Time(received->header.stamp)).seconds();
    const double diagnostic_age=(consumer->now()-*metadata).seconds();
    require(header_age==1. && diagnostic_age < -1000000.,"expected domain mismatch not observed");
    // SmoothStop also constructs standalone ROS clocks, without the node TimeSource.
    rclcpp::Clock standalone(RCL_ROS_TIME);
    require(!standalone.ros_time_is_active() && standalone.now().nanoseconds()>1000000000000000LL,
      "standalone ROS-clock fallback differs from expected host time");
    // The actual Controller::check_cyclic_message_timeout compares age > 0.9.
    require(header_age>.9 && !(diagnostic_age>.9),"cyclic-timeout comparison did not diverge");
    std::cout<<std::setprecision(17)<<"ATLAS_POLLING_RESULT {\"header_age_sec\":"<<header_age
      <<",\"polling_metadata_age_sec\":"<<diagnostic_age
      <<",\"expected_cyclic_timeout\":true,\"actual_comparison_timeout\":false,\"mismatch_reproduced\":true,\"unattached_ros_clock_uses_host_time\":true}"<<std::endl;
  } catch(const std::exception & e){std::cerr<<"FAIL: "<<e.what()<<std::endl;status=1;}
  rclcpp::shutdown();return status;
}
