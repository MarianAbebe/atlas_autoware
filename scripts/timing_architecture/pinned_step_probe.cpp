// Component-only timing experiment; actual pinned simulator, no replacement dynamics.
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Odom = nav_msgs::msg::Odometry;
using Control = autoware_control_msgs::msg::Control;
using Gear = autoware_vehicle_msgs::msg::GearCommand;
void require(bool value, const char * message) {if (!value) throw std::runtime_error(message);}

int probe(bool controlled, int stall_ms)
{
  auto * yaml = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  auto file = ament_index_cpp::get_package_share_directory("atlas_closed_loop_demo") + "/config/simulator.param.yaml";
  require(rcl_parse_yaml_file(file.c_str(), yaml), "parameter parse failed");
  std::vector<rclcpp::Parameter> parameters;
  for (const auto & entry : rclcpp::parameter_map_from(yaml, ""))
    parameters.insert(parameters.end(),entry.second.begin(),entry.second.end());
  rcl_yaml_node_struct_fini(yaml);
  auto set = [&](rclcpp::Parameter p) {
    parameters.erase(std::remove_if(parameters.begin(),parameters.end(),[&](const auto & old) {
      return old.get_name()==p.get_name();}),parameters.end());
    parameters.push_back(p);
  };
  set(rclcpp::Parameter("use_sim_time",controlled));
  set(rclcpp::Parameter("initialize_source","ORIGIN"));
  set(rclcpp::Parameter("vehicle_model_type","IDEAL_STEER_ACC_GEARED"));
  set(rclcpp::Parameter("initial_engage_state",true));
  set(rclcpp::Parameter("simulated_frame_id","timing_probe_truth"));
  auto library = ament_index_cpp::get_package_prefix("autoware_simple_planning_simulator")+
    "/lib/libautoware_simple_planning_simulator.so";
  class_loader::ClassLoader loader(library);
  const std::string type="rclcpp_components::NodeFactoryTemplate<autoware::simulator::simple_planning_simulator::SimplePlanningSimulator>";
  auto factory=loader.createInstance<rclcpp_components::NodeFactory>(type);
  rclcpp::NodeOptions options;
  options.use_global_arguments(false).use_intra_process_comms(false).parameter_overrides(parameters);
  options.arguments({"--ros-args","-r","__ns:=/atlas_timing/sim","-r","/clock:=/atlas_timing/clock"});
  auto component=factory->create_node_instance(options);
  auto driver=std::make_shared<rclcpp::Node>("step_coordinator","/atlas_timing");
  auto clock=driver->create_publisher<rosgraph_msgs::msg::Clock>("clock",10);
  auto commands=driver->create_publisher<Control>("sim/input/ackermann_control_command",10);
  auto gear=driver->create_publisher<Gear>("sim/input/gear_command",10);
  std::vector<Odom> samples;
  auto sub=driver->create_subscription<Odom>("sim/output/odometry",100,
    [&](Odom::ConstSharedPtr msg){samples.push_back(*msg);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(driver);exec.add_node(component.get_node_base_interface());
  auto pump=[&](auto predicate, std::chrono::milliseconds limit=2000ms) {
    auto end=std::chrono::steady_clock::now()+limit;
    while (std::chrono::steady_clock::now()<end) {
      exec.spin_some();
      if (predicate()) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  };
  require(pump([&]{return commands->get_subscription_count() && gear->get_subscription_count() &&
    (!controlled || clock->get_subscription_count());}),"discovery timeout");
  Control command;command.longitudinal.acceleration=.2;command.longitudinal.velocity=1.;
  Gear drive;drive.command=Gear::DRIVE;
  commands->publish(command);gear->publish(drive);
  require(commands->wait_for_all_acked(1s) && gear->wait_for_all_acked(1s),"input acknowledgement failed");
  for(int i=0;i<20;++i){exec.spin_some();std::this_thread::sleep_for(1ms);}
  int64_t tick=1000000000;
  auto step=[&] {
    tick+=20000000;
    rosgraph_msgs::msg::Clock msg;msg.clock=static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(tick));clock->publish(msg);
    require(pump([&]{return !samples.empty() && rclcpp::Time(samples.back().header.stamp).nanoseconds()==tick;}),
      "step output barrier timeout");
  };
  if (controlled) {for(int i=0;i<10;++i) step();}
  else require(pump([&]{return samples.size()>=10;}),"wall-time warmup failed");
  const auto before=samples.back();
  const auto count=samples.size();
  // Deliberately stall this executor; no commands or clocks are advanced.
  std::this_thread::sleep_for(std::chrono::milliseconds(stall_ms));
  if (controlled) {
    for(int i=0;i<10;++i){exec.spin_some();std::this_thread::sleep_for(1ms);}
    require(samples.size()==count,"controlled simulator advanced while clock paused");
    step();
  } else require(pump([&]{return samples.size()>count;}),"post-stall output missing");
  const auto after=samples.back();
  const double dt=(rclcpp::Time(after.header.stamp)-rclcpp::Time(before.header.stamp)).seconds();
  const double dv=after.twist.twist.linear.x-before.twist.twist.linear.x;
  require(std::abs(dv-.2*dt)<.003,"held acceleration integration mismatch");
  if (controlled) {
    require(std::abs(dt-.02)<1e-9 && std::abs(dv-.004)<1e-5,"controlled step changed with wall stall");
    // Several more acknowledged steps establish that resumption is repeatable.
    for(int i=0;i<10;++i) step();
  } else require(stall_ms>=450 && dt>.45 && dv>.09,"did not expose variable elapsed-time update");
  std::cout << std::setprecision(17) << "ATLAS_TIMING_RESULT {\"mode\":\""<<(controlled?"controlled":"realtime")
    <<"\",\"wall_stall_sec\":"<<stall_ms/1000.0<<",\"state_dt_sec\":"<<dt<<",\"velocity_increment\":"<<dv
    <<",\"samples\":"<<samples.size()<<",\"final_x\":"<<samples.back().pose.pose.position.x
    <<",\"final_vx\":"<<samples.back().twist.twist.linear.x<<",\"passed\":true}"<<std::endl;
  exec.remove_node(component.get_node_base_interface());exec.remove_node(driver);
  return 0;
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  int status=0;
  try {
    const int stall_ms=argc>2?std::stoi(argv[2]):500;
    require(stall_ms>=0 && stall_ms<=1000,"stall must be 0..1000 ms");
    status=probe(argc>1 && std::string(argv[1])=="controlled",stall_ms);
  }
  catch(const std::exception & error) {std::cerr<<"FAIL: "<<error.what()<<std::endl;status=1;}
  rclcpp::shutdown();return status;
}
