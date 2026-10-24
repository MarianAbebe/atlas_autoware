// Real CDR -> registered conversion, native-frame RANSAC, and pinned clustering.
// All algorithm execution occurs in unchanged component libraries.
#include <ament_index_cpp/get_package_prefix.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sys/resource.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <iterator>
#include <thread>
using Cloud = sensor_msgs::msg::PointCloud2;
using Objects = autoware_perception_msgs::msg::DetectedObjects;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
void require(bool condition, const std::string & text) {
  if (!condition) throw std::runtime_error(text);
}
template<class T> T load(const char * path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  require(bool(f) && f.tellg()>0 && f.tellg()<4000000, "invalid input CDR size");
  const size_t size = f.tellg(); f.seekg(0);
  rclcpp::SerializedMessage bytes(size);
  auto & raw = bytes.get_rcl_serialized_message();
  f.read(reinterpret_cast<char *>(raw.buffer),size); raw.buffer_length=size;
  require(bool(f), "input CDR read failed");
  T msg; rclcpp::Serialization<T>().deserialize_message(&bytes,&msg); return msg;
}
template<class T> void save(const std::string & path, const T & msg) {
  rclcpp::SerializedMessage bytes;
  rclcpp::Serialization<T>().serialize_message(&msg,&bytes);
  const auto & raw=bytes.get_rcl_serialized_message();
  std::ofstream f(path,std::ios::binary);
  f.write(reinterpret_cast<const char *>(raw.buffer),raw.buffer_length);
  require(bool(f),"output CDR write failed");
}
long rss_kib() {
  std::ifstream f("/proc/self/status"); std::string line;
  while (std::getline(f,line)) if (line.rfind("VmRSS:",0)==0) return std::stol(line.substr(6));
  throw std::runtime_error("cannot measure RSS");
}
int main(int argc,char **argv) {
 try {
  require(argc==3,"usage: real_lidar_perception_probe input.cdr output_directory");
  const auto input=load<Cloud>(argv[1]);
  const std::string out=argv[2]; std::filesystem::create_directories(out);
  rclcpp::init(0,nullptr);
  const auto sensor=ament_index_cpp::get_package_prefix("atlas_sensor_kit");
  const auto detector=ament_index_cpp::get_package_prefix("autoware_euclidean_cluster_object_detector");
  class_loader::ClassLoader converters(sensor+"/lib/libpointcloud_components.so");
  class_loader::ClassLoader ground(ATLAS_RANSAC_LIBRARY);
  class_loader::ClassLoader cluster(detector+"/lib/libautoware_euclidean_cluster_object_detector_node_core.so");
  auto factory=[](class_loader::ClassLoader & loader,const std::string & type) {
    return loader.createInstance<rclcpp_components::NodeFactory>(
      "rclcpp_components::NodeFactoryTemplate<"+type+">");
  };
  auto vf=factory(converters,"atlas_sensor_kit::VelodyneToExtendedComponent");
  auto cf=factory(converters,"atlas_sensor_kit::ExtendedToCompactComponent");
  auto gf=factory(ground,"autoware::ground_segmentation::RANSACGroundFilterComponent");
  auto ef=factory(cluster,"autoware::euclidean_cluster::EuclideanClusterNode");
  auto options=[](std::vector<std::string> args) {
    rclcpp::NodeOptions o; o.use_intra_process_comms(false).use_global_arguments(false);
    args.insert(args.begin(),{"--ros-args","-r","__ns:=/atlas_real_perception"});
    o.arguments(args); return o;
  };
  auto vo=options({"-r","~/input/pointcloud:=raw","-r","~/output/pointcloud:=extended"});
  vo.parameter_overrides({rclcpp::Parameter("time_is_relative_seconds",true)});
  auto co=options({"-r","~/input/pointcloud:=compact_input","-r","~/output/pointcloud:=compact"});
  auto go=options({"--params-file",ATLAS_RANSAC_PARAMS,"-r","input:=ground_input","-r","output:=filtered"});
  go.parameter_overrides({rclcpp::Parameter("base_frame",input.header.frame_id),
    rclcpp::Parameter("input_frame",input.header.frame_id),rclcpp::Parameter("output_frame",input.header.frame_id)});
  auto eo=options({"--params-file",ATLAS_CLUSTER_PARAMS,"-r","input:=cluster_input", "-r","output:=objects"});
  auto v=vf->create_node_instance(vo), c=cf->create_node_instance(co),
       g=gf->create_node_instance(go), e=ef->create_node_instance(eo);
  auto probe=std::make_shared<rclcpp::Node>("probe","/atlas_real_perception",
    rclcpp::NodeOptions().use_intra_process_comms(false));
  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto n : {v.get_node_base_interface(),c.get_node_base_interface(),g.get_node_base_interface(),e.get_node_base_interface()}) executor.add_node(n);
  executor.add_node(probe);
  std::map<std::string,Cloud::ConstSharedPtr> clouds;
  Objects::ConstSharedPtr objects;
  std::map<std::string,Clock::time_point> arrived;
  std::vector<rclcpp::Subscription<Cloud>::SharedPtr> subs;
  for (const std::string name : {"extended","compact","filtered","debug/clusters"}) {
    subs.push_back(probe->create_subscription<Cloud>(name,rclcpp::SensorDataQoS(),
      [&,name](Cloud::ConstSharedPtr msg){clouds[name]=msg;arrived[name]=Clock::now();}));
  }
  auto osub=probe->create_subscription<Objects>("objects",rclcpp::QoS(1),
    [&](Objects::ConstSharedPtr msg){objects=msg;arrived["objects"]=Clock::now();});
  std::map<std::string,rclcpp::Publisher<Cloud>::SharedPtr> pubs;
  for (const std::string name : {"raw","compact_input","ground_input","cluster_input"})
    pubs[name]=probe->create_publisher<Cloud>(name,rclcpp::SensorDataQoS());
  auto wait=[&](auto ready){
    auto deadline=Clock::now()+10s;
    while (!ready() && Clock::now()<deadline) {executor.spin_some();std::this_thread::sleep_for(1ms);}
    require(ready(),"bounded ROS wait timed out");
  };
  wait([&]{return std::all_of(pubs.begin(),pubs.end(),[](auto & p){return p.second->get_subscription_count()>0;}) &&
    osub->get_publisher_count()>0 && std::all_of(subs.begin(),subs.end(),[](auto & s){return s->get_publisher_count()>0;});});
  std::ofstream metrics(out+"/metrics.json");
  metrics << "{\"libraries\":{\"converter\":" << std::quoted(sensor+"/lib/libpointcloud_components.so")
    << ",\"ground\":" << std::quoted(std::string(ATLAS_RANSAC_LIBRARY))
    << ",\"clustering\":" << std::quoted(detector+"/lib/libautoware_euclidean_cluster_object_detector_node_core.so")
    << "},\"parameter_files\":{\"ground\":" << std::quoted(std::string(ATLAS_RANSAC_PARAMS))
    << ",\"clustering\":" << std::quoted(std::string(ATLAS_CLUSTER_PARAMS))
    << "},\"rss_before_kib\":" << rss_kib() << ",\"stages\":[";
  bool first=true;
  auto record=[&](const std::string & name,Clock::time_point start,Clock::time_point finish){
    if (!first) metrics << ','; first=false;
    metrics << "{\"name\":\""<<name<<"\",\"publish_to_receipt_ms\":"
      << std::chrono::duration<double,std::milli>(finish-start).count()
      << ",\"rss_after_kib\":"<<rss_kib()<<'}';
  };
  auto stage=[&](const std::string & name,const std::string & port,const Cloud & msg,const std::string & result){
    clouds.erase(result); auto start=Clock::now(); pubs[port]->publish(msg);
    wait([&]{return clouds.count(result)>0;});
    require(clouds[result]->header==input.header,"stage header mismatch");
    record(name,start,arrived[result]); save(out+"/"+result+".cdr",*clouds[result]);
  };
  stage("velodyne_conversion","raw",input,"extended");
  stage("compact_conversion","compact_input",*clouds.at("extended"),"compact");
  stage("ransac_sensor_z","ground_input",*clouds.at("compact"),"filtered");
  for (const std::string mode : {"filtered","bypass"}) {
    objects.reset(); clouds.erase("debug/clusters");
    const auto & msg=*clouds.at(mode=="filtered" ? "filtered" : "compact");
    auto start=Clock::now(); pubs["cluster_input"]->publish(msg);
    wait([&]{return objects && clouds.count("debug/clusters");});
    require(objects->header==input.header && clouds.at("debug/clusters")->header==input.header,"cluster header mismatch");
    record("clustering_"+mode,start,std::max(arrived["objects"],arrived["debug/clusters"]));
    save(out+"/"+mode+"_objects.cdr",*objects);
    save(out+"/"+mode+"_clusters.cdr",*clouds.at("debug/clusters"));
  }
  rusage usage{};require(getrusage(RUSAGE_SELF,&usage)==0,"resource measurement failed");
  metrics << "],\"process_peak_rss_kib\":"<<usage.ru_maxrss<<"}\n";
  require(bool(metrics),"metrics write failed");
  rclcpp::shutdown(); return 0;
 } catch(const std::exception & ex){std::cerr<<ex.what()<<'\n';return 1;}
}
