#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware_lanelet2_extension/traffic_rules/autoware_traffic_rules.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

class Fixture final : public rclcpp::Node
{
public:
  Fixture() : Node("route_adapter")
  {
    dir_=declare_parameter<std::string>("results_dir","/tmp/atlas_closed_loop_results");
    start_id_=declare_parameter<int64_t>("start_lanelet_id",38184);
    goal_id_=declare_parameter<int64_t>("goal_lanelet_id",23892);
    std::filesystem::create_directories(dir_);
    const auto latched=rclcpp::QoS(1).reliable().transient_local();
    route_pub_=create_publisher<autoware_planning_msgs::msg::LaneletRoute>("route",latched);
    route_path_pub_=create_publisher<nav_msgs::msg::Path>("route_path",latched);
    path_pub_=create_publisher<autoware_planning_msgs::msg::Path>("path",1);
    trajectory_path_pub_=create_publisher<nav_msgs::msg::Path>("trajectory_visualization",1);
    markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("markers",1);
    map_sub_=create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "/atlas_map_demo/vector_map",latched,[this](autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
        if (ready_) return;
        try { select_route(*msg); ready_=true; }
        catch(const std::exception & e) {
          RCLCPP_ERROR(get_logger(),"FAIL: route selection: %s",e.what());
          std::ofstream(dir_+"/failure.txt")<<e.what()<<'\n';
        }
      });
    path_sub_=create_subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>(
      "path_with_lane_id",1,[this](autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg) {
        // This pinned conversion copies every point, bound, header and velocity;
        // it removes lane IDs only. No custom path/trajectory/profile generation.
        path_pub_->publish(autoware::motion_utils::convertToPath(*msg));
      });
    trajectory_sub_=create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "trajectory",1,[this](autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
        nav_msgs::msg::Path view;view.header=msg->header;
        for(const auto & p:msg->points) {
          geometry_msgs::msg::PoseStamped pose;pose.header=msg->header;pose.pose=p.pose;view.poses.push_back(pose);
        }
        trajectory_path_pub_->publish(view);
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){if(ready_)publish_markers();});
  }
private:
  void select_route(const autoware_map_msgs::msg::LaneletMapBin & msg) {
    if(msg.header.frame_id!="map" || msg.data.empty())throw std::runtime_error("invalid loaded map");
    const auto map=autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg);
    const auto [graph,rules]=autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      map,lanelet::autoware::DefaultLocation,lanelet::Participants::Vehicle);
    const auto computed=graph->getRoute(map->laneletLayer.get(start_id_),map->laneletLayer.get(goal_id_),0,false);
    if(!computed)throw std::runtime_error("no actual route between selected lanelets");
    const auto lanes=computed->shortestPath();
    if(lanes.size()<2 || lanes.front().id()!=start_id_ || lanes.back().id()!=goal_id_)
      throw std::runtime_error("route does not contain requested start and goal lanelets");
    auto has=[](const auto & v,lanelet::Id id){return std::any_of(v.begin(),v.end(),[&](const auto & l){return l.id()==id;});};
    double gap=0.0;
    for(size_t i=1;i<lanes.size();++i) {
      const auto followers=graph->following(lanes[i-1],false), previous=graph->previous(lanes[i],false);
      if(!has(followers,lanes[i].id())||!has(previous,lanes[i-1].id())||!rules->canPass(lanes[i-1],lanes[i]))
        throw std::runtime_error("invalid successor transition");
      const auto a=lanes[i-1].centerline().back(),b=lanes[i].centerline().front();
      const double join=std::hypot(a.x()-b.x(),a.y()-b.y());gap=std::max(gap,join);
      if(join>0.05)throw std::runtime_error("route centerline join gap");
    }
    nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=now();
    double length=0;
    for(const auto & l:lanes) {
      autoware_planning_msgs::msg::LaneletSegment segment;
      segment.preferred_primitive.id=l.id();segment.preferred_primitive.primitive_type="lane";
      segment.primitives.push_back(segment.preferred_primitive);route_.segments.push_back(segment);
      const auto c=l.centerline();
      for(size_t i=0;i<c.size();++i) {
        const auto & p=c[i];
        if(p.x()<-79.563538 || p.x()>122.839691 || p.y()<-76.59005 || p.y()>119.276711)
          throw std::runtime_error("chosen route outside PCD XY bbox");
        if(i)length+=std::hypot(p.x()-c[i-1].x(),p.y()-c[i-1].y());
        geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position=autoware::experimental::lanelet2_utils::to_ros(p);pose.pose.orientation.w=1;
        if(path.poses.empty()||std::hypot(p.x()-path.poses.back().pose.position.x,p.y()-path.poses.back().pose.position.y)>1e-6)
          path.poses.push_back(pose);
      }
    }
    for(size_t i=0;i<path.poses.size();++i) {
      const auto & a=path.poses[i?i-1:0].pose.position;
      const auto & b=path.poses[std::min(i+1,path.poses.size()-1)].pose.position;
      const double yaw=std::atan2(b.y-a.y,b.x-a.x);
      path.poses[i].pose.orientation.z=std::sin(yaw/2);path.poses[i].pose.orientation.w=std::cos(yaw/2);
    }
    route_.header=path.header;route_.start_pose=path.poses.front().pose;route_.goal_pose=path.poses.back().pose;
    route_.uuid.uuid[0]=0x54;route_.uuid.uuid[1]=0x47;route_.allow_modification=false;
    route_pub_->publish(route_);route_path_pub_->publish(path);
    std::ofstream out(dir_+"/selected_route.json");out<<std::setprecision(12)
      <<"{\"start_lanelet_id\":"<<start_id_<<",\"goal_lanelet_id\":"<<goal_id_
      <<",\"lanelet_count\":"<<lanes.size()<<",\"route_length_m\":"<<length
      <<",\"maximum_transition_gap_m\":"<<gap<<",\"frame\":\"map\",\"traffic_rules\":\"autoware/vehicle\"}\n";
    RCLCPP_INFO(get_logger(),"Selected Lanelet2 route %lld -> %lld across %zu lanelets: %.3f m, max join %.4f m",
      static_cast<long long>(start_id_),static_cast<long long>(goal_id_),lanes.size(),length,gap);
  }
  void publish_markers() {
    nav_msgs::msg::Odometry odom;odom.header.frame_id="map";odom.header.stamp=now();
    visualization_msgs::msg::MarkerArray markers;
    for(int i=0;i<2;++i) {
      visualization_msgs::msg::Marker m;m.header=odom.header;m.ns="route_endpoints";m.id=i;m.type=m.SPHERE;m.action=m.ADD;
      m.pose=i==1?route_.goal_pose:route_.start_pose;m.scale.x=m.scale.y=m.scale.z=2.;
      m.color.a=1.;m.color.r=i!=0;m.color.g=i!=1;m.color.b=0.;markers.markers.push_back(m);
      m.type=m.TEXT_VIEW_FACING;m.ns="labels";m.pose.position.z+=3.;m.scale.z=2.;m.color.r=m.color.g=m.color.b=1.;
      m.text=i==0?"Route start":"Terminal goal";markers.markers.push_back(m);
    }
    markers_pub_->publish(markers);
  }
  std::string dir_;int64_t start_id_{38184},goal_id_{23892};bool ready_{false};autoware_planning_msgs::msg::LaneletRoute route_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr path_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_path_pub_,trajectory_path_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Fixture>());rclcpp::shutdown();return 0;
}
