#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/traffic_rules/autoware_traffic_rules.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

using Key = std::pair<lanelet::Id, bool>;
static Key key(const lanelet::ConstLanelet & ll) { return {ll.id(), ll.inverted()}; }
static double distance(const lanelet::ConstPoint3d & a, const lanelet::ConstPoint3d & b)
{ return std::hypot(a.x() - b.x(), a.y() - b.y()); }
static double length(const lanelet::ConstLanelet & ll)
{
  double value = 0.0;
  const auto c = ll.centerline();
  for (size_t i = 1; i < c.size(); ++i) value += distance(c[i - 1], c[i]);
  return value;
}
template<class T> static void array(std::ostream & out, const std::vector<T> & values)
{
  out << '[';
  for (size_t i = 0; i < values.size(); ++i) { if (i) out << ','; out << values[i]; }
  out << ']';
}

struct GraphInfo
{
  std::vector<lanelet::ConstLanelet> lanes;
  std::map<Key, int> index;
  std::vector<std::vector<int>> outgoing, incoming, weak, successors;
  std::vector<int> components, component_sizes, scc_ids, scc_sizes;
  size_t successor_edges{0}, lane_change_edges{0}, denied{0};
};

static GraphInfo inspect(
  const lanelet::LaneletMapConstPtr & map,
  const lanelet::routing::RoutingGraphConstPtr & graph,
  const lanelet::traffic_rules::TrafficRulesPtr & rules)
{
  GraphInfo g;
  for (const auto & ll : map->laneletLayer) {
    if (!rules->canPass(ll)) ++g.denied;
    for (auto candidate : {lanelet::ConstLanelet(ll), ll.invert()}) {
      if (rules->canPass(candidate)) { g.index[key(candidate)] = g.lanes.size(); g.lanes.push_back(candidate); }
    }
  }
  const int n = g.lanes.size();
  g.outgoing.resize(n); g.incoming.resize(n); g.weak.resize(n); g.successors.resize(n);
  for (int i = 0; i < n; ++i) {
    for (const auto & target : graph->following(g.lanes[i], true)) {
      auto it = g.index.find(key(target));
      if (it == g.index.end()) continue;
      const int j = it->second;
      g.outgoing[i].push_back(j); g.incoming[j].push_back(i);
      g.weak[i].push_back(j); g.weak[j].push_back(i);
    }
    const auto following = graph->following(g.lanes[i], false);
    for (const auto & target : following) {
      auto it = g.index.find(key(target));
      if (it != g.index.end()) g.successors[i].push_back(it->second);
    }
    g.successor_edges += following.size();
    g.lane_change_edges += g.outgoing[i].size() - following.size();
  }
  g.components.assign(n, -1);
  for (int i = 0; i < n; ++i) {
    if (g.components[i] >= 0) continue;
    const int component = g.component_sizes.size();
    int size = 0;
    std::queue<int> q; q.push(i); g.components[i] = component;
    while (!q.empty()) {
      auto v = q.front(); q.pop(); ++size;
      for (int j : g.weak[v]) if (g.components[j] < 0) { g.components[j] = component; q.push(j); }
    }
    g.component_sizes.push_back(size);
  }
  std::vector<bool> seen(n, false); std::vector<int> order;
  std::function<void(int)> visit = [&](int v) {
    seen[v] = true;
    for (int j : g.outgoing[v]) if (!seen[j]) visit(j);
    order.push_back(v);
  };
  for (int i = 0; i < n; ++i) if (!seen[i]) visit(i);
  std::fill(seen.begin(), seen.end(), false);
  g.scc_ids.assign(n, -1);
  std::function<int(int,int)> collect = [&](int v, int component) {
    seen[v] = true; g.scc_ids[v] = component; int size = 1;
    for (int j : g.incoming[v]) if (!seen[j]) size += collect(j, component);
    return size;
  };
  int scc_id = 0;
  for (auto it = order.rbegin(); it != order.rend(); ++it) if (!seen[*it]) {
    g.scc_sizes.push_back(collect(*it, scc_id)); ++scc_id;
  }
  return g;
}

class Probe : public rclcpp::Node
{
public:
  Probe() : Node("map_routing_probe")
  {
    dir_ = declare_parameter<std::string>("results_dir", "/tmp/atlas_map_demo_results");
    std::filesystem::create_directories(dir_);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    route_pub_ = create_publisher<autoware_planning_msgs::msg::LaneletRoute>("route", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("route_path", qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("diagnostic_markers", qos);
    map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "vector_map", qos, [this](autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
        if (map_ready_ || failed_) return;
        map_sec_ = elapsed(); map_bytes_ = msg->data.size();
        try {
          if (msg->header.frame_id != "map" || msg->data.empty()) throw std::runtime_error("invalid LaneletMapBin frame or payload");
          validate_route(*msg); map_ready_ = true; write_status();
        } catch (const std::exception & e) { fail(e.what()); }
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_map", qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (cloud_ready_ || failed_) return;
        cloud_sec_ = elapsed(); cloud_points_ = static_cast<uint64_t>(msg->width) * msg->height;
        cloud_bytes_ = msg->data.size();
        if (msg->header.frame_id != "map" || cloud_points_ != 1004344 || msg->data.empty()) {
          fail("pointcloud map frame, point count, or payload invalid"); return;
        }
        bool x=false,y=false,z=false;
        for (auto & f : msg->fields) { x |= f.name=="x"; y |= f.name=="y"; z |= f.name=="z"; }
        if (!x || !y || !z) { fail("pointcloud lacks XYZ fields"); return; }
        cloud_ready_ = true; write_status();
      });
    projector_sub_ = create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
      "/map/map_projector_info", qos, [this](autoware_map_msgs::msg::MapProjectorInfo::ConstSharedPtr msg) {
        if (projector_ready_ || failed_) return;
        projector_sec_ = elapsed();
        if (msg->projector_type != "LocalCartesian" || msg->vertical_datum != "WGS84" ||
            std::abs(msg->map_origin.latitude - 51.08080935188234) > 1e-10 ||
            std::abs(msg->map_origin.longitude + 114.1331143659806) > 1e-10 || msg->map_origin.altitude != 0.0) {
          fail("projector publication differs from supplied LocalCartesian configuration"); return;
        }
        projector_ready_ = true; write_status();
      });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (!failed_ && elapsed() > 60 && !(map_ready_ && cloud_ready_ && projector_ready_)) fail("map publication timeout");
      if (map_ready_ && cloud_ready_ && projector_ready_ && !logged_) {
        logged_ = true;
        RCLCPP_INFO(get_logger(), "PASS: original map publications, Autoware routing, path, and diagnostics validated");
      }
    });
  }
private:
  double elapsed() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count(); }
  void fail(const std::string & why) {
    failed_ = true; RCLCPP_ERROR(get_logger(), "FAIL: %s", why.c_str());
    std::ofstream(dir_ + "/failure.txt") << why << '\n'; write_status();
  }
  void write_status() {
    std::ofstream out(dir_ + "/publications.json.tmp"); out << std::boolalpha << std::setprecision(12)
      << "{\"map_ready\":" << map_ready_ << ",\"cloud_ready\":" << cloud_ready_
      << ",\"projector_ready\":" << projector_ready_ << ",\"failed\":" << failed_
      << ",\"map_bytes\":" << map_bytes_ << ",\"pcd_points\":" << cloud_points_
      << ",\"pcd_bytes\":" << cloud_bytes_ << ",\"map_received_sec\":" << map_sec_
      << ",\"pcd_received_sec\":" << cloud_sec_ << ",\"projector_received_sec\":" << projector_sec_ << "}\n";
    out.close(); std::filesystem::rename(dir_ + "/publications.json.tmp", dir_ + "/publications.json");
  }
  void validate_route(const autoware_map_msgs::msg::LaneletMapBin & msg) {
    const auto begin = std::chrono::steady_clock::now();
    const auto map = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg);
    const auto [graph, rules] = autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      map, lanelet::autoware::DefaultLocation, lanelet::Participants::Vehicle);
    auto g = inspect(map, graph, rules);
    const auto errors = graph->checkValidity(false);
    if (g.lanes.empty() || !errors.empty()) throw std::runtime_error("Autoware routing graph empty or invalid");
    const int largest = std::max_element(g.component_sizes.begin(), g.component_sizes.end()) - g.component_sizes.begin();
    std::vector<int> candidates;
    for (size_t i=0; i<g.lanes.size(); ++i) if (g.components[i]==largest) candidates.push_back(i);
    std::sort(candidates.begin(), candidates.end(), [&](int a,int b) {
      const auto ca=g.lanes[a].centerline().front(), cb=g.lanes[b].centerline().front();
      const auto da=std::hypot(ca.x(),ca.y()), db=std::hypot(cb.x(),cb.y());
      return da==db ? key(g.lanes[a])<key(g.lanes[b]) : da<db;
    });
    lanelet::Optional<lanelet::routing::Route> route;
    int selected_start=-1, selected_goal=-1;
    for (int start : candidates) {
      // Select a reachable destination about 200 m along successor edges.
      std::map<int,double> cost; cost[start]=length(g.lanes[start]);
      std::queue<int> q; q.push(start); int goal=-1; double best=1e9;
      while (!q.empty()) {
        int v=q.front(); q.pop();
        if (v!=start && cost[v] > 50 && cost[v] < 400 && std::abs(cost[v]-200)<best) {goal=v;best=std::abs(cost[v]-200);}
        if (cost[v]>450) continue;
        for (auto & next : graph->following(g.lanes[v],false)) {
          auto found=g.index.find(key(next));
          if (found==g.index.end() || g.components[found->second]!=largest || cost.count(found->second)) continue;
          cost[found->second]=cost[v]+length(next); q.push(found->second);
        }
      }
      if (goal<0) continue;
      route = graph->getRoute(g.lanes[start],g.lanes[goal],0,false);
      if (route) { selected_start=start; selected_goal=goal; break; }
    }
    if (!route) throw std::runtime_error("no successor-only route longer than 50 m in largest component");
    const auto path = route->shortestPath();
    if (path.size()<2) throw std::runtime_error("route does not contain distinct lanelets");
    nav_msgs::msg::Path ros_path; ros_path.header.frame_id="map"; ros_path.header.stamp=now();
    autoware_planning_msgs::msg::LaneletRoute ros_route; ros_route.header=ros_path.header;
    ros_route.uuid.uuid[0]=0x41; ros_route.uuid.uuid[1]=0x54; ros_route.allow_modification=false;
    double total=0,max_gap=0; std::vector<lanelet::Id> route_ids;
    for (size_t i=0;i<path.size();++i) {
      const auto ll=path[i]; route_ids.push_back(ll.id()); total+=length(ll);
      if (!rules->canPass(ll) || g.components[g.index.at(key(ll))]!=largest) throw std::runtime_error("route leaves allowed largest component");
      if (i) {
        const auto followers=graph->following(path[i-1],false), previous=graph->previous(ll,false);
        auto has=[](const auto & lanes,const auto & value) { return std::any_of(lanes.begin(),lanes.end(),[&](const auto & x){return key(x)==key(value);}); };
        if (!has(followers,ll) || !has(previous,path[i-1]) || !rules->canPass(path[i-1],ll)) throw std::runtime_error("invalid predecessor/successor transition");
        max_gap=std::max(max_gap,distance(path[i-1].centerline().back(),ll.centerline().front()));
      }
      autoware_planning_msgs::msg::LaneletSegment segment;
      segment.preferred_primitive.id=ll.id(); segment.preferred_primitive.primitive_type="lane";
      segment.primitives.push_back(segment.preferred_primitive); ros_route.segments.push_back(segment);
      for (const auto & pt : ll.centerline()) {
        geometry_msgs::msg::PoseStamped pose; pose.header=ros_path.header;
        pose.pose.position=autoware::experimental::lanelet2_utils::to_ros(pt); pose.pose.orientation.w=1;
        ros_path.poses.push_back(pose);
      }
    }
    if (max_gap>0.05 || total<50 || total>500 || !std::isfinite(total)) throw std::runtime_error("route length or centerline continuity implausible");
    for (size_t i=0;i<ros_path.poses.size();++i) {
      const auto & a=ros_path.poses[i?i-1:i].pose.position;
      const auto & b=ros_path.poses[std::min(i+1,ros_path.poses.size()-1)].pose.position;
      double yaw=std::atan2(b.y-a.y,b.x-a.x);
      ros_path.poses[i].pose.orientation.z=std::sin(yaw/2); ros_path.poses[i].pose.orientation.w=std::cos(yaw/2);
    }
    ros_route.start_pose=ros_path.poses.front().pose; ros_route.goal_pose=ros_path.poses.back().pose;
    path_pub_->publish(ros_path); route_pub_->publish(ros_route);
    publish_diagnostics(ros_route);
    std::ofstream centers(dir_+"/centerlines.csv"); centers<<"lanelet_id,index,x,y,z,component\n"<<std::setprecision(12);
    for (size_t i=0;i<g.lanes.size();++i) {
      int j=0; for(const auto & pt:g.lanes[i].centerline()) centers<<g.lanes[i].id()<<','<<j++<<','<<pt.x()<<','<<pt.y()<<','<<pt.z()<<','<<g.components[i]<<'\n';
    }
    std::ofstream directed(dir_+"/directed_lanelets.csv");
    directed<<"node,id,inverted,weak_component,strong_component,point_index,x,y,z\n"<<std::setprecision(12);
    for (size_t i=0;i<g.lanes.size();++i) {
      size_t point_index=0;
      for (const auto & pt:g.lanes[i].centerline())
        directed<<i<<','<<g.lanes[i].id()<<','<<(g.lanes[i].inverted()?1:0)<<','<<g.components[i]<<','<<g.scc_ids[i]<<','<<point_index++<<','<<pt.x()<<','<<pt.y()<<','<<pt.z()<<'\n';
    }
    std::ofstream edges(dir_+"/successor_edges.csv"); edges<<"from_node,to_node\n";
    for (size_t i=0;i<g.successors.size();++i) for (int j:g.successors[i]) edges<<i<<','<<j<<'\n';
    std::ofstream boundaries(dir_+"/boundaries.csv"); boundaries<<"way_id,index,x,y,z\n"<<std::setprecision(12);
    std::set<lanelet::Id> used;
    for (const auto & ll:map->laneletLayer) for (const auto & bound:{ll.leftBound(),ll.rightBound()}) {
      if (!used.insert(bound.id()).second) continue;
      int j=0; for (const auto & pt:bound) boundaries<<bound.id()<<','<<j++<<','<<pt.x()<<','<<pt.y()<<','<<pt.z()<<'\n';
    }
    std::ofstream routefile(dir_+"/route.csv"); routefile<<"x,y,z\n"<<std::setprecision(12);
    for (auto & pose:ros_path.poses) routefile<<pose.pose.position.x<<','<<pose.pose.position.y<<','<<pose.pose.position.z<<'\n';
    const auto [other_graph,other_rules]=autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(map,lanelet::Locations::Germany,lanelet::Participants::Vehicle);
    auto other=inspect(map,other_graph,other_rules);
    std::ofstream out(dir_+"/routing.json");out<<std::setprecision(12);
    out<<"{\"rules\":\""<<rules->location()<<"/"<<rules->participant()<<"\",\"map_lanelets\":"<<map->laneletLayer.size()
       <<",\"routable_directed_vertices\":"<<g.lanes.size()<<",\"denied_original_lanelets\":"<<g.denied
       <<",\"successor_edges\":"<<g.successor_edges<<",\"lane_change_edges\":"<<g.lane_change_edges
       <<",\"weak_components\":"<<g.component_sizes.size()<<",\"largest_weak_component\":"<<g.component_sizes[largest]
       <<",\"strong_components\":"<<g.scc_sizes.size()<<",\"largest_strong_component\":"<<*std::max_element(g.scc_sizes.begin(),g.scc_sizes.end())
       <<",\"weak_component_sizes\":";array(out,g.component_sizes);out<<",\"strong_component_sizes\":";array(out,g.scc_sizes);
    out<<",\"route_lanelet_ids\":";array(out,route_ids);
    out<<",\"route_lanelets\":"<<path.size()<<",\"route_length_m\":"<<total<<",\"max_transition_gap_m\":"<<max_gap
       <<",\"start_id\":"<<g.lanes[selected_start].id()<<",\"goal_id\":"<<g.lanes[selected_goal].id()
       <<",\"start_inverted\":"<<(g.lanes[selected_start].inverted()?"true":"false")
       <<",\"goal_inverted\":"<<(g.lanes[selected_goal].inverted()?"true":"false")
       <<",\"route_points\":"<<ros_path.poses.size()<<",\"graph_validity_errors\":"<<errors.size()
       <<",\"germany_vertices\":"<<other.lanes.size()<<",\"germany_weak_components\":"<<other.component_sizes.size()
       <<",\"germany_successor_edges\":"<<other.successor_edges<<",\"germany_lane_change_edges\":"<<other.lane_change_edges
       <<",\"routing_compute_sec\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<"}\n";
    RCLCPP_INFO(get_logger(),"ROUTE: %zu lanelets, %.3f m, max join gap %.6f m; graph %zu vertices, %zu weak components, largest %d",
      path.size(),total,max_gap,g.lanes.size(),g.component_sizes.size(),g.component_sizes[largest]);
  }
  void publish_diagnostics(const autoware_planning_msgs::msg::LaneletRoute & route) {
    visualization_msgs::msg::MarkerArray markers;
    for (int i=0;i<2;++i) {
      visualization_msgs::msg::Marker m; m.header=route.header;m.ns="endpoints";m.id=i;m.type=m.SPHERE;m.action=m.ADD;
      m.pose=i?route.goal_pose:route.start_pose;m.scale.x=m.scale.y=m.scale.z=4;m.color.a=1;m.color.r=i?1:0;m.color.g=i?0:1;
      markers.markers.push_back(m);
      m.ns="labels";m.type=m.TEXT_VIEW_FACING;m.pose.position.z+=5;m.scale.z=3;m.color.r=m.color.g=m.color.b=1;m.text=i?"Destination":"Start";markers.markers.push_back(m);
    }
    for (int i=0;i<3;++i) {
      visualization_msgs::msg::Marker m;m.header=route.header;m.ns="map_axes";m.id=i;m.type=m.ARROW;m.action=m.ADD;m.pose.orientation.w=1;
      geometry_msgs::msg::Point a,b; if(i==0)b.x=15;if(i==1)b.y=15;if(i==2)b.z=15;m.points={a,b};
      m.scale.x=.4;m.scale.y=.8;m.scale.z=1.2;m.color.a=1;m.color.r=i==0;m.color.g=i==1;m.color.b=i==2;markers.markers.push_back(m);
    }
    markers_pub_->publish(markers);
  }
  std::string dir_; bool map_ready_{false},cloud_ready_{false},projector_ready_{false},failed_{false},logged_{false};
  double map_sec_{0},cloud_sec_{0},projector_sec_{0};size_t map_bytes_{0},cloud_bytes_{0};uint64_t cloud_points_{0};
  std::chrono::steady_clock::time_point start_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr projector_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<Probe>()); }
  catch(const std::exception & e) { std::cerr<<"FAIL: "<<e.what()<<'\n';return 1; }
  rclcpp::shutdown();return 0;
}
