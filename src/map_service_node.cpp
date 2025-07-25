#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gmserver/srv/load_map.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

class MapServiceNode : public rclcpp::Node
{
public:
    MapServiceNode() : Node("map_service_node")
    {
        // Create service for loading map data
        service_ = this->create_service<gmserver::srv::LoadMap>(
            "load_map",
            std::bind(&MapServiceNode::handleLoadMapService, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Map service node initialized. Service '/load_map' is ready.");
    }

private:
    void handleLoadMapService(
        const std::shared_ptr<gmserver::srv::LoadMap::Request> request,
        std::shared_ptr<gmserver::srv::LoadMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Loading map from: %s", request->map_file_path.c_str());
        
        try {
            if (loadMapFromJSON(request->map_file_path, response)) {
                response->success = true;
                response->message = "Map loaded successfully";
                RCLCPP_INFO(this->get_logger(), "Map loaded successfully: %zu nodes, %zu links", 
                           response->nodes.poses.size(), response->links.poses.size());
            } else {
                response->success = false;
                response->message = "Failed to load map file";
                RCLCPP_ERROR(this->get_logger(), "Failed to load map from: %s", 
                            request->map_file_path.c_str());
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception while loading map: %s", e.what());
        }
    }
    
    bool loadMapFromJSON(const std::string& file_path, 
                        std::shared_ptr<gmserver::srv::LoadMap::Response> response)
    {
        try {
            std::ifstream file(file_path);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", file_path.c_str());
                return false;
            }
            
            json map_json;
            file >> map_json;
            
            // Initialize response arrays
            response->nodes.poses.clear();
            response->links.poses.clear();
            response->nodes.header.frame_id = "map";
            response->links.header.frame_id = "map";
            response->nodes.header.stamp = this->get_clock()->now();
            response->links.header.stamp = this->get_clock()->now();
            
            // Store node positions for link processing
            std::unordered_map<std::string, geometry_msgs::msg::Pose> node_positions;
            
            // Parse Node array from JSON
            if (map_json.contains("Node") && map_json["Node"].is_array()) {
                for (const auto& node : map_json["Node"]) {
                    geometry_msgs::msg::Pose pose;
                    
                    if (node.contains("GpsInfo") && node.contains("ID")) {
                        const auto& gps = node["GpsInfo"];
                        std::string node_id = node["ID"];
                        
                        if (gps.contains("Lat") && gps.contains("Long") && gps.contains("Alt")) {
                            // Use UTM coordinates if available, otherwise convert GPS
                            if (node.contains("UtmInfo")) {
                                const auto& utm = node["UtmInfo"];
                                if (utm.contains("Easting") && utm.contains("Northing")) {
                                    pose.position.x = utm["Easting"].get<double>();
                                    pose.position.y = utm["Northing"].get<double>();
                                    pose.position.z = gps["Alt"].get<double>();
                                }
                            } else {
                                // Simple GPS to local coordinate conversion
                                pose.position.x = gps["Long"].get<double>() * 111320.0;
                                pose.position.y = gps["Lat"].get<double>() * 110540.0;
                                pose.position.z = gps["Alt"].get<double>();
                            }
                            
                            // Set orientation (no rotation for nodes)
                            pose.orientation.x = 0.0;
                            pose.orientation.y = 0.0;
                            pose.orientation.z = 0.0;
                            pose.orientation.w = 1.0;
                            
                            response->nodes.poses.push_back(pose);
                            node_positions[node_id] = pose;
                        }
                    }
                }
            }
            
            // Parse Link array from JSON
            if (map_json.contains("Link") && map_json["Link"].is_array()) {
                for (const auto& link : map_json["Link"]) {
                    if (link.contains("FromNodeID") && link.contains("ToNodeID")) {
                        std::string from_node_id = link["FromNodeID"];
                        std::string to_node_id = link["ToNodeID"];
                        
                        // Find corresponding nodes
                        auto from_it = node_positions.find(from_node_id);
                        auto to_it = node_positions.find(to_node_id);
                        
                        if (from_it != node_positions.end() && to_it != node_positions.end()) {
                            geometry_msgs::msg::Pose link_pose;
                            
                            // Set link position as midpoint between from and to nodes
                            link_pose.position.x = (from_it->second.position.x + to_it->second.position.x) / 2.0;
                            link_pose.position.y = (from_it->second.position.y + to_it->second.position.y) / 2.0;
                            link_pose.position.z = (from_it->second.position.z + to_it->second.position.z) / 2.0;
                            
                            // Calculate orientation from from_node to to_node
                            double dx = to_it->second.position.x - from_it->second.position.x;
                            double dy = to_it->second.position.y - from_it->second.position.y;
                            double yaw = atan2(dy, dx);
                            
                            // Convert yaw to quaternion
                            link_pose.orientation.x = 0.0;
                            link_pose.orientation.y = 0.0;
                            link_pose.orientation.z = sin(yaw / 2.0);
                            link_pose.orientation.w = cos(yaw / 2.0);
                            
                            response->links.poses.push_back(link_pose);
                        }
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Parsed %zu nodes and %zu links from JSON", 
                       response->nodes.poses.size(), response->links.poses.size());
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON: %s", e.what());
            return false;
        }
    }
    
    rclcpp::Service<gmserver::srv::LoadMap>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}