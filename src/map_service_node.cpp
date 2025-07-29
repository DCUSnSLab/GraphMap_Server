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
                           response->nodes.poses.size(), response->link_from_node_indices.size());
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
            response->node_ids.clear();
            response->link_from_node_indices.clear();
            response->link_to_node_indices.clear();
            response->nodes.header.frame_id = "map";
            response->nodes.header.stamp = this->get_clock()->now();
            
            // Store node positions and IDs for link processing
            std::unordered_map<std::string, size_t> node_id_to_index;
            std::vector<std::string> node_id_list;
            
            // Parse Node array from JSON
            if (map_json.contains("Node") && map_json["Node"].is_array()) {
                size_t node_index = 0;
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
                            
                            // Store node data
                            response->nodes.poses.push_back(pose);
                            response->node_ids.push_back(node_id);
                            node_id_to_index[node_id] = node_index;
                            node_id_list.push_back(node_id);
                            node_index++;
                        }
                    }
                }
            }
            
            // Parse Link array from JSON to create connectivity information
            if (map_json.contains("Link") && map_json["Link"].is_array()) {
                for (const auto& link : map_json["Link"]) {
                    if (link.contains("FromNodeID") && link.contains("ToNodeID")) {
                        std::string from_node_id = link["FromNodeID"];
                        std::string to_node_id = link["ToNodeID"];
                        
                        // Find corresponding node indices
                        auto from_it = node_id_to_index.find(from_node_id);
                        auto to_it = node_id_to_index.find(to_node_id);
                        
                        if (from_it != node_id_to_index.end() && to_it != node_id_to_index.end()) {
                            // Store connectivity as indices
                            response->link_from_node_indices.push_back(static_cast<int32_t>(from_it->second));
                            response->link_to_node_indices.push_back(static_cast<int32_t>(to_it->second));
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Link references unknown nodes: %s -> %s", 
                                       from_node_id.c_str(), to_node_id.c_str());
                        }
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Parsed %zu nodes and %zu links from JSON", 
                       response->nodes.poses.size(), response->link_from_node_indices.size());
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