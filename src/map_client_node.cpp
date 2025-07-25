#include <rclcpp/rclcpp.hpp>
#include <gmserver/srv/load_map.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class MapClient : public rclcpp::Node
{
public:
    MapClient() : Node("map_client_node")
    {
        // Declare parameter for map file path
        this->declare_parameter<std::string>("map_file_path", "");
        
        // Get map file path parameter
        std::string map_file_path;
        this->get_parameter("map_file_path", map_file_path);
        
        if (map_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Map file path parameter is required!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Map Client initialized with file: %s", map_file_path.c_str());
        
        // Create service client
        client_ = this->create_client<gmserver::srv::LoadMap>("load_map");
        
        // Create publishers for visualization
        nodes_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("map_nodes", 10);
        links_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("map_links", 10);
        
        // Store map file path as member variable
        map_file_path_ = map_file_path;
        
        // Wait for service and call it
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&MapClient::timer_callback, this));
    }

private:
    void timer_callback()
    {
        call_service(map_file_path_);
    }
    
    void call_service(const std::string& map_file_path)
    {
        // Stop timer after first call
        timer_->cancel();
        
        if (!client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
            return;
        }

        auto request = std::make_shared<gmserver::srv::LoadMap::Request>();
        request->map_file_path = map_file_path;

        RCLCPP_INFO(this->get_logger(), "Calling map service with file: %s", map_file_path.c_str());

        auto result = client_->async_send_request(request);
        
        // Wait for result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Map loaded successfully!");
                RCLCPP_INFO(this->get_logger(), "Nodes: %zu, Links: %zu", 
                           response->nodes.poses.size(), response->links.poses.size());
                
                // Publish map data for visualization
                nodes_publisher_->publish(response->nodes);
                links_publisher_->publish(response->links);
                
                RCLCPP_INFO(this->get_logger(), "Map data published to /map_nodes and /map_links topics");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

    rclcpp::Client<gmserver::srv::LoadMap>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr nodes_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr links_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string map_file_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}