#ifndef AURORA_PUBLISHER_NODE_HPP
#define AURORA_PUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <aurora_ndi_ros2_driver/msg/aurora_data.hpp>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <optional>
#include <deque>

// Forward declaration
namespace AuroraDriver {
    class ndi_aurora;
    struct ToolPose;
}

namespace aurora_ndi_ros2_driver
{

class AuroraPublisherNode : public rclcpp::Node
{
public:
    AuroraPublisherNode();
    ~AuroraPublisherNode();

private:
    // =========================================================================
    // PARAMETER STRUCTURES
    // =========================================================================
    
    struct Parameters {
        // Serial communication
        std::string serial_port;
        int baud_rate;
        double serial_timeout_sec;
        
        // Multi-sensor configuration
        int num_sensors;  
        std::vector<std::string> tool_rom_files;
        std::vector<std::string> port_handles;
        std::vector<std::string> port_handles_autoconfig;  
        std::vector<std::string> topic_names;
        std::vector<std::string> child_frame_names;
        
        // Aurora system
        std::string reference_port;
        int track_reply_option;
        std::string measure_reply_option;
        bool status_reply;
        
        // ROS topics
        std::string frame_id;
        double publish_rate_hz;
        int queue_size;
        
        // Data processing
        bool enable_data_filtering;
        int filter_window_size;
        double position_scale_factor;
        double orientation_scale_factor;
        double error_scale_factor;
        
        // Connection and retry
        double command_timeout_sec;
        int max_connection_retries;
        double retry_delay_sec;
        
        // Logging
        bool debug_mode;
        bool log_raw_data;
    };
    
    // =========================================================================
    // DATA STRUCTURES
    // =========================================================================
    
    struct AuroraData {
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
        std::string handle;
        int sensor_index{-1};
        bool visible{false};
        std::array<double, 3> position{{0.0, 0.0, 0.0}};      // x, y, z in mm
        std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};  // quaternion: x, y, z, w
        double error{0.0};
    };
    
    // =========================================================================
    // PARAMETER MANAGEMENT
    // =========================================================================
    
    void declare_parameters();
    void load_parameters();
    bool validate_parameters();
    
    // =========================================================================
    // AURORA SYSTEM MANAGEMENT
    // =========================================================================
    
    bool setup_aurora();
    
    // =========================================================================
    // DATA PROCESSING
    // =========================================================================
    
    std::vector<std::optional<AuroraData>> parse_aurora_data_multi(
        const std::vector<AuroraDriver::ToolPose>& poses,
        const std::vector<int>& port_indexes,
        const std::vector<int>& visible_tools);
    
    AuroraData apply_filtering(const AuroraData& new_data, int sensor_index);
    
    // =========================================================================
    // THREAD FUNCTIONS
    // =========================================================================
    
    void read_thread_function();
    
    // =========================================================================
    // TIMER CALLBACKS
    // =========================================================================
    
    void tf_publish_callback();
    void aurora_data_publish_callback();
    
    // =========================================================================
    // SHUTDOWN
    // =========================================================================
    
    void shutdown();
    
    // =========================================================================
    // MEMBER VARIABLES
    // =========================================================================
    
    // Parameters
    Parameters params_;
    
    // Aurora driver
    std::unique_ptr<AuroraDriver::ndi_aurora> aurora_driver_;
    
    // ROS publishers (one per sensor)
    std::vector<rclcpp::Publisher<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr> aurora_data_publishers_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr aurora_data_timer_;
    
    // Threading
    std::thread read_thread_;
    std::atomic<bool> tracking_active_;
    std::mutex data_mutex_;
    
    // Data storage (one per sensor)
    std::vector<AuroraData> latest_data_;
    std::vector<bool> has_valid_data_;
    std::vector<std::deque<AuroraData>> data_buffers_;  // For filtering, one buffer per sensor
};

} // namespace aurora_ndi_ros2_driver

#endif // AURORA_PUBLISHER_NODE_HPP