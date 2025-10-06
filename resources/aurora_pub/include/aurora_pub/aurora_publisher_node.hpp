#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/header.hpp>
#include "aurora_pub/ndi_aurora_ros2.hpp"
#include "aurora_pub/msg/aurora_data.hpp"

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <cstdint>

namespace aurora_pub
{

/**
 * @brief Aurora NDI Publisher Node for ROS2
 * 
 * This class provides direct communication with Aurora NDI tracking system
 * via the ndi_aurora driver, publishing TF transforms instead of pose topics.
 * 
 * Features:
 * - Direct Aurora communication using ndi_aurora driver
 * - TF broadcasting: aurora_base -> endo_aurora
 * - Configurable via YAML parameters
 * - Data filtering and averaging
 * - Robust error handling and reconnection
 */
class AuroraPublisherNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes node and Aurora communication
     */
    AuroraPublisherNode();

    /**
     * @brief Destructor - cleanly shuts down Aurora and threads
     */
    ~AuroraPublisherNode();

private:
    /**
     * @brief Configuration parameters structure
     */
    struct Parameters {
        // Serial communication settings
        std::string serial_port;           ///< Aurora device serial port (/dev/ttyUSB0)
        int baud_rate;                     ///< Communication baud rate (230400)
        double serial_timeout_sec;         ///< Serial operation timeout

        // Aurora system configuration
        std::string tool_handle;           ///< Primary tool handle ("0A")
        std::vector<std::string> tool_rom_files;  ///< ROM file paths
        std::vector<std::string> port_handles;    ///< Enabled port handles
        std::string reference_port;        ///< Reference tool port ("10")
        int track_reply_option;            ///< Track reply option (80)
        std::string measure_reply_option;  ///< Measure reply option ("0001")
        bool status_reply;                 ///< Include status in replies

        // TF and ROS configuration           
        std::string child_frame_name;      ///< Child frame name ("endo_aurora")
        std::string topic_aurora_name;     ///< Aurora data topic ("/aurora_data")
        std::string frame_id;              ///< Parent frame ID ("aurora_base")
        double publish_rate_hz;            ///< Publishing frequency (40.0)
        int queue_size;                    ///< Publisher queue size

        // Data processing settings
        bool enable_data_filtering;        ///< Enable moving average filter
        int filter_window_size;            ///< Filter window size (4)
        double position_scale_factor;      ///< Position scaling (1.0)
        double orientation_scale_factor;   ///< Orientation scaling (1.0)
        double error_scale_factor;         ///< Error scaling (1.0)

        // Connection and retry settings
        double command_timeout_sec;        ///< Aurora command timeout
        int max_connection_retries;        ///< Max serial connection retries
        double retry_delay_sec;            ///< Delay between retries

        // Logging configuration
        bool debug_mode;                   ///< Enable debug logging
        bool log_raw_data;                 ///< Log raw Aurora data strings
    };

    /**
     * @brief Aurora sensor data structure
     */
    struct AuroraData {
        std::string handle;                ///< Tool handle ("0A", "0B", etc.)
        double position[3];                ///< Position [x, y, z] in mm
        double orientation[4];             ///< Quaternion [qx, qy, qz, qw]
        bool visible;                      ///< Tool visibility status
        double error;                      ///< RMS error in mm
        std::chrono::steady_clock::time_point timestamp;  ///< Data timestamp
    };

    // ==========================================================================
    // INITIALIZATION AND SETUP METHODS
    // ==========================================================================

    /**
     * @brief Declare all ROS parameters with defaults
     */
    void declare_parameters();

    /**
     * @brief Load parameters from ROS parameter server
     */
    void load_parameters();

    /**
     * @brief Validate loaded parameters for correctness
     * @return true if all parameters are valid
     */
    bool validate_parameters();

    /**
     * @brief Clean shutdown of Aurora system and threads
     */
    void shutdown();

    // ==========================================================================
    // AURORA SYSTEM METHODS (using ndi_aurora driver)
    // ==========================================================================

    /**
     * @brief Setup Aurora system using ndi_aurora driver
     * @return true if setup successful
     */
    bool setup_aurora();

    /**
     * @brief Parse Aurora data from driver structures
     * @param poses Vector of ToolPose from driver
     * @param port_indexes Vector of port indexes from driver
     * @param visible_tools Vector of visible tool indexes from driver
     * @return Parsed Aurora data if successful, nullopt otherwise
     */
    std::optional<AuroraData> parse_aurora_data(
        const std::vector<AuroraDriver::ToolPose>& poses, 
        const std::vector<int>& port_indexes, 
        const std::vector<int>& visible_tools);

    // ==========================================================================
    // DATA PROCESSING METHODS
    // ==========================================================================

    /**
     * @brief Apply data filtering (moving average)
     * @param new_data New Aurora data point
     * @return Filtered Aurora data
     */
    AuroraData apply_filtering(const AuroraData& new_data);

    // ==========================================================================
    // THREAD FUNCTIONS
    // ==========================================================================

    /**
     * @brief Background thread function for reading Aurora data
     * Continuously calls measureTool and parses responses using ndi_aurora driver
     */
    void read_thread_function();

    /**
     * @brief Timer callback for publishing TF transforms
     * Called at the configured publish rate
     */
    void tf_publish_callback();       
    
    /**
     * @brief Timer callback for publishing aurora data messages
     */
    void aurora_data_publish_callback();

    // ==========================================================================
    // MEMBER VARIABLES
    // ==========================================================================

    /// Configuration parameters
    Parameters params_;

    /// TF broadcaster for aurora_base -> endo_aurora
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// ROS2 publisher for aurora diagnostic data
    rclcpp::Publisher<aurora_pub::msg::AuroraData>::SharedPtr aurora_data_publisher_;

    /// Timers for periodic publishing
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr aurora_data_timer_;

    /// Aurora driver instance
    std::unique_ptr<AuroraDriver::ndi_aurora> aurora_driver_;

    /// Latest valid Aurora data
    AuroraData latest_data_;

    /// Mutex for thread-safe data access
    std::mutex data_mutex_;

    /// Flag for active tracking
    std::atomic<bool> tracking_active_;

    /// Flag for valid data availability
    std::atomic<bool> has_valid_data_;

    /// Background thread for reading Aurora data
    std::thread read_thread_;

    /// Data buffer for filtering
    std::vector<AuroraData> data_buffer_;
};

} // namespace aurora_pub