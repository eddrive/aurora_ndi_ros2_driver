#include "aurora_pub/aurora_publisher_node.hpp"
#include "aurora_pub/aurora_utils.hpp"
#include "aurora_pub/ndi_aurora_ros2.hpp" 

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

using namespace aurora_pub;

// =============================================================================
// CONSTRUCTOR AND DESTRUCTOR
// =============================================================================

AuroraPublisherNode::AuroraPublisherNode() : Node("aurora_publisher_node")
{
    // Declare all parameters
    declare_parameters();
    
    // Load parameters from YAML
    load_parameters();
    
    // Validate parameters
    if (!validate_parameters()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter validation failed. Shutting down.");
        rclcpp::shutdown();
        return;
    }
    
    // Initialize member variables
    tracking_active_ = false;
    has_valid_data_ = false;
    
    // Initialize publishers
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    aurora_data_publisher_ = this->create_publisher<aurora_pub::msg::AuroraData>(
        params_.topic_aurora_name, params_.queue_size);
    
    // Initialize Aurora driver
    aurora_driver_ = std::make_unique<AuroraDriver::ndi_aurora>(params_.serial_port.c_str());
    
    // Setup Aurora system
    if (!setup_aurora()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup Aurora system. Shutting down.");
        rclcpp::shutdown();
        return;
    }
    
    // Start reading thread
    tracking_active_ = true;
    read_thread_ = std::thread(&AuroraPublisherNode::read_thread_function, this);
    
    // Start publishing timer
    auto timer_period = std::chrono::duration<double>(1.0 / params_.publish_rate_hz);
    tf_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&AuroraPublisherNode::tf_publish_callback, this));
    
    aurora_data_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&AuroraPublisherNode::aurora_data_publish_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Aurora Publisher Node initialized successfully");
}

AuroraPublisherNode::~AuroraPublisherNode()
{
    shutdown();
}

// =============================================================================
// PARAMETER MANAGEMENT (unchanged)
// =============================================================================

void AuroraPublisherNode::declare_parameters()
{
    // Serial communication parameters
    this->declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
    this->declare_parameter("baud_rate", 230400);  // Changed default to 115200
    this->declare_parameter("serial_timeout_sec", 2.0);
    
    // Aurora system parameters
    this->declare_parameter("tool_handle", std::string("0A"));
    this->declare_parameter("tool_rom_files", std::vector<std::string>{"/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"});
    this->declare_parameter("port_handles", std::vector<std::string>{"0A"});
    this->declare_parameter("reference_port", std::string("10"));
    this->declare_parameter("track_reply_option", 80);
    this->declare_parameter("measure_reply_option", std::string("0001"));
    this->declare_parameter("status_reply", true);
    
    // ROS topic parameters
    this->declare_parameter("child_frame_name", std::string("endo_aurora"));
    this->declare_parameter("topic_aurora_name", std::string("aurora_data"));
    this->declare_parameter("frame_id", std::string("aurora_base"));
    this->declare_parameter("publish_rate_hz", 40.0);
    this->declare_parameter("queue_size", 10);
    
    // Data processing parameters
    this->declare_parameter("enable_data_filtering", true);
    this->declare_parameter("filter_window_size", 4);
    this->declare_parameter("position_scale_factor", 1.0);  // Driver handles mm conversion
    this->declare_parameter("orientation_scale_factor", 1.0);
    this->declare_parameter("error_scale_factor", 1.0);
    
    // Connection and retry parameters
    this->declare_parameter("command_timeout_sec", 2.0);
    this->declare_parameter("max_connection_retries", 3);
    this->declare_parameter("retry_delay_sec", 2.0);
    
    // Logging parameters
    this->declare_parameter("debug_mode", false);
    this->declare_parameter("log_raw_data", false);
}

void AuroraPublisherNode::load_parameters()
{
    // Load serial communication parameters
    params_.serial_port = this->get_parameter("serial_port").as_string();
    params_.baud_rate = this->get_parameter("baud_rate").as_int();
    params_.serial_timeout_sec = this->get_parameter("serial_timeout_sec").as_double();
    
    // Load Aurora system parameters
    params_.tool_handle = this->get_parameter("tool_handle").as_string();
    params_.tool_rom_files = this->get_parameter("tool_rom_files").as_string_array();
    params_.port_handles = this->get_parameter("port_handles").as_string_array();
    params_.reference_port = this->get_parameter("reference_port").as_string();
    params_.track_reply_option = this->get_parameter("track_reply_option").as_int();
    params_.measure_reply_option = this->get_parameter("measure_reply_option").as_string();
    params_.status_reply = this->get_parameter("status_reply").as_bool();
    
    // Load ROS topic parameters
    params_.child_frame_name = this->get_parameter("child_frame_name").as_string();
    params_.topic_aurora_name = this->get_parameter("topic_aurora_name").as_string();
    params_.frame_id = this->get_parameter("frame_id").as_string();
    params_.publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    params_.queue_size = this->get_parameter("queue_size").as_int();
    
    // Load data processing parameters
    params_.enable_data_filtering = this->get_parameter("enable_data_filtering").as_bool();
    params_.filter_window_size = this->get_parameter("filter_window_size").as_int();
    params_.position_scale_factor = this->get_parameter("position_scale_factor").as_double();
    params_.orientation_scale_factor = this->get_parameter("orientation_scale_factor").as_double();
    params_.error_scale_factor = this->get_parameter("error_scale_factor").as_double();
    
    // Load connection and retry parameters
    params_.command_timeout_sec = this->get_parameter("command_timeout_sec").as_double();
    params_.max_connection_retries = this->get_parameter("max_connection_retries").as_int();
    params_.retry_delay_sec = this->get_parameter("retry_delay_sec").as_double();
    
    // Load logging parameters
    params_.debug_mode = this->get_parameter("debug_mode").as_bool();
    params_.log_raw_data = this->get_parameter("log_raw_data").as_bool();
}

bool AuroraPublisherNode::validate_parameters()
{
    bool valid = true;
    
    // Validate serial port
    if (params_.serial_port.empty()) {
        RCLCPP_ERROR(this->get_logger(), "serial_port parameter is empty");
        valid = false;
    }
    
    // Validate baud rate
    if (params_.baud_rate <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid baud_rate: %d", params_.baud_rate);
        valid = false;
    }
    
    // Validate tool handle
    if (!utils::validate_tool_handle(params_.tool_handle)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid tool_handle: %s", params_.tool_handle.c_str());
        valid = false;
    }
    
    // Validate publish rate
    if (params_.publish_rate_hz <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid publish_rate_hz: %.2f", params_.publish_rate_hz);
        valid = false;
    }
    
    return valid;
}

// =============================================================================
// AURORA SYSTEM SETUP (using ndi_aurora driver)
// =============================================================================

bool AuroraPublisherNode::setup_aurora()
{
    RCLCPP_INFO(this->get_logger(), "Setting up Aurora system using NDI driver...");
    
    // Start serial communication
    if (!aurora_driver_->startSerial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start serial communication");
        return false;
    }
    
    // Change baud rate if specified
    if (params_.baud_rate != 9600) {
        if (!aurora_driver_->changeBaudRate(params_.baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to change baud rate to %d", params_.baud_rate);
            return false;
        }
    }
    
    // Initialize Aurora
    if (!aurora_driver_->initAurora()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Aurora");
        return false;
    }
    
    // Send a beep to confirm connection
    if (!aurora_driver_->sendBeep()) {
        RCLCPP_WARN(this->get_logger(), "Aurora beep failed, but continuing...");
    }
    
    // Initialize port handles with tool definitions
    std::vector<std::string> toolDefinitions = params_.tool_rom_files;
    std::vector<std::string> portHandles = params_.port_handles;
    
    if (!aurora_driver_->initPortHandleWT(toolDefinitions, portHandles)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize port handles");
        return false;
    }
    
    // Initialize tool vector
    aurora_driver_->initToolVector();
    
    // Start tracking mode
    if (!aurora_driver_->startTrackingMode(params_.track_reply_option)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start tracking mode");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Aurora system setup completed successfully");
    return true;
}

// =============================================================================
// DATA PARSING (modified for ndi_aurora)
// =============================================================================

std::optional<AuroraPublisherNode::AuroraData> AuroraPublisherNode::parse_aurora_data(
    const std::vector<AuroraDriver::ToolPose>& poses, 
    const std::vector<int>& port_indexes, 
    const std::vector<int>& visible_tools)
{
    if (poses.empty()) {
        return std::nullopt;
    }
    
    // Find data for our tool handle
    int target_handle = std::stoi(params_.tool_handle, nullptr, 16);
    
    for (size_t i = 0; i < poses.size(); ++i) {
        if (port_indexes[i] == target_handle) {
            AuroraData aurora_data;
            aurora_data.timestamp = std::chrono::steady_clock::now();
            aurora_data.handle = params_.tool_handle;
            
            // Check if tool is visible
            aurora_data.visible = std::find(visible_tools.begin(), visible_tools.end(), target_handle) 
                                 != visible_tools.end();
            
            if (aurora_data.visible) {
                // Position (already in mm from driver)
                aurora_data.position[0] = poses[i].x * params_.position_scale_factor;
                aurora_data.position[1] = poses[i].y * params_.position_scale_factor;
                aurora_data.position[2] = poses[i].z * params_.position_scale_factor;
                
                // Orientation (quaternion)
                aurora_data.orientation[0] = poses[i].qx * params_.orientation_scale_factor;
                aurora_data.orientation[1] = poses[i].qy * params_.orientation_scale_factor;
                aurora_data.orientation[2] = poses[i].qz * params_.orientation_scale_factor;
                aurora_data.orientation[3] = poses[i].qw * params_.orientation_scale_factor;
                
                // Normalize quaternion
                utils::normalize_quaternion(aurora_data.orientation);
                
                // Error (set to 0 for now, could be extracted from driver if needed)
                aurora_data.error = 0.0;
            }
            
            return aurora_data;
        }
    }
    
    return std::nullopt;
}

// =============================================================================
// DATA FILTERING (unchanged)
// =============================================================================

AuroraPublisherNode::AuroraData AuroraPublisherNode::apply_filtering(const AuroraData& new_data)
{
    if (!params_.enable_data_filtering) {
        return new_data;
    }
    
    // Add new data to buffer
    data_buffer_.push_back(new_data);
    
    // Keep buffer size within window
    if (static_cast<int>(data_buffer_.size()) > params_.filter_window_size) {
        data_buffer_.erase(data_buffer_.begin());
    }
    
    // If not enough data for filtering, return original
    if (data_buffer_.size() < 2) {
        return new_data;
    }
    
    AuroraData filtered_data = new_data;  // Keep handle, visible, timestamp, error
    
    // Apply moving average to position
    for (int i = 0; i < 3; ++i) {
        std::vector<double> position_values;
        for (const auto& data : data_buffer_) {
            if (data.visible) {
                position_values.push_back(data.position[i]);
            }
        }
        
        if (!position_values.empty()) {
            filtered_data.position[i] = utils::calculate_moving_average(position_values);
        }
    }
    
    // Apply moving average to orientation
    for (int i = 0; i < 4; ++i) {
        std::vector<double> orientation_values;
        for (const auto& data : data_buffer_) {
            if (data.visible) {
                orientation_values.push_back(data.orientation[i]);
            }
        }
        
        if (!orientation_values.empty()) {
            filtered_data.orientation[i] = utils::calculate_moving_average(orientation_values);
        }
    }
    
    // Renormalize quaternion after averaging
    utils::normalize_quaternion(filtered_data.orientation);
    
    return filtered_data;
}

// =============================================================================
// THREAD FUNCTIONS (modified for ndi_aurora)
// =============================================================================

void AuroraPublisherNode::read_thread_function()
{
    RCLCPP_INFO(this->get_logger(), "Aurora data reading thread started");
    
    while (tracking_active_ && rclcpp::ok()) {
        try {
            std::vector<AuroraDriver::ToolPose> poses;
            std::vector<int> port_indexes;
            std::vector<int> visible_tools;
            
            // Get measurement data from driver
            if (aurora_driver_->measureTool(params_.measure_reply_option, poses, 
                                          params_.status_reply, port_indexes, visible_tools)) {
                
                // Parse the data for our tool
                auto parsed_data = parse_aurora_data(poses, port_indexes, visible_tools);
                
                if (parsed_data) {
                    // Apply filtering if enabled
                    AuroraData filtered_data = apply_filtering(*parsed_data);
                    
                    // Update latest data with thread safety
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        latest_data_ = filtered_data;
                        has_valid_data_ = filtered_data.visible;
                    }
                    
                    if (params_.debug_mode && filtered_data.visible) {
                        RCLCPP_DEBUG(this->get_logger(), 
                            "Tool %s: pos[%.2f, %.2f, %.2f] quat[%.4f, %.4f, %.4f, %.4f] err=%.2f",
                            filtered_data.handle.c_str(),
                            filtered_data.position[0], filtered_data.position[1], filtered_data.position[2],
                            filtered_data.orientation[0], filtered_data.orientation[1], 
                            filtered_data.orientation[2], filtered_data.orientation[3],
                            filtered_data.error);
                    }
                }
            } else {
                if (params_.debug_mode) {
                    RCLCPP_DEBUG(this->get_logger(), "measureTool returned false");
                }
            }
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Exception in read thread: %s", e.what());
        }
        
        // Sleep to maintain reading rate
        std::this_thread::sleep_for(std::chrono::milliseconds(25));  // 40 Hz
    }
    
    RCLCPP_INFO(this->get_logger(), "Aurora data reading thread stopped");
}

void AuroraPublisherNode::tf_publish_callback()
{
    if (!has_valid_data_) return;
    
    AuroraData current_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_data = latest_data_;
    }
    
    if (!current_data.visible) return;
    
    // Publish TF: aurora_base -> endo_aurora
    geometry_msgs::msg::TransformStamped tf_msg;
    
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = params_.frame_id;           // "aurora_base"
    tf_msg.child_frame_id = params_.child_frame_name;    // "endo_aurora"
    
    // Position (mm to meters)
    tf_msg.transform.translation.x = current_data.position[0] / 1000.0;
    tf_msg.transform.translation.y = current_data.position[1] / 1000.0;
    tf_msg.transform.translation.z = current_data.position[2] / 1000.0;
    
    // Orientation
    tf_msg.transform.rotation.x = current_data.orientation[0];
    tf_msg.transform.rotation.y = current_data.orientation[1];
    tf_msg.transform.rotation.z = current_data.orientation[2];
    tf_msg.transform.rotation.w = current_data.orientation[3];
    
    tf_broadcaster_->sendTransform(tf_msg);
    
    if (params_.debug_mode) {
        RCLCPP_DEBUG(this->get_logger(), "Published TF: %s -> %s", 
                    params_.frame_id.c_str(), params_.child_frame_name.c_str());
    }
}

void AuroraPublisherNode::aurora_data_publish_callback()
{
    
    if (!has_valid_data_ && latest_data_.handle.empty()) {
        return;  // No data ever received since startup
    }
    
    // Thread-safe copy of the latest Aurora data
    AuroraData current_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_data = latest_data_;
    }
    
    // ALWAYS PUBLISH THE AURORA MESSAGE (even when not visible)
    auto aurora_msg = aurora_pub::msg::AuroraData();
    
    // Keep original position data in millimeters (even if invalid)
    aurora_msg.position.x = current_data.position[0];  // mm
    aurora_msg.position.y = current_data.position[1];  // mm
    aurora_msg.position.z = current_data.position[2];  // mm
    
    // Copy orientation quaternion (even if invalid)
    aurora_msg.orientation.x = current_data.orientation[0];
    aurora_msg.orientation.y = current_data.orientation[1];
    aurora_msg.orientation.z = current_data.orientation[2];
    aurora_msg.orientation.w = current_data.orientation[3];
    
    // Include Aurora-specific data 
    aurora_msg.error = current_data.error;
    aurora_msg.visible = current_data.visible;  // Indicates if data is valid
    aurora_msg.port_handle = std::stoi(params_.tool_handle, nullptr, 16);
    
    // Publish the custom Aurora message
    aurora_data_publisher_->publish(aurora_msg);
    
    if (params_.debug_mode) {
        RCLCPP_DEBUG(this->get_logger(), "Published aurora data for tool %s (visible: %s)", 
                    current_data.handle.c_str(), current_data.visible ? "true" : "false");
    }
}

// =============================================================================
// SHUTDOWN PROCEDURE (modified for ndi_aurora)
// =============================================================================

void AuroraPublisherNode::shutdown()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Aurora Publisher Node...");
    
    // Stop tracking thread
    tracking_active_ = false;
    
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    
    // Stop Aurora system using driver
    if (aurora_driver_) {
        try {
            aurora_driver_->stopTrackingMode();
            aurora_driver_->stopSerial();
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Exception during Aurora shutdown: %s", e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Aurora Publisher Node shutdown complete");
}

// =============================================================================
// MAIN FUNCTION (unchanged)
// =============================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<AuroraPublisherNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}