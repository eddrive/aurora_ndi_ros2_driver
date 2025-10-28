#include "aurora_ndi_ros2_driver/aurora_publisher_node.hpp"
#include "aurora_ndi_ros2_driver/aurora_utils.hpp"
#include "aurora_ndi_ros2_driver/ndi_aurora_ros2.hpp" 

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
    
    // Resize vectors for multi-sensor support
    has_valid_data_.resize(params_.num_sensors, false);
    latest_data_.resize(params_.num_sensors);
    data_buffers_.resize(params_.num_sensors);
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize publishers for each sensor
    aurora_data_publishers_.resize(params_.num_sensors);
    for (int i = 0; i < params_.num_sensors; ++i) {
        aurora_data_publishers_[i] = this->create_publisher<aurora_pub::msg::AuroraData>(
            params_.topic_names[i], params_.queue_size);
        RCLCPP_INFO(this->get_logger(), "Created publisher for sensor %d on topic: %s", 
                    i, params_.topic_names[i].c_str());
    }
    
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
    
    RCLCPP_INFO(this->get_logger(), "Aurora Publisher Node initialized successfully with %d sensors", 
                params_.num_sensors);
}

AuroraPublisherNode::~AuroraPublisherNode()
{
    shutdown();
}

// =============================================================================
// PARAMETER MANAGEMENT
// =============================================================================

void AuroraPublisherNode::declare_parameters()
{
    // Serial communication parameters
    this->declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
    this->declare_parameter("baud_rate", 230400);
    this->declare_parameter("serial_timeout_sec", 2.0);
    
    // Multi-sensor configuration
    this->declare_parameter("num_sensors", 1);  // Total number of sensors (including autoconfig)
    this->declare_parameter("tool_rom_files", std::vector<std::string>{});  // May be empty for all-autoconfig
    this->declare_parameter("port_handles", std::vector<std::string>{"0A"});
    this->declare_parameter("port_handles_autoconfig", std::vector<std::string>{});  // autoconfigured ports
    this->declare_parameter("topic_names", std::vector<std::string>{"/aurora_data_sensor0"});
    this->declare_parameter("child_frame_names", std::vector<std::string>{"endo_aurora_sensor0"});
    
    // Aurora system parameters
    this->declare_parameter("reference_port", std::string("10"));
    this->declare_parameter("track_reply_option", 80);
    this->declare_parameter("measure_reply_option", std::string("0001"));
    this->declare_parameter("status_reply", true);
    
    // ROS topic parameters
    this->declare_parameter("frame_id", std::string("aurora_base"));
    this->declare_parameter("publish_rate_hz", 40.0);
    this->declare_parameter("queue_size", 10);
    
    // Data processing parameters
    this->declare_parameter("enable_data_filtering", true);
    this->declare_parameter("filter_window_size", 4);
    this->declare_parameter("position_scale_factor", 1.0);
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
    
    // Load multi-sensor configuration
    params_.num_sensors = this->get_parameter("num_sensors").as_int();
    
    // Load tool_rom_files - may contain empty strings as placeholders for all-autoconfig setups
    auto rom_files_raw = this->get_parameter("tool_rom_files").as_string_array();
    params_.tool_rom_files.clear();
    for (const auto& rom_file : rom_files_raw) {
        if (!rom_file.empty()) {
            params_.tool_rom_files.push_back(rom_file);
        }
    }
    
    params_.port_handles = this->get_parameter("port_handles").as_string_array();
    
    // Load port_handles_autoconfig - may contain empty strings as placeholders
    auto autoconfig_raw = this->get_parameter("port_handles_autoconfig").as_string_array();
    params_.port_handles_autoconfig.clear();
    for (const auto& handle : autoconfig_raw) {
        if (!handle.empty()) {
            params_.port_handles_autoconfig.push_back(handle);
        }
    }
    
    params_.topic_names = this->get_parameter("topic_names").as_string_array();
    params_.child_frame_names = this->get_parameter("child_frame_names").as_string_array();
    
    // Load Aurora system parameters
    params_.reference_port = this->get_parameter("reference_port").as_string();
    params_.track_reply_option = this->get_parameter("track_reply_option").as_int();
    params_.measure_reply_option = this->get_parameter("measure_reply_option").as_string();
    params_.status_reply = this->get_parameter("status_reply").as_bool();
    
    // Load ROS topic parameters
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
    
    // Validate num_sensors
    if (params_.num_sensors < 1 || params_.num_sensors > 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid num_sensors: %d (must be 1-4)", params_.num_sensors);
        valid = false;
    }
    
    // Validate ROM files size considering autoconfigured sensors
    int num_autoconfig = params_.port_handles_autoconfig.size();
    int expected_rom_files = params_.num_sensors - num_autoconfig;
    
    if (static_cast<int>(params_.tool_rom_files.size()) != expected_rom_files) {
        RCLCPP_ERROR(this->get_logger(), 
                     "tool_rom_files size (%zu) doesn't match expected (%d = num_sensors %d - autoconfig %d)", 
                     params_.tool_rom_files.size(), expected_rom_files, params_.num_sensors, num_autoconfig);
        valid = false;
    }
    
    // Validate port_handles size matches num_sensors
    if (static_cast<int>(params_.port_handles.size()) != params_.num_sensors) {
        RCLCPP_ERROR(this->get_logger(), "port_handles size (%zu) doesn't match num_sensors (%d)", 
                     params_.port_handles.size(), params_.num_sensors);
        valid = false;
    }
    
    if (static_cast<int>(params_.topic_names.size()) != params_.num_sensors) {
        RCLCPP_ERROR(this->get_logger(), "topic_names size (%zu) doesn't match num_sensors (%d)", 
                     params_.topic_names.size(), params_.num_sensors);
        valid = false;
    }
    
    if (static_cast<int>(params_.child_frame_names.size()) != params_.num_sensors) {
        RCLCPP_ERROR(this->get_logger(), "child_frame_names size (%zu) doesn't match num_sensors (%d)", 
                     params_.child_frame_names.size(), params_.num_sensors);
        valid = false;
    }
    
    // Validate that autoconfig ports are subset of port_handles
    for (const auto& autoconfig_handle : params_.port_handles_autoconfig) {
        bool found = false;
        for (const auto& handle : params_.port_handles) {
            if (handle == autoconfig_handle) {
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Autoconfig port '%s' not found in port_handles list", 
                        autoconfig_handle.c_str());
            valid = false;
        }
    }
    
    // Validate tool handles
    for (const auto& handle : params_.port_handles) {
        if (!utils::validate_tool_handle(handle)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid port_handle: %s", handle.c_str());
            valid = false;
        }
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
    
    // Separate port handles into regular and autoconfigured
    std::vector<std::string> toolDefinitions;
    std::vector<std::string> portHandlesWithROM;
    std::vector<std::string> portHandlesAutoconfig = params_.port_handles_autoconfig;
    
    // Build lists of ports with ROM files vs autoconfigured
    int rom_file_index = 0;
    for (int i = 0; i < params_.num_sensors; ++i) {
        const std::string& handle = params_.port_handles[i];
        
        // Check if this port is in the autoconfig list
        bool is_autoconfig = false;
        for (const auto& autoconfig_handle : portHandlesAutoconfig) {
            if (handle == autoconfig_handle) {
                is_autoconfig = true;
                break;
            }
        }
        
        if (!is_autoconfig) {
            // Regular sensor with ROM file
            portHandlesWithROM.push_back(handle);
            toolDefinitions.push_back(params_.tool_rom_files[rom_file_index]);
            rom_file_index++;
            
            RCLCPP_INFO(this->get_logger(), "  Sensor %d: handle=%s, rom=%s", 
                        i, handle.c_str(), params_.tool_rom_files[rom_file_index-1].c_str());
        } else {
            // Autoconfigured sensor (e.g., NDI pen)
            RCLCPP_INFO(this->get_logger(), "  Sensor %d: handle=%s (AUTOCONFIGURED - no ROM file)", 
                        i, handle.c_str());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Initializing %d port handles (%d with ROM, %d autoconfigured)...", 
                params_.num_sensors, (int)portHandlesWithROM.size(), (int)portHandlesAutoconfig.size());
    
    // Call initPortHandleWT with autoconfig parameter
    // IMPORTANT: Second parameter must be ALL port handles (not just ROM ports)
    if (!aurora_driver_->initPortHandleWT(toolDefinitions, params_.port_handles, portHandlesAutoconfig, params_.reference_port)) {
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
    
    RCLCPP_INFO(this->get_logger(), "Aurora system setup completed successfully with %d sensors", 
                params_.num_sensors);
    return true;
}

// =============================================================================
// DATA PARSING - Modified for multi-sensor support
// =============================================================================

std::vector<std::optional<AuroraPublisherNode::AuroraData>> AuroraPublisherNode::parse_aurora_data_multi(
    const std::vector<AuroraDriver::ToolPose>& poses, 
    const std::vector<int>& port_indexes, 
    const std::vector<int>& visible_tools)
{
    std::vector<std::optional<AuroraData>> results(params_.num_sensors, std::nullopt);
    
    if (poses.empty()) {
        return results;
    }
    
    // Parse data for each configured sensor
    for (int sensor_idx = 0; sensor_idx < params_.num_sensors; ++sensor_idx) {
        int target_handle = std::stoi(params_.port_handles[sensor_idx], nullptr, 16);
        
        // Find this sensor's data in the poses
        for (size_t i = 0; i < poses.size(); ++i) {
            if (port_indexes[i] == target_handle) {
                AuroraData aurora_data;
                aurora_data.timestamp = std::chrono::steady_clock::now();
                aurora_data.handle = params_.port_handles[sensor_idx];
                aurora_data.sensor_index = sensor_idx;
                
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
                    utils::normalize_quaternion(aurora_data.orientation.data());
                    
                    // Error (set to 0 for now, could be extracted from driver if needed)
                    aurora_data.error = 0.0;
                }
                
                results[sensor_idx] = aurora_data;
                break;  // Found this sensor, move to next
            }
        }
    }
    
    return results;
}

// =============================================================================
// DATA FILTERING - Modified for multi-sensor support
// =============================================================================

AuroraPublisherNode::AuroraData AuroraPublisherNode::apply_filtering(
    const AuroraData& new_data, int sensor_index)
{
    if (!params_.enable_data_filtering || sensor_index >= static_cast<int>(data_buffers_.size())) {
        return new_data;
    }
    
    // Add new data to this sensor's buffer
    data_buffers_[sensor_index].push_back(new_data);
    
    // Keep buffer size within window
    if (static_cast<int>(data_buffers_[sensor_index].size()) > params_.filter_window_size) {
        data_buffers_[sensor_index].erase(data_buffers_[sensor_index].begin());
    }
    
    // If not enough data for filtering, return original
    if (data_buffers_[sensor_index].size() < 2) {
        return new_data;
    }
    
    AuroraData filtered_data = new_data;  // Keep handle, visible, timestamp, error
    
    // Apply moving average to position
    for (int i = 0; i < 3; ++i) {
        std::vector<double> position_values;
        for (const auto& data : data_buffers_[sensor_index]) {
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
        for (const auto& data : data_buffers_[sensor_index]) {
            if (data.visible) {
                orientation_values.push_back(data.orientation[i]);
            }
        }
        
        if (!orientation_values.empty()) {
            filtered_data.orientation[i] = utils::calculate_moving_average(orientation_values);
        }
    }
    
    // Renormalize quaternion after averaging
    utils::normalize_quaternion(filtered_data.orientation.data());
    
    return filtered_data;
}

// =============================================================================
// THREAD FUNCTIONS - Modified for multi-sensor support
// =============================================================================

void AuroraPublisherNode::read_thread_function()
{
    RCLCPP_INFO(this->get_logger(), "Aurora data reading thread started for %d sensors", 
                params_.num_sensors);
    
    while (tracking_active_ && rclcpp::ok()) {
        try {
            std::vector<AuroraDriver::ToolPose> poses;
            std::vector<int> port_indexes;
            std::vector<int> visible_tools;
            
            // Get measurement data from driver (reads all sensors at once)
            if (aurora_driver_->measureTool(params_.measure_reply_option, poses, 
                                          params_.status_reply, port_indexes, visible_tools)) {
                
                // Parse the data for all sensors
                auto parsed_data_multi = parse_aurora_data_multi(poses, port_indexes, visible_tools);
                
                // Process each sensor's data
                for (int sensor_idx = 0; sensor_idx < params_.num_sensors; ++sensor_idx) {
                    if (parsed_data_multi[sensor_idx]) {
                        // Apply filtering if enabled
                        AuroraData filtered_data = apply_filtering(*parsed_data_multi[sensor_idx], sensor_idx);
                        
                        // Update latest data with thread safety
                        {
                            std::lock_guard<std::mutex> lock(data_mutex_);
                            latest_data_[sensor_idx] = filtered_data;
                            has_valid_data_[sensor_idx] = filtered_data.visible;
                        }
                        
                        if (params_.debug_mode && filtered_data.visible) {
                            RCLCPP_DEBUG(this->get_logger(), 
                                "Sensor %d (handle %s): pos[%.2f, %.2f, %.2f] quat[%.4f, %.4f, %.4f, %.4f] err=%.2f",
                                sensor_idx, filtered_data.handle.c_str(),
                                filtered_data.position[0], filtered_data.position[1], filtered_data.position[2],
                                filtered_data.orientation[0], filtered_data.orientation[1], 
                                filtered_data.orientation[2], filtered_data.orientation[3],
                                filtered_data.error);
                        }
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
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Publish TF for each sensor
    for (int sensor_idx = 0; sensor_idx < params_.num_sensors; ++sensor_idx) {
        if (!has_valid_data_[sensor_idx]) continue;
        
        const AuroraData& current_data = latest_data_[sensor_idx];
        if (!current_data.visible) continue;
        
        // Publish TF: aurora_base -> sensor_frame
        geometry_msgs::msg::TransformStamped tf_msg;
        
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = params_.frame_id;  // "aurora_base"
        tf_msg.child_frame_id = params_.child_frame_names[sensor_idx];  // e.g., "endo_aurora_sensor0"
        
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
            RCLCPP_DEBUG(this->get_logger(), "Published TF for sensor %d: %s -> %s", 
                        sensor_idx, params_.frame_id.c_str(), params_.child_frame_names[sensor_idx].c_str());
        }
    }
}

void AuroraPublisherNode::aurora_data_publish_callback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Publish data for each sensor
    for (int sensor_idx = 0; sensor_idx < params_.num_sensors; ++sensor_idx) {
        // Check if we have ever received data for this sensor
        if (latest_data_[sensor_idx].handle.empty()) {
            continue;  // No data ever received for this sensor
        }
        
        const AuroraData& current_data = latest_data_[sensor_idx];
        
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
        aurora_msg.port_handle = std::stoi(params_.port_handles[sensor_idx], nullptr, 16);
        
        // Publish the custom Aurora message
        aurora_data_publishers_[sensor_idx]->publish(aurora_msg);
        
        if (params_.debug_mode) {
            RCLCPP_DEBUG(this->get_logger(), "Published aurora data for sensor %d (handle %s, visible: %s)", 
                        sensor_idx, current_data.handle.c_str(), current_data.visible ? "true" : "false");
        }
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
// MAIN FUNCTION
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