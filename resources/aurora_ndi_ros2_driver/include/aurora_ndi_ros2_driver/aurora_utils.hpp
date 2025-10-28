#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <algorithm> 
#include <cctype> 

namespace aurora_ndi_ros2_driver
{
namespace utils
{

/**
 * @brief Simplified Aurora NDI utility functions for ndi_aurora driver
 * Only includes functions still needed when using the ndi_aurora driver
 */

/**
 * @brief Normalize quaternion to unit length
 * Ensures quaternion represents a valid rotation
 * @param quat Quaternion array [qx, qy, qz, qw] - modified in place
 */
void normalize_quaternion(double quat[4]);

/**
 * @brief Validate Aurora tool handle format
 * Tool handles must be exactly 2 hexadecimal characters (e.g., "0A", "1B", "FF")
 * @param handle Tool handle string to validate
 * @return true if handle format is valid
 */
bool validate_tool_handle(const std::string& handle);

/**
 * @brief Calculate moving average for a data series
 * Used for data filtering to smooth noisy measurements
 * @param data_buffer Vector of historical data points
 * @return Moving average value, 0.0 if buffer is empty
 */
double calculate_moving_average(const std::vector<double>& data_buffer);

/**
 * @brief Validate Aurora baud rate
 * Checks if baud rate is supported by Aurora system
 * @param baud_rate Baud rate value to validate
 * @return true if baud rate is valid for Aurora
 */
bool validate_baud_rate(int baud_rate);

/**
 * @brief Validate ROM file path format
 * Checks if path has correct .rom extension (case insensitive)
 * @param rom_path ROM file path string to validate
 * @return true if path format is valid (.rom extension)
 */
bool validate_rom_path(const std::string& rom_path);

/**
 * @brief Convert position from millimeters to meters (ROS standard)
 * @param position_mm Position in millimeters
 * @return Position in meters
 */
inline double mm_to_meters(double position_mm) {
    return position_mm / 1000.0;
}

/**
 * @brief Check if Aurora measurement quality is acceptable
 * @param error_mm RMS error in millimeters
 * @param max_acceptable_error_mm Maximum acceptable error threshold
 * @return true if measurement quality is acceptable
 */
inline bool is_measurement_quality_ok(double error_mm, double max_acceptable_error_mm = 2.0) {
    return error_mm <= max_acceptable_error_mm;
}

} // namespace utils
} // namespace aurora_ndi_ros2_driver