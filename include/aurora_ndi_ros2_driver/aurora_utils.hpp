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

void normalize_quaternion(double quat[4]);

bool validate_tool_handle(const std::string& handle);

double calculate_moving_average(const std::vector<double>& data_buffer);

bool validate_baud_rate(int baud_rate);

bool validate_rom_path(const std::string& rom_path);

inline double mm_to_meters(double position_mm) {
    return position_mm / 1000.0;
}

inline bool is_measurement_quality_ok(double error_mm, double max_acceptable_error_mm = 2.0) {
    return error_mm <= max_acceptable_error_mm;
}

} // namespace utils
} // namespace aurora_ndi_ros2_driver