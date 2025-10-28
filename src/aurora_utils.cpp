#include "aurora_pub/aurora_utils.hpp"

#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cctype>

namespace aurora_pub
{
namespace utils
{

void normalize_quaternion(double quat[4])
{
    // Calculate magnitude
    double magnitude = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1] + 
                                quat[2]*quat[2] + quat[3]*quat[3]);
    
    // Avoid division by zero
    if (magnitude < 1e-8) {
        // Set to identity quaternion
        quat[0] = 0.0; // qx
        quat[1] = 0.0; // qy  
        quat[2] = 0.0; // qz
        quat[3] = 1.0; // qw
        return;
    }
    
    // Normalize
    quat[0] /= magnitude;
    quat[1] /= magnitude;
    quat[2] /= magnitude;
    quat[3] /= magnitude;
}

bool validate_tool_handle(const std::string& handle)
{
    // Tool handle should be exactly 2 hexadecimal characters
    if (handle.length() != 2) {
        return false;
    }
    
    return std::all_of(handle.begin(), handle.end(), [](char c) {
        return std::isxdigit(c);
    });
}

double calculate_moving_average(const std::vector<double>& data_buffer)
{
    if (data_buffer.empty()) {
        return 0.0;
    }
    
    double sum = 0.0;
    for (double value : data_buffer) {
        sum += value;
    }
    
    return sum / data_buffer.size();
}

bool validate_baud_rate(int baud_rate)
{
    // Aurora supported baud rates
    const std::vector<int> supported_rates = {
        9600, 19200, 38400, 57600, 115200, 230400
    };
    
    return std::find(supported_rates.begin(), supported_rates.end(), baud_rate) 
           != supported_rates.end();
}

bool validate_rom_path(const std::string& rom_path)
{
    if (rom_path.empty()) {
        return false;
    }
    
    // Convert to lowercase for case-insensitive comparison
    std::string lower_path = rom_path;
    std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    // Check if string ends with ".rom"
    const std::string suffix = ".rom";
    if (lower_path.length() >= suffix.length()) {
        return lower_path.compare(lower_path.length() - suffix.length(), 
                                 suffix.length(), suffix) == 0;
    }
    return false;
}

} // namespace utils
} // namespace aurora_pub