#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace rosbag2_exporter {

std::string get_cam_name(const std::string& path) {
    const std::string prefix = "/sensor/camera/";
    size_t start = path.find(prefix);
    if (start == std::string::npos) {
        return ""; // Prefix not found
    }
    
    // Move past "/sensor/camera/"
    start += prefix.length(); 
    
    size_t end = path.find('/', start);
    return path.substr(start, end - start);
}

// Helper function to convert builtin_interfaces::msg::Time to nanoseconds
int64_t toNanoseconds(const builtin_interfaces::msg::Time& timestamp) {
    return static_cast<int64_t>(timestamp.sec) * std::pow(10, 9) + timestamp.nanosec;
}

// Function to find the closest index in a sorted vector for a given timestamp
size_t find_closest_timestamp(const std::vector<DataMeta>& target_vector, 
                              const builtin_interfaces::msg::Time& timestamp, 
                              size_t& last_index) {
                                
    int64_t timestamp_ns = toNanoseconds(timestamp);

    // Start from the last known index
    size_t closest_index = last_index;
    int64_t min_diff = std::numeric_limits<int64_t>::max();

    // Iterate forward through the sorted vector
    for (size_t i = last_index; i < target_vector.size(); ++i) {
        int64_t target_ns = toNanoseconds(target_vector[i].timestamp);
        int64_t diff = std::abs(target_ns - timestamp_ns);

        // Update closest index and minimum difference
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        } else {
            // Since the vector is sorted, stop when differences start increasing
            break;
        }
    }

    // Update the last index for the next call
    last_index = closest_index;
    return closest_index;
}

// Helper to  produce a stdout progress bar
void print_progress(int percentage) {
    // Static is safe as this runs only until 100% is reached 
    static int last_percentage = -1;

    // Never exceeds 100%
    if (percentage > 100) percentage = 100;
    // Prevent duplicate prints 
    if (percentage == last_percentage) return; 

    last_percentage = percentage;

    int width = 50;
    int pos = width * percentage / 100;

    std::cout << "\r\t\t\t\t\t\t [";
    for (int i = 0; i < width; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }

    std::cout << "] " << percentage << "%" << std::flush;

    // Print a newline once at 100%
    if (percentage >= 100)
        std::cout << std::endl;  
}


} // namespace rosbag2_exporter

#endif // UTILS_HPP
