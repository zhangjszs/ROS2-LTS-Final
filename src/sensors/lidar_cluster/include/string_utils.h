#pragma once

#include <cctype>
#include <format>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace lidar_cluster {

/**
 * @brief 解析以逗号分隔的浮点数字符串 (C++20 std::string_view 零拷贝切片，消除 std::istringstream)
 */
inline void parseCsvDoubles(std::string_view in_string, std::vector<double>& out_array) {
    size_t start = 0;
    while (start < in_string.size()) {
        size_t end = in_string.find(',', start);
        std::string_view token =
            (end == std::string_view::npos) ? in_string.substr(start) : in_string.substr(start, end - start);

        // 去除前后空白字符
        while (!token.empty() && std::isspace(static_cast<unsigned char>(token.front()))) {
            token.remove_prefix(1);
        }
        while (!token.empty() && std::isspace(static_cast<unsigned char>(token.back()))) {
            token.remove_suffix(1);
        }

        if (!token.empty()) {
            try {
                out_array.push_back(std::stod(std::string(token)));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("lidar_cluster"), "%s",
                             std::format("[lidar_cluster] Failed to parse token '{}' in config string '{}': {}", token,
                                         in_string, e.what())
                                 .c_str());
            }
        }

        if (end == std::string_view::npos) {
            break;
        }
        start = end + 1;
    }
}

/**
 * @brief 使用 C++20 std::format 序列化浮点数容器为紧凑可读字符串
 */
inline std::string formatCsvDoubles(const std::vector<double>& values, std::string_view delimiter = ", ") {
    std::string result;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            result.append(delimiter);
        }
        result.append(std::format("{:.2f}", values[i]));
    }
    return result;
}

}  // namespace lidar_cluster