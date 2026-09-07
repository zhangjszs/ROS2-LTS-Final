/**
 * @file Time.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Time 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Time.hpp"

#include <rclcpp/rclcpp.hpp>

std::map<std::string, std::chrono::steady_clock::time_point> Time::clocks_;

void Time::tick(const std::string &clockName) {
    std::map<std::string, std::chrono::steady_clock::time_point>::iterator it = clocks_.find(clockName);
    if (it != clocks_.end()) {
        RCLCPP_WARN(rclcpp::get_logger("urinay"), "[urinay] Duplicate tick() call");
        it->second = std::chrono::steady_clock::now();
    } else {
        clocks_.emplace(clockName, std::chrono::steady_clock::now());
    }
}

std::chrono::duration<double> Time::tock(const std::string &clockName) {
    std::map<std::string, std::chrono::steady_clock::time_point>::iterator it = clocks_.find(clockName);
    std::chrono::duration<double> res{0.0};
    if (it == clocks_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("urinay"), "[urinay] tock() called before tick()");
    } else {
        res = std::chrono::steady_clock::now() - it->second;
        RCLCPP_INFO(rclcpp::get_logger("urinay"), "[urinay] %s elapsed: %.2f ms", it->first.c_str(), res.count() * 1000.0);
        clocks_.erase(it);
    }
    return res;
}
