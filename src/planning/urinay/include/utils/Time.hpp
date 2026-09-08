/**
 * @file Time.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Time 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <chrono>
#include <map>
#include <stdexcept>
#include <string>

/**
 * @brief 抽象静态类，用于更方便地量化
 * 一组特定操作所花费的时间。
 */
class Time {
   private:
    static std::map<std::string, std::chrono::steady_clock::time_point> clocks_;

   public:
    Time() = delete;

    /**
     * @brief 创建一个名为 clockName 的时钟。
     *
     * @param[in] clockName
     */
    static void tick(const std::string& clockName);

    /**
     * @brief 停止名为 clockName 的时钟，返回持续时间
     * 并输出时钟名称和持续时间。
     *
     * @param[in] clockName
     */
    static std::chrono::duration<double> tock(const std::string& clockName);
};
