/**
 * @file constants.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含整个项目中使用的常量。
 * @version 1.0
 * @date 2023-05-01
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <cstdint>

/**
 * @brief 这个数是在计算哈希值时使用的移位数字，
 * 用于哈希由多个数字定义的对象。鉴于哈希元素最多的对象
 * 是三角形（3 个），这个数字已相应调整，即 21*3 == 63，63 < 64 位。
 * 这也意味着最大的锥桶 ID 必须小于 2^HASH_SHIFT_NUM-3。
 * 最后的 "3" 是因为超级三角形。我们必须能够创建它。
 */
const uint32_t HASH_SHIFT_NUM = 21;

/**
 * @brief 能够进行闭环所需的最小中点数量。
 * 如果大小小于 MIN_LOOP_SIZE，则不会进行闭环。
 * 该常量用于确定闭环的最小长度，即在闭环达到该长度之前，不会进行闭环操作。
 */
const uint32_t MIN_LOOP_SIZE = 25;

/**
 * @brief 如果两个中点之间的距离（彼此之间的距离）小于
 * SAME_MIDPOINT_DIST_THRESHOLD 米，则将被视为同一点。
 */
const double SAME_MIDPOINT_DIST_THRESHOLD = 0.2;

/**
 * @brief 车辆前方所需的最小中点数量，以避免触发故障保护。
 *  - 如果设置为 1：只有当车辆前方没有任何中点时，才会激活故障保护。
 *  - 如果设置为 2：当车辆前方仍有一个中点时，就会激活故障保护。这个选项可能更可取。
 */
const uint32_t MIN_FAILSAFE_WAY_SIZE = 2;
