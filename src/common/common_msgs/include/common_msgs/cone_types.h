#pragma once

#include <concepts>
#include <cstdint>
#include <type_traits>
#include <utility>

// Canonical cone labels for this stack.
//
// HuatCone.type and HuatVisionDetections.color_types share Color.
// HuatConeCluster.type is Size only — never copy it into HuatCone.type.
namespace huat_cone {

/**
 * @brief 现代 C++20 强类型锥桶颜色枚举 (Scoped Enumeration)
 */
enum class Color : uint32_t {
    Blue = 0,
    YellowSmall = 1,
    Yellow = 1,  // vision emits unified yellow; lidar size may split it
    YellowBig = 2,
    Red = 3,
    None = 4
};

/**
 * @brief 现代 C++20 强类型锥桶尺寸枚举 (Scoped Enumeration)
 */
enum class Size : uint8_t {
    Unknown = 0,
    Large = 1,
    Small = 2
};

/**
 * @brief C++20 辅助工具函数：安全提取枚举底层类型数值
 */
template <typename Enum>
    requires std::is_enum_v<Enum>
[[nodiscard]] constexpr auto to_underlying(Enum e) noexcept {
    return static_cast<std::underlying_type_t<Enum>>(e);
}

// -------------------------------------------------------------
// 向后兼容别名与常量（保障旧代码及 ROS 消息无缝编译）
// -------------------------------------------------------------
inline constexpr uint32_t BLUE = to_underlying(Color::Blue);
inline constexpr uint32_t YELLOW_SMALL = to_underlying(Color::YellowSmall);
inline constexpr uint32_t YELLOW = to_underlying(Color::Yellow);
inline constexpr uint32_t YELLOW_BIG = to_underlying(Color::YellowBig);
inline constexpr uint32_t RED = to_underlying(Color::Red);
inline constexpr uint32_t NONE = to_underlying(Color::None);

inline constexpr uint8_t SIZE_UNKNOWN = to_underlying(Size::Unknown);
inline constexpr uint8_t SIZE_LARGE = to_underlying(Size::Large);
inline constexpr uint8_t SIZE_SMALL = to_underlying(Size::Small);

/**
 * @brief 判定是否为已知有效颜色 (强类型 Color 重载)
 */
[[nodiscard]] constexpr bool IsKnownColor(Color color) noexcept {
    return to_underlying(color) < to_underlying(Color::None);
}

/**
 * @brief 判定是否为已知有效颜色 (向后兼容原始数值重载)
 */
[[nodiscard]] constexpr bool IsKnownColor(uint32_t color) noexcept {
    return color < to_underlying(Color::None);
}

/**
 * @brief 视觉颜色与雷达尺寸融合算法 (强类型重载，C++20 constexpr)
 * 视觉黄色为尺寸无关统一黄色。若激光雷达分类出大小尺寸，则进行细分。
 */
[[nodiscard]] constexpr Color MergeVisionColorWithLidarSize(Color vision_color, Size lidar_size) noexcept {
    if (!IsKnownColor(vision_color))
        return Color::None;
    if (vision_color == Color::Yellow || vision_color == Color::YellowBig) {
        if (lidar_size == Size::Large)
            return Color::YellowBig;
        if (lidar_size == Size::Small)
            return Color::YellowSmall;
    }
    return vision_color;
}

/**
 * @brief 视觉颜色与雷达尺寸融合算法 (原始数值重载，保持 ABI 与旧调用 100% 兼容)
 */
[[nodiscard]] constexpr uint32_t MergeVisionColorWithLidarSize(uint8_t vision_color, uint8_t lidar_size) noexcept {
    return to_underlying(MergeVisionColorWithLidarSize(
        static_cast<Color>(vision_color), static_cast<Size>(lidar_size)));
}

}  // namespace huat_cone
