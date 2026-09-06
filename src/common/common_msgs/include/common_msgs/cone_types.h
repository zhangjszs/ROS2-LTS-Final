#pragma once

#include <cstdint>

// Canonical cone labels for this stack.
//
// HuatCone.type and HuatVisionDetections.color_types share Color.
// HuatConeCluster.type is Size only — never copy it into HuatCone.type.
namespace huat_cone {

enum Color : uint32_t {
    BLUE = 0,
    YELLOW_SMALL = 1,
    YELLOW = 1,  // vision emits unified yellow; lidar size may split it
    YELLOW_BIG = 2,
    RED = 3,
    NONE = 4
};

enum Size : uint8_t { SIZE_UNKNOWN = 0, SIZE_LARGE = 1, SIZE_SMALL = 2 };

inline bool IsKnownColor(uint32_t color) {
    return color < NONE;
}

// Vision yellow is size-agnostic. When lidar classified large/small, split it.
inline uint32_t MergeVisionColorWithLidarSize(uint8_t vision_color, uint8_t lidar_size) {
    if (!IsKnownColor(vision_color))
        return NONE;
    if (vision_color == YELLOW || vision_color == YELLOW_BIG) {
        if (lidar_size == SIZE_LARGE)
            return YELLOW_BIG;
        if (lidar_size == SIZE_SMALL)
            return YELLOW_SMALL;
    }
    return vision_color;
}

}  // namespace huat_cone
