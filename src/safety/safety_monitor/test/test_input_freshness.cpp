#include <gtest/gtest.h>

#include "safety_monitor/input_freshness.h"

TEST(InputFreshness, OneVehicleSampleWithContinuousTenHzPathMustExpire) {
    safety_monitor::InputFreshness inputs(0.3, 0.5);
    inputs.observeVehicle(true, 100.0, 100.0);
    inputs.observePath(true, 100.0, 100.0);
    ASSERT_TRUE(inputs.healthy(100.0, 100.0));
    for (int tick = 1; tick <= 10; ++tick) {
        const double now = 100.0 + tick * 0.1;
        inputs.observePath(true, now, now);
        if (tick >= 4)
            EXPECT_FALSE(inputs.healthy(now, now)) << "vehicle last rx=100, path rx=" << now;
    }
}
