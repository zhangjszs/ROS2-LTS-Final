#pragma once

#include <deque>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class ImuData {
   public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    struct RPY {
        double heading = 0.0;
        double pitch = 0.0;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    RPY rpy;

   public:
};
