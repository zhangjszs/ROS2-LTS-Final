#include "race/race_director.h"

#include <algorithm>
#include <cmath>

RaceDirector::RaceDirector(int number_of_stopped_turns, double start_x, double start_y, double enter_radius,
                           double exit_radius)
    : number_of_stopped_turns_(number_of_stopped_turns),
      start_x_(start_x),
      start_y_(start_y),
      enter_radius_(enter_radius),
      exit_radius_(std::max(exit_radius, enter_radius + 0.1)) {}

bool RaceDirector::update(double car_x, double car_y) {
    const double dist = std::hypot(car_x - start_x_, car_y - start_y_);

    if (in_decision_area_) {
        if (dist > exit_radius_) {
            in_decision_area_ = false;
            has_left_start_ = true;
        }
    } else if (dist < enter_radius_) {
        in_decision_area_ = true;
        if (has_left_start_) {
            lap_count_++;
        }
    }
    return lap_count_ > number_of_stopped_turns_;
}

bool RaceDirector::shouldPublishStop() const {
    if (!stop_published_) {
        return true;
    }
    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - last_publish_time_).count();
    return elapsed_s > STOP_HEARTBEAT_INTERVAL_S;
}

void RaceDirector::markStopPublished() {
    stop_published_ = true;
    last_publish_time_ = std::chrono::steady_clock::now();
}

int RaceDirector::lapCount() const {
    return lap_count_;
}
