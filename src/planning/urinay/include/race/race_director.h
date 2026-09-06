#ifndef RACE_DIRECTOR_H
#define RACE_DIRECTOR_H

#include <chrono>

class RaceDirector {
   public:
    RaceDirector(int number_of_stopped_turns, double start_x = 0.0, double start_y = 0.0, double enter_radius = 2.0,
                 double exit_radius = 6.0);

    // 使用当前车辆位置 (x, y) 进行更新。
    // 如果比赛结束（圈数超过限制），返回 true。
    bool update(double car_x, double car_y);

    // 如果现在应该发布停止信号，则返回 true
    // （首次完成或心跳间隔已过）。
    bool shouldPublishStop() const;

    // 在发布停止信号后调用。
    void markStopPublished();

    int lapCount() const;

   private:
    int number_of_stopped_turns_;
    double start_x_;
    double start_y_;
    double enter_radius_;
    double exit_radius_;
    int lap_count_ = 0;
    bool in_decision_area_ = false;
    bool has_left_start_ = false;
    bool stop_published_ = false;
    std::chrono::steady_clock::time_point last_publish_time_;
    static constexpr double STOP_HEARTBEAT_INTERVAL_S = 0.5;
};

#endif
