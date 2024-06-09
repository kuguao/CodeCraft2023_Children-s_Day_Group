#pragma once

#include "common.hpp"
#include "env.hpp"
#include "astar.hpp"
#include "sdf.hpp"
#include "trajectory_optimization.hpp"

inline static double angle_norm(double angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }

    return angle;
}

class DiffModel {
public:
    Vec2d pos_;
    double angle_;

    DiffModel(): pos_(0.0, 0.0), angle_(0.0) {}
    DiffModel(Vec2d pos, double angle): pos_(pos), angle_(angle) {}
    DiffModel update_diff_model(Vec2d cvel, double dt) const {
        double mid_angle = angle_norm(angle_ + cvel.y_ * dt / 2.0);
        double new_angle = angle_norm(angle_ + cvel.y_ * dt);
        Vec2d new_pos(pos_.x_ + cvel.x_ * cos(mid_angle) * dt, pos_.y_ + cvel.x_ * sin(mid_angle) * dt);
        return DiffModel(new_pos, new_angle);
    }
};

inline Vec2d compute_vel(Vec2d pos, double angle, Vec2d goal, double angular_fb, const double max_vel, const double velp) {
    Vec2d delta_pos = goal - pos;
    double dis = delta_pos.mod();
    double delta_angle;
    double ang = delta_pos.ang();
    if (fabs(angle - ang) > M_PI) {
        if (angle > ang) {
            delta_angle = -(M_PI - angle + ang + M_PI);
        } else {
            delta_angle = M_PI - ang + angle + M_PI;
        }
    } else {
        delta_angle = angle - ang;
    }

    // double vel = 6.0;
    
    double linear = LIMIT_R(dis * velp, -2, max_vel);
    if (fabs(delta_angle) > M_PI * 1.0 / 4) {
        linear = 0.0;
    }
    static double last_delta_angle = 0.0;
    double angular = -LIMIT_R(delta_angle * 10.0 + angular_fb * 0.0, -M_PI, M_PI);
    last_delta_angle = delta_angle;
    if (dis < 0.1) {
        angular = 0.0;
    }
    return Vec2d(linear, angular);
}

inline double get_angle_delta(Vec2d pos, double angle, Vec2d goal) {
    Vec2d delta_pos = goal - pos;
    double dis = delta_pos.mod();
    double delta_angle;
    double ang = delta_pos.ang();
    if (fabs(angle - ang) > M_PI) {
        if (angle > ang) {
            delta_angle = -(M_PI - angle + ang + M_PI);
        } else {
            delta_angle = M_PI - ang + angle + M_PI;
        }
    } else {
        delta_angle = angle - ang;
    }
    return delta_angle;
}
