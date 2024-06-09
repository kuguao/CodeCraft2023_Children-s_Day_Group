#include <iostream>
#include <iomanip>

#include "trajectory_optimization.hpp"
#include "common.hpp"
#include "newton_method.hpp"

using namespace std;

static Vec2d *obs_pos;
static Vec2d *obs_vel;
static bool avoid_enable;
static vector<Vec2d> dynamic_objects;
int flag;

static double cost_function(void *instance, const VectorXd &x, VectorXd &g) {
#define LENGTH_COST_W 0.6
#define CONTROL_COST_W 2.0
#define SDF_COST_W 0.5
#define COLLISION_COST_W 0.15
#define COLLISION2_COST_W 0.2
    double length_cost = 0.0;
    g = VectorXd::Zero(g.rows());
    for (int i = 0; i < x.rows() / 2.0 - 1; i++) {
        double dx = x((i + 1) * 2) - x(i * 2);
        double dy = x((i + 1) * 2 + 1) - x(i * 2 + 1);
        double norm = dx * dx + dy * dy;
        length_cost += norm * LENGTH_COST_W;
        g(i * 2) -= 2 * dx * LENGTH_COST_W;
        g(i * 2 + 1) -= 2 * dy * LENGTH_COST_W;
        g((i + 1) * 2) += 2 * dx * LENGTH_COST_W;
        g((i + 1) * 2 + 1) += 2 * dy * LENGTH_COST_W;
    }
    double control_cost = 0.0;
    for (int i = 0; i < x.rows() / 2.0 - 4; i++) {
        double dx = x(i * 2) + x((i + 2) * 2) - 2 * x((i + 1) * 2);
        double dy = x(i * 2 + 1) + x((i + 2) * 2 + 1) - 2 * x((i + 1) * 2 + 1);
        double norm = dx * dx + dy * dy;
        control_cost += norm * CONTROL_COST_W;
        g(i * 2) += 2 * dx * CONTROL_COST_W;
        g(i * 2 + 1) += 2 * dy * CONTROL_COST_W;
        g((i + 1) * 2) -= 4 * dx * CONTROL_COST_W;
        g((i + 1) * 2 + 1) -= 4 * dy * CONTROL_COST_W;
        g((i + 2) * 2) += 2 * dx * CONTROL_COST_W;
        g((i + 2) * 2 + 1) += 2 * dy * CONTROL_COST_W;
    }
    double sdf_cost = 0;
#define RISK_DIS 1.0
    for (int i = 0; i < x.rows() / 2.0; i++) {
        auto sdf = ((SdfMap *)instance)->get_dist_with_grad_bilinear(Vec2d(x(i * 2), x(i * 2 + 1)));
        double dis = sdf.first;
        Vec2d grad = sdf.second;
        if (dis > RISK_DIS) {

        } else {
            sdf_cost += (dis - RISK_DIS) * (dis - RISK_DIS) * SDF_COST_W;
            g(i * 2) += 2 * (dis - RISK_DIS) * grad.x_ * SDF_COST_W;
            g(i * 2 + 1) += 2 * (dis - RISK_DIS) * grad.y_ * SDF_COST_W;
        }
    }

    double collsion_cost = 0;
#define RISK_RADIUS 2.0
    if (avoid_enable) {
        for (int i = 0; i < x.rows() / 2.0; i++) {
            for (int j = 0; j < 3; j++) {
                double dx = x(i * 2) - obs_pos[j].x_ - obs_vel[j].x_ * 0.05 * i;
                double dy = x(i * 2 + 1) - obs_pos[j].y_ - obs_vel[j].y_ * 0.05 * i;
                double norm = dx * dx + dy * dy;
                if (norm > RISK_RADIUS) {

                } else {
                    collsion_cost += (RISK_RADIUS - norm) * COLLISION_COST_W;
                    g(i * 2) -= 2 * dx * COLLISION_COST_W;
                    g(i * 2 + 1) -= 2 * dy * COLLISION_COST_W;
                }
            }
        }
    }

    double collsion_cost2 = 0;
    if (flag == 1) {
        for (int i = 0; i < x.rows() / 2.0; i++) {
            for (int j = 0; j < dynamic_objects.size(); j++) {
                double dx = x(i * 2) - dynamic_objects[j].x_;
                double dy = x(i * 2 + 1) - dynamic_objects[j].y_;
                double norm = dx * dx + dy * dy;
                if (norm > RISK_RADIUS) {

                } else {
                    collsion_cost2 += (RISK_RADIUS - norm) * COLLISION2_COST_W;
                    g(i * 2) -= 2 * dx * COLLISION2_COST_W;
                    g(i * 2 + 1) -= 2 * dy * COLLISION2_COST_W;
                }
            }
        }
    }

    g(0) = g(1) = 0;
    g(x.rows() - 2) = g(x.rows() - 1) = 0;
    // cerr << length_cost << " " << control_cost << " " << sdf_cost << " " << collsion_cost << endl;

    return length_cost + control_cost + sdf_cost + collsion_cost + collsion_cost2;
}

vector<Vec2d> path_optimize(Vec2d pos, vector<Vec2d> raw_path, Vec2d obs[3], Vec2d vel[3], SdfMap &sdf, bool avoid_enable_, vector<Vec2d> dynamic_objects_, int flag_) {
    if (raw_path.size() < 2) {
        return raw_path;
    }
    raw_path[0] = pos;
    obs_pos = obs;
    obs_vel = vel;
    avoid_enable = avoid_enable_;
    dynamic_objects = dynamic_objects_;
    flag = flag_;
    // cerr << obs_vel[2].x_ << endl;
    // vector<Vec2d> down_sample_path;
    // for (int i = 0; i < raw_path.size(); i++) {
    //     if (i % 2 == 0 || i == raw_path.size() - 1)
    //         down_sample_path.push_back(raw_path[i]);
    // }
    // raw_path = down_sample_path;
    
    vector<Vec2d> left_path;
    if (raw_path.size() > 50) {
        for (int i = 50; i < raw_path.size(); i++) {
            left_path.push_back(raw_path[i]);
        }
        raw_path.erase((raw_path.begin() + 50), raw_path.end());
    }
    if (raw_path.size() < 10) {
        // vector<Vec2d> refine_path;
        // double dis = 0;
        // int index = 0;
        // refine_path.push_back(raw_path[0]);
        // double t = 0;
        // Vec2d p1 = raw_path[0];
        // while (index < raw_path.size() - 1) {
        //     t += 0.1;
        //     Vec2d p2 = raw_path[index] * (1 - t) + raw_path[index + 1] * t;
        //     if (t > 1) {
        //         t = 0;
        //         index++;
        //     } else {
        //         dis += (p2 - p1).mod();
        //         if (dis >= 0.1) {
        //             refine_path.push_back(p2);
        //             dis = 0.0;
        //         }
        //     }
        //     p1 = p2;
        // }
        // raw_path = refine_path;
    }

    VectorXd x(raw_path.size() * 2);
    for (int i = 0; i < raw_path.size(); i++) {
        x(i * 2) = raw_path[i].x_;
        x(i * 2 + 1) = raw_path[i].y_;
    }

    double finalCost = 0.0;

    auto t0 = clock();
    int ret = lbfgs_optimize(x, finalCost, cost_function, &sdf);
    auto t1 = clock();
    
    // cerr << setprecision(4)
    //     << "================================" << endl
    //     << "L-BFGS Optimization Returned: " << ret << endl
    //     << "Minimized Cost: " << finalCost << endl
    //     << "Spend " << (t1 - t0) << "us" << endl
    //     << "Optimal Variables num: " << x.size() << endl;

    vector<Vec2d> path;
    for (int i = 0; i < x.rows() / 2.0; i++) {
        path.push_back(Vec2d(x(i * 2), x(i * 2 + 1)));
    }

    //轨迹细化
    vector<Vec2d> refine_path;
    double dis = 0;
    int index = 0;
    refine_path.push_back(path[0]);
    double t = 0;
    Vec2d p1 = path[0];
    while (index < raw_path.size() - 1) {
        t += 0.1;
        Vec2d p2 = path[index] * (1 - t) + path[index + 1] * t;
        if (t > 1) {
            t = 0;
            index++;
        } else {
            dis += (p2 - p1).mod();
            if (dis >= 0.1) {
                refine_path.push_back(p2);
                dis = 0.0;
            }
        }
        p1 = p2;
    }

    for (int i = 0; i < left_path.size(); i++) {
        path.push_back(left_path[i]);
    }

    return refine_path;
}

vector<Vec2d> path_adjustment(vector<Vec2d> raw_path, SdfMap &sdf) {
    if (raw_path.size() < 2) {
        return raw_path;
    }

    vector<Vec2d> path = raw_path;
    for (int i = 1; i < path.size() - 1; i++) {
        // while (true) {
            auto sdfret = sdf.get_dist_with_grad_bilinear(path[i]);
            double dis = sdfret.first;
            Vec2d grad = sdfret.second;
            // if (dis < 0.45) {
            //     path[i] += grad
            // }
            if (dis < 0.6) {
                // path[i] = path[i] + grad * 0.4;
            }
        // }
    }

    //轨迹细化
    vector<Vec2d> refine_path;
    double dis = 0;
    int index = 0;
    refine_path.push_back(path[0]);
    double t = 0;
    Vec2d p1 = path[0];
    while (index < raw_path.size() - 1) {
        t += 0.1;
        Vec2d p2 = path[index] * (1 - t) + path[index + 1] * t;
        if (t > 1) {
            t = 0;
            index++;
        } else {
            dis += (p2 - p1).mod();
            if (dis >= 0.1) {
                refine_path.push_back(p2);
                dis = 0.0;
            }
        }
        p1 = p2;
    }

    return path;
}