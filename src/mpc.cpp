#include <iostream>
#include <iomanip>
#include <queue>

#include "mpc.hpp"
#include "common.hpp"
#include "newton_method.hpp"
#include "sdf.hpp"

using namespace std;

Vec2d *obs_pos;

double cost_function(void *instance, const VectorXd &x, VectorXd &g) {
#define LENGTH_COST_W 0.6
#define CONTROL_COST_W 2.0
#define SDF_COST_W 0.5
#define COLLISION_COST_W 0.2
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
#define RISK_RADIUS 1.2
    for (int i = 0; i < x.rows() / 2.0; i++) {
        for (int j = 0; j < 3; j++) {
            double dx = x(i * 2) - obs_pos[j].x_;
            double dy = x(i * 2 + 1) - obs_pos[j].y_;
            double norm = dx * dx + dy * dy;
            if (norm > RISK_RADIUS) {

            } else {
                collsion_cost += (RISK_RADIUS - norm) * COLLISION_COST_W;
                g(i * 2) -= 2 * dx * COLLISION_COST_W;
                g(i * 2 + 1) -= 2 * dy * COLLISION_COST_W;
            }
        }
    }

    g(0) = g(1) = 0;
    g(x.rows() - 2) = g(x.rows() - 1) = 0;
    // cerr << length_cost << " " << control_cost << " " << sdf_cost << " " << collsion_cost << endl;

    return length_cost + control_cost + sdf_cost + collsion_cost;
}

vector<Vec2d> mpc(const Vec2d pos, const Vec2d vel, const double heading, const double angular, vector<Vec2d> raw_path, SdfMap &sdf) {
    if (raw_path.size() < 1) {
        return vector<Vec2d>();
    }
    int horizon = 5;
    VectorXd x(horizon * 2);
    //生成初始解
    for (int i = 0; i < horizon; i++) {

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

    return vector<Vec2d>();
}