#include <iostream>
#include <iomanip>
#include <queue>

#include "circle_fit.hpp"
#include "common.hpp"
#include "newton_method.hpp"
#include "sdf.hpp"

using namespace std;

static double cost_function(void *instance, const VectorXd &x, VectorXd &g) {
    vector<Vec2d> *ps = (vector<Vec2d> *)instance;
    double cost = 0.0;
    g = VectorXd::Zero(g.rows());
    for (int i = 0; i < ps->size(); i++) {
        double dx = (*ps)[i].x_ - x(0);
        double dy = (*ps)[i].y_ - x(1);
        double norm = dx * dx + dy * dy;
        cost += (norm - x(2) * x(2)) * (norm - x(2) * x(2));
        g(0) += 2 * ((norm - x(2) * x(2))) * (-2 * dx);
        g(1) += 2 * ((norm - x(2) * x(2))) * (-2 * dy);
        g(2) += 2 * ((norm - x(2) * x(2))) * (-2 * x(2));
    }
    if (x(2) > 0.6) {
        cost += (x(2) - 0.6) * (x(2) - 0.6);
        g(2) += 2 * (x(2) - 0.6);
    } else if (x(2) < 0.4) {
        cost += (0.4 - x(2)) * (0.4 - x(2));
        g(2) -= 2 * (0.4 - x(2));
    }

    return cost;
}

pair<Vec2d, double> circle_fit(vector<Vec2d> points) {
    VectorXd x(3);
    x(2) = 0.45; //半径猜测
    Vec2d mid(0.0, 0.0);
    int cnt = 0;
    for (auto p : points) {
        mid = mid + p;
        cnt++;
    }
    mid = mid / cnt;
    x(0) = mid.x_;
    x(1) = mid.y_;

    double finalCost = 0.0;

    auto t0 = clock();
    int ret = lbfgs_optimize(x, finalCost, cost_function, &points);
    auto t1 = clock();
    
    // cerr << setprecision(4)
    //     << "================================" << endl
    //     << "L-BFGS Optimization Returned: " << ret << endl
    //     << "Minimized Cost: " << finalCost << endl
    //     << "Spend " << (t1 - t0) << "us" << endl
    //     << "Optimal Variables num: " << x.size() << endl;
    // cerr << x(0) << " " << x(1) << " " << x(2) << endl;

    return make_pair(Vec2d(x(0), x(1)), x(2));
}