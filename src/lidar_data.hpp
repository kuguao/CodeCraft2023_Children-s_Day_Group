#pragma once

#include <queue>

#include "common.hpp"
#include "env.hpp"
#include "sdf.hpp"

using namespace std;

class DynamicObsFilter {
public:
    vector<Vec2d> pcls_;

    DynamicObsFilter() {}
    bool can_see(Vec2d p1, Vec2d p2, SdfMap &sdf) {
        bool can = true;
        if ((p1 - p2).mod() > 25) {
            can = false;
        } else {
            for (double t = 0; t < 1; t += 0.01) {
                if (sdf.get_dist_with_grad_bilinear(p1 * t + p2 * (1 - t)).first < 0.1) {
                    can = false;
                    break;
                }
            }
        }
        return can;
    }
    void update(vector<Robot> robots, vector<Vec2d> pcl, SdfMap &sdf) {
        //利用视场删除旧障碍物
        vector<Vec2d> new_pcls;
        for (int i = 0; i < pcls_.size(); i++) {
            Vec2d p_ = pcls_[i];
            bool can = false;
            for (auto r : robots) {
                if (can_see(p_, r.pos_, sdf)) {
                    can = true;
                    break;
                }
            }
            if (can) {
                bool is = false;
                for (auto p : pcl) {
                    if ((p - p_).mod() < 0.25) {
                        is = true;
                        break;
                    }
                }
                if (!is) {

                } else {
                    new_pcls.push_back(p_);
                }
            } else {
                new_pcls.push_back(p_);
            }
        }
        pcls_ = new_pcls;
        // pcls_.clear();
        //添加障碍物
        for (auto p : pcl) {
            bool have = false;
            for (auto p_ : pcls_) {
                if ((p_ - p).mod() < 0.25) {
                    have = true;
                    break;
                }
            }
            if (!have) {
                pcls_.push_back(p);
            }
        }
    }
};
