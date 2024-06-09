#pragma once

#include <iostream>
#include <queue>
#include <limits>
#include <math.h>

#include "common.hpp"

using namespace std;

#define CHECK_POS_IN_MAP(x, y) (x >=0.0 && x <= 50.0 && y >= 0.0 && y <= 50.0)

class SdfMap
{
public:
    double resolution_;
    int wid_voxel_num_;
    int hei_voxel_num_;
    vector<double> sdf_map_;
public:
    SdfMap(double resolution, vector<vector<int>> &grid_map);
    ~SdfMap();
    inline int to_address(int x, int y) {return y * wid_voxel_num_ + x;}
    inline double get_dis(Vec2i idx) {return sdf_map_[idx.x_ + (hei_voxel_num_ - idx.y_ + 1) * (wid_voxel_num_ + 2)];}
    template <typename T> inline pair<T, Vec2<T>> get_dist_with_grad_bilinear(Vec2<T> pos);
private:
    inline int to_address_(int x, int y) {return y * (wid_voxel_num_ + 2) + x;}
    template <typename F_get_val, typename F_set_val>
    inline void fill_esdf(F_get_val f_get_val, F_set_val f_set_val, int start, int end) {
        int v[end - start + 1];
        double z[end - start + 2];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q) k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }
};

inline SdfMap::SdfMap(double resolution, vector<vector<int>> &grid_map)
{
    auto t0 = clock();
    resolution_ = resolution;
    wid_voxel_num_ = int(50.0 / resolution_);
    hei_voxel_num_ = int(50.0 / resolution_);
    vector<double> tmp_buf1((hei_voxel_num_ + 2) * (wid_voxel_num_ + 2));
    vector<double> pdis_buf((hei_voxel_num_ + 2) * (wid_voxel_num_ + 2));
    vector<double> ndis_buf((hei_voxel_num_ + 2) * (wid_voxel_num_ + 2));
    double resolution_ratio = resolution_ / 0.5;

    //positive
    for (int y = 0; y < hei_voxel_num_ + 2; y++) {
        fill_esdf(
            [&](int x) {
                if (y > 0 && y < hei_voxel_num_ + 1 && x > 0 && x < wid_voxel_num_ + 1)
                    return grid_map[int((y - 1) * resolution_ratio)][int((x - 1) * resolution_ratio)] == 1 ?
                        0.0 :
                        numeric_limits<double>::max();
                else
                    return 0.0;
            },
            [&](int x, double val) {tmp_buf1[to_address_(x, y)] = val;}, 0,
            wid_voxel_num_ + 1);
    }

    for (int x = 0; x < wid_voxel_num_ + 2; x++) {
        fill_esdf(
            [&](int y) {
                return tmp_buf1[to_address_(x, y)];
            },
            [&](int y, double val) {pdis_buf[to_address_(x, y)] = resolution_ * sqrt(val);}, 0,
            hei_voxel_num_ + 1);
    }
    for (int y = 0; y < hei_voxel_num_ + 2; y++) {
        fill_esdf(
            [&](int x) {
                if (y > 0 && y < hei_voxel_num_ + 1 && x > 0 && x < wid_voxel_num_ + 1)
                    return grid_map[int((y - 1) * resolution_ratio)][int((x - 1) * resolution_ratio)] == 0 ?
                        0.0 :
                        numeric_limits<double>::max();
                else
                    return numeric_limits<double>::max();
            },
            [&](int x, double val) {tmp_buf1[to_address_(x, y)] = val;}, 0,
            wid_voxel_num_ + 1);
    }

    for (int x = 0; x < wid_voxel_num_ + 2; x++) {
        fill_esdf(
            [&](int y) {
                return tmp_buf1[to_address_(x, y)];
            },
            [&](int y, double val) {ndis_buf[to_address_(x, y)] = resolution_ * sqrt(val);}, 0,
            hei_voxel_num_ + 1);
    }
    // for (int i = 0; i < pdis_buf.size(); i++) {
    //     if (ndis_buf[i] > 0.0)
    //         sdf_map_.push_back(pdis_buf[i] - ndis_buf[i] + resolution_);
    //     else
    //         sdf_map_.push_back(pdis_buf[i]);
    // }
    for (int y = 0; y < hei_voxel_num_ + 2; y++) {
        for (int x = 0; x < wid_voxel_num_ + 2; x++) {
            // if (y > 0 && y < hei_voxel_num_ + 1 && x > 0 && x < wid_voxel_num_ + 1) {
            if (ndis_buf[to_address_(x, y)] > 0.0)
                sdf_map_.push_back(pdis_buf[to_address_(x, y)] - ndis_buf[to_address_(x, y)] + resolution_ / 2.0);
            else
                sdf_map_.push_back(pdis_buf[to_address_(x, y)] - resolution_ / 2.0);
            // }
        }
    }
    cerr << "Build ESDF, spend " << (clock() - t0) / 1.0e3 << "ms" << endl;
}

inline SdfMap::~SdfMap()
{

}

template <typename T>
inline pair<T, Vec2<T>> SdfMap::get_dist_with_grad_bilinear(Vec2<T> pos) {
    if (!CHECK_POS_IN_MAP(pos.x_, pos.y_)) {
        return make_pair(0.0, Vec2<T>(0.0, 0.0));
    }

    Vec2i idx(int((pos.x_ + 0.5 * resolution_) / resolution_), int((pos.y_ + 0.5 * resolution_) / resolution_));
    Vec2<T> idx_pos((idx.x_ - 0.5) * resolution_, (idx.y_ - 0.5) * resolution_);
    Vec2<T> diff = (pos - idx_pos) / resolution_;
    T values[2][2];
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            Vec2i current_idx = idx + Vec2i(x, y);
            values[x][y] = get_dis(current_idx);
        }
    }
    T v0 = (1 - diff.x_) * values[0][0] + diff.x_ * values[1][0];
    T v1 = (1 - diff.x_) * values[0][1] + diff.x_ * values[1][1];
    T dis = (1 - diff.y_) * v0 + diff.y_ * v1;
    Vec2<T> grad;
    grad.y_ = (v1 - v0) / resolution_;
    grad.x_ = ((1 - diff.y_) * (values[1][0] - values[0][0]) + diff.y_ * (values[1][1] - values[0][1])) / resolution_;
    return make_pair(dis, grad);
}