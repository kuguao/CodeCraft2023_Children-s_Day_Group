#pragma once

#include "common.hpp"
#include "env.hpp"

#define CHECK_IN_MAP(x,y) (x >= 0 && x < 200 && y >= 0 && y < 200)

class AstarSearch {
public:
    GridMap *map_;
    GridMap map_1_;
    GridMap map_2_;
    GridMap map_3_;

public:
    AstarSearch() {}
    AstarSearch(GridMap map);
    ~AstarSearch() {}
    Vec2i pos_to_index(Vec2d pos) {
        return Vec2i(int(pos.x_ / 0.25), int((50.0 - pos.y_) / 0.25));
    }
    Vec2d index_to_pos(Vec2i idx) {
        return Vec2d(idx.x_ * 0.25 + 0.125, 50.0 - idx.y_ * 0.25 - 0.125);
    }
    double huristic(Vec2i pos1, Vec2i pos2) {
        // return (index_to_pos(pos1) - index_to_pos(pos2)).mod();
        Vec2d d = index_to_pos(pos1) - index_to_pos(pos2);
        d.x_ = fabs(d.x_);
        d.y_ = fabs(d.y_);
        double minv = min(min(d.x_, d.y_), 0.0);
        double maxv = max(d.x_, d.y_);
        double midv = d.x_ + d.y_- minv - maxv;
        return d.x_ + d.y_
            + (sqrt(3) - 3) * minv
            + (sqrt(2) - 2) * (midv - minv);
    }
    void find_neighbor(Vec2i idx, vector<Vec2i> &neig, vector<vector<int>> &map, bool no_check) {
        if (CHECK_IN_MAP(idx.x_ + 1, idx.y_ - 1) && map[idx.y_ - 1][idx.x_ + 1] == 0 && (map[idx.y_ - 1][idx.x_] == 0 && map[idx.y_][idx.x_ + 1] == 0 || no_check))
            neig.push_back(Vec2i(idx.x_ + 1, idx.y_ - 1));
        if (CHECK_IN_MAP(idx.x_ + 1, idx.y_ + 1) && map[idx.y_ + 1][idx.x_ + 1] == 0 && (map[idx.y_][idx.x_ + 1] == 0 && map[idx.y_ + 1][idx.x_] == 0 || no_check))
            neig.push_back(Vec2i(idx.x_ + 1, idx.y_ + 1));
        if (CHECK_IN_MAP(idx.x_ - 1, idx.y_ - 1) && map[idx.y_ - 1][idx.x_ - 1] == 0 && (map[idx.y_][idx.x_ - 1] == 0 && map[idx.y_ - 1][idx.x_] == 0 || no_check))
            neig.push_back(Vec2i(idx.x_ - 1, idx.y_ - 1));
        if (CHECK_IN_MAP(idx.x_ - 1, idx.y_ + 1) && map[idx.y_ + 1][idx.x_ - 1] == 0 && (map[idx.y_][idx.x_ - 1] == 0 && map[idx.y_ + 1][idx.x_] == 0 || no_check))
            neig.push_back(Vec2i(idx.x_ - 1, idx.y_ + 1));
        if (CHECK_IN_MAP(idx.x_, idx.y_ - 1) && map[idx.y_ - 1][idx.x_] == 0)
            neig.push_back(Vec2i(idx.x_, idx.y_ - 1));
        if (CHECK_IN_MAP(idx.x_, idx.y_ + 1) && map[idx.y_ + 1][idx.x_] == 0)
            neig.push_back(Vec2i(idx.x_, idx.y_ + 1));
        if (CHECK_IN_MAP(idx.x_ + 1, idx.y_) && map[idx.y_][idx.x_ + 1] == 0)
            neig.push_back(Vec2i(idx.x_ + 1, idx.y_));
        if (CHECK_IN_MAP(idx.x_ - 1, idx.y_) && map[idx.y_][idx.x_ - 1] == 0)
            neig.push_back(Vec2i(idx.x_ - 1, idx.y_));
    }
    bool search(Vec2d start_pos, Vec2d goal_pos, vector<Vec2d> &path, char grid_visited[200][200], bool is_inflate);
    bool search(
        Vec2d start_pos, Vec2d goal_pos, 
        vector<Vec2d> &path, char grid_visited[200][200], 
        bool is_inflate, vector<vector<Vec2d>> paths,
        vector<Vec2d> poss, bool check_pos_all_time,
        vector<int> &collision_robots, vector<int> &collision_robots_cnt,
        vector<Vec2d> lidar_pcl = vector<Vec2d>(), bool check_lidar = false);
    Vec2d bfs(Vec2d robot_pos, vector<Vec2d> obs, Vec2d poss, double dir, char grid_visited[200][200],vector<Vec2d> other_robots);
};

inline double calculate_path_dis(vector<Vec2d> &path) {
    double dis = 0.;
    if (path.size() == 0) {
        // cerr << " == 0" << endl;
        return 1000000;
    }
    for (int i = 0; i < path.size() - 1; i++) {
        dis += (path[i + 1] - path[i]).mod();
    }
    return dis;
}