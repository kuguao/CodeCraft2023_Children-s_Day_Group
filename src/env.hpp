#ifndef _ENV_HPP
#define _ENV_HPP

#include <tuple>
#include <vector>

#include "common.hpp"

using namespace std;

class Workbench {
public:
    int index_;
    int type_;
    Vec2d pos_;
    int remain_time_;
    int material_state_;
    int have_product_;
};

class Robot {
public:
    int index_;
    int workbech_id_;
    int item_;
    double tv_factor_;
    double cv_factor_;
    double angular_;
    Vec2d linear_;
    double angle_;
    Vec2d pos_;
};

class RobotCmd {
public:
    double linear_;
    double angular_;
    bool buy_;
    bool sell_;
    bool destroy_;
};

const char OBSTACLE = '#';

class GridMap {
  public:
    GridMap() {}
    GridMap(const vector<vector<char>>& map) : map_(map) {
        rows_ = map.size();
        cols_ = map[0].size();
        grid_.resize(rows_, vector<int>(cols_, 0));
        build_grid_map();
    }

    void build_grid_map();
    int Getgridvalue(int row, int col);

    vector<vector<char>> map_;//原始地图
    vector<vector<int>> grid_;//栅格地图
    vector<vector<char>> ela_map_;//原始地图精细
    vector<vector<int>> ela_grid_;//精细栅格地图
    
    int rows_;
    int cols_;
};

bool dfs(int x, int y);
// tuple <GridMap, vector<Workbench>, vector<Robot>, vector<vector<int>>> Init();
tuple <GridMap, vector<Workbench>, vector<Robot>, vector<vector<int>>, vector<Workbench>, vector<Robot>> Init();
// tuple<int, int, vector<Robot>> get_state(vector<Workbench> &workbenchs, vector<Robot> &robots);
tuple<int, int, vector<Robot>, vector<vector<double>>> get_state(vector<Workbench> &workbenchs, vector<Robot> &robots, vector<vector<double>> &lidar_data);
vector<vector<int>> Connect_detect(vector<Workbench> workbenchs, vector<Robot>& robots);
void set_robot(RobotCmd rcs[4], int frame_id);
inline void env_start() {puts("OK");fflush(stdout);}
bool check_wb_sell(vector<Workbench> &workbenchs, const int& i, const int& j);
bool check_wb_buy(vector<Workbench> &workbenchs, const int& i, const int& j); 
extern int FLAG;
extern double MAX_VEL;

#endif