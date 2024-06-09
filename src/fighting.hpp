#pragma once

#include <queue>

#include "common.hpp"
#include "astar.hpp"
#include "sdf.hpp"

using namespace std;

class FightingData {
public:
    Vec2d enemy_pos_;
    Vec2d past_enemy_pos_;
    // Vec2d past2_enemy_pos_;
    Vec2d pos_ob_;
    Vec2d vel_ob_;
    bool enemy_locked_;
    int loss_cnt_;

    bool reach_wb_;


    FightingData() : 
        enemy_pos_(0.0, 0.0),
        past_enemy_pos_(0.0, 0.0),
        enemy_locked_(false),
        loss_cnt_(0),
        reach_wb_(false) {}
};

enum FightType {
    IDLE = 0,
    BLOCK_ENEMY,
    BLOCK_WB,
    BLOCK_STATIC
};

class FightingTask {
public:
    FightType fight_type_;
    int blocked_wb_;
    Vec2d blocked_pos_;
    
    FightingTask() : fight_type_(IDLE), blocked_wb_(0), blocked_pos_(0.0, 0.0) {}
};

int block_enemy(
    vector<Vec2d> dynamic_objects, vector<double> dynamic_objects_radius,
    Robot robot, FightingData &data, AstarSearch *astar,
    SdfMap &sdf_map, RobotCmd &cmd, vector<Vec2d> &path,
    vector<Workbench> &other_workbenchs);

int block_workbench(
    Vec2d wb_pos, vector<Vec2d> dynamic_objects,
    vector<double> dynamic_objects_radius,
    Robot robot, FightingData &data,
    AstarSearch *astar, SdfMap &sdf_map,
    RobotCmd &cmd, vector<Vec2d> &path);

int block_workbench_static(
    Vec2d wb_pos, vector<Vec2d> dynamic_objects,
    vector<double> dynamic_objects_radius,
    Robot robot, FightingData &data,
    AstarSearch *astar, SdfMap &sdf_map,
    RobotCmd &cmd, vector<Vec2d> &path);

void fighting_scheduler(
    RobotTask *tasks, vector<Robot> &robot,
    FightingTask *ftasks, FightingData *data,
    vector<Workbench> &other_workbenchs, int map_id,
    int frame_id);