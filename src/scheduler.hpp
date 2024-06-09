#pragma once

#include <iostream>
#include <queue>
#include "common.hpp"
#include "env.hpp"
#include "astar.hpp"

class RobotTask {
public:
    int index_;
    bool is_idle_;
    bool is_fighting_;

    int from_;  //去哪个工作台买
    int to_;    //去哪个工作台卖
    bool is_bought_;    //买到了吗
    double dis_;
    int start_frame_;
    bool init1_;
    bool init2_;
    vector<Vec2d> path1_;
    vector<Vec2d> path2_;
    Vec2d start_pos_;
    int last_to_;
    vector<Vec2d> normal_path;
    vector<Vec2d> no_delete_path;
    int from_type_;
    int to_type_;
    double start_time_;
    double estimate_time_;
    bool timeout_;

    RobotTask *sub_task_;
    int sub_num_;

    RobotTask():
        index_(0), is_idle_(true), is_fighting_(false),
        from_(0), to_(0), is_bought_(false), dis_(0.0), start_frame_(0), init1_(false), init2_(false), sub_task_(nullptr), sub_num_(0), timeout_(false) {}
    RobotTask(int index, bool is_idle, int from, int to, bool is_bought, double dis, int start_frame):
        index_(index), is_idle_(is_idle), is_fighting_(false),
        from_(from), to_(to), is_bought_(is_bought),
        dis_(dis), start_frame_(start_frame), init1_(false), init2_(false), sub_task_(nullptr), sub_num_(0), timeout_(false) {}
    vector<Vec2d> get_now_path() {
        if (is_idle_)
            return vector<Vec2d>();
        if (is_bought_)
            return path2_;
        else
            return path1_;
    }
    vector<Vec2d> get_no_delete_path() {
        if (is_idle_)
            return vector<Vec2d>();
        return no_delete_path;
    }
};

class BSPair {
public:
    int from_;
    int to_;
    double dis_;

    BSPair():
        from_(0), to_(0), dis_(0.0) {}
    BSPair(int from, int to, double dis):
        from_(from), to_(to), dis_(dis) {}
};

class TaskSolution {
public:
    BSPair *bspair_;
    double total_dis_;
    double cost_;

    TaskSolution():
        bspair_(nullptr), total_dis_(0.0), cost_(0.0) {}
    TaskSolution(BSPair *pair, double total_dis, double cost):
        bspair_(pair), total_dis_(total_dis), cost_(cost) {}
};

extern vector<bool> workbench_mark;
extern int workbench_mask_cnt[9];
extern vector<vector<int>> new_wb_index;
extern vector<vector<BSPair>> sellpairs;
extern vector<vector<BSPair>> buypairs;
extern double buy_price[];
extern double sell_price[];
extern vector<int*> material_masks;
extern vector<bool> product_masks;
extern int num_4;//生产4的数量
extern int num_5;//生产5的数量
extern int num_6;//生产6的数量
extern bool balance_flag;

inline bool have_time_complete_task(double dis, int frame_id) {
    // double dis = (from - robot).mod() + (to - from).mod();
    double time = dis / 4.0;
    if (time < (12000 - frame_id) / 50.0) {
        return true;
    } else {
        return false;
    }
    // return true;
}

inline bool have_time_complete_final_task(double dis, int frame_id) {
    double time = dis / 3.0;
    if (time < (12000 - frame_id) / 50.0) {
        return true;
    } else {
        return false;
    }
    // return true;
}

inline double get_move_time(Vec2d pos1, Vec2d pos2, double speed) {
    return (pos1 - pos2).mod() / speed * 50.0;
}

inline double get_solution_cost(Robot &robot, Workbench &from, Workbench &to, double dis, RobotTask &task, int frame_id) {
    double direct_reward = sell_price[from.type_ - 1] - buy_price[from.type_ - 1];
    double future_reward = 0.0;

    //强行平均456
    if ((num_4 <= num_5) && (num_4 <= num_6) && (((num_5 - num_4) > 1) || ((num_6 - num_4) > 1)) 
        && (to.type_ == 4) && balance_flag) {
        direct_reward = 10 * direct_reward;
    }
    if ((num_5 <= num_4) && (num_5 <= num_6) && (((num_4 - num_5) > 1) || ((num_6 - num_5) > 1)) 
        && (to.type_ == 5) && balance_flag) {
        direct_reward = 10 * direct_reward;
    }
    if ((num_6 <= num_5) && (num_6 <= num_4) && (((num_5 - num_6) > 1) || ((num_4 - num_6) > 1)) 
        && (to.type_ == 6) && balance_flag) {
        direct_reward = 10 * direct_reward;
    }
    
    //增加去7的收益

    if(to.type_ == 7) {
        direct_reward = 100 * direct_reward;
    }
    if(to.type_ == 8) {
        direct_reward = 100000000000 * direct_reward;
    }

    // if((to.material_state_ > 0) && (map_id == 1)) {
    //     direct_reward = 100 * direct_reward;
    // }

    if((to.material_state_ > 0)) {
        direct_reward = 100 * direct_reward;
    }

    return (direct_reward + future_reward) / dis;
}

inline bool check_to_need_from(Workbench &from, Workbench &to) {
#if 1
    if (material_masks[to.index_][from.type_ - 1] == false) {
        return true;
    }
#else
    if (material_masks[to.index_][from.type_ - 1] == false) {
        return true;
    } else {
        if (to.type_ == 4 && material_masks[to.index_][2 - 1] == 1 && material_masks[to.index_][1 - 1] == 1 && (from.type_ == 1 || from.type_ == 2) && from.remain_time_ != 0) {
            int cnt = 0;
            for (int i = 0; i < 4; i++) {
                if (!tasks[i].is_idle_ && tasks[i].is_bought_ && tasks[i].is_bought_ && tasks[i].to_ == to.index_ && tasks[i].from_type_ == from.type_) {
                    cnt++;
                }
            }
            if (cnt == 1) {
                return true;
            }
        } else if (to.type_ == 5 && material_masks[to.index_][1 - 1] == 1 && material_masks[to.index_][3 - 1] == 1 && (from.type_ == 1 || from.type_ == 3) && from.remain_time_ != 0) {
            int cnt = 0;
            for (int i = 0; i < 4; i++) {
                if (!tasks[i].is_idle_ && tasks[i].is_bought_ && tasks[i].is_bought_ && tasks[i].to_ == to.index_ && tasks[i].from_type_ == from.type_) {
                    cnt++;
                }
            }
            if (cnt == 1) {
                return true;
            }
        } else if (to.type_ == 6 && material_masks[to.index_][2 - 1] == 1 && material_masks[to.index_][3 - 1] == 1 && (from.type_ == 2 || from.type_ == 3) && from.remain_time_ != 0) {
            int cnt = 0;
            for (int i = 0; i < 4; i++) {
                if (!tasks[i].is_idle_ && tasks[i].is_bought_ && tasks[i].is_bought_ && tasks[i].to_ == to.index_ && tasks[i].from_type_ == from.type_) {
                    cnt++;
                }
            }
            if (cnt == 1) {
                return true;
            }
        }
    }
#endif
    return false;
}

inline bool check_is_optimal(Workbench &from, Workbench &to, Robot &robot, int robotindex, RobotTask *robottask, vector<Robot> &robots) {
    // return true;
    double dis = (robot.pos_ - from.pos_).mod();
    for (int i = 0; i < 4; i++) {
        if (i != robotindex && !robottask[i].is_idle_ && robottask[i].is_bought_ && robottask[i].to_ == from.index_) {
            if ((robots[i].pos_ - from.pos_).mod() < dis) {
                return false;
            }
        }
    }
    return true;
}

inline bool switch_task(int &cnt, RobotTask &task, vector<Workbench> &workbenchs, int index, RobotCmd *cmds, vector<Robot> &robots, AstarSearch *astar, vector<Vec2d> &path, double min_dis, char grid_visited[200][200]) {
    bool found = false;
    bool have_robot = false;
    for (int m = 0; m < new_wb_index[index].size(); m++) {
        int i = new_wb_index[index][m];
        if ((workbenchs[i].type_ == workbenchs[task.to_].type_) && (workbenchs[i].index_ != workbenchs[task.to_].index_) && check_to_need_from(workbenchs[task.from_], workbenchs[i])) {
            //检查是否有自己人在周围
            for (int j = 0; j < robots.size(); j++) {
                if (j == index) continue;
                if ((robots[j].pos_ - workbenchs[i].pos_).mod() < 0.8) {
                    have_robot = true;
                    continue;
                }
            }
            //检查通过
            if (!have_robot) {
                // cerr << "switch_task from " << workbenchs[task.to_].index_ << "to " << workbenchs[i].index_ << "---------------------" << endl;
                found = astar->search(robots[index].pos_, workbenchs[i].pos_, path, grid_visited, true);
                double dis;
                if (found) {
                    dis = calculate_path_dis(path);
                    if (dis > min_dis) {
                        continue;
                    } else {
                        task.to_ = workbenchs[i].index_;
                        cnt++;
                        return true;
                    }
                }
            }
        }  
    }

    if (!found) {
        for (int m = 0; m < new_wb_index[index].size(); m++) {
            int i = new_wb_index[index][m];
            if ((workbenchs[i].type_ == 9) && (workbenchs[task.to_].type_ == 8)) {
                // cerr << "switch_task from " << workbenchs[task.to_].index_ << "to " << workbenchs[i].index_ << endl;
                task.to_ = workbenchs[i].index_;
                cnt++;
                return true;
            }
        }
    }
    return false;
}

void scheduler_run(
    RobotTask robottask[4],
    vector<Workbench> &workbenchs, vector<Robot> &robots,
    RobotCmd *cmds, int frame_id, AstarSearch *astar_search);