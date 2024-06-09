#include "scheduler.hpp"

vector<bool> workbench_mark;
int workbench_mask_cnt[9];
vector<vector<int>> new_wb_index;

vector<vector<BSPair>> sellpairs(4);
vector<vector<BSPair>> buypairs(4);

double buy_price[] = {3000, 4400, 5800, 15400, 17200, 19200, 76000};
double sell_price[] = {6000, 7600, 9200, 22500, 25000, 27500, 105000};

vector<int*> material_masks;
vector<bool> product_masks;

int num_4;//生产4的数量
int num_5;//生产4的数量
int num_6;//生产4的数量
bool balance_flag;

static char grid_visited[200][200];

void scheduler_run(
    RobotTask robottask[4],
    vector<Workbench> &workbenchs, vector<Robot> &robots,
    RobotCmd *cmds, int frame_id, AstarSearch *astar_search) {
    static bool is_init = false;
    balance_flag = true;

    if (!is_init) {
        for (int i = 0; i < workbenchs.size(); i++) {
            auto mask = new int[8];
            for (int j = 0; j < 8; j++) {
                mask[j] = -1;
            }
            if (workbenchs[i].type_ == 4) {
                mask[0] = mask [1] = false;
            } else if (workbenchs[i].type_ == 5) {
                mask[0] = mask [2] = false;
            } else if (workbenchs[i].type_ == 6) {
                mask[1] = mask [2] = false;
            } else if (workbenchs[i].type_ == 7) {
                mask[3] = mask[4] = mask [5] = false;
            } else if (workbenchs[i].type_ == 8) {
                mask[6] = false;
            } else if (workbenchs[i].type_ == 9) {
                mask[0] = mask[1] = mask [2] = mask[3] = false;
                mask[4] = mask [5] = mask[6] = false;
            }
            material_masks.push_back(mask);
            auto prud = new bool;
            *prud = false;
            product_masks.push_back(prud);
        }
    }

    if (!is_init) {
        for (int i = 0; i < 4; i++) {
            robottask[i].index_ = i;
        }
    }

    if (!is_init) {
        for (int i = 0; i < workbenchs.size(); i++) {
            workbench_mark.push_back(false);
        }
    }

    //创建空闲机器人的容器
    vector<int> idle_robots;
    for (int i = 0; i < 4; i++) {
        if (robottask[i].is_idle_ && !robottask[i].is_fighting_) {
            idle_robots.push_back(robottask[i].index_);
        }
    }
    // cerr << "idle_robots.size: " << idle_robots.size() << endl;

    //更新material_masks
    for (int i = 0; i < workbenchs.size(); i++) {
        auto mask = material_masks[i];
        if (workbenchs[i].type_ == 4) {
            mask[0] = (workbenchs[i].material_state_ >> 1) & 1;
            mask[1] = (workbenchs[i].material_state_ >> 2) & 1;
            product_masks[i] = workbenchs[i].have_product_;
        } else if (workbenchs[i].type_ == 5) {
            mask[0] = (workbenchs[i].material_state_ >> 1) & 1;
            mask[2] = (workbenchs[i].material_state_ >> 3) & 1;
            product_masks[i] = workbenchs[i].have_product_;
        } else if (workbenchs[i].type_ == 6) {
            mask[1] = (workbenchs[i].material_state_ >> 2) & 1;
            mask[2] = (workbenchs[i].material_state_ >> 3) & 1;
            product_masks[i] = workbenchs[i].have_product_;
        } else if (workbenchs[i].type_ == 7) {
            mask[3] = (workbenchs[i].material_state_ >> 4) & 1;
            mask[4] = (workbenchs[i].material_state_ >> 5) & 1;
            mask[5] = (workbenchs[i].material_state_ >> 6) & 1;
            product_masks[i] = workbenchs[i].have_product_;
        } else if (workbenchs[i].type_ == 8) {
            mask[6] = false;
        }
        product_masks[i] = true;
    }
    for (int r = 0; r < 4; r++) {
        if (!robottask[r].is_idle_) {
            if (workbenchs[robottask[r].to_].type_ >= 4 && workbenchs[robottask[r].to_].type_ <= 8) {
                material_masks[robottask[r].to_][workbenchs[robottask[r].from_].type_ - 1] = true;
            }
            if (workbenchs[robottask[r].from_].type_ >= 1 && workbenchs[robottask[r].from_].type_ <= 7 && !robottask[r].is_bought_) {
                product_masks[robottask[r].from_] = false;
            } /*else {
                product_masks[robottask[r].from_] = true;
            }*/
        }
    }

    if (idle_robots.size() != 0) {//如果有机器人空闲
        for (int i = 0; i < idle_robots.size(); i++) {
            int robotindex = idle_robots[i];
            Robot &robot = robots[robotindex];
            vector<TaskSolution> solutions;
            TaskSolution optimal(nullptr, 0, 0);
            //查找所有可行解
            // cerr << "sellpairs[robotindex].size: " << sellpairs[robotindex].size() << endl;
            for (int j = 0; j < sellpairs[robotindex].size(); j++) {
                // const BSPair &bs = bspairs[j];
                const BSPair &bs = sellpairs[robotindex][j];
                Workbench &from = workbenchs[bs.from_];
                Workbench &to = workbenchs[bs.to_];
                
                if (from.type_ <= 3) {
                    if ((from.have_product_ || (from.remain_time_ >= 0 && from.remain_time_ < get_move_time(robot.pos_, from.pos_, 4.0))) &&
                        /*product_masks[from.index_] &&*/
                        check_to_need_from(from, to)) {
                        double dis = 0; 
                        double buy_dis = 0;
                        if (robot.workbech_id_ > 0) {
                            for (int i = 0; i < buypairs[robotindex].size(); i++) {
                                if ((buypairs[robotindex][i].from_ == robot.workbech_id_) && (buypairs[robotindex][i].to_ == bs.from_)) {
                                    buy_dis = buypairs[robotindex][i].dis_;
                                    break;
                                }
                            }
                        }
                        else {
                            vector<Vec2d> path;
                            astar_search->search(robot.pos_, from.pos_, path, grid_visited, false);
                            buy_dis = calculate_path_dis(path);
                        }
                        dis = bs.dis_ + buy_dis;
                        double cost = get_solution_cost(robot, from, to, dis, robottask[robotindex], frame_id);
                        if(!have_time_complete_task(dis, frame_id)) {
                            cost = -1;
                        }
                        auto solution = TaskSolution(&sellpairs[robotindex][j], dis, cost);
                        solutions.push_back(solution);
                        if (solution.cost_ > optimal.cost_) {
                            optimal = solution;
                        }
                    }
                } else {
                    if ((from.have_product_ || (from.remain_time_ >= 0 && from.remain_time_ < get_move_time(robot.pos_, from.pos_, 4.0))) &&
                        product_masks[from.index_] &&
                        check_to_need_from(from, to) &&
                        check_is_optimal(from, to, robot, robotindex, robottask, robots)) {
                        double dis = 0; 
                        double buy_dis = 0;
                        if (robot.workbech_id_ > 0) {
                            for (int i = 0; i < buypairs[robotindex].size(); i++) {
                                if ((buypairs[robotindex][i].from_ == robot.workbech_id_) && (buypairs[robotindex][i].to_ == bs.from_)) {
                                    buy_dis = buypairs[robotindex][i].dis_;
                                    break;
                                }
                            }
                        }
                        else {
                            vector<Vec2d> path;
                            astar_search->search(robot.pos_, from.pos_, path, grid_visited, false);
                            buy_dis = calculate_path_dis(path);
                        }
                        dis = bs.dis_ + buy_dis;
                    
                        double cost = get_solution_cost(robot, from, to, dis, robottask[robotindex], frame_id);

                        if(!have_time_complete_task(dis, frame_id)) {
                            cost = -1;
                        }
                        auto solution = TaskSolution(&sellpairs[robotindex][j], dis, cost);
                        solutions.push_back(solution);
                        if (solution.cost_ > optimal.cost_) {
                            optimal = solution;
                        }
                    }
                }
            }
            // cerr << "robot " << robotindex << " check optimal bspair cost: " << optimal.cost_ << endl;
            if (optimal.bspair_ && (optimal.cost_ > 0) ) {
                robottask[robotindex].is_idle_ = false;
                robottask[robotindex].is_bought_ = false;
                robottask[robotindex].dis_ = optimal.total_dis_;
                robottask[robotindex].from_ = optimal.bspair_->from_;
                robottask[robotindex].last_to_ = robottask[robotindex].to_;
                robottask[robotindex].to_ = optimal.bspair_->to_;
                robottask[robotindex].init1_ = true;
                robottask[robotindex].init2_ = true;
                robottask[robotindex].start_frame_ = frame_id;
                robottask[robotindex].sub_num_ = 0;
                robottask[robotindex].start_pos_ = robots[robotindex].pos_;
                robottask[robotindex].path1_ = vector<Vec2d>();
                robottask[robotindex].path2_ = vector<Vec2d>();
                robottask[robotindex].normal_path = vector<Vec2d>();
                robottask[robotindex].from_type_ = workbenchs[robottask[robotindex].from_].type_;
                robottask[robotindex].to_type_ = workbenchs[robottask[robotindex].to_].type_;
                robottask[robotindex].start_time_ = frame_id;
                robottask[robotindex].estimate_time_ = optimal.total_dis_ / 3.5; //秒为单位
                robottask[robotindex].timeout_ = false;
                // cerr << "robot " << robotindex << " robottask[robotindex].to_" << robottask[robotindex].to_ << endl;
                // cerr << "type: " << workbenchs[robottask[robotindex].to_].type_ << endl;
                if (workbenchs[robottask[robotindex].to_].type_ == 4) {
                    num_4++;
                }
                if (workbenchs[robottask[robotindex].to_].type_ == 5) {
                    num_5++;
                }
                if (workbenchs[robottask[robotindex].to_].type_ == 6) {
                    num_6++;
                }
                // cerr << "num_4: " << num_4 << endl;
                // cerr << "num_5: " << num_5 << endl;
                // cerr << "num_6: " << num_6 << endl;


                if (workbenchs[optimal.bspair_->from_].type_ <= 3) {
                    product_masks[optimal.bspair_->from_] = false;
                } else {
                    // workbench_mark[optimal.bspair_->from_] = true;
                    product_masks[optimal.bspair_->from_] = false;
                }
                // workbench_mark[optimal.bspair_->to_] = true;

                if (workbenchs[optimal.bspair_->to_].type_ >= 4 && workbenchs[optimal.bspair_->to_].type_ <= 8) {
                    material_masks[optimal.bspair_->to_][workbenchs[optimal.bspair_->from_].type_ - 1] = true;
                }
            }
        }
    }
    is_init = true;
    return;
}
