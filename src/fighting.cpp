#include <iostream>
#include <queue>
#include <map>
#include <algorithm>
#include <cerrno>
#include <memory>

#include "common.hpp"
#include "env.hpp"
#include "astar.hpp"
#include "sdf.hpp"
#include "trajectory_optimization.hpp"
#include "car_model.hpp"
#include "circle_fit.hpp"
#include "scheduler.hpp"
#include "fighting.hpp"

using namespace std;

static char grid_visited[200][200];

int block_enemy(
    vector<Vec2d> dynamic_objects, vector<double> dynamic_objects_radius,
    Robot robot, FightingData &data, AstarSearch *astar,
    SdfMap &sdf_map, RobotCmd &cmd, vector<Vec2d> &path,
    vector<Workbench> &other_workbenchs) {
    Vec2d wbg;
    bool have_wbg = false;
    int other_num_7 = 0, other_num_8 = 0, other_num_9 = 0;
    for (int i = 0; i < other_workbenchs.size(); i++) {
        if (other_workbenchs[i].type_ == 7) {
            other_num_7++;
        } else if (other_workbenchs[i].type_ == 8) {
            other_num_8++;
        } else if (other_workbenchs[i].type_ == 9) {
            other_num_9++;
        }
    }
    if (other_num_8 == 1) {
        for (int i = 0; i < other_workbenchs.size(); i++) {
            if (other_workbenchs[i].type_ == 8) {
                wbg = other_workbenchs[i].pos_;
                have_wbg = true;
            }
        }
    }

    if (!data.enemy_locked_) { //查找目标
FIND_BLOCK_ENEMY:
        double min_dis = INFINITY;
        Vec2d obj_pos;
        for (auto obj : dynamic_objects) {
            // double dis = (obj - robot.pos_).mod();
            vector<Vec2d> path;
            auto found = astar->search(robot.pos_, obj, path, grid_visited, false);
            double dis = 0.0;
            if (found) {
                dis = calculate_path_dis(path);
            } else {
                dis = INFINITY;
            }
            if (dis < min_dis) {
                min_dis = dis;
                obj_pos = obj;
            }
        }
        if (min_dis > 1000 || ((obj_pos - wbg).mod() > 10.0 && have_wbg)) {
            auto found = astar->search(robot.pos_, wbg, path, grid_visited, false);
            path = path_optimize(robot.pos_, path, nullptr, nullptr, sdf_map, false, vector<Vec2d>(), FLAG);
            while (true) {
                if (path.size() > 1 && (robot.pos_ - path[0]).mod() < 1.0) {
                    path.erase(path.begin());
                } else {
                    break;
                }
            }
            if (path.size() > 1) {
                auto vel = compute_vel(robot.pos_, robot.angle_, path[0], robot.angular_, MAX_VEL, 10.0);
                cmd.linear_ = vel.x_;
                cmd.angular_ = vel.y_;
            }
            return -1;
        } else {
            data.enemy_locked_ = true;
            data.enemy_pos_ = obj_pos;
            data.loss_cnt_ = 0;
        }
    } else { //根据上一帧目标位置实现目标追踪
        double min_dis = INFINITY;
        Vec2d obj_pos;
        for (auto obj : dynamic_objects) {
            double dis = (obj - data.enemy_pos_).mod();
            if (dis < min_dis) {
                min_dis = dis;
                obj_pos = obj;
            }
        }
        if (min_dis > 2.0 || ((obj_pos - wbg).mod() > 10.0 && have_wbg)) { //目标丢失
            data.loss_cnt_++;
            if (data.loss_cnt_ > 5) {
                data.enemy_locked_ = false;
                goto FIND_BLOCK_ENEMY;
            }
            
        } else {
            data.loss_cnt_ = 0;
            data.past_enemy_pos_ = data.enemy_pos_;
            data.enemy_pos_ = obj_pos;
        }
    }

    //追踪目标
    Vec2d vel;
    auto found = astar->search(robot.pos_, data.enemy_pos_, path, grid_visited, false);
    path = path_optimize(robot.pos_, path, nullptr, nullptr, sdf_map, false, vector<Vec2d>(), 0);
    while (true) {
        if (path.size() > 1 && (robot.pos_ - path[0]).mod() < 1.0) {
            path.erase(path.begin());
        } else {
            break;
        }
    }
    if (!found || path.size() < 1 || (data.enemy_pos_ - robot.pos_).mod() < 1.0) {
        vel = compute_vel(robot.pos_, robot.angle_, data.enemy_pos_, robot.angular_, MAX_VEL, 200.0);
    } else {
        vel = compute_vel(robot.pos_, robot.angle_, path[0], robot.angular_, MAX_VEL, 200.0);
    }
    cmd.linear_ = vel.x_;
    cmd.angular_ = vel.y_;
    return 1;
}

int block_workbench(
    Vec2d wb_pos, vector<Vec2d> dynamic_objects,
    vector<double> dynamic_objects_radius,
    Robot robot, FightingData &data,
    AstarSearch *astar, SdfMap &sdf_map,
    RobotCmd &cmd, vector<Vec2d> &path) {
    Vec2d vel;
    if (!data.reach_wb_) {
        auto found = astar->search(robot.pos_, wb_pos, path, grid_visited, false);
        path = path_optimize(robot.pos_, path, nullptr, nullptr, sdf_map, false, vector<Vec2d>(), 0);
        while (true) {
            if (path.size() > 1 && (robot.pos_ - path[0]).mod() < 1.0) {
                path.erase(path.begin());
            } else {
                break;
            }
        }
        if (!found) {
            return -1;
        }
        if ((wb_pos - robot.pos_).mod() < 1.0) {
            vel = compute_vel(robot.pos_, robot.angle_, wb_pos, robot.angular_, MAX_VEL, 10.0);
        } else {
            vel = compute_vel(robot.pos_, robot.angle_, path[0], robot.angular_, MAX_VEL, 10.0);
        }
        if ((wb_pos - robot.pos_).mod() < 0.2) {
            data.reach_wb_ = true;
        }
    } else {
        if (!data.enemy_locked_) { //查找目标
FIND_BLOCK_ENEMY:
            double min_dis = INFINITY;
            Vec2d obj_pos;
            for (auto obj : dynamic_objects) {
                // double dis = (obj - robot.pos_).mod();
                vector<Vec2d> path;
                auto found = astar->search(robot.pos_, obj, path, grid_visited, false);
                double dis = 0.0;
                if (found) {
                    dis = calculate_path_dis(path);
                } else {
                    dis = INFINITY;
                }
                if (dis < min_dis) {
                    min_dis = dis;
                    obj_pos = obj;
                }
            }
            if (min_dis > 3.5) {
                // return -1;
            } else {
                data.enemy_locked_ = true;
                data.enemy_pos_ = obj_pos;
                data.loss_cnt_ = 0;
            }
        } else { //根据上一帧目标位置实现目标追踪
            double min_dis = INFINITY;
            Vec2d obj_pos;
            for (auto obj : dynamic_objects) {
                double dis = (obj - data.enemy_pos_).mod();
                if (dis < min_dis) {
                    min_dis = dis;
                    obj_pos = obj;
                }
            }
            if ((obj_pos - wb_pos).mod() > 4.0) {
                data.enemy_locked_ = false;
                goto FIND_BLOCK_ENEMY;
            }
            else if (min_dis > 2.0) { //目标丢失
                data.loss_cnt_++;
                if (data.loss_cnt_ > 5) {
                    data.enemy_locked_ = false;
                    goto FIND_BLOCK_ENEMY;
                }
            } else {
                data.loss_cnt_ = 0;
                data.past_enemy_pos_ = data.enemy_pos_;
                data.enemy_pos_ = obj_pos;
            }
        }
        // cerr << data.enemy_locked_ << endl;
        // if (data.enemy_locked_) {
        //     auto found = astar->search(robot.pos_, data.enemy_pos_, path, grid_visited, false);
        //     path = path_optimize(robot.pos_, path, nullptr, nullptr, sdf_map, false, vector<Vec2d>());
        //     while (true) {
        //         if (path.size() > 1 && (robot.pos_ - path[0]).mod() < 1.0) {
        //             path.erase(path.begin());
        //         } else {
        //             break;
        //         }
        //     }
        //     if (!found || path.size() < 1 || (data.enemy_pos_ - robot.pos_).mod() < 1.0) {
        //         vel = compute_vel(robot.pos_, robot.angle_, data.enemy_pos_, robot.angular_, MAX_VEL, 200.0);
        //     } else {
        //         vel = compute_vel(robot.pos_, robot.angle_, path[0], robot.angular_, MAX_VEL, 200.0);
        //     }

        //     if ((data.enemy_pos_ - wb_pos).mod() > 3.0) {
        //         vel = compute_vel(robot.pos_, robot.angle_, wb_pos, robot.angular_, MAX_VEL, 200.0);
        //     }
        // } else {
        //     vel = compute_vel(robot.pos_, 
        //         robot.angle_ > 0 ? robot.angle_ - M_PI : robot.angle_ + M_PI, 
        //         wb_pos, robot.angular_, MAX_VEL, 5.0);
        //     vel.x_ = -vel.x_;
        // }

        // data.pos_ob_ = data.pos_ob_ + data.vel_ob_ * 0.02;
        // Vec2d ob_err = data.enemy_pos_ - data.pos_ob_;
        // data.pos_ob_ = data.pos_ob_ + ob_err * 0.1;
        // data.vel_ob_ = data.vel_ob_ + ob_err * 0.3;
        // cerr << data.enemy_pos_.x_ << " " << data.enemy_pos_.y_ << " -> " << data.pos_ob_.x_ << " " << data.pos_ob_.y_ << endl;
        Vec2d enemy_vel = (data.enemy_pos_ - data.past_enemy_pos_) / 0.02;
        LIMIT(enemy_vel.x_, -2, 7);
        LIMIT(enemy_vel.y_, -2, 7);
        Vec2d delta_pos = data.enemy_pos_ - robot.pos_;
        double dis = delta_pos.mod();
        double delta_angle;
        double ang = delta_pos.ang();
        if (fabs(robot.angle_ - ang) > M_PI) {
            if (robot.angle_ > ang) {
                delta_angle = -(M_PI - robot.angle_ + ang + M_PI);
            } else {
                delta_angle = M_PI - ang + robot.angle_ + M_PI;
            }
        } else {
            delta_angle = robot.angle_ - ang;
        }
        // double t_theta = fabs(delta_angle) / (M_PI / 2.0);
        // double t_colli = (data.enemy_pos_ + enemy_vel * t_theta - robot.pos_).mod() / (3.0 + enemy_vel.mod()) + t_theta;
        double t_colli = 0.3;
        Vec2d colli_pos = data.enemy_pos_ + enemy_vel * t_colli;
        double remain_dis = (colli_pos - wb_pos).mod();
        // cerr << remain_dis << " " << enemy_vel.mod() << " " << t_colli << " " << (data.enemy_pos_ - robot.pos_).mod() << endl;
        if ((remain_dis < 1.5 || (data.enemy_pos_ - wb_pos).mod() < 1.5) && (robot.pos_ - wb_pos).mod() < 0.4) {
            vel = compute_vel(robot.pos_, robot.angle_, data.enemy_pos_, robot.angular_, MAX_VEL, 4.0);
        } else {
            vel = compute_vel(robot.pos_, 
                robot.angle_, 
                wb_pos, robot.angular_, MAX_VEL, 5.0);
            // vel = compute_vel(robot.pos_, 
            //     robot.angle_ > 0 ? robot.angle_ - M_PI : robot.angle_ + M_PI, 
            //     wb_pos, robot.angular_, MAX_VEL, 5.0);
            // vel.x_ = -vel.x_;
            if ((wb_pos - robot.pos_).mod() < 0.1) {
                vel = compute_vel(robot.pos_, robot.angle_, data.enemy_pos_, robot.angular_, MAX_VEL, 10.0);
                vel.x_ = 0.0;
            }
        }
    }
    cmd.linear_ = vel.x_;
    cmd.angular_ = vel.y_;
    return 1;
}

int block_workbench_static(
    Vec2d wb_pos, vector<Vec2d> dynamic_objects,
    vector<double> dynamic_objects_radius,
    Robot robot, FightingData &data,
    AstarSearch *astar, SdfMap &sdf_map,
    RobotCmd &cmd, vector<Vec2d> &path) {
    //　前往工作台
    data.reach_wb_ = false;
    Vec2d vel;
    double dist = (wb_pos - robot.pos_).mod();
    if(dist > 3) {
        auto found = astar->search(robot.pos_, wb_pos, path, grid_visited, false);
        path = path_optimize(robot.pos_, path, nullptr, nullptr, sdf_map, false, vector<Vec2d>(), 0);
        while (true) {
            if (path.size() > 1 && (robot.pos_ - path[0]).mod() < 1.0) {
                path.erase(path.begin());
            } else {
                break;
            }
        }
        if (!found) {
            return -1;
        }
        vel = compute_vel(robot.pos_, robot.angle_, path[0], robot.angular_, MAX_VEL, 200.0);
    }
    else {
        vel = compute_vel(robot.pos_, robot.angle_, wb_pos, robot.angular_, MAX_VEL, 10.0);
    }
    cmd.linear_ = vel.x_;
    cmd.angular_ = vel.y_;
    return 1;
}

void fighting_scheduler(
    RobotTask *tasks, vector<Robot> &robots,
    FightingTask *ftasks, FightingData *datas,
    vector<Workbench> &other_workbenchs, int map_id,
    int frame_id) {
    for (int i = 0; i < 4; i++) {
        if (!tasks[i].is_idle_) {
            ftasks[i].fight_type_ = IDLE;
        }
    }

    int other_num_7 = 0, other_num_9 = 0;
    for (int i = 0; i < other_workbenchs.size(); i++) {
        if (other_workbenchs[i].type_ == 7) {
            other_num_7++;
        } else if (other_workbenchs[i].type_ == 9) {
            other_num_9++;
        }
    }
    if (map_id == 2) {
        for (int i = 0; i < 4; i++) {
            RobotTask &task = tasks[i];
            FightingTask &ftask = ftasks[i];
            Robot &robot = robots[i];
            FightingData &data = datas[i];
            if (task.is_idle_ && (ftask.fight_type_ == IDLE)) {
                ftask.fight_type_ = BLOCK_ENEMY;
            }
        }
    } else if (map_id == 4) {
        if (other_num_7 <= 3) { //分配最多三个机器人占领7号工作台
            for (int i = 0; i < other_num_7; i++) {
                RobotTask &task = tasks[i];
                FightingTask &ftask = ftasks[i];
                Robot &robot = robots[i];
                FightingData &data = datas[i];
                if (task.is_idle_ && (ftask.fight_type_ == IDLE)) {
                    ftask.fight_type_ = BLOCK_WB;
                    for (int j = 0, k = 0; j < other_workbenchs.size(); j++) {
                        if (other_workbenchs[j].type_ == 7) {
                            if (k++ == i) {
                                ftask.blocked_wb_ = j;
                                ftask.blocked_pos_ = other_workbenchs[j].pos_;
                            }
                        }
                    }
                }
            }
            for (int i = other_num_7; i < 4; i++) {
                RobotTask &task = tasks[i];
                FightingTask &ftask = ftasks[i];
                Robot &robot = robots[i];
                FightingData &data = datas[i];
                if (task.is_idle_ && (ftask.fight_type_ == IDLE)) {
                    ftask.fight_type_ = BLOCK_ENEMY;
                }
            }
        } else {
            for (int i = 0; i < 4; i++) {
                RobotTask &task = tasks[i];
                FightingTask &ftask = ftasks[i];
                Robot &robot = robots[i];
                FightingData &data = datas[i];
                if (task.is_idle_ && (ftask.fight_type_ == IDLE)) {
                    ftask.fight_type_ = BLOCK_ENEMY;
                }
            }
        }
    } else {
        if (other_num_7 == 1 && FLAG == 0) {
            for (int i = 0; i < 4; i++) {
                RobotTask &task = tasks[i];
                FightingTask &ftask = ftasks[i];
                Robot &robot = robots[i];
                FightingData &data = datas[i];
                if (task.is_idle_ && (ftask.fight_type_ == IDLE)) {
                    ftask.fight_type_ = BLOCK_WB;
                    for (int j = 0, k = 0; j < other_workbenchs.size(); j++) {
                        if (other_workbenchs[j].type_ == 7) {
                            if (k++ == i) {
                                ftask.blocked_wb_ = j;
                                ftask.blocked_pos_ = other_workbenchs[j].pos_;
                            }
                        }
                    }
                }
            }
        } else {
            for (int i = 0; i < 4; i++) {
                RobotTask &task = tasks[i];
                FightingTask &ftask = ftasks[i];
                Robot &robot = robots[i];
                FightingData &data = datas[i];
                if (task.is_idle_ && (ftask.fight_type_ == IDLE)) {
                    ftask.fight_type_ = BLOCK_ENEMY;
                }
            }
        }
    }
}
