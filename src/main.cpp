#include <iostream>
#include <queue>
#include <map>
#include <algorithm>
#include <cerrno>
#include <memory>

// #define ROS
#ifdef ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <tf/tf.h>
#endif

#include "common.hpp"
#include "env.hpp"
#include "astar.hpp"
// #include "matplotlibcpp.hpp"
#include "sdf.hpp"
#include "trajectory_optimization.hpp"
#include "car_model.hpp"
#include "circle_fit.hpp"
#include "scheduler.hpp"
#include "fighting.hpp"
#include "lidar_data.hpp"

using namespace std;
// namespace plt = matplotlibcpp;

int frame_id = 0, score = 0;
int map_id = 0;

static GridMap grid_map;
static vector<Workbench> workbenchs;
static vector<Workbench> other_workbenchs;
static vector<Robot> other_robots;
static vector<Robot> robots;
static vector<vector<double>> lidar_data(4, vector<double>(360));
static vector<vector<Vec2d>> paths[100];
// vector<vector<vector<Vec2d>>> paths_sell(50, vector<vector<Vec2d>>(50));
// vector<vector<vector<Vec2d>>> paths_buy(50, vector<vector<Vec2d>>(50));
static char grid_visited[200][200];
shared_ptr<AstarSearch> astar_search;
vector<Vec2d> robot_init_pos(4);

inline double collision_cost_function2(const Vec2d pos, const vector<Vec2d> obs, const double radius) {
    double min_dis = 9999999;
    for (int i = 0; i < obs.size(); i++) {
        double dis = (pos - obs[i]).mod();
        if (dis < min_dis) {
            min_dis = dis;
        }
    }
    if (min_dis <= radius * 1.5) {
        return INFINITY;
    } else {
        if (min_dis <= radius * 4.0)
            return 1 / (min_dis - radius * 1.5) - 1 / (2.5 * radius);
        else
            return 0.0;
    }
}

inline double cost_function2(
    const Vec2d pos, const double angle, const Vec2d goal,
    const Vec2d cvel, const Vec2d last_cvel, const vector<Vec2d> obs,
    const double q, const double r, const double c, SdfMap &sdf,
    DiffModel diffmodel, bool active_avoid) {
    double target_cost = (goal - pos).mod(); //target代价
    double control_cost = (cvel - last_cvel).mod(); //控制代价
    double coll_cost = 0; //碰撞代价

    double delta_angle;
    double ang = (goal - pos).ang();
    if (fabs(angle - ang) > M_PI) {
        if (angle > ang) {
            delta_angle = -(M_PI - angle + ang + M_PI);
        } else {
            delta_angle = M_PI - ang + angle + M_PI;
        }
    } else {
        delta_angle = angle - ang;
    }
    // printInfo("ang:%f angle:%f delta_angle:%f\n\r", ang, angle, delta_angle);
    delta_angle = fabs(delta_angle);
    double heading_cost = 0.0;
    if (delta_angle > M_PI / 4.0) {
        heading_cost = (delta_angle - M_PI / 4.0) / (M_PI * 3 / 4.0);
    }

    double risk_radius = 0.43;
    if (map_id == 4) {
        risk_radius = 0.46;
    }

    coll_cost = collision_cost_function2(pos, obs, risk_radius);
    if (coll_cost > 0.0) {
        delta_angle = 0;
    }
    double heading_weight = 3.0;
    // if (coll_cost > 0.05) {
    //     heading_weight = 0.0;
    // }
    double sdf_cost = 0.0;
    DiffModel model = diffmodel;
    for (int i = 1; i < 9; i++) {
        model = model.update_diff_model(cvel, 0.02);
        double sdf_cost_ = sdf.get_dist_with_grad_bilinear(model.pos_).first;
        // sdf_cost_ = exp(-(sdf_cost_ - 0.60) * 1);
        if (sdf_cost_ > 0.6) {
            sdf_cost_ = 0.0;
        } else {
            sdf_cost_ = pow(sdf_cost_ - 0.6, 2);
        }
        sdf_cost += sdf_cost_;
    }
    sdf_cost /= 8.0;
    // cerr << sdf_cost << endl;
    if (active_avoid) {
        // target_cost = heading_cost = 0.0;
        // coll_cost *= 10.0;
        coll_cost = 0.0;
    }
    if (cvel.x_ > 0)
        return 0.4 * target_cost + 0.0 * control_cost + 0.0 * coll_cost + heading_weight * heading_cost + (6 - fabs(cvel.x_)) / 60.0 * 1.5 + sdf_cost * 1.5;
    else
        return 0.4 * target_cost + 0.0 * control_cost + 0.0 * coll_cost + heading_weight * heading_cost + (2 - fabs(cvel.x_)) * 3.0 / 60.0 * 1.5 + sdf_cost * 1.5;
    // return 0.4 * target_cost + heading_weight * heading_cost + (2 - fabs(cvel.x_)) * 3.0 / 60.0 * 2.0 + sdf_cost * 0;
}

#define DwaSearchNodePtr DwaSearchNode*

class DwaSearchNode {
public:
    Vec2d pos_;
    double angle_;
    Vec2d cvel_;
    double g_cost_;
    double cost_;
    int step_;
    DwaSearchNodePtr parent_;

    DwaSearchNode(): pos_(0.0, 0.0), angle_(0.0), cvel_(0.0, 0.0), g_cost_(0.0), cost_(0.0), step_(0), parent_(nullptr) {}
    DwaSearchNode(Vec2d pos, double angle, Vec2d cvel, double g_cost, double cost, int step, DwaSearchNodePtr parent):
        pos_(pos), angle_(angle), cvel_(cvel), g_cost_(g_cost), cost_(cost), step_(step), parent_(parent) {}
};

class DwaSearchNodeComparator {
public:
    bool operator()(DwaSearchNodePtr node1, DwaSearchNodePtr node2) {
        return node1->cost_ > node2->cost_;
    }
};

inline Vec2d dwa_search(Vec2d pos, double angle, Vec2d cvel, Vec2d goal, Vec2d obs_pos[3], double obs_angle[3], Vec2d obs_cvel[3], SdfMap &sdf, bool active_avoid) {
    int horizon = 1;

    vector<Vec2d> cvels;
    for (int i = 0; i < 9 * 4; i++) {
        for (int j = 0; j < 9 * 4; j++) {
            cvels.push_back(Vec2d(-2 + i * 1.0 / 4.35, -M_PI + M_PI * j / 4.0 / 4.0));
        }
    }

    DiffModel init_obs_model[3];
    for (int i = 0; i < 3; i++) {
        init_obs_model[i].pos_ = obs_pos[i];
        init_obs_model[i].angle_ = obs_angle[i];
    }

    DiffModel obsmodel[horizon][3];
    vector<Vec2d> obs_predict_pos[horizon];
    for (int i = 0; i < horizon; i++) {
        if (0 == i) {
            for (int j = 0; j < 3; j++) {
                obsmodel[i][j] = init_obs_model[j].update_diff_model(obs_cvel[j], 0.02 * 5.5);
                obs_predict_pos[i].push_back(obsmodel[i][j].pos_);
            }
        } else {
            for (int j = 0; j < 3; j++) {
                obsmodel[i][j] = obsmodel[i - 1][j].update_diff_model(obs_cvel[j], 0.02 * 5.5);
                obs_predict_pos[i].push_back(obsmodel[i][j].pos_);
            }
        }
    }

    priority_queue<DwaSearchNodePtr, vector<DwaSearchNodePtr>, DwaSearchNodeComparator> openset;
    vector<DwaSearchNodePtr> nodeset;

    DwaSearchNodePtr node = new DwaSearchNode(pos, angle, cvel, 0, 0, 0, nullptr);
    nodeset.push_back(node);
    openset.push(node);

    DwaSearchNodePtr end_node = nullptr;

    // printInfo("ff\n\r");

    int cnt = 0;
    
    while (!openset.empty()) {
        DwaSearchNodePtr node = openset.top();
        openset.pop();
        DiffModel diffmodel(node->pos_, node->angle_);

        if (node->step_ > horizon - 1) {
            end_node = node;
            break;
        }
        // printInfo("(%f %f %f) vel:(%f %f) step:%d\n\r",
        //     diffmodel.pos_.x_, diffmodel.pos_.y_, diffmodel.angle_, node->cvel_.x_, node->cvel_.y_, node->step_);

        for (int i = 0; i < cvels.size(); i++) {
            DiffModel new_diff_model = diffmodel.update_diff_model(cvels[i], 0.02 * 5.5);
            // if ((new_diff_model.pos_ - goal).mod() < 0.1) {
            //     end_node = node;
            //     cerr << "sa" << endl;
            //     break;
            // }
            // printInfo("(%f %f %f)->(%f %f %f) vel:(%f %f) ",
            //     diffmodel.pos_.x_, diffmodel.pos_.y_, diffmodel.angle_,
            //     new_diff_model.pos_.x_, new_diff_model.pos_.y_, new_diff_model.angle_,
            //     cvels[i].x_, cvels[i].y_);
            
            double now_cost = cost_function2(new_diff_model.pos_, new_diff_model.angle_, goal, cvels[i], node->cvel_, obs_predict_pos[node->step_], 0.0, 0.0, 0.0, sdf, diffmodel, active_avoid);
            double g_cost = now_cost + node->g_cost_;
            double cost = g_cost + now_cost * (horizon - node->step_ - 1);
            cost = now_cost;
            // printInfo("cost:%f\n\r", cost);
            DwaSearchNodePtr new_node = new DwaSearchNode(new_diff_model.pos_, new_diff_model.angle_, cvels[i], g_cost, cost, node->step_ + 1, node);
            nodeset.push_back(new_node);
            openset.push(new_node);
            // printInfo("dd\n\r");
        }
    }
    DwaSearchNodePtr start_node = end_node;
    while (start_node->parent_ != nullptr && start_node->parent_->parent_ != nullptr) {
        start_node = start_node->parent_;
    }
    for (int i = 0; i < nodeset.size(); i++) {
        delete nodeset[i];
    }
    // printInfo("end: %f, %f\n\r", start_node->cvel_.x_, start_node->cvel_.y_);

    return start_node->cvel_;
}

int get_task_goal_index(RobotTask &task) {
    if (task.is_bought_) {
        return task.to_;
    } else {
        return task.from_;
    }
}

void System_init() {
    cerr << "System_init" << endl;
    vector<Vec2d> path;
    auto init = Init();
    grid_map = get<0>(init);
    workbenchs = get<1>(init);
    // if ((workbenchs[0].type_ == 2) && (workbenchs[1].type_ == 1) && (workbenchs[2].type_ == 7)  && (workbenchs[3].type_ == 7)) {
    //     map_id = 1;
    // }
    // if ((workbenchs[0].type_ == 8) && (workbenchs[1].type_ == 1) && (workbenchs[2].type_ == 8) && (workbenchs[3].type_ == 8)) {
    //     map_id = 2;
    // }
    robots = get<2>(init);
    for (int i = 0; i < 4; i++) {
        robot_init_pos[i] = robots[i].pos_;
    }
    new_wb_index = get<3>(init);
    // cerr << "workbenchs.size()" << workbenchs.size() << endl;
    cerr << "new_wb_index" << endl;
    for (int i = 0; i < new_wb_index.size(); i++) {
        for(int j = 0; j < new_wb_index[i].size(); j++) {
            cerr << new_wb_index[i][j] << " , ";
        }
        cerr << endl;
    }
    int new_wb_zero_count;
    //判断地图
    for (int i = 0; i < new_wb_index.size(); i++) {
        if(new_wb_index[i].size() == 0) {
            new_wb_zero_count++;
        }
    }
    if(new_wb_zero_count == 1) {
        map_id = 2;
    }
    if(new_wb_zero_count == 4) {
        map_id = 4;
    }
    if ((workbenchs.size() > 3) && (workbenchs[0].type_ == 1) && (workbenchs[1].type_ == 2) && (workbenchs[2].type_ == 4)) {
        map_id = 3;
    }
    if ((workbenchs.size() > 3) && (workbenchs[0].type_ == 3) && (workbenchs[1].type_ == 3) && (workbenchs[2].type_ == 7)) {
        map_id = 1;
    }
    cerr << "map_id" << map_id << endl;
    other_workbenchs = get<4>(init);
    other_robots = get<5>(init);
    astar_search = make_shared<AstarSearch>(AstarSearch(grid_map));
    auto t_0 = clock();
    for (int r = 0; r < new_wb_index.size(); r++) {
        // cerr << "sell-------------------------" <<endl;
        for (int n = 0; n < new_wb_index[r].size(); n++) {
            int i = new_wb_index[r][n];
            //买工作台去卖的路径 膨胀后的路径
            //123->456
            //456->7/9
            //7->8/9
            for (int m = 0; m < new_wb_index[r].size(); m++) {
                int j = new_wb_index[r][m];
                path.clear();
                if (check_wb_sell(workbenchs, i, j)) {
                    if (r == 0) {
                        astar_search->search(workbenchs[i].pos_, workbenchs[j].pos_, path, grid_visited, true);
                        // paths_sell[i][j] = path;
                        double dis = 0;
                        dis = calculate_path_dis(path);
                        // cerr << "i = " << i << "j = " << j << "dis =" << dis <<endl;
                        sellpairs[r].push_back(BSPair(i, j, dis));
                    }
                    else {
                        bool found = false;
                        // cerr << "------------------------r: " << r << endl;
                        for (int a = 0; a < r; a++) {
                            for (int b = 0; b < sellpairs[a].size(); b++) {
                                if (sellpairs[a][b].from_ == i && sellpairs[a][b].to_ == j) {
                                    double dis = sellpairs[a][b].dis_;
                                    sellpairs[r].push_back(BSPair(i, j, dis));
                                    // cerr << "copy i j dis" << ", i = "<< i << " , " << "j = " << j << "dis = " << dis << endl;
                                    found = true;
                                    break;
                                }
                            }
                            if(found){
                                break;
                            }
                        }
                        if(!found)
                        {
                            astar_search->search(workbenchs[i].pos_, workbenchs[j].pos_, path, grid_visited, true);
                            // paths_sell[i][j] = path;
                            double dis = 0;
                            dis = calculate_path_dis(path);
                            // cerr << "i = " << i << "j = " << j << "dis =" << dis <<endl;
                            sellpairs[r].push_back(BSPair(i, j, dis));
                        }
                        // cerr << "i = " << i << "j = " << j << "dis =" << dis <<endl;
                    }
                }
            }
        }
        //去工作台买的路径
        //不膨胀
        //456789->1234567
        // cerr << "buy-------------------------" <<endl;
        for (int n = 0; n < new_wb_index[r].size(); n++) {
            int i = new_wb_index[r][n];
            for (int m = 0; m < new_wb_index[r].size(); m++) {
                int j = new_wb_index[r][m];
                path.clear();
                if (check_wb_buy(workbenchs, i, j)) {
                    if (r == 0) {
                        astar_search->search(workbenchs[i].pos_, workbenchs[j].pos_, path, grid_visited, false);
                        // paths_sell[i][j] = path;
                        double dis = 0;
                        dis = calculate_path_dis(path);
                        // cerr << "i = " << i << "j = " << j << "dis =" << dis <<endl;
                        buypairs[r].push_back(BSPair(i, j, dis));
                    }
                    else {
                        bool found = false;
                        // cerr << "------------------------r: " << r << endl;
                        for (int a = 0; a < r; a++) {
                            for (int b = 0; b < buypairs[a].size(); b++) {
                                if (buypairs[a][b].from_ == i && buypairs[a][b].to_ == j) {
                                    double dis = buypairs[a][b].dis_;
                                    buypairs[r].push_back(BSPair(i, j, dis));
                                    // cerr << "copy i j dis" << ", i = "<< i << " , " << "j = " << j << "dis = " << dis << endl;
                                    found = true;
                                    break;
                                }
                            }
                            if(found){
                                break;
                            }
                        }
                        if(!found)
                        {
                            astar_search->search(workbenchs[i].pos_, workbenchs[j].pos_, path, grid_visited, false);
                            // paths_sell[i][j] = path;
                            double dis = 0;
                            dis = calculate_path_dis(path);
                            cerr << "i = " << i << "j = " << j << "dis =" << dis <<endl;
                            buypairs[r].push_back(BSPair(i, j, dis));
                        }
                    }
                }
            }
        }
    }
    auto t_1 = clock();

    cerr << "-------------A* find all path, spend------------ " << (t_1 - t_0) / 1000.0 << "ms" << endl;
}

inline Vec2i pos_to_index(Vec2d& pos) {
    return Vec2i(min(max(int((50 - pos.y_ - 0.25) / 0.5),0),100), min(max(int((pos.x_ - 0.25) / 0.5), 0), 100));
}

//输入雷达数据，机器人
//返回障碍物坐标
inline vector<Vec2d> get_move_obstacle(vector<Robot> &robots, vector<vector<double>> &lidar_data, SdfMap &sdf) {
    Vec2d robot_pos;
    int angle = 0;
    vector<vector<Vec2d>> lidar_point_pos_w(4, vector<Vec2d>(360));
    for (int i = 0; i < 4; i++) {
        // robot_pos = robots[i].pos_;
        for (int j = 0; j < lidar_data[i].size(); j++) {
            //计算雷达上的点对应地图上的点
            Vec2d lidar_point_pos_r;//body坐标系
            double radians = M_PI / 180.0 * angle;
            // lidar_point_pos_r.x_ = (lidar_data[i][j] + 0.35) * cos(radians);
            // lidar_point_pos_r.y_ = (lidar_data[i][j] + 0.35) * sin(radians);
            lidar_point_pos_r.x_ = lidar_data[i][j] * cos(radians);
            lidar_point_pos_r.y_ = lidar_data[i][j] * sin(radians);
            
            // Vec2d lidar_point_pos_w;//世界坐标系
            lidar_point_pos_w[i][j].x_ = lidar_point_pos_r.x_ * cos(robots[i].angle_) - lidar_point_pos_r.y_ * sin(robots[i].angle_) + robots[i].pos_.x_;
            lidar_point_pos_w[i][j].y_ = lidar_point_pos_r.x_ * sin(robots[i].angle_) + lidar_point_pos_r.y_ * cos(robots[i].angle_) + robots[i].pos_.y_;
            angle += 1;
        }
    }
    vector<Vec2d> oprobot_pos;
    for (int i = 0; i < 4; i++) {
        // robot_pos = robots[i].pos_;
        for (int j = 0; j < lidar_point_pos_w[i].size(); j++) {
            // if(lidar_point_pos_w[i][j])
            Vec2i lidar_point_index = pos_to_index(lidar_point_pos_w[i][j]);
            // cerr << "lidar_point_pos_w[i].size()" << lidar_point_pos_w[i].size() << endl;
            if (sdf.get_dist_with_grad_bilinear(lidar_point_pos_w[i][j]).first > 0.1/*(lidar_point_index.x_ > 0) && (lidar_point_index.x_ < 99) && (lidar_point_index.y_ > 0) && (lidar_point_index.y_ < 99) &&
                grid_map.grid_[lidar_point_index.x_][lidar_point_index.y_] == 0 && grid_map.grid_[lidar_point_index.x_ - 1][lidar_point_index.y_] == 0 &&
                grid_map.grid_[lidar_point_index.x_ + 1][lidar_point_index.y_] == 0 && grid_map.grid_[lidar_point_index.x_][lidar_point_index.y_ + 1] == 0 &&
                grid_map.grid_[lidar_point_index.x_][lidar_point_index.y_ - 1] == 0 && grid_map.grid_[lidar_point_index.x_ - 1][lidar_point_index.y_ - 1] == 0 &&
                grid_map.grid_[lidar_point_index.x_ - 1][lidar_point_index.y_ + 1] == 0 && grid_map.grid_[lidar_point_index.x_ + 1][lidar_point_index.y_ - 1] == 0 &&
                grid_map.grid_[lidar_point_index.x_ + 1][lidar_point_index.y_ + 1] == 0*/){
                bool is_own = false;
                for (int k = 0; k < 4; k++) {
                    if (k == i)
                        continue;
                    double radius = 0.0;
                    if (robots[k].item_ == 0) {
                        radius = 0.45 * 1.08;
                    } else {
                        radius = 0.53 * 1.08;
                    }
                    if ((lidar_point_pos_w[i][j] - robots[k].pos_).mod() < radius) {
                        is_own = true;
                        break;
                    }
                }
                // cerr << "lidar_point_index.x_" << lidar_point_index.x_ << endl;
                // cerr << "lidar_point_index.y_" << lidar_point_index.y_ << endl;
                if (is_own)
                    continue;
                oprobot_pos.push_back(lidar_point_pos_w[i][j]);
            }
        }
    }
    // cerr << "oprobot_pos.size()" << oprobot_pos.size() << endl;
    return oprobot_pos;
}

#ifdef ROS
void publish_scan(ros::Publisher &pub, vector<Vec2d> oprobot_pos, vector<double> colors) {
    sensor_msgs::PointCloud pcl;
    pcl.header.frame_id = "map";
    // cerr << colors.size() << endl;
    // pcl.channels.resize(1);
    pcl.channels.resize(1);
    for (int i = 0; i < oprobot_pos.size(); i++) {
        geometry_msgs::Point32 p;
        p.x = oprobot_pos[i].x_;
        p.y = oprobot_pos[i].y_;
        p.z = 0.0;
        pcl.points.push_back(p);
        pcl.channels[0].name = "intensities";
        pcl.channels[0].values.push_back(colors[i]);
    }
    pub.publish(pcl);
}

void publish_grid(ros::Publisher &pub, vector<vector<int>> &grid) {
    nav_msgs::OccupancyGrid myMap;
    // 地图数据 
    myMap.info.map_load_time = ros::Time(0);
    myMap.info.resolution = 0.5;
    myMap.info.width = 100;
    myMap.info.height = 100;
    myMap.info.origin.position.x = 0;
    myMap.info.origin.position.y = 0;
    myMap.info.origin.position.z = 0;
    myMap.info.origin.orientation.x = 0;
    myMap.info.origin.orientation.y = 0;
    myMap.info.origin.orientation.z = 0;
    // myMap.info.origin.orientation.w = 0;
    // for (std::size_t i = 0; i < 10; i++)
    // {
    //     int8_t a;
    //     if (i <= 6)
    //         a = 20 * i;
    //     else if (i == 7)
    //         a = -10;
    //     else
    //         a = i;
    //     myMap.data.push_back(a);
    // }
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[i].size(); j++) {
            myMap.data.push_back(grid[grid.size() - 1 - i][j] * 100);
        }
    }
    myMap.header.frame_id = "map";
    pub.publish(myMap);
}

void publish_path(ros::Publisher &pub, vector<Vec2d> &path) {
    nav_msgs::Path plan;
    plan.header.frame_id = "map";
    for (int i = 0; i < path.size(); i++) {
        geometry_msgs::PoseStamped pos;
        pos.header.frame_id = "map";
        pos.pose.position.x = path[i].x_;
        pos.pose.position.y = path[i].y_;
        plan.poses.push_back(pos);
    }
    pub.publish(plan);
}

void publish_sdf(ros::Publisher &pub, SdfMap &sdf) {
    sensor_msgs::PointCloud pc;
    pc.header.frame_id = "map";
    pc.channels.resize(1);
    for (double x = 0; x < 50.0; x += 0.25) {
        for (double y = 0; y < 50.0; y += 0.25) {
            geometry_msgs::Point32 p;
            p.x = x;
            p.y = y;
            p.z = 0;
            pc.points.push_back(p);
            pc.channels[0].name = "intensities";
            auto dis = sdf.get_dist_with_grad_bilinear(Vec2d(x, y)).first;
            pc.channels[0].values.push_back(10 + LIMIT(dis, -10, 20));
        }
    }
    pub.publish(pc);
}

void publish_pos(ros::Publisher &pub, Robot &r) {
    geometry_msgs::PoseStamped pos;
    pos.header.frame_id = "map";
    pos.pose.position.x = r.pos_.x_;
    pos.pose.position.y = r.pos_.y_;
    auto q = tf::createQuaternionFromYaw(r.angle_);
    pos.pose.orientation.w = q.getW();
    pos.pose.orientation.x = q.getX();
    pos.pose.orientation.y = q.getY();
    pos.pose.orientation.z = q.getZ();
    pub.publish(pos);
}

void publish_dynamic_object(ros::Publisher &pub, vector<Vec2d> mid, vector<double> r) {
    jsk_recognition_msgs::PolygonArray polygons;
    polygons.header.frame_id = "map";
    polygons.header.stamp = ros::Time::now();
    for (int i = 0; i < mid.size(); i++) {
        geometry_msgs::PolygonStamped polygon;
        polygon.header.frame_id = "map";
        polygon.header.stamp = ros::Time::now();
        geometry_msgs::Point32 point;
        for (int j = 0; j < 360; j+=10) {
            point.x = mid[i].x_ + (r[i]) * cos(j / 180.0 * M_PI);
            point.y = mid[i].y_ + (r[i]) * sin(j / 180.0 * M_PI);
            point.z = 0.0;
            polygon.polygon.points.push_back(point);
        }
        // polygon.polygon.points.
        polygons.polygons.push_back(polygon);
    }
    pub.publish(polygons);
}
#endif
class MotionControlState {
public:
    double dis_drived_;
    Vec2d last_pos_;

    MotionControlState() : dis_drived_(0.0), last_pos_(0.0, 0.0) {}
};

bool check_can_go_back(int from, int robot_index, RobotTask *tasks) {
    for (int i = 0; i < 4; i++) {
        if (i == robot_index)
            continue;
        if (!tasks[i].is_idle_) {
            if (!tasks[i].is_bought_) {
                if (tasks[i].from_ == from) {
                    return false;
                }
            } else {
                if (tasks[i].to_ == from) {
                    return false;
                }
            }
        }
    }
    return true;
}

Vec2d vel_adjustment(Vec2d vel, int rindex, vector<Robot> robots, double max_vel) {
    double min_vel[4] = {2, 2, 2, 2};//{0, 0.8, 1.6, 2};//-0.5, 0.5, 1.2, 2
    double collision_risk = 0.0;
    // for (int t = 0; t < 25; t+=25) {
        Vec2d pos = robots[rindex].pos_ + robots[rindex].linear_ * 0.02 * 10;
        for (int i = 0; i < 4; i++) {
            // collision_risk += 
            if (i == rindex)
                continue;
            double dis = (pos - robots[i].pos_).mod();
            Vec2d dir = robots[i].pos_ - robots[rindex].pos_;
            dir = dir / dir.mod();
            double proj = dir * (robots[rindex].linear_ / robots[rindex].linear_.mod());
            LIMIT(proj, 0, 1);
            if (dis > 4.0) {

            } else {
                collision_risk += (dis - 4.0) * (dis - 4.0) * proj;
            }
        }
    // }
    collision_risk /= 8.0;
    LIMIT(collision_risk, 0, 1);
    LIMIT(vel.x_, -2, min_vel[rindex] + (max_vel - min_vel[rindex]) * (1 - collision_risk));
    // cerr << collision_risk << endl;
    return vel;
}

Vec2d turn_vel_adjustment(Vec2d raw_vel, vector<Vec2d> path, double max_vel) {
    double rotation = fabs(raw_vel.y_);
    LIMIT(rotation, 0, M_PI);
    if (fabs(max_vel - 6.0) < 0.1)
        LIMIT(raw_vel.x_, 0, 3.5 + 2.5 * (M_PI - rotation) / (M_PI));
    else
        LIMIT(raw_vel.x_, 0, 3.5 + 3.5 * (M_PI - rotation) / (M_PI));
    return raw_vel;
}

class ClusterNode{
public:
    Vec2d pos_;
    int class_;
    bool checked_;
    ClusterNode() {}
    ClusterNode(Vec2d pos, int class__, bool checked) : pos_(pos), class_(class__), checked_(checked) {};
};

// inline void cluster_add_operation(vector<Vec2d> &data, Vec2d point, vector<Vec2d> &nodes) {
//     while (!data.empty()) {
//         Vec2d p = data.back();
//         if ((p - point).mod() < 1.0) {
            
//         }
//     }
// }

tuple<vector<double>, vector<Vec2d>, vector<double>> cluster_lidar_data(const vector<Vec2d> &data) {
    static vector<Vec2d> past_dynamic_objs_pos;
    static vector<Vec2d> past_dynamic_objs_vel;
    vector<Vec2d> dynamic_objs_pos;
    vector<Vec2d> dynamic_objs_vel;
    vector<ClusterNode> clusternodes(data.size());
    for (int i = 0; i < data.size(); i++) {
        clusternodes[i] = ClusterNode(data[i], -1, false);
    }

    // //根据跟踪对象聚类
    // for (int i = 0; i < dynamic_objs_pos.size(); i++) {
    //     for (int j = 0; j < data.size(); j++) {
    //         if ((data[j] - dynamic_objs_pos[i]).mod() < 0.7 && clusternodes[j].checked_ && clusternodes[j].class_ != -1) {
    //             clusternodes[j].class_ = i;
    //             clusternodes[j].checked_ = true;
    //         }
    //     }
    // }

    
    int num_of_class = 0;
    for (int i = 0; i < data.size(); i++) {
        if (!clusternodes[i].checked_) {
            for (int j = 0; j < data.size(); j++) {
                if (j == i || clusternodes[j].checked_ || clusternodes[j].class_ != -1) {
                    continue;
                }
                if ((data[j] - data[i]).mod() < 0.8) {
                    if (clusternodes[i].class_ == -1) {
                        clusternodes[i].class_ = num_of_class++;
                    }
                    clusternodes[j].class_ = clusternodes[i].class_;
                }
            }
            clusternodes[i].checked_ = true;
        }
    }
    // cerr << num_of_class << endl;
    vector<Vec2d> mids;
    vector<double> radius;
    vector<double> color;
    for (int i = 0; i < num_of_class; i++) {
        Vec2d mid(0.0, 0.0);
        int cnt = 0;
        vector<Vec2d> ps;
        for (int j = 0; j < data.size(); j++) {
            if (clusternodes[j].class_ == i) {
                ps.push_back(data[j]);
            }
        }
        if (ps.size() < 3)
            continue;
        auto c = circle_fit(ps);
        if (c.second > 0.7)
            continue;
        mid = c.first;
        radius.push_back(c.second);
        mids.push_back(mid);
    }
    for (int j = 0; j < data.size(); j++) {
        color.push_back(clusternodes[j].class_);
    }
    return make_tuple(color, mids, radius);
}

bool have_enemy_nearby(Robot robot, vector<Vec2d> dynamic_objs) {
    for (auto obj : dynamic_objs) {
        if ((obj - robot.pos_).mod() < 3.0) {
            return true;
        }
    }
    return false;
}

bool check_need_back(Robot robot, RobotTask task, vector<Vec2d> dynamic_objects) {
    return true;
}

double get_task_pri_cost(RobotTask &task) {
    double cost = 0;
    if (!task.is_idle_ && task.is_bought_) {
        cost = workbenchs[task.from_].type_;
    }
    if (!task.is_idle_ && !task.is_bought_) {
        cost = workbenchs[task.from_].type_ - 10;
    }
    if (!task.is_idle_ && ((frame_id - task.start_frame_) * 0.02 > 30.0 || task.timeout_)) {
        cost = -10;
    }
    if (task.is_idle_) {
        cost = -10;
    }
    if (task.is_fighting_) {
        cost = -20;
    }
    return cost;
}

int main(int argc, char** argv) {
    System_init();
    #ifdef ROS
    ros::init(argc, argv, "planner");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");
    ros::Publisher raw_grid_pub = nh.advertise<nav_msgs::OccupancyGrid> ("raw_grid_map", 1);
    ros::Publisher sdf_pub = nh.advertise<sensor_msgs::PointCloud> ("sdf_map", 1);
    ros::Publisher astar_pub[4] = {
        nh.advertise<nav_msgs::Path> ("robot1/global_plan", 1),
        nh.advertise<nav_msgs::Path> ("robot2/global_plan", 1),
        nh.advertise<nav_msgs::Path> ("robot3/global_plan", 1),
        nh.advertise<nav_msgs::Path> ("robot4/global_plan", 1)
    };
    ros::Publisher pos_pub[4] = {
        nh.advertise<geometry_msgs::PoseStamped> ("robot1/pos", 1),
        nh.advertise<geometry_msgs::PoseStamped> ("robot2/pos", 1),
        nh.advertise<geometry_msgs::PoseStamped> ("robot3/pos", 1),
        nh.advertise<geometry_msgs::PoseStamped> ("robot4/pos", 1)
    };
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud> ("scan", 1);
    ros::Publisher dynamic_object_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("/dynamic_object", 1);
    #endif
    
    SdfMap sdf_map(0.25, grid_map.grid_);
    
    env_start();
    cerr << "env_start()" << endl;

    // cerr << sdf_map.get_dist_with_grad_bilinear(Vec2d(16.25, 30.25)).first << endl;
    // exit(0);

    // timeval start_time;
    // RECORD_TIME(start_time);

    int cnt = 0;

    double mean_vel = 0.0;
    int vel_cnt = 0;

    int selled_7_num = 0;

    //OrcaLocalPlanner planner;
    //planner.init();
    double update_ratio = 0.0;

    MotionControlState motionstate[4];
    bool active_avoid[4] = {false, false, false, false};
    int block_state[4] = {0, 0, 0, 0};
    int block_state_flag[4] = {0, 0, 0, 0};
    int path_fixed_cnt[4] = {0, 0, 0, 0};
    // pri
    bool is_avoiding[4] = {false, false, false, false};
    static bool last_found[4] = {false, false, false, false};
    int avoiding_cnt[4] = {0, 0, 0, 0};
    int avoid_priority[4] = {0, 1, 2, 3};
    vector<Vec2d> normal_path[4];
    int avoid_id[4] = {-1, -1, -1, -1};
    static int sell_cnt[4] = {0, 0, 0, 0};
    FightingData fightingdata[4];
    FightingTask fightingtask[4];
    RobotTask tasks[4];
    int switch_cnt[4] = {0, 0, 0, 0};//切换工作台计数
    DynamicObsFilter dynamic_obs_filter;

    while (true) {
        auto t0 = clock();
        auto state = get_state(workbenchs, robots, lidar_data);
        frame_id = get<0>(state);
        // if (frame_id < 0) {
        //     break;
        // }
        score = get<1>(state);
        lidar_data = get<3>(state);
        vector<Vec2d> lidar_pcl;
        lidar_pcl = get_move_obstacle(robots, lidar_data, sdf_map);
        auto cluster = cluster_lidar_data(lidar_pcl);
        vector<Vec2d> dynamic_objects = get<1>(cluster);
        auto dynamic_objects_radius = get<2>(cluster);
        auto cluster_pcl = get<0>(cluster);
        dynamic_obs_filter.update(robots, lidar_pcl, sdf_map);
        // cerr << "dynamic_obs_filter.update spend " << (clock() - t0) / 1000.0 << "ms"  << endl;
#ifdef ROS

        publish_scan(pcl_pub, lidar_pcl, cluster_pcl);
        publish_dynamic_object(dynamic_object_pub, dynamic_objects, dynamic_objects_radius);
#endif
        RobotCmd cmds[4];
        RobotCmd last_cmds[4];
        for (int i = 0; i < 4; i++) {
            cmds[i].linear_ = robots[i].linear_.mod();
            cmds[i].angular_ = robots[i].angular_;
            cmds[i].buy_ = false;
            cmds[i].sell_ = false;
            cmds[i].destroy_ = false;
        }
        // timeval abs_time;
        // RECORD_TIME(abs_time);
        // float rel_time = TIME_ITV(start_time, abs_time) / 1.0e6;    
        // if (!tasks[frame_id % 4].is_bought_ && !last_cmds[frame_id % 4].buy_)
        //     tasks[frame_id % 4].is_idle_ = true;

        if ((map_id == 1) && (FLAG == 0)) {
            tasks[0].is_fighting_ = true;
        }
        if ((map_id == 3) && (FLAG == 0)) {
            tasks[3].is_fighting_ = true;
        }
        if ((map_id == 3) && (FLAG == 1)) {
            tasks[0].is_fighting_ = true;
            tasks[1].is_fighting_ = true;
        }
        // cerr << "get state and task spend " << (clock() - t0) / 1000.0 << "ms"  << endl;

        scheduler_run(tasks, workbenchs, robots, last_cmds, frame_id, astar_search.get());
        fighting_scheduler(tasks, robots,
            fightingtask, fightingdata,
            other_workbenchs, map_id, frame_id);
        //将未携带物体的机器人的优先集调到最低
        for (int i = 0; i < 3; i++) {
            int index = i;
            int max_type = get_task_pri_cost(tasks[avoid_priority[i]]);
            // if (tasks[avoid_priority[i]].is_bought_ && !tasks[avoid_priority[i]].is_idle_)
            //     continue;
            for (int j = i + 1; j < 4; j++) {
                if (get_task_pri_cost(tasks[avoid_priority[j]]) > max_type) {
                    index = j;
                    max_type = workbenchs[tasks[avoid_priority[j]].from_].type_;
                }
            }
            if (index > i) {
                int tmp = avoid_priority[i];
                avoid_priority[i] = avoid_priority[index];
                avoid_priority[index] = tmp;
            }
        }
        // for (int i = 0; i < 4; i++) {
        //     cerr << avoid_priority[i] << "(" << tasks[avoid_priority[i]].is_bought_ << "," 
        //         << is_avoiding[avoid_priority[i]] << "," 
        //         << workbenchs[tasks[avoid_priority[i]].from_].type_ << ","
        //         << get_task_pri_cost(tasks[avoid_priority[i]]) << ") ";
        // }
        // cerr << endl;

        // for (int i = 0; i < 4; i++) {
        
        t0 = clock();
        for (int pri = 0; pri < 4; pri++) {
            int i = avoid_priority[pri];
            auto &task = tasks[i];
            vector<Vec2d> path;
            motionstate[i].dis_drived_ = (robots[i].pos_ - motionstate[i].last_pos_).mod();
            vector<vector<Vec2d>> obs_paths;
            vector<Vec2d> obs_poss;
            vector<int> collision_robots, collision_robots_cnt;
            vector<int> collision_index;
            int b1, b2, b3;
            if (0 == i) {
                b1 = 1, b2 = 2, b3 = 3;
            } else if (1 == i) {
                b1 = 0, b2 = 2, b3 = 3;
                // obs_poss.push_back(robots[0].pos_);
            } else if (2 == i) {
                b1 = 0, b2 = 1, b3 = 3;
                // obs_poss.push_back(robots[0].pos_);
                // obs_poss.push_back(robots[1].pos_);
            } else if (3 == i) {
                b1 = 0, b2 = 1, b3 = 2;
                // obs_poss.push_back(robots[0].pos_);
                // obs_poss.push_back(robots[1].pos_);
                // obs_poss.push_back(robots[2].pos_);
            }
            int priority = 0;
            for (int m = 0; m < 4; m++) {
                if (avoid_priority[m] == i) {
                    priority = m;
                    break;
                }
            }
            for (int m = 0; m < priority; m++) {
                obs_poss.push_back(robots[avoid_priority[m]].pos_);
                obs_paths.push_back(tasks[avoid_priority[m]].get_now_path());
                collision_index.push_back(avoid_priority[m]);
            }

            double ahead_dis = robots[i].linear_.mod() * 0.1;
            LIMIT(ahead_dis, 0.3, 0.6);
            
            Vec2d obs_pos[] = {robots[b1].pos_, robots[b2].pos_, robots[b3].pos_};
            Vec2d obs_vel[] = {robots[b1].linear_, robots[b2].linear_, robots[b3].linear_};
            double obs_angle[] = {robots[b1].angle_, robots[b2].angle_, robots[b3].angle_};
            Vec2d obs_cvel[] = {
                Vec2d(robots[b1].linear_.mod() * (1 - update_ratio) + cmds[b1].linear_ * update_ratio, robots[b1].angular_ * (1 - update_ratio) + cmds[b1].angular_ * update_ratio),
                Vec2d(robots[b2].linear_.mod() * (1 - update_ratio) + cmds[b2].linear_ * update_ratio, robots[b2].angular_ * (1 - update_ratio) + cmds[b2].angular_ * update_ratio),
                Vec2d(robots[b3].linear_.mod() * (1 - update_ratio) + cmds[b3].linear_ * update_ratio, robots[b3].angular_ * (1 - update_ratio) + cmds[b3].angular_ * update_ratio)};
            
            if (!task.is_idle_) {
                if (robots[i].item_ == workbenchs[task.from_].type_ && !task.is_bought_) {
                    task.is_bought_ = true;
                    workbench_mark[task.from_] = false;
                    last_found[i] = false;
                }
                if (!task.is_bought_) {
                    sell_cnt[i] = 0;
                    Vec2d vel;
                    if ((task.path1_.size() < 1 || true) && (path_fixed_cnt[i] <= 0 || task.path1_.size() < 2)) {
                        auto found = astar_search->search(
                                        robots[i].pos_, workbenchs[task.from_].pos_,
                                        path, grid_visited, false, obs_paths,
                                        obs_poss, true, collision_robots, collision_robots_cnt,
                                        dynamic_obs_filter.pcls_, true);
                        if (!found) {
                            found = astar_search->search(
                                        robots[i].pos_, workbenchs[task.from_].pos_,
                                        path, grid_visited, false, obs_paths,
                                        obs_poss, true, collision_robots, collision_robots_cnt,
                                        dynamic_obs_filter.pcls_, false);
                        }
                        if (found && is_avoiding[i]) {
                            avoiding_cnt[i]++;
                            if (avoiding_cnt[i] > 40) {
                                is_avoiding[i] = false;
                                task.is_idle_ = true;
                            }
                        } else if (!found && is_avoiding[i]) {
                            avoiding_cnt[i] = 0;
                            found = false;
                        }
                        if (path_fixed_cnt[i] > 0) {
                            found = false;
                        }
                        last_found[i] = found;
                        if (!found) {
                            // cerr << i << " not found" << endl;
                            // if (check_can_go_back(task.last_to_, i, tasks)) {
                                if (!is_avoiding[i]) {
                                    is_avoiding[i] = true;
                                    avoiding_cnt[i] = 0;
                                    path_fixed_cnt[i] = 0;
                                }
                                int max_cnt = 0;
                                int collision_r = -1;
                                for (int ci = 0; ci < collision_robots.size(); ci++) {
                                    if (collision_robots_cnt[ci] > max_cnt) {
                                        max_cnt = collision_robots_cnt[ci];
                                        collision_r = collision_index[collision_robots[ci]];
                                    }
                                }
                                avoid_id[i] = collision_r;
                                vector<Vec2d> tmp_poss;
                                tmp_poss.push_back(robots[collision_r].pos_);
                                // Vec2d avoid_goal = astar_search->bfs(robots[i].pos_, robots[i].angle_, tasks[collision_r].get_no_delete_path(), tmp_poss, grid_visited);
                                Vec2d search_point;
                                double dir = 0;
                                //cerr <<i << " " << (robots[x].pos_-robots[i].pos_).mod() << " ";
                                for(int p = 0; p < tasks[collision_r].get_no_delete_path().size();p++) {
                                    Vec2d path_x = tasks[collision_r].get_no_delete_path()[p];
                                    if((path_x-robots[i].pos_).mod() < 1.2){
                                        search_point = path_x;
                                        if(p > 0) {
                                            dir = atan2(tasks[collision_r].get_no_delete_path()[p].y_ -tasks[collision_r].get_no_delete_path()[p-1].y_ ,tasks[collision_r].get_no_delete_path()[p].x_ -tasks[collision_r].get_no_delete_path()[p-1].x_ );
                                        }
                                        break;
                                    }
                                }
                                Vec2d avoid_goal = astar_search->bfs(robots[i].pos_, tasks[collision_r].get_no_delete_path(), robots[collision_r].pos_, dir, grid_visited, vector<Vec2d>());
                                found = astar_search->search(robots[i].pos_, avoid_goal, path, grid_visited, false);
                                // found = astar_search->search(robots[i].pos_, task.start_pos_, path, grid_visited, false);
                                // if (calculate_path_dis(path) < 1.0)
                                //     found = false;
                            // }
                            if (!found) {
                                is_avoiding[i] = false;
                                avoiding_cnt[i] = 0;
                                path_fixed_cnt[i] = 0;
                                // found = astar_search->search(robots[i].pos_, workbenchs[task.from_].pos_, path, grid_visited, false);
                                if (!found) {
                                    cerr << "robot: " << i << "still not found" << endl;
                                    cmds[i].linear_ = -2.0;
                                    cmds[i].angular_ = 0;
                                    continue;
                                }
                            }
                        } else {
                            is_avoiding[i] = false;
                            task.normal_path = path;
                            avoid_id[i] = -1;
                            // cerr << "robot: " << i << " from: " << task.from_ << " find path" << endl;
                        }
                        task.path1_ = path;
                        motionstate[i].last_pos_ = robots[i].pos_;
                        motionstate[i].dis_drived_ = 0.0;
                    } else {
                        path = task.path1_;
                    }
                    if (path_fixed_cnt[i] > 0) {
                        path_fixed_cnt[i]--;
                    }
                    task.no_delete_path = path;
                    path = path_optimize(robots[i].pos_, path, obs_pos, obs_vel, sdf_map, !is_avoiding[i], dynamic_objects, FLAG);
                    task.path1_ = path;
                    // task.no_delete_path = path;
                    while (true)
                        if (path.size() > 1 && (robots[i].pos_ - path[0]).mod() < ahead_dis) {
                            path.erase(path.begin());
                            task.path1_.erase(task.path1_.begin());
                        } else {
                            break;
                        }

                    // vel = dwa_search(robots[i].pos_, robots[i].angle_, Vec2d(robots[i].linear_.mod(), robots[i].angular_), path[0], obs_pos, obs_angle, obs_cvel, sdf_map, active_avoid[i]);
                    if (is_avoiding[i])
                        vel = compute_vel(robots[i].pos_, robots[i].angle_, path[0], robots[i].angular_, MAX_VEL, 10.0);
                    else
                        vel = compute_vel(robots[i].pos_, robots[i].angle_, path[0], robots[i].angular_, MAX_VEL, 20.0);
                    if (!is_avoiding[i]) {
                        if (calculate_path_dis(path) < 1.0) {
                            LIMIT(vel.x_, -2.0, MAX_VEL);
                        }
                        
                        if ((robots[i].pos_ - workbenchs[task.from_].pos_).mod() < 0.5) {
                            vel = compute_vel(robots[i].pos_, robots[i].angle_, workbenchs[task.from_].pos_, robots[i].angular_, MAX_VEL, 20.0);
                        }

                        if ((robots[i].pos_ - workbenchs[task.from_].pos_).mod() <= 0.4) {
                            //补丁，再次计算购买时间
                            vector<Vec2d> path;
                            double dis_to;
                            astar_search->search(robots[i].pos_, workbenchs[task.to_].pos_, path, grid_visited, true);
                            dis_to = calculate_path_dis(path);
                            if(have_time_complete_final_task(dis_to, frame_id)) {
                                cmds[i].buy_ = true;
                            } else {
                                task.is_idle_ = true;
                            }
                        }
                        // vel = vel_adjustment(vel, i, robots, MAX_VEL);
                    }
                    if (!have_enemy_nearby(robots[i], dynamic_objects) || FLAG == 1)
                        vel = turn_vel_adjustment(vel, path, MAX_VEL);
                    cmds[i].linear_ = vel.x_;
                    cmds[i].angular_ = vel.y_;
                } else {
                    if (robots[i].item_ == 0) {
                        task.is_idle_ = true;
                        last_found[i] = false;
                        task.is_bought_ = false;
                        if (workbenchs[task.to_].type_ == 9) {
                            workbench_mask_cnt[8]--;
                        }
                        double frame = frame_id - task.start_frame_;
                        double vel = 0.0;
                        if (frame > 0) {
                            vel = task.dis_ / (frame / 50.0);
                            mean_vel += vel;
                            vel_cnt++;
                        }
                        if (workbenchs[task.to_].type_ == 8) {
                            selled_7_num++;
                        }
                    }
                    Vec2d vel;
                    if ((task.path2_.size() < 1 || true)  && (path_fixed_cnt[i] <= 0 || task.path2_.size() < 2)) {
                        auto found = astar_search->search(
                                        robots[i].pos_, workbenchs[task.to_].pos_,
                                        path, grid_visited, true, obs_paths,
                                        obs_poss, true, collision_robots, collision_robots_cnt,
                                        dynamic_obs_filter.pcls_, true);
                        if (!found) {
                            // cerr << "research" << endl;
                            found = astar_search->search(
                                        robots[i].pos_, workbenchs[task.to_].pos_,
                                        path, grid_visited, true, obs_paths,
                                        obs_poss, true, collision_robots, collision_robots_cnt,
                                        dynamic_obs_filter.pcls_, false);
                        }
                        // if (last_found[i]) {
                        //     if (calculate_path_dis(tasks[i].path2_) < calculate_path_dis(path) * 0.50) {
                        //         path = tasks[i].path2_;
                        //     }
                        // }
                        // if (found) {
                        //     if (calculate_path_dis(path) > calculate_path_dis(task.normal_path) + 5.0 && !is_avoiding[i]) {
                        //         found = false;
                        //     }
                        // }
                        // cerr << i << " found = " << found << " obs_poss.size = " << obs_poss.size() << " path.size = " << path.size() << endl;
                        // if (!found) {
                            // int max_cnt = 0;
                            // int collision_r = -1;
                            // for (int ci = 0; ci < collision_robots.size(); ci++) {
                            //     if (collision_robots_cnt[ci] > max_cnt) {
                            //         max_cnt = collision_robots_cnt[ci];
                            //         collision_r = collision_index[collision_robots[ci]];
                            //     }
                            // }
                            // avoid_id[i] = collision_r;
                            // vector<Vec2d> cpath = tasks[collision_r].get_no_delete_path();
                            // bool should_avoid = false;
                            // for (int ci = 0; ci < cpath.size(); ci++) {
                            //     if ((cpath[ci] - robots[i].pos_).mod() < 1.2) {
                            //         should_avoid = true;
                            //     }
                            // }
                            // if (!should_avoid) {
                            //     found = astar_search->search(robots[i].pos_, workbenchs[task.to_].pos_, path, grid_visited, true);
                            // }
                        // }
                        if (found && is_avoiding[i]) {
                            avoiding_cnt[i]++;
                            if (avoiding_cnt[i] > 40) {
                                is_avoiding[i] = false;
                            }
                        } else if (!found && is_avoiding[i]) {
                            avoiding_cnt[i] = 0;
                            found = false;
                        }
                        if (path_fixed_cnt[i] > 0) {
                            found = false;
                        }
                        if (path_fixed_cnt[i] > 0) {
                            found = false;
                        }
                        last_found[i] = found;
                        if (!found) {
                            // cerr << i << " not found" << endl;
                            // if (check_can_go_back(task.from_, i, tasks)) {
                                if (!is_avoiding[i]) {
                                    is_avoiding[i] = true;
                                    avoiding_cnt[i] = 0;
                                    path_fixed_cnt[i] = 0;
                                }
                                int max_cnt = 0;
                                int collision_r = 0;
                                for (int ci = 0; ci < collision_robots.size(); ci++) {
                                    if (collision_robots_cnt[ci] > max_cnt) {
                                        max_cnt = collision_robots_cnt[ci];
                                        collision_r = collision_index[collision_robots[ci]];
                                    }
                                }
                                avoid_id[i] = collision_r;
                                vector<Vec2d> tmp_poss;
                                tmp_poss.push_back(robots[collision_r].pos_);
                                // Vec2d avoid_goal = astar_search->bfs(robots[i].pos_, robots[i].angle_, tasks[collision_r].get_no_delete_path(), tmp_poss, grid_visited);
                                Vec2d search_point;
                                double dir = 0;
                                //cerr <<i << " " << (robots[x].pos_-robots[i].pos_).mod() << " ";
                                for(int p = 0; p < tasks[collision_r].get_no_delete_path().size();p++) {
                                    Vec2d path_x = tasks[collision_r].get_no_delete_path()[p];
                                    if((path_x-robots[i].pos_).mod() < 1.2){
                                        search_point = path_x;
                                        if(p > 0) {
                                            dir = atan2(tasks[collision_r].get_no_delete_path()[p].y_ -tasks[collision_r].get_no_delete_path()[p-1].y_ ,tasks[collision_r].get_no_delete_path()[p].x_ -tasks[collision_r].get_no_delete_path()[p-1].x_ );
                                        }
                                        break;
                                    }
                                }
                                Vec2d avoid_goal = astar_search->bfs(robots[i].pos_, tasks[collision_r].get_no_delete_path(), robots[collision_r].pos_, dir, grid_visited, vector<Vec2d>());
                                found = astar_search->search(robots[i].pos_, avoid_goal, path, grid_visited, false);
                                // found = astar_search->search(robots[i].pos_, workbenchs[task.from_].pos_, path, grid_visited, false);
                                // if (calculate_path_dis(path) < 1.0)
                                //     found = false;
                            if (!found) {
                                // found = astar_search->search(robots[i].pos_, workbenchs[task.to_].pos_, path, grid_visited, true);
                                if (!found) {
                                    is_avoiding[i] = false;
                                    avoiding_cnt[i] = 0;
                                    path_fixed_cnt[i] = 0;
                                    // cerr << "robot: " << i << " to: " << task.to_ << " failed to find path" << endl;
                                    cerr << "robot: " << i << "still not found" << endl;
                                    cmds[i].linear_ = -2.0;
                                    cmds[i].angular_ = 0;
                                    continue;
                                }
                            }
                        } else {
                            is_avoiding[i] = false;
                            task.normal_path = path;
                            avoid_id[i] = -1;
                            // cerr << "robot: " << i << " to: " << task.to_ << " find path" << endl;
                        }
                        task.path2_ = path;
                        motionstate[i].last_pos_ = robots[i].pos_;
                        motionstate[i].dis_drived_ = 0.0;
                    } else {
                        path = task.path2_;
                    }
                    if (path_fixed_cnt[i] > 0) {
                        path_fixed_cnt[i]--;
                    }
                    task.no_delete_path = path;
                    path = path_optimize(robots[i].pos_, path, obs_pos, obs_vel, sdf_map, !is_avoiding[i], dynamic_objects, FLAG);
                    task.path2_ = path;
                    while (true)
                        if (path.size() > 1 && (robots[i].pos_ - path[0]).mod() < ahead_dis) {
                            path.erase(path.begin());
                            task.path2_.erase(task.path2_.begin());
                        } else {
                            break;
                        }
                    // vel = dwa_search(robots[i].pos_, robots[i].angle_, Vec2d(robots[i].linear_.mod(), robots[i].angular_), path[0], obs_pos, obs_angle, obs_cvel, sdf_map, active_avoid[i]);
                    // if (is_avoiding[i])
                    if (is_avoiding[i])
                        vel = compute_vel(robots[i].pos_, robots[i].angle_, path[0], robots[i].angular_, MAX_VEL, 10.0);
                    else
                        vel = compute_vel(robots[i].pos_, robots[i].angle_, path[0], robots[i].angular_, MAX_VEL, 20.0);
                    if (!is_avoiding[i]) {
                        if (calculate_path_dis(path) < 1.0) {
                            LIMIT(vel.x_, -2.0, MAX_VEL);
                        }
                        
                        if ((robots[i].pos_ - workbenchs[task.to_].pos_).mod() < 0.5) {
                            vel = compute_vel(robots[i].pos_, robots[i].angle_, workbenchs[task.to_].pos_, robots[i].angular_, MAX_VEL, 20.0);
                            if (fabs(get_angle_delta(robots[i].pos_, robots[i].angle_, workbenchs[task.from_].pos_)) < M_PI / 4.0) {
                                task.init2_ = false;
                                // cerr << "**********" << endl;
                            }
                        }
                        if ((robots[i].pos_ - workbenchs[task.to_].pos_).mod() <= 0.6) {
                            cmds[i].sell_ = true;
                        }
                        // vel = vel_adjustment(vel, i, robots, MAX_VEL);
                    }
                    if (!have_enemy_nearby(robots[i], dynamic_objects) || FLAG == 1)
                        vel = turn_vel_adjustment(vel, path, MAX_VEL);
                    cmds[i].linear_ = vel.x_;
                    cmds[i].angular_ = vel.y_;
                }
            } 
            else {
                if (frame_id > 12000) {
RETURN_TO_INIT_POS:
                    last_found[i] = false;
                    // cerr << i << ":idle" << endl;
                    Vec2d goal = robot_init_pos[i];
                    auto found = astar_search->search(robots[i].pos_, goal, path, grid_visited, false, obs_paths, obs_poss, true, collision_robots, collision_robots_cnt);
                    path = path_optimize(robots[i].pos_, path, obs_pos, obs_vel, sdf_map, !is_avoiding[i], dynamic_objects, FLAG);
                    // Vec2d goal = workbenchs[task.to_].pos_ + Vec2d(5.0, 0.0);
                    Vec2d vel;
                    vel = dwa_search(robots[i].pos_, robots[i].angle_, Vec2d(robots[i].linear_.mod(), robots[i].angular_), goal, obs_pos, obs_angle, obs_cvel, sdf_map, active_avoid[i]);
                    if ((robots[i].pos_ - goal).mod() < 0.4) {
                        vel = compute_vel(robots[i].pos_, robots[i].angle_, goal, robots[i].angular_, MAX_VEL, 6.0);
                    }
                    cmds[i].linear_ = vel.x_;
                    cmds[i].angular_ = vel.y_;
                } else {
                    // block_enemy(dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    
                    // for (auto w : other_workbenchs) {
                    //     if (w.type_ == 7) {
                    //         block_workbench(w.pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    //         break;
                    //     }
                    // }
                        
                    // }
                    // if ((map_id == 4) && (FLAG == 0) && i == 1) {
                    //     continue;
                    // }
                    // if ((map_id == 1) && (FLAG == 0)) {
                    //     block_workbench(other_workbenchs[2].pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    //     // block_workbench_static(other_workbenchs[12].pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    // } else if ((map_id == 3) && (FLAG == 0)) {
                    //     // block_workbench_static(other_workbenchs[3].pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    //     block_workbench(other_workbenchs[4].pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    // } else if ((map_id == 3) && (FLAG == 1)) {
                    //     block_workbench(other_workbenchs[8].pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    // } else if ((map_id == 4) && (FLAG == 0)) {
                    //     block_workbench(other_workbenchs[10].pos_, dynamic_objects, dynamic_objects_radius, robots[i], fightingdata[i], astar_search.get(), sdf_map, cmds[i], path);
                    // } else {
                    //     goto RETURN_TO_INIT_POS;
                    // }
                    if (fightingtask[i].fight_type_ == BLOCK_WB) {
                        block_workbench(other_workbenchs[fightingtask[i].blocked_wb_].pos_,
                            dynamic_objects, dynamic_objects_radius,
                            robots[i], fightingdata[i], astar_search.get(),
                            sdf_map, cmds[i], path);
                    } else if (fightingtask[i].fight_type_ == BLOCK_ENEMY) {
                        block_enemy(dynamic_objects, dynamic_objects_radius,
                            robots[i], fightingdata[i], astar_search.get(),
                            sdf_map, cmds[i], path, other_workbenchs);
                    } else {
                        goto RETURN_TO_INIT_POS;
                    }
                    
                }
            }
#ifdef ROS
            publish_path(astar_pub[i], path);
#endif
        }

        for (int i = 0; i < 4; i++) {
#ifdef ROS
            publish_pos(pos_pub[i], robots[i]);
#endif
        }
        // cerr << "control spend " << (clock() - t0) / 1000.0 << "ms"  << endl;

        // for (int i = 0; i < 4; i++) {
        //     cerr << is_avoiding[i] << " ";
        // }
        // cerr << endl;

        //防守策略
        //1.工作台被占领，直接切换工作台
        //2.任务长时间无法完成，摧毁+关闭相关任务
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < dynamic_objects.size(); j++) {
                if(!tasks[i].is_idle_) {
                    //1.任务只能切换一次
                    vector<Vec2d> path;
                    double min_dis = 80;
                    bool switch_succ = false;
                    if (!(switch_cnt[i]) && (dynamic_objects[j] - workbenchs[tasks[i].to_].pos_).mod() <= 0.6) {
                        switch_succ = switch_task(switch_cnt[i], tasks[i], workbenchs, i, cmds, robots, astar_search.get(), path, min_dis, grid_visited);
                        if(!switch_succ) {
                            tasks[i].timeout_ = true;
                        }
                    }
                    //2.再次切换任务
                    if((((frame_id - tasks[i].start_frame_) * 0.02) > (2 * tasks[i].estimate_time_)) && (!switch_succ) && ((frame_id - tasks[i].start_frame_) * 0.02 > 30)){
                        switch_succ = switch_task(switch_cnt[i], tasks[i], workbenchs, i, cmds, robots, astar_search.get(), path, 200, grid_visited);
                        if(!switch_succ) {
                            tasks[i].timeout_ = true;
                        }
                    }
                    //2.摧毁+关闭相关任务
                    double max_time = 60 * 3.0;
                    // if (robots[i].item_ <= 3) {
                    //     max_time = 30.0;
                    // }
                    if(((frame_id - tasks[i].start_frame_) * 0.02) > (3 * tasks[i].estimate_time_) && ((frame_id - tasks[i].start_frame_) * 0.02 > max_time) &&
                    (((dynamic_objects[j] - workbenchs[tasks[i].to_].pos_).mod() <= 1) || ((dynamic_objects[j] - robots[i].pos_).mod() <= 1))) { //任务时间已经是实际时间的五倍了，且周围检测到机器人，被彻底堵死
                        if (tasks[i].is_bought_) {
                            cmds[i].destroy_ = true;
                        }
                        //从sell_pair去掉相关的任务
                        // for (int r = 0; r < 4; r++) {
                        //     for (int b = 0; b < sellpairs[r].size(); b++) {
                        //         if (sellpairs[r][b].to_ == tasks[i].to_) {
                        //             sellpairs[r][b].dis_ = 1000000;
                        //         }
                        //         if (sellpairs[r][b].from_ == tasks[i].to_) {
                        //             sellpairs[r][b].dis_ = 10000000;
                        //         }
                        //     }
                        // }
                        tasks[i].is_idle_ = true; //idle，重新分配任务
                    }
                }
            }
        }
        for (int i = 0; i < 4; i++) {
            if (block_state[i] >= 1) {
                cmds[i].linear_ = -2.0;
                if (block_state_flag[i] == 1)
                    cmds[i].angular_ = (rand() / RAND_MAX) * M_PI;
                else
                    cmds[i].angular_ = 0.0;
                block_state[i]--;
            }
        }

        //堵死检测
        static int blocked_cnt[4];
        for (int i = 0; i < 4; i++) {
            if (!tasks[i].is_idle_) {
                if (block_state[i] == 0) {
                    bool reach_goal = false;
                    vector<Vec2d> path = tasks[i].get_now_path();
                    if (calculate_path_dis(path) < 3.0 && FLAG == 1) {
                        reach_goal = true;
                    }
                    if ((fabs(cmds[i].linear_) > 0.5 && fabs(robots[i].linear_.mod()) < 0.2 && !reach_goal) ||
                        (fabs(cmds[i].linear_) > 0.5 && fabs(robots[i].linear_.mod()) < 1.5 && reach_goal)) {
                        blocked_cnt[i]++;
                    } else {
                        blocked_cnt[i] = 0;
                    }
                    bool is_enemy = false;
                    for (auto ene : dynamic_objects) {
                        if ((ene - robots[i].pos_).mod() < 1.5) {
                            is_enemy = true;
                        }
                    }
                    int thresh = 75;
                    if (is_enemy && FLAG == 0) {
                        thresh = 200;
                    }
                    if (blocked_cnt[i] > thresh) {
                        blocked_cnt[i] = 0;
                        if (is_enemy) {
                            block_state_flag[i] = 1;
                            block_state[i] = 50;
                        } else {
                            block_state_flag[i] = 0;
                            block_state[i] = 50;
                        }
                    }
                }
            }
        }

        //碰撞检查
        int cnts[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            if (!tasks[i].is_idle_) {
                for (int j = i + 1; j < 4; j++) {
                    vector<Vec2d> *path1;
                    if (tasks[i].is_bought_) {
                        path1 = &tasks[i].path1_;
                    } else {
                        path1 = &tasks[i].path2_;
                    }
                    vector<Vec2d> *path2;
                    if (tasks[j].is_bought_) {
                        path2 = &tasks[j].path1_;
                    } else {
                        path2 = &tasks[j].path2_;
                    }
                    int siz = min(int(min(path1->size(), path2->size())), 20);
                    for (int t = 0; t < siz; t++) {
                        if (((*path1)[t] - (*path2)[t]).mod() < 0.2) {
                            cnts[j]++;
                        }
                    }
                }
            }
        }
        for (int i = 0; i < 4; i++) {
            if (cnts[i] > 1) {
                active_avoid[i] = true;
            } else {
                active_avoid[i] = false;
            }
        }
        
        for (int i = 0; i < 4; i++) {
            last_cmds[i] = cmds[i];
        }

        set_robot(cmds, frame_id);
#ifdef ROS
        publish_grid(raw_grid_pub, grid_map.grid_);
        publish_sdf(sdf_pub, sdf_map);
        ros::spinOnce();
#endif
    }

    if (vel_cnt > 0) {
        mean_vel /= vel_cnt;
        cerr << "mean velocity: " << mean_vel << endl;
    }
    cerr << "selled_7_num: " << selled_7_num << endl;

    return 0;
}
