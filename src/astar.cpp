#include <iostream>
#include <queue>
#include <map>
#include <algorithm>

#include "astar.hpp"

#define GRID_WIDTH 0.25
#define GRID_HEIGHT 0.25

using namespace std;

#define SearchNodePtr SearchNode*

extern int map_id;

class Cost {
public:
    double g_cost_[200][200];
    double cost_[200][200];
};

class SearchNode {
public:
    Vec2i grid_idx_;
    static Cost costs_;
    static SearchNodePtr parents_[200][200];
    double &g_cost_;
    double &cost_;
    int step_;
    SearchNodePtr &parent_;

    SearchNode(): grid_idx_(0, 0), g_cost_(costs_.g_cost_[grid_idx_.y_][grid_idx_.x_]), cost_(costs_.cost_[grid_idx_.y_][grid_idx_.x_]), step_(0), parent_(parents_[grid_idx_.y_][grid_idx_.x_]) {}
    SearchNode(Vec2i idx, double g_cost, double cost, int step, SearchNodePtr parent):
        grid_idx_(idx), g_cost_(costs_.g_cost_[idx.y_][idx.x_]), cost_(costs_.cost_[idx.y_][idx.x_]), step_(step), parent_(parents_[grid_idx_.y_][grid_idx_.x_]) {
        g_cost_ = g_cost, cost_ = cost, parent_ = parent;
    }
};

Cost SearchNode::costs_;
SearchNodePtr SearchNode::parents_[200][200];

class SearchNodeComparator {
public:
    bool operator()(SearchNodePtr node1, SearchNodePtr node2) {
        return node1->cost_ > node2->cost_;
    }
};

bool compare(SearchNodePtr node1, SearchNodePtr node2) {
    return node1->cost_ > node2->cost_;
}

AstarSearch::AstarSearch(GridMap map) {
    map_1_ = map;
    map_2_ = map;
    map_3_ = map;
    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 200; j++) {
            if (map_1_.ela_grid_[i][j] == 1) {
                Vec2i idx(j, i);
                if (CHECK_IN_MAP(idx.x_, idx.y_ - 1) && map_1_.ela_map_[idx.y_ - 1][idx.x_] == '.')
                    map_2_.ela_grid_[idx.y_ - 1][idx.x_] = 1;
                if (CHECK_IN_MAP(idx.x_, idx.y_ + 1) && map_1_.ela_map_[idx.y_ + 1][idx.x_] == '.')
                    map_2_.ela_grid_[idx.y_ + 1][idx.x_] = 1;
                if (CHECK_IN_MAP(idx.x_ + 1, idx.y_) && map_1_.ela_map_[idx.y_][idx.x_ + 1] == '.')
                    map_2_.ela_grid_[idx.y_][idx.x_ + 1] = 1;
                if (CHECK_IN_MAP(idx.x_ - 1, idx.y_) && map_1_.ela_map_[idx.y_][idx.x_ - 1] == '.')
                    map_2_.ela_grid_[idx.y_][idx.x_ - 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ - 1, idx.y_-1) && map_1_.ela_map_[idx.y_-1][idx.x_ - 1] == '.')
                //     map_2_.ela_grid_[idx.y_-1][idx.x_ - 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ - 1, idx.y_+1) && map_1_.ela_map_[idx.y_+1][idx.x_ - 1] == '.')
                //     map_2_.ela_grid_[idx.y_+1][idx.x_ - 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ + 1, idx.y_-1) && map_1_.ela_map_[idx.y_-1][idx.x_ +1] == '.')
                //     map_2_.ela_grid_[idx.y_-1][idx.x_ + 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ + 1, idx.y_+ 1) && map_1_.ela_map_[idx.y_+1][idx.x_ + 1] == '.')
                //     map_2_.ela_grid_[idx.y_+1][idx.x_ + 1] = 1;
            }
        }
    }

    // for (int i = 0; i < 100; i++){
    //     for (int j = 0; j < 100; j++) {
    //         cerr << map_2_.ela_grid_[i][j];
    //     }
    //     cerr << endl;
    // }


    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 200; j++) {
            if (map_2_.ela_grid_[i][j] == 1) {
                Vec2i idx(j, i);
                if (CHECK_IN_MAP(idx.x_, idx.y_ - 1) && map_1_.ela_map_[idx.y_ - 1][idx.x_] == '.')
                    map_3_.ela_grid_[idx.y_ - 1][idx.x_] = 1;
                if (CHECK_IN_MAP(idx.x_, idx.y_ + 1) && map_1_.ela_map_[idx.y_ + 1][idx.x_] == '.')
                    map_3_.ela_grid_[idx.y_ + 1][idx.x_] = 1;
                if (CHECK_IN_MAP(idx.x_ + 1, idx.y_) && map_1_.ela_map_[idx.y_][idx.x_ + 1] == '.')
                    map_3_.ela_grid_[idx.y_][idx.x_ + 1] = 1;
                if (CHECK_IN_MAP(idx.x_ - 1, idx.y_) && map_1_.ela_map_[idx.y_][idx.x_ - 1] == '.')
                    map_3_.ela_grid_[idx.y_][idx.x_ - 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ - 1, idx.y_-1) && map_1_.ela_map_[idx.y_-1][idx.x_ - 1] == '.')
                //     map_3_.ela_grid_[idx.y_-1][idx.x_ - 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ - 1, idx.y_+1) && map_1_.ela_map_[idx.y_+1][idx.x_ - 1] == '.')
                //     map_3_.ela_grid_[idx.y_+1][idx.x_ - 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ + 1, idx.y_-1) && map_1_.ela_map_[idx.y_-1][idx.x_ +1] == '.')
                //     map_3_.ela_grid_[idx.y_-1][idx.x_ + 1] = 1;
                // if (CHECK_IN_MAP(idx.x_ + 1, idx.y_+ 1) && map_1_.ela_map_[idx.y_+1][idx.x_ + 1] == '.')
                //     map_3_.ela_grid_[idx.y_+1][idx.x_ + 1] = 1;
            }
        }
    }

    // for (int i = 0; i < 200; i++) {
    //     for (int j = 0; j < 200; j++) {
    //         cerr << map_2_.ela_grid_[i][j];
    //     }
    //     cerr << endl;
    // }
}

bool AstarSearch::search(Vec2d start_pos, Vec2d goal_pos, vector<Vec2d> &path, char grid_visited[200][200], bool is_inflate) {
    if (is_inflate) {
        map_ = &map_3_;
    } else {
        map_ = &map_2_;
    }
    auto t0 = clock();
    priority_queue<SearchNodePtr, vector<SearchNodePtr>, SearchNodeComparator> openset;
    // vector<SearchNodePtr> openset;
    vector<SearchNodePtr> nodeset;
    Vec2i goal_idx = pos_to_index(goal_pos);
    Vec2i start_idx = pos_to_index(start_pos);
    // cerr << "start:" << start_idx.x_ << ' ' << start_idx.y_ << " goal: " << goal_idx.x_ << ' ' << goal_idx.y_ << endl;
    // char grid_visited[100][100];
    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 200; j++) {
            grid_visited[i][j] = 0;
            SearchNode::costs_.g_cost_[i][j] = 0;
            SearchNode::costs_.cost_[i][j] = 0;
        }
    }

    SearchNodePtr node = new SearchNode(start_idx, 0, 0, 0, nullptr);
    grid_visited[start_idx.y_][start_idx.x_] = 1;
    nodeset.push_back(node);
    openset.push(node);

    SearchNodePtr end_node = nullptr;

    // printInfo("ff\n\r");

    int cnt = 0;
    
    while (!openset.empty()) {
        // cerr << "0000" << endl;
        SearchNodePtr node = openset.top();
        openset.pop();
        // cerr << node->grid_idx_.x_ << "," << node->grid_idx_.y_ << " " << node->cost_ << endl;
        grid_visited[node->grid_idx_.y_][node->grid_idx_.x_] = -1;
        int step = node->step_ + 1;

        if (node->grid_idx_ == goal_idx) {
            auto t1 = clock();
            // cerr << "A* find avaliable path, spend " << (t1 - t0) / 1000.0 << "ms" << endl;
            end_node = node;
            break;
        }
        
        vector<Vec2i> neig;
        find_neighbor(node->grid_idx_, neig, map_->ela_grid_, step == 1);
        // cerr << neig.size() << endl;
        for (int i = 0; i < neig.size(); i++) {
            Vec2i idx = neig[i];
            if (map_->ela_grid_[idx.y_][idx.x_] == 0 && grid_visited[idx.y_][idx.x_] == 0) {
                grid_visited[idx.y_][idx.x_] = 1;
                double g_cost = node->g_cost_ + (index_to_pos(idx) - index_to_pos(node->grid_idx_)).mod();
                double cost = g_cost + huristic(idx, goal_idx);
                SearchNodePtr new_node = new SearchNode(idx, g_cost, cost, node->step_ + 1, node);
                nodeset.push_back(new_node);
                openset.push(new_node);
            } else if (map_->ela_grid_[idx.y_][idx.x_] == 0 && grid_visited[idx.y_][idx.x_] == 1) {
                double g_cost = node->g_cost_ + (index_to_pos(idx) - index_to_pos(node->grid_idx_)).mod();
                if (g_cost < SearchNode::costs_.g_cost_[idx.y_][idx.x_]) {
                    SearchNode::costs_.g_cost_[idx.y_][idx.x_] = g_cost;
                    SearchNode::costs_.cost_[idx.y_][idx.x_] = g_cost + huristic(idx, goal_idx);
                    SearchNode::parents_[idx.y_][idx.x_] = node;
                }
            }
            // sort(openset.begin(), openset.end(), compare);
        }

        if (++cnt > 40000) {
            end_node = nullptr;
            break;
        }
    }
    if (end_node == nullptr) {
        auto t1 = clock();
        for (int i = 0; i < nodeset.size(); i++) {
            delete nodeset[i];
        }
        // cerr << "A* failed, spend " << (t1 - t0) / 1000.0 << "ms" << endl;
        return false;
    }
    SearchNodePtr start_node = end_node;
    path.clear();
    if (start_node != nullptr)
        path.push_back(index_to_pos(start_node->grid_idx_));
    while (start_node != nullptr && start_node->parent_ != nullptr && start_node->parent_->parent_ != nullptr) {
        start_node = start_node->parent_;
        path.push_back(index_to_pos(start_node->grid_idx_));
    }
    reverse(path.begin(), path.end());
    for (int i = 0; i < nodeset.size(); i++) {
        delete nodeset[i];
    }

    return true;
}

bool AstarSearch::search(
    Vec2d start_pos, Vec2d goal_pos, 
    vector<Vec2d> &path, char grid_visited[200][200], 
    bool is_inflate, vector<vector<Vec2d>> paths,
    vector<Vec2d> poss, bool check_pos_all_time,
    vector<int> &collision_robots, vector<int> &collision_robots_cnt,
    vector<Vec2d> lidar_pcl, bool check_lidar) {
    if (is_inflate) {
        map_ = &map_3_;
    } else {
        map_ = &map_2_;
    }
    auto t0 = clock();
    priority_queue<SearchNodePtr, vector<SearchNodePtr>, SearchNodeComparator> openset;
    vector<SearchNodePtr> nodeset;
    Vec2i goal_idx = pos_to_index(goal_pos);
    Vec2i start_idx = pos_to_index(start_pos);

    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 200; j++) {
            grid_visited[i][j] = 0;
            SearchNode::costs_.g_cost_[i][j] = 0;
            SearchNode::costs_.cost_[i][j] = 0;
        }
    }

    SearchNodePtr node = new SearchNode(start_idx, 0, 0, 0, nullptr);
    grid_visited[start_idx.y_][start_idx.x_] = 1;
    nodeset.push_back(node);
    openset.push(node);

    SearchNodePtr end_node = nullptr;

    int cnt = 0;
    //　使用广搜
    while (!openset.empty()) {
        SearchNodePtr node = openset.top();
        openset.pop();
        grid_visited[node->grid_idx_.y_][node->grid_idx_.x_] = 1;
        int step = node->step_ + 1;
        //　搜索到目标
        if (node->grid_idx_ == goal_idx) {
            auto t1 = clock();
            // cerr << "A* find avaliable path, spend " << (t1 - t0) / 1000.0 << "ms" << endl;
            end_node = node;
            break;
        }
        
        vector<Vec2i> neig;
        //　当前节点node->grid_idx_，邻居节点，精细删格地图，返回：neig
        //　遍历八个方向
        find_neighbor(node->grid_idx_, neig, map_->ela_grid_, step == 1);
        for (int i = 0; i < neig.size(); i++) {
            Vec2i idx = neig[i];
            bool collision = false;
            int max_step = 30;
            //　遍历其他几个机器人的位置
            for (int ri = 0; ri < poss.size(); ri++) {
                //　前５次遍历默认就进行判断避障，或者当机器人远离目标并且当前规划步数小于最大步数。
                if (step < 5 || (check_pos_all_time && (index_to_pos(idx) - goal_pos).mod() > 1.0 && step < max_step)) {//30
                    if ((index_to_pos(idx) - poss[ri]).mod() < 1.5) {
                        collision = true;
                        bool have_ri = false;
                        for (int cr = 0; cr < collision_robots.size(); cr++) {
                            if (collision_robots[cr] == ri) {
                                collision_robots_cnt[cr]++;
                                have_ri = true;
                                break;
                            }
                        }
                        if (!have_ri) {
                            collision_robots.push_back(ri);
                            collision_robots_cnt.push_back(1);
                        }
                    }
                }
            }
            if (!collision && check_lidar) { //检查lidar点云
                for (auto pcl : lidar_pcl) {
                    if ((index_to_pos(idx) - goal_pos).square_norm() > 1.0 * 1.0) {
                        if ((index_to_pos(idx) - pcl).square_norm() < 0.8 * 0.8) {
                            collision = true;
                            break;
                        }
                    }
                }
            }
            //　相当于把机器人考虑进去
            if (!collision) {
                if (grid_visited[idx.y_][idx.x_] == 0) {
                    grid_visited[idx.y_][idx.x_] = 1;
                    //　目标代价
                    double g_cost = node->g_cost_ + (index_to_pos(idx) - index_to_pos(node->grid_idx_)).mod();
                    double cost = g_cost + huristic(idx, goal_idx);
                    //　把父亲节点存进去
                    SearchNodePtr new_node = new SearchNode(idx, g_cost, cost, node->step_ + 1, node);
                    nodeset.push_back(new_node);
                    openset.push(new_node);
                } else if (grid_visited[idx.y_][idx.x_] == 1) {
                    double g_cost = node->g_cost_ + (index_to_pos(idx) - index_to_pos(node->grid_idx_)).mod();
                    //　更新删咯里面的最小代价
                    if (g_cost < SearchNode::costs_.g_cost_[idx.y_][idx.x_]) {
                        SearchNode::costs_.g_cost_[idx.y_][idx.x_] = g_cost;
                        SearchNode::costs_.cost_[idx.y_][idx.x_] = g_cost + huristic(idx, goal_idx);
                        SearchNode::parents_[idx.y_][idx.x_] = node;
                    }
                }
            }
            // sort(openset.begin(), openset.end(), compare);
        }
        //　规划超时
        if (++cnt > 40000) {
            end_node = nullptr;
            break;
        }
    }
    if (end_node == nullptr) {
        auto t1 = clock();
        for (int i = 0; i < nodeset.size(); i++) {
            delete nodeset[i];
        }
        // cerr << "A* failed, spend " << (t1 - t0) / 1000.0 << "ms" << endl;
        return false;
        
    }
    //　反向找到所有路径
    SearchNodePtr start_node = end_node;
    path.clear();
    if (start_node != nullptr)
        path.push_back(index_to_pos(start_node->grid_idx_));
    while (start_node != nullptr && start_node->parent_ != nullptr && start_node->parent_->parent_ != nullptr) {
        start_node = start_node->parent_;
        path.push_back(index_to_pos(start_node->grid_idx_));
    }
    reverse(path.begin(), path.end());
    for (int i = 0; i < nodeset.size(); i++) {
        delete nodeset[i];
    }

    return true;
 }


// Vec2d AstarSearch::bfs(Vec2d robot_pos, double angle, vector<Vec2d> obs, vector<Vec2d> poss,char grid_visited[200][200]) {
//     Vec2i robot_idx = pos_to_index(robot_pos);
//     Vec2d avoid_dir = robot_pos - poss[0];
//     vector<vector<bool>> is_visited(200,vector<bool>(200,false));
//     queue<Vec2i> que;
//     que.push(Vec2i(robot_idx.y_,robot_idx.x_));
//     Vec2d tmp_goal(0,0);
//     while(!que.empty()) {
//         Vec2i t = que.front();
//         que.pop();
//         //求出删格对应的真实位置
//             int i = t.x_, j = t.y_;
//         Vec2d bfs_pos = index_to_pos(Vec2i(j,i));
//         //cerr << bfs_pos.x_<<" "<<bfs_pos.y_<<endl;
//         //　计算栅格距离路径最近的距离
//         //终止条件
//         double dist = 999;
//         for(Vec2d p:obs) {
//             //cerr << p.x_<<" "<<p.y_<<endl;
//             if((p-bfs_pos).mod()<dist){
//                 dist=(p-bfs_pos).mod();
//             }
//         }
//         //cerr << dist << endl;
//         double safe_dis = 2.0;
//         if(dist > safe_dis && ((bfs_pos - poss[0]).mod() > 2.0)/* && (bfs_pos - poss[0]) * avoid_dir < 0)*/){
//             tmp_goal = bfs_pos;
//             vector<Vec2d> path;
//             vector<vector<Vec2d>> obspath;
//             // bool found = search(robot_pos,  tmp_goal, path, grid_visited,false,obspath,poss,false);
//             bool found = search(robot_pos,  tmp_goal, path, grid_visited,false);
//             if(found){
//                     //cerr <<"找到临时目标"<<endl;
//             break;
//             }
//         }
//         vector<vector<int>> dirs = { {0,1},{1,0}, {-1,0},  {0,-1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
//         // if(angle>=0 && angle < 90.0/180*3.14)dirs = { {-1,0},  {0,-1}};
//         // if(angle>=90.0/180*3.14 && angle < 180.0/180*3.14)dirs = { {1,0},  {0,-1}};
//         // if(angle>=-180.0/180*3.14 && angle < -90.0/180*3.14)dirs = { {1,0},  {0,1}};
//         // if(angle>=-90.0/180*3.14 && angle < 0)dirs = { {-1,0},  {0,1}};
//         for(auto d : dirs) {
//             int x = i + d[0], y = j + d[1];
//             Vec2d tmp_pos = index_to_pos(Vec2i(y,x));
//             if(x >= 0 && x < 200 && y >= 0 && y < 200 &&
//                 !is_visited[x][y]&& map_3_.ela_grid_[x][y]==0 &&/*(tmp_pos - poss[0]).mod() > 1.0*/
//                 !((tmp_pos - poss[0]).mod() < 2.0 && (tmp_pos - poss[0]) * avoid_dir < 0)) {
//                 is_visited[x][y] = true;
//                 que.push(Vec2i(x,y));
//                 // cerr << x <<" "<< y<<endl;
//             }
//         }
//     }
//     return  tmp_goal;
// }

inline bool cmp(pair<int, pair<int,int>> a , pair<int, pair<int,int>> b) {
    if(a.second.second > b.second.second)return true;
    else return false;
}

Vec2d AstarSearch::bfs(Vec2d robot_pos, vector<Vec2d> obs, Vec2d poss, double dir, char grid_visited[200][200],vector<Vec2d> other_robots) {
    Vec2i robot_idx = pos_to_index(robot_pos);
    vector<vector<bool>> is_visited(200,vector<bool>(200,false));
    queue<Vec2i> que;
    que.push(Vec2i(robot_idx.y_,robot_idx.x_));
    Vec2d tmp_goal(0,0);
    while(!que.empty()) {
        Vec2i t = que.front();
        que.pop();
        //求出删格对应的真实位置
        int i = t.x_, j = t.y_;
        Vec2d bfs_pos = index_to_pos(Vec2i(j,i));
        //cerr << bfs_pos.x_<<" "<<bfs_pos.y_<<endl;
        //计算栅格距离路径最近的距离
        //终止条件
        double dist = 999;
        for(Vec2d p:obs) {
            //cerr << p.x_<<" "<<p.y_<<endl;
            if((p-bfs_pos).mod()<dist){
                dist=(p-bfs_pos).mod();
            }
        }
        for(Vec2d p:other_robots) {
            //cerr << p.x_<<" "<<p.y_<<endl;
            if((p-bfs_pos).mod()<dist){
                dist=(p-bfs_pos).mod();
            }
        }
        //cerr << dist << endl;
        double safe_dis = 2.0;
        if(dist > safe_dis){
            tmp_goal = bfs_pos;
            vector<Vec2d> path;
            vector<vector<Vec2d>> obspath;
            // bool found = search(robot_pos,  tmp_goal, path, grid_visited,false,obspath,poss,false);
            bool found = search(robot_pos,  tmp_goal, path, grid_visited,false);
            if(found){
                    //cerr <<"找到临时目标"<<endl;
            break;
            }
        }
        vector<pair<int, pair<int,double>>> dirs = {{1,{1,0}},{1,{-1,0}},{-1,{1,0}},{-1,{-1,0}},{1,{0,0}},{0,{1,0}},{0,{1,0}},{0,{-1,0}}};
        for(auto& d : dirs) {
            int x = i + d.first, y = j + d.second.first;
            Vec2d tmp_pos = index_to_pos(Vec2i(y,x));
            double dir_robot = atan2(tmp_pos.y_ - robot_pos.y_,tmp_pos.x_ - robot_pos.x_ );
            double rank  = dir_robot - dir;
            rank = fabs(LIMIT(rank, -M_PI, M_PI));
            d.second.second = rank;
        }
        sort(dirs.begin(),dirs.end(), cmp);

        // if(angle>=0 && angle < 90.0/180*3.14)dirs = { {-1,0},  {0,-1}};
        // if(angle>=90.0/180*3.14 && angle < 180.0/180*3.14)dirs = { {1,0},  {0,-1}};
        // if(angle>=-180.0/180*3.14 && angle < -90.0/180*3.14)dirs = { {1,0},  {0,1}};
        // if(angle>=-90.0/180*3.14 && angle < 0)dirs = { {-1,0},  {0,1}};
        for(auto d : dirs) {
            int x = i + d.first, y = j + d.second.first;
            Vec2d tmp_pos = index_to_pos(Vec2i(y,x));
            if(x >= 0 && x < 200 && y >= 0 && y < 200 && !is_visited[x][y]&& map_3_.ela_grid_[x][y]==0 && (tmp_pos - poss).mod() > 0.8) {
                is_visited[x][y] = true;
                que.push(Vec2i(x,y));
                // cerr << x <<" "<< y<<endl;
            }
        }
    }
    return  tmp_goal;
}