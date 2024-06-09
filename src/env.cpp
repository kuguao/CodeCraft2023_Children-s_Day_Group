#include <iostream>
#include <tuple>
#include <vector>
#include <cstring>

#include "env.hpp"
#include "common.hpp"

using namespace std;

void GridMap::build_grid_map() {

    vector<vector<char>> map2 = map_;
    for (int i = 1; i < 99; i++) {
        for (int j = 1; j < 99; j++) {
            if ((map2[i][j] == '.' && map2[i-1][j+1] == '#'&& map2[i+1][j-1] == '#') ||
                (map2[i][j] == '.' && map2[i-1][j-1] == '#'&& map2[i+1][j+1] == '#'))
                map2[i][j] = 'T';
        }
    }

    for (int i = 1; i < 100; i++) {
        for (int j = 1; j < 100; j++) {
            if ((map2[i][j] == 'T'))
                map2[i][j] = '#';
        }
    }

    for (int i = 0; i < rows_; ++i) 
        for (int j = 0; j < cols_; ++j) {
            if (map2[i][j] == OBSTACLE)
                grid_[i][j] = 1;
            else
                grid_[i][j] = 0;
        }

    ela_grid_.resize(2*rows_, vector<int>(2*cols_, 0));
    for (int i = 0; i < rows_; ++i) {
        for (int j = 0; j < cols_; ++j) {
            if (grid_[i][j] == 1){
                ela_grid_[2*i][2*j] = 1;
                ela_grid_[2*i][2*j+1] = 1;
                ela_grid_[2*i+1][2*j] = 1;
                ela_grid_[2*i+1][2*j+1] = 1;
            } 
            else {
                ela_grid_[2*i][2*j] = 0;
                ela_grid_[2*i][2*j+1] = 0;
                ela_grid_[2*i+1][2*j] = 0;
                ela_grid_[2*i+1][2*j+1] = 0;
            }
        }
    }
    ela_map_.resize(2*rows_, vector<char>(2*cols_, '.'));
    for (int i = 0; i < rows_; ++i) {
        for (int j = 0; j < cols_; ++j) {
            ela_map_[2*i][2*j] = map_[i][j];
            ela_map_[2*i][2*j+1] = map_[i][j];
            ela_map_[2*i+1][2*j] = map_[i][j];
            ela_map_[2*i+1][2*j+1] = map_[i][j]; 
        }
    }
    // for (int i = 0; i < ela_map_.size(); ++i) {
    //     for (int j = 0; j < ela_map_[0].size(); ++j) {
    //         cerr << ela_map_[i][j];
    //     }
    // cerr << endl;
    // }
    // for (int i = 0; i < 100; i++) {
    //     for (int j = 0; j < 100; j++) {
    //         cerr << map2[i][j];
    //     }
    //     cerr << endl;
    // }
}

int GridMap::Getgridvalue(int row, int col) {
    if (row < 0 || row >= rows_ || col < 0 || col >= cols_)
        return -1;
    else
        return grid_[row][col];
}

tuple<int, int, vector<Robot>, vector<vector<double>>> get_state(vector<Workbench> &workbenchs, vector<Robot> &robots, vector<vector<double>> &lidar_data) {
    char line[10240];
    int cnt = 0;
    int frameID, score;
    int workbench_num = 0;
    if (scanf("%d", &frameID) != EOF) {
        while (fgets(line, sizeof line, stdin)) {
            // cerr << line;
            if (line[0] == 'O' && line[1] == 'K') {
                break;
            }
            if (cnt == 0) {
                sscanf(line, "%d", &score);
            } else if (cnt == 1) {
                sscanf(line, "%d", &workbench_num);
            } else if (cnt > 1 && cnt <= 1 + workbench_num) {
                int i = cnt -2;
                sscanf(line,
                    "%d %lf %lf %d %d %d", 
                    &workbenchs[i].type_, &workbenchs[i].pos_.x_, &workbenchs[i].pos_.y_, 
                    &workbenchs[i].remain_time_, &workbenchs[i].material_state_, 
                    &workbenchs[i].have_product_);
            } else if (cnt > 1 + workbench_num && cnt <= workbench_num + 5){
                int j = cnt -2 -workbench_num;
                sscanf(line, 
                    "%d %d %lf %lf %lf %lf %lf %lf %lf %lf", 
                    &robots[j].workbech_id_, &robots[j].item_, &robots[j].tv_factor_,
                    &robots[j].cv_factor_, &robots[j].angular_, &robots[j].linear_.x_,
                    &robots[j].linear_.y_, &robots[j].angle_,
                    &robots[j].pos_.x_, &robots[j].pos_.y_);
            } else {
                //雷达数据：
                // cerr << line;
                int k = cnt -6 - workbench_num;
                int i = 0;
                char *ptr = strtok(line, " "); // 使用空格作为分隔符，将一行数据分割为多个子串
                while (ptr != NULL) {
                    // cerr << "i = " << i << endl;
                    sscanf(ptr, "%lf", &lidar_data[k][i]); // 将子串转换为双精度浮点型数据，存储到数组中
                    ptr = strtok(NULL, " ");
                    i++;
                }
            }
            cnt++;
        }
        return make_tuple(frameID, score, robots, lidar_data);
    } else {
        return make_tuple(-1, -1, robots, lidar_data);
    }
}

void set_robot(RobotCmd rcs[4], int frame_id) {
    printf("%d\n", frame_id);
    for(int robotId = 0; robotId < 4; robotId++){
        printf("forward %d %f\n", robotId, rcs[robotId].linear_);
        printf("rotate %d %f\n", robotId, rcs[robotId].angular_);
        if (rcs[robotId].buy_) {
            printf("buy %d\n", robotId);
        }
        if (rcs[robotId].sell_) {
            printf("sell %d\n", robotId);
        }
        if (rcs[robotId].destroy_) {
            printf("destroy %d\n", robotId);
        }
    }
    
    printf("OK\n");
    fflush(stdout);
}

vector<vector<char>> map(100, vector<char>(100, '.'));
vector<vector<char>> dilate_map(100, vector<char>(100, '.'));
int FLAG = 0; //红蓝机器人 0:蓝方 1:红方
double MAX_VEL;

bool readUntilOK() {
    char line[1024];
    int row = 0;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        if (line[0] == 'B' && line[1] == 'L' && line[2] == 'U' && line[3] == 'E') {
            FLAG = 0;
            cerr << "blue" << endl;

        }
        if (line[0] == 'R' && line[1] == 'E' && line[2] == 'D') {
            FLAG = 1;
            cerr << "red" << endl;

        }
        if ((line[0] != 'B') && (line[0] != 'R')) {
            for (int i = 0; i < 100; i++) {
                map[row][i] = line[i];
            }
            row++;
            
        }  
    }
    return false;
}

//read map 
tuple <GridMap, vector<Workbench>, vector<Robot>, vector<vector<int>>, vector<Workbench>, vector<Robot>> Init() {
    readUntilOK();
    if (FLAG == 0) {
        MAX_VEL = 6.0;
    } else {
        MAX_VEL = 7.0;
    }
    GridMap gridmap = GridMap(map);
    vector<Workbench> workbenchs;
    vector<Workbench> other_workbechs;
    vector<Robot> robots;
    vector<Robot> other_robots;
    if (FLAG == 0) {//蓝方
        for (int i = 0; i < map.size(); i++) {
            for (int j = 0; j < map[i].size(); j++) {
                for (int k = 49; k < 58; k++) { //蓝方工作台
                    if (map[i][j] == k) {
                    workbenchs.push_back(Workbench());
                    Workbench &w = workbenchs[workbenchs.size() - 1];
                    w.index_ = workbenchs.size() - 1;
                    w.type_ = k - 48;
                    w.pos_.x_ = j * 0.5 + 0.25;
                    w.pos_.y_ = 50 - (i * 0.5 + 0.25);
                }  
            }
                for (int m = 97; m < 106; m++) {
                    if (map[i][j] == m) { //红方工作台
                    other_workbechs.push_back(Workbench());
                    Workbench &w = other_workbechs[other_workbechs.size() - 1];
                    w.index_ = other_workbechs.size() - 1;
                    w.type_ = m - 96;
                    w.pos_.x_ = j * 0.5 + 0.25;
                    w.pos_.y_ = 50 - (i * 0.5 + 0.25);
                }  
            }            
            if (map[i][j] == 'A') {//蓝方机器人
                robots.push_back(Robot());
                Robot &r = robots[robots.size() - 1];
                r.index_ = robots.size() - 1;
                r.pos_.x_ = j * 0.5 + 0.25;
                r.pos_.y_ = 50 - (i * 0.5 + 0.25);
            }
            if (map[i][j] == 'B') {//红方机器人
                other_robots.push_back(Robot());
                Robot &r = other_robots[other_robots.size() - 1];
                r.index_ = other_robots.size() - 1;
                r.pos_.x_ = j * 0.5 + 0.25;
                r.pos_.y_ = 50 - (i * 0.5 + 0.25);
            } 
        }
        }
    }

    if (FLAG == 1) {//红方
        for (int i = 0; i < map.size(); i++) {
            for (int j = 0; j < map[i].size(); j++) {
                for (int k = 97; k < 106; k++) {
                    if (map[i][j] == k) {//红方工作台
                    workbenchs.push_back(Workbench());
                    Workbench &w = workbenchs[workbenchs.size() - 1];
                    w.index_ = workbenchs.size() - 1;
                    w.type_ = k - 96;
                    w.pos_.x_ = j * 0.5 + 0.25;
                    w.pos_.y_ = 50 - (i * 0.5 + 0.25);
                }  
            }
            for (int k = 49; k < 58; k++) {
                    if (map[i][j] == k) {//蓝方工作台
                    other_workbechs.push_back(Workbench());
                    Workbench &w = other_workbechs[other_workbechs.size() - 1];
                    w.index_ = other_workbechs.size() - 1;
                    w.type_ = k - 48;
                    w.pos_.x_ = j * 0.5 + 0.25;
                    w.pos_.y_ = 50 - (i * 0.5 + 0.25);
                }  
            }
            if (map[i][j] == 'B') {//红方机器人
                robots.push_back(Robot());
                Robot &r = robots[robots.size() - 1];
                r.index_ = robots.size() - 1;
                r.pos_.x_ = j * 0.5 + 0.25;
                r.pos_.y_ = 50 - (i * 0.5 + 0.25);
            } 
            if (map[i][j] == 'A') {//蓝方机器人
                other_robots.push_back(Robot());
                Robot &r = other_robots[other_robots.size() - 1];
                r.index_ = other_robots.size() - 1;
                r.pos_.x_ = j * 0.5 + 0.25;
                r.pos_.y_ = 50 - (i * 0.5 + 0.25);
            }
        }
        }
    }
    cerr << "FLAG= " << FLAG << endl;
    cerr << "workbenchs.size()" << workbenchs.size() << endl;
    cerr << "other_workbechs.size()" << other_workbechs.size() << endl;
    // for (int i = 0; i < 4; i++) {
    //     cerr << "my_robots" << robots[i].pos_.x_ << " " << robots[i].pos_.y_ << endl;
    // }
    // for (int i = 0; i < 4; i++) {
    //     cerr << "other_robots" << other_robots[i].pos_.x_ << " " << other_robots[i].pos_.y_ << endl;
    // }
    // vector<vector<int>> newworkbenchs_index(4);
    auto newworkbenchs_index = Connect_detect(workbenchs, robots);
    return make_tuple(gridmap, workbenchs, robots, newworkbenchs_index, other_workbechs, other_robots);
    // cerr << "line";
}

//连通性检测：
//传入每个机器人的位置,all workbench
//已这个位置进行搜索，找到所有可达的工作台
//返回所有可达的工作台，之后才进行路径搜索
inline Vec2i pos_to_index(Vec2d& pos) {
    return Vec2i(int ((50 - pos.y_ - 0.25) / 0.5), int ((pos.x_ - 0.25) / 0.5));
}

int m = 100;
int visited[100][100];

vector<vector<int>> Connect_detect(vector<Workbench> workbenchs, vector<Robot>& robots) {
    vector<vector<int>> newwb_index(4);
    //先膨胀地图
    if (FLAG == 0) //蓝方
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 100; j++) {
            if (map[i][j] == '#') {
                dilate_map[i][j] = '#';
                if (((i - 1) > 0) && ((i + 1) < m) && (map[i-1][j] == '.' || map[i-1][j] > '3')) {
                    dilate_map[i-1][j] = '#';
                }
                if (((i - 1) > 0) && ((i + 1) < m) && (map[i+1][j] == '.' || map[i+1][j] > '3')) {
                    dilate_map[i+1][j] = '#';
                }
                if (((j - 1) > 0) && ((j + 1) < m) && (map[i][j-1] == '.' || map[i][j-1] > '3')) {
                    dilate_map[i][j-1] = '#';
                }
                if (((j - 1) > 0) && ((j + 1) < m) && (map[i][j+1] == '.' || map[i][j+1] > '3')) {
                    dilate_map[i][j+1] = '#';
                }
            }
        }
    }
    if (FLAG == 1) //红方
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 100; j++) {
            if (map[i][j] == '#') {
                dilate_map[i][j] = '#';
                if (((i - 1) > 0) && ((i + 1) < m) && (map[i-1][j] == '.' || map[i-1][j] > 'c')) {
                    dilate_map[i-1][j] = '#';
                }
                if (((i - 1) > 0) && ((i + 1) < m) && (map[i+1][j] == '.' || map[i+1][j] > 'c')) {
                    dilate_map[i+1][j] = '#';
                }
                if (((j - 1) > 0) && ((j + 1) < m) && (map[i][j-1] == '.' || map[i][j-1] > 'c')) {
                    dilate_map[i][j-1] = '#';
                }
                if (((j - 1) > 0) && ((j + 1) < m) && (map[i][j+1] == '.' || map[i][j+1] > 'c')) {
                    dilate_map[i][j+1] = '#';
                }
            }
        }
    }

    // for (int i = 0; i < 100; i++){
    //     for (int j = 0; j < 100; j++) {
    //         cerr << dilate_map[i][j];
    //     }
    //     cerr << endl;
    // }

    for (int r = 0; r < robots.size(); r++) {
        Vec2i init_index = pos_to_index(robots[r].pos_);
        for (int i = 0; i < 100; i++)
            for (int j = 0; j < 100; j++) {
                visited[i][j] = 0;
        }
        dfs(init_index.x_, init_index.y_);
        for (int i = 0; i < workbenchs.size(); i++) {
            Vec2i wb_pos_index = pos_to_index(workbenchs[i].pos_);
            if (visited[wb_pos_index.x_][wb_pos_index.y_] == 1) {
                newwb_index[r].push_back(i);
            }  
        }
    //         for (int i = 0; i < 100; i++){
    //     for (int j = 0; j < 100; j++) {
    //         cerr << visited[i][j];
    //     }
    //     cerr << endl;
    // }
    }
    return newwb_index;
}

bool dfs(int x, int y) {
    if (x < 0 || x >= m || y < 0 || y >= m) return false;
    if (map[x][y] == '#' || visited[x][y]) return false;
    visited[x][y] = 1;
    dfs(x - 1, y);
    dfs(x + 1, y);
    dfs(x, y - 1);
    dfs(x, y + 1);
    return true;
}
bool check_wb_sell(vector<Workbench> &workbenchs, const int& i, const int& j) {
    if ((workbenchs[i].type_==1 && workbenchs[j].type_ == 4) ||
        (workbenchs[i].type_==1 && workbenchs[j].type_ == 5) ||
        (workbenchs[i].type_==1 && workbenchs[j].type_ == 9) ||
        (workbenchs[i].type_==2 && workbenchs[j].type_ == 4) ||
        (workbenchs[i].type_==2 && workbenchs[j].type_ == 6) ||
        (workbenchs[i].type_==2 && workbenchs[j].type_ == 9) ||
        (workbenchs[i].type_==3 && workbenchs[j].type_ == 5) ||
        (workbenchs[i].type_==3 && workbenchs[j].type_ == 6) ||
        (workbenchs[i].type_==3 && workbenchs[j].type_ == 9) ||

        (workbenchs[i].type_==4 && workbenchs[j].type_ == 7) ||
        (workbenchs[i].type_==4 && workbenchs[j].type_ == 9) ||
        (workbenchs[i].type_==5 && workbenchs[j].type_ == 7) ||
        (workbenchs[i].type_==5 && workbenchs[j].type_ == 9) ||
        (workbenchs[i].type_==6 && workbenchs[j].type_ == 7) ||
        (workbenchs[i].type_==6 && workbenchs[j].type_ == 9) ||

        (workbenchs[i].type_==7 && workbenchs[j].type_ == 8) ||
        (workbenchs[i].type_==7 && workbenchs[j].type_ == 9))
    return true;
    else 
        return false;
}

bool check_wb_buy(vector<Workbench> &workbenchs, const int& i, const int& j) {
    if (
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==4 && workbenchs[j].type_ == 7) ||

    (workbenchs[i].type_==5 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==5 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==5 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==5 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==5 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==5 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==5 && workbenchs[j].type_ == 7) ||

    (workbenchs[i].type_==6 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==6 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==6 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==6 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==6 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==6 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==6 && workbenchs[j].type_ == 7) ||

    (workbenchs[i].type_==7 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 7) ||

    (workbenchs[i].type_==7 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==7 && workbenchs[j].type_ == 7) ||

    (workbenchs[i].type_==8 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==8 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==8 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==8 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==8 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==8 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==8 && workbenchs[j].type_ == 7) ||

    (workbenchs[i].type_==9 && workbenchs[j].type_ == 1) ||
    (workbenchs[i].type_==9 && workbenchs[j].type_ == 2) ||
    (workbenchs[i].type_==9 && workbenchs[j].type_ == 3) ||
    (workbenchs[i].type_==9 && workbenchs[j].type_ == 4) ||
    (workbenchs[i].type_==9 && workbenchs[j].type_ == 5) ||
    (workbenchs[i].type_==9 && workbenchs[j].type_ == 6) ||
    (workbenchs[i].type_==9 && workbenchs[j].type_ == 7))
    return true;
    else
        return false;
}

