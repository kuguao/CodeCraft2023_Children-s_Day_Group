#pragma once

#include <iostream>
#include <queue>

#include "common.hpp"
#include "sdf.hpp"

using namespace std;

vector<Vec2d> path_optimize(Vec2d pos, vector<Vec2d> raw_path, Vec2d obs[3], Vec2d vel[3], SdfMap &sdf, bool avoid_enable_, vector<Vec2d> dynamic_objects_, int flag_);
vector<Vec2d> path_adjustment(vector<Vec2d> raw_path, SdfMap &sdf);