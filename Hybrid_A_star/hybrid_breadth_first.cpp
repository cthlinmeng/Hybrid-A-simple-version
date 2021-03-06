#include "stdafx.h"
#include "hybrid_breadth_first.h"

#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846 
#endif // !M_PI



using namespace std;


/**
* Initializes HBF
*/
HBF::HBF() {

}

HBF::~HBF() {}

bool HBF::compare_maze_s(const HBF::maze_s & lhs, const HBF::maze_s & rhs) {
	// 在sort函数中使用，按f值从小到大排序
	return lhs.f < rhs.f;
}

double HBF::heuristic(double x, double y, vector<int> goal) {
	return fabs(x - goal[0]) + fabs(y - goal[1]); //两个直线距离dx+dy
}

int HBF::theta_to_stack_number(double theta) {
	/*
	eg：NUM_THETA_CELLS=10，每个小模块0.2pi，[1.9pi,0.1pi)时stack_number=0，[0.1pi,0.3pi)时stack_number=1;
	*/

	double new_theta = fmod((theta + 2 * M_PI), (2 * M_PI));
	int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2 * M_PI))) % NUM_THETA_CELLS;  // round 四舍五入取整
	return stack_number;
}


int HBF::idx(double float_num) {
	/*
	Returns the index into the grid for continuous position. So if x is 3.621, then this
	would return 3 to indicate that 3.621 corresponds to array index 3.
	*/

	return int(floor(float_num));
}


vector<HBF::maze_s> HBF::expand(HBF::maze_s state, vector<int> goal) {
	int g = state.g;
	double x = state.x;
	double y = state.y;
	double theta = state.theta;

	int g2 = g + 1;
	vector<HBF::maze_s> next_states;
	for (double delta_i = -35; delta_i < 40; delta_i += 5)
	{

		double delta = M_PI / 180.0 * delta_i;
		double omega = SPEED / LENGTH * tan(delta);
		double theta2 = theta + omega;
		if (theta2 < 0)
		{
			theta2 += 2 * M_PI;
		}
		double x2 = x + SPEED * cos(theta);
		double y2 = y + SPEED * sin(theta);
		HBF::maze_s state2;
		state2.f = g2 + heuristic(x2, y2, goal);
		state2.g = g2;
		state2.x = x2;
		state2.y = y2;
		state2.theta = theta2;
		next_states.push_back(state2);

	}
	return next_states;
}

vector< HBF::maze_s> HBF::reconstruct_path(vector< vector< vector<HBF::maze_s> > > came_from, vector<double> start, HBF::maze_s final) {

	// 在 came from 中选取出最终的path，思路是从终点倒推到起点。

	vector<maze_s> path = { final };

	int stack = theta_to_stack_number(final.theta);

	maze_s current = came_from[stack][idx(final.x)][idx(final.y)];

	stack = theta_to_stack_number(current.theta);

	double x = current.x;
	double y = current.y;
	while (x != start[0] || y != start[1])
	{
		path.push_back(current);
		current = came_from[stack][idx(x)][idx(y)];
		x = current.x;
		y = current.y;
		stack = theta_to_stack_number(current.theta);
	}

	return path;

}

HBF::maze_path HBF::search(vector< vector<int> > grid, vector<double> start, vector<int> goal) {
	// breadth first search method
	vector< vector< vector<int> > > closed(NUM_THETA_CELLS, vector<vector<int>>(grid.size(), vector<int>(grid[0].size())));  
	vector< vector< vector<maze_s> > > came_from(NUM_THETA_CELLS, vector<vector<maze_s>>(grid.size(), vector<maze_s>(grid[0].size())));
	double theta = start[2];
	int stack = theta_to_stack_number(theta);
	int g = 0;

	maze_s state;
	state.g = g;
	state.x = start[0];
	state.y = start[1];
	state.f = g + heuristic(state.x, state.y, goal);
	state.theta = theta;

	closed[stack][idx(state.x)][idx(state.y)] = 1;
	came_from[stack][idx(state.x)][idx(state.y)] = state;
	int total_closed = 1;
	vector<maze_s> opened = { state };
	bool finished = false;
	while (!opened.empty()
	{
		sort(opened.begin(), opened.end(), compare_maze_s);  //sort open
		maze_s current = opened[0]; //grab first elment
		opened.erase(opened.begin()); //pop first element

		int x = current.x;
		int y = current.y;

		if (idx(x) == goal[0] && idx(y) == goal[1])             //是否到达终点
		{
			cout << "found path to goal in " << total_closed << " expansions" << endl;
			maze_path path;
			path.came_from = came_from;
			path.closed = closed;
			path.final = current;
			return path;

		}
		vector<maze_s> next_state = expand(current, goal);

		for (int i = 0; i < next_state.size(); i++)
		{
			int g2 = next_state[i].g;
			double x2 = next_state[i].x;
			double y2 = next_state[i].y;
			double theta2 = next_state[i].theta;

			if ((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))      // x,y 不越界
			{
				//invalid cell
				continue;
			}
			int stack2 = theta_to_stack_number(theta2);

			if (closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0)  // x,y 没有被close，并且没有障碍物
			{

				opened.push_back(next_state[i]);
				closed[stack2][idx(x2)][idx(y2)] = 1;
				came_from[stack2][idx(x2)][idx(y2)] = current;
				total_closed += 1;
			}
		}

	}
	cout << "no valid path." << endl;
	HBF::maze_path path;
	path.came_from = came_from;     // no valid path，返回初始状态
	path.closed = closed;
	path.final = state;
	return path;
}


