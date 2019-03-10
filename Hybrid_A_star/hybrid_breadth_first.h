#ifndef HYBRID_BREADTH_FIRST_H_
#define HYBRID_BREADTH_FIRST_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class HBF {
public:

	int NUM_THETA_CELLS = 90; // the number of cells a circle is divided into
	double SPEED = 3.45;  // speed 
	double LENGTH = 0.5;  // wheelbase 

	struct maze_s {

		int g;	// ��ǰ���˶��ٲ�
		int f;  // ��ǰ���˶��ٲ�+��������=���յ�Ԥ��cost 
		double x;
		double y;
		double theta;
	};

	struct maze_path {

		vector< vector< vector<int> > > closed;        // vector��ӦССդ����,����,������ intΪ 0��1 �Ƿ�close 
		vector< vector< vector<maze_s> > > came_from;
		maze_s final;

	};


	/**
	* Constructor
	*/
	HBF();

	/**
	* Destructor
	*/
	virtual ~HBF();

	static bool compare_maze_s(const HBF::maze_s & lhs, const HBF::maze_s & rhs);

	double heuristic(double x, double y, vector<int> goal);

	int theta_to_stack_number(double theta);

	int idx(double float_num);

	vector<maze_s> expand(maze_s state, vector<int> goal);

	maze_path search(vector< vector<int> > grid, vector<double> start, vector<int> goal);

	vector<maze_s> reconstruct_path(vector< vector< vector<maze_s> > > came_from, vector<double> start, HBF::maze_s final);


};

#endif

