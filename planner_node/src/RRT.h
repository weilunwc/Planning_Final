/*
	RRT.h
	
	by Peter Jan
	Nov. 20 2018
*/

#ifndef _RRT_H_
#define _RRT_H_

#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <vector>
#include "kdTree.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>


#define EPS 0.3
#define NUMVTX 100000
#define GOALBIAS 2

/*****************************
 * Parameters of the planner *
 ******************************/

#define NUMOFDOF 3
#define GOALSAMPLE 10    //10%
#define EPSILON 0.5
#define CHECKLEN 0.05

/****************************
 * Constants of the planner *
 *****************************/
#define REACHED 1
#define ADVANCED 2
#define TRAPPED 3


//Core RRT Planner
class RRT
{
	public:
		RRT(int xSize,int ySize,int zSize,std::vector<double>  start,std::vector<double>  goal,octomap::OcTree* octo);
		~RRT();
		void plan();
        int Extend(std::vector<double> q_rand);

		//Collision Checking
		bool validNewConf(std::vector<double>  q,std::vector<double>  q_near,std::vector<double>  &q_new);
		bool comparePt(std::vector<double>  p1,std::vector<double>  p2,int dim);
		bool check_result(octomap::OcTreeNode* node);


		//Generate Sample Point
		inline double genRand(double lb,double ub){return  lb+((double) rand()/RAND_MAX)*(ub-lb);}
		std::vector<double>  random_config();

        double Distance(std::vector<double> p1, std::vector<double> p2);
        std::vector<std::vector<double> > exportPlan();
		std::vector<double> start_;
		std::vector<double> goal_;

	private:
		octomap::OcTree* octoTree;
		bool isValidPt;
		int xSize_;
		int ySize_;
		int zSize_;
		kdTree* tree_;
};
bool rayCast_collision(octomap::point3d origin, octomap::point3d des_pos, octomap::OcTree* tree);

#endif
