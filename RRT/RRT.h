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

#define EPS 0.3
#define NUMVTX 100000
#define GOALBIAS 20

/*****************************
 * Parameters of the planner *
 ******************************/

#define NUMOFDOF 3
#define GOALSAMPLE 10    //10%
#define EPSILON 0.2
#define CHECKLEN 0.05

/****************************
 * Constants of the planner *
 *****************************/
#define REACH 1
#define ADVANCED 2
#define TRAPPED 3


//Core RRT Planner
class RRT
{
	public:
		RRT(double*	map,int xSize,int ySize,int zSize,double* start,double* goal);
		~RRT();
		void planner();
		void connect(double* newPt);
		void nearestNeighbor(double* sample);
		void extend(double* sample);
    
        /***************************
         * NEW DECLARED FUNCTIONS  *
         ****************************/
    
        void random_config(double* map_size);
        int Extend(kdTree* tree ,double* q_rand );
        bool validcheck(double* p1, double* p2);
        bool Check_invalid(double* point);
        double Distance(double*p1, double*p2);
        vector<double*> makeplan(kdTree* tree, double* goal);
        double* Newpoint(double* q_near, double* q_rand);

		//Collision Checking
		bool validNewConf(double* point);

		//Generate Sample Point
		inline double genRand(double lb,double ub){return  lb+((double) rand()/RAND_MAX)*(ub-lb);}

	private:
		double* map_;
		int xSize_;
		int ySize_;
		int zSize_;
		double* start_;
		double*goal_;
		kdTree* tree_;
};



#endif
