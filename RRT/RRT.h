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

//Core RRT Planner
class RRT
{
	public:
		RRT(double*	map,int xSize,int ySize,int zSize,double* start,double* goal);
		~RRT();
		void plan();
		void connect(double* newPt);
		void nearestNeighbor(double* sample);
		void extend(double* sample);

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