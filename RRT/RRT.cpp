/*
	RRT.cpp
	
	by Peter Jan
	Nov. 20 2018
*/

#include "RRT.h"

RRT::RRT(double* map,int xSize,int ySize,int zSize,double* start,double* goal)
{
	map_ = map;
	xSize_ = xSize;
	ySize_ = ySize;
	zSize_ = zSize;
	start_ = start;
	goal_ = goal;
}

RRT::~RRT()
{
	//Might want to do something here later if we allocate on heap
}

void RRT::plan()
{

}

void RRT::connect(double* sample)
{

}

void RRT::nearestNeighbor(double* sample)
{

}

void RRT::extend(double* sample)
{

}