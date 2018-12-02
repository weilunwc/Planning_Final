/*
	RRT.cpp
	
	by Peter Jan
	Nov. 20 2018
*/

#include "RRT.h"



using namespace std;

/*****************************
 * Parameters of the planner *
******************************/

#define NUMOFDOF 3
#define GOALSAMPLE 10    //10% 
#define EPSILON 0.2

/****************************
 * Constants of the planner *
*****************************/
#define REACH 1
#define ADVANCED 2
#define TRAPPED 3 

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



RRT::RRT(double* map,int xSize,int ySize,int zSize,double* start,double* goal)
{
	map_ = map;
	xSize_ = xSize;
	ySize_ = ySize;
	zSize_ = zSize;
	start_ = start;
	goal_ = goal;
}
void RRT::planner()
{	
	bool foundpath = false;
	double map_size[NUMOFDOF] = {xSize_, ySize_, zSize_}; 
        double cSpace[2 * NUMOFDOF];	
	for(int i = 0; i<NUMOFDOF; i++)
	{
		cSpace[2*i] = 0;
		cSpace[2*i +1] = map_size[i];
	}
	
	boundingBox* bBox = new boundingBox(cSpace);

	
	
/***********************************************************
 * 1. T.init                                               *
 *                                                         *
 * 2. For every step                                       * 
 *    2-1. make a random point                             *
 *    2-2. EXTEND between tree and a random point          *
 *                                                         *
 * 3. In EXTEND function                                   *
 *                                                         *
 ***********************************************************/
	srand(time(NULL));

	//T.init
	kdTree* tree = new kdTree(NUMOFDOF,bBox);
	tree->root = tree->insert(start,tree->root,0);

	vector<double*> plan;
	double q_rand[NUMOFDOF];
	bool extend_success = 0;
	int count_tree = 0;
	int max_step = 300;
	
	for(int step = 0; step < max_step; step++)
	{
		bool result = false;
		random_config(q_rand, map_size);
		extend_success = Extend(tree,q_rand);

		if(extend_success != TRAPPED) count_tree++; //check whether this is necessary when make the plan
		
		if(step % GOALSAMPLE == 0)
		{
			q_rand = goal;
			result = validcheck(tree->bestPoint , goal);
	
			if(result)
			{
				foundpath = true;
				break;
			}
			else continue;
		}
	}
	
	plan = makeplan(tree,goal); 
	
	return plan; 
	
}

RRT::~RRT()
{
	//Might want to do something here later if we allocate on heap
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


void random_config(double* q_rand,double* map_size)
{
	
	for(int i = 0; i < NUMOFDOF; i++)
	{
		q_rand[i] = fmod(rand(),map_size[i]);
	}
	
	return;	
}

int Extend(kdTree* tree ,double* q_rand )
{
	bool config_result = 0;
	double distance = 0;
	double* q_near;
	double* q_new;
	
        
        // Find the nearest point and distance between random point and nearest point
	tree->NN(q_rand, tree->root, 0, tree->BB->bounds);
	q_near = tree->bestPoint;
	distance = Distance(q_near, q_rand);
     
	if(distance <= EPSILON)
	{
		q_new = q_rand;
	}
	else
	{
		q_new = Newpoint(q_near, q_rand);
	}

         
	config_result = (map.check(q_new) && validcheck(q_new, tree->bestPoint));
	
	 
        // config_result -> 1: Valid , 0: Invalid
	if(config_result)
	{
		Node* point;

		tree->NN(q_new,tree->root,0,tree->BB->bounds);
		point = tree->insert(q_new, tree->root,0);
		//save the parent 
		point->parent = q_near;

		if(q_new == q_rand) return REACH;
		
		else return ADVANCED;
	}
	return TRAPPED;
}


double* Newpoint(double* q_near, double* q_rand)
{
	double dir[NUMOFDOF];
	double q_new[NUMOFDOF];
	double distance;

	for(int i = 0; i < NUMOFDOF; i++)
	{
		dir[i] = q_rand - q_near;
		distance = Distance(q_rand, q_near);
		q_new[i] = q_near[i] + dir[i] * EPSILON/distance; 
	}
}



bool validcheck(double* p1, double* p2)
{
	bool result = 0;
	double num_interpol = 0;
	double check_points[NUMOFDOF];
	double distance = 0;
	double rate1 = 0;
	double rate2 = 0;
	
	distance = Distance(p1,p2);
	num_interpol = (int)distance/EPSILON;	

	for(int i = 0; i < num_interpol; i++)
	{
		for(int j = 0; j < NUMOFDOF; j++)
		{
			rate1 = i / num_interpol;
			rate2 = 1 - i/num_interpol;
			check_points[j] = rate1*p1[i] + rate2*p2[i];
		}

		result |= Check_invalid(check_points);
		
		if(result == 1)
		{
			return false;
		}		 
	}
	
	return true;
	
}

bool Check_invalid(double* point)
{
	return map.check(point);
}

double Distance(double*p1, double*p2)
{

	double distance_tmp = 0;
	double distance = 0;	

	for(int i = 0; i < NUMOFDOF; i++)
	{
		distance_tmp =(p1[i] - p2[i]) * (p1[i] - p2[i]);
	}
	
	distance += distance_tmp;
	distance = sqrt(distance);

	return distance;
}

vector<double*> makeplan(kdTree* tree, double* goal)
{
	vector<double*>plan;
	tree->NN(goal,tree->root,0,tree->BB->bounds);
	Node* tmp = tree->nearestNode;
	
	while(tmp != NULL)
	{
		plan.push_back(tmp->data);
		tmp = tmp->parent;
	}
	
	return plan;
}
