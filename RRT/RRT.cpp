/*
	RRT.cpp
	
	by Peter Jan
	Nov. 20 2018
*/

#include "RRT.h"



using namespace std;


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

	srand(time(NULL));

	// 1.T.init
	kdTree* tree = new kdTree(NUMOFDOF,bBox);
	tree->root = tree->insert(start,tree->root,0);

	vector<double*> plan;
	double* q_rand;
	bool extend_success = 0;
	int count_tree = 0;
	int max_step = 300;
    bool goalPt = false;
    double* goal_copy = new double[tree->dim];
    
    for(int i = 0; i < NUMOFDOF; i++)
    {
        goal_copy[i] = goal[i];
    }
	
    
    
	for(int step = 0; step < max_step; step++)
	{
		bool result = false;
		random_config(q_rand, map_size);
		extend_success = Extend(tree,q_rand,map,x_size,y_size);

        if(i%GOALSAMPLE == 0)
        {
            q_rand = armgoal_anglesV_rad;
            goalPt = true;
        }
        else
        {
            random_config(q_rand, map_size);
            goalPt = false;
        }
        
        tree->bestDist = INF; //done
        
        //extend tree
        if(Extend(tree,sample,map,x_size,y_size) == 1 && goalPt == true)
        {
            cout << "FOUND GOAL" << endl;
            cout << "count: " << i << endl;
            foundpath = true;
            //Export Plan
            break;
        }
	}
    
    vector<double*> vector_plan;
	if(foundpath)
    {
        vector_plan = makeplan(tree,armgoal_anglesV_rad);
    }
    else
    {
        cout << "No path found" <<endl;
        return;
    }
    
    int pathlength = vector_plan.size();
    
    *plan = (double**) malloc(pathlength * sizeof(double*));
    for(int i = 0; i<NUMOFDOF; i++)
    {
        (*plan)[i] = (double*) malloc(NUMOFDOF * sizeof(double));
        for(int j=0; j<NUMOFDOF; j++)
        {
            (*plan)[i][j] = vector_plan[vector_plan.size() -i -1][j];
        }
    }
    
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


void RRT::random_config(double* q_rand,double* map_size)
{
	
	for(int i = 0; i < NUMOFDOF; i++)
	{
		q_rand[i] = fmod(rand(),map_size[i]);
	}
	
	return;	
}

int RRT::Extend(kdTree* tree ,double* q_rand )
{
	bool config_result = 0;
	double distance = 0;
	double* q_near;
	double* q_new;
    double* g_rand_copy = new double[tree->dim];
    
    for(int i = 0; i < NUMOFDOF, i++)
    {
        q_rand_copy[i] = q_rand[i];
    }
        
        // Find the nearest point and distance between random point and nearest point
	tree->NN(q_rand, tree->root, 0, tree->BB->bounds);
	q_near = tree->bestPoint;
	distance = Distance(q_near, q_rand);
     
	if(distance <= EPSILON)
	{
		q_new = q_rand_copy;
	}
	else
	{
		q_new = Newpoint(q_new,q_near, q_rand_copy);
	}

    tree->NN(q_new, tree->root, 0, tree->BB->bounds);
	config_result = (validcheck(q_new, tree->bestPoint,map)); //change using map info
	
	 
        // config_result -> 1: Valid , 0: Invalid
	if(config_result)
	{
		Node* point;
        tree->NN(q_new,tree->root,0, tree->BB->bounds);
		tree->insert(q_new, tree->root,0);
		
		if(q_new == q_rand) return REACH;
		
		else return ADVANCED;
	}
	return TRAPPED;
}


void RRT::Newpoint(double* q_new, double* q_near, double* q_rand)
{
	double dir[NUMOFDOF];
	double distance = Distance(q_near,q_rand);

	for(int i = 0; i < NUMOFDOF; i++)
	{
		dir[i] = q_rand[i] - q_near[i];
		q_new[i] = q_near[i] + dir[i] * EPSILON/distance; 
	}
}



bool RRT::validcheck(double* p1, double* p2)
{
	bool result = 1;
	double num_interpol = 0;
	double check_points[NUMOFDOF];
	double distance = 0;
	double rate1 = 0;
	double rate2 = 0;
	
	distance = Distance(p1,p2);
	num_interpol = 4;

	for(int i = 0; i < num_interpol; i++)
	{
		for(int j = 0; j < NUMOFDOF; j++)
		{
			rate1 = i / num_interpol;
			rate2 = 1 - i/num_interpol;
			check_points[j] = rate1*p1[i] + rate2*p2[i];
		}

		result &= map.check(check_points);
		
		if(result == 1)
		{
			return true;
		}		 
	}
	
	return false;
	
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
	vector<double*> vector_plan;
	tree->NN(goal,tree->root,0,tree->BB->bounds);
	Node* tmp = tree->nearestNode;
    
    vector_plan.push_back(goal);
	
	while(tmp != NULL)
	{
		plan.push_back(tmp->data);
		tmp = tmp->parent;
	}
	
	return vector_plan;
}
