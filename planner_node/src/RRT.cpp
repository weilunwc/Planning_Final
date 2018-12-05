/*
	RRT.cpp
	
	by Peter Jan
	Nov. 20 2018 	
*/

#include "RRT.h"

using namespace std;

RRT::RRT(int xSize,int ySize,int zSize,vector<double>  start,vector<double>  goal,octomap::OcTree* octo)
{
	xSize_ = xSize;
	ySize_ = ySize;
	zSize_ = zSize;
	start_ = start;
	goal_ = goal;

	//Bounding Box for NN search
	vector<double> cSpace;
	cSpace.push_back(-10);
	cSpace.push_back(xSize_);
	cSpace.push_back(-10);
	cSpace.push_back(ySize_);
	cSpace.push_back(0);
	cSpace.push_back(zSize_);

	boundingBox* bBox = new boundingBox(cSpace);

	//Init KD-Tree
	tree_ = new kdTree(3,bBox);

	//Init Root
	tree_->root = tree_->insert(start,tree_->root,0);

	//Collision Check
	octoTree = octo;

	//seed Random
	srand(time(NULL));
}

RRT::~RRT()
{
	//Might want to do something here later if we allocate on heap
}

bool RRT::validNewConf(vector<double>  q_rand,vector<double>  q_near,vector<double>  &q_new)
{
  octomap::OcTreeNode* result;
  octomap::point3d query;
  double dist2sample =  Distance(q_near,q_rand);
//   cout<< endl << "dist2Sample: " <<dist2sample << endl;
//   std::cout << "rand: " << q_rand[0] << " " << q_rand[1] << " " << q_rand[2] << std::endl;
//   std::cout << "near: " << q_near[0] << " " << q_near[1] << " " << q_near[2] << std::endl;
  if(dist2sample>EPSILON)
  {
	  q_new[0] = q_near[0]+EPSILON*((q_rand[0]-q_near[0])/dist2sample);
	  q_new[1] = q_near[1]+EPSILON*((q_rand[1]-q_near[1])/dist2sample);
	  q_new[2] = q_near[2]+EPSILON*((q_rand[2]-q_near[2])/dist2sample);
  }
  else
  {
	  q_new[0] = q_rand[0];
	  q_new[1] = q_rand[1];
	  q_new[2] = q_rand[2];
  }

	query = octomap::point3d(q_new[0], q_new[1], q_new[2]);
	result = octoTree->search(query);

	// tree_->NN(q_new,tree_->root,0,tree_->BB->bounds);	
	cout << "treesize: " << tree_->size << endl;

  if(!check_result(result))
  {
	std::cout << "new: " << q_new[0] << " " << q_new[1] << " " << q_new[2] << std::endl;
	tree_->insert(q_new,tree_->root,0);
  } 
  return !check_result(result);
}

vector<double>  RRT::random_config()
{
	vector<double> retVec;
	retVec.push_back(genRand(-30,30));
	retVec.push_back(genRand(-30,30));
	retVec.push_back( genRand(0,3));
	return retVec;
}


void RRT::plan()			
{	
	for(int i=0;i<NUMVTX;i++)
	{
		std::cout << std::endl << 	"Planning: " << i << std::endl;
		vector<double> q_rand;
		int status;
		if(i%GOALBIAS == 0)
		{
			q_rand = goal_;
		}
		else
		{
			q_rand = random_config();
		}
		status = Extend(q_rand);
		if(status == REACHED && q_rand == goal_)
		{
			cout << " FUCKKK YEEEAAA!!! " <<endl;
			exportPlan();
			break;
		}
	}
}

int RRT::Extend(vector<double> q_rand )
{
	vector<double>  q_near;
	vector<double>  q_new;
	q_near.resize(3);
	q_new.resize(3);

	// std::cout << "rand: " << q_rand[0] << " " << q_rand[1] << " " << q_rand[2] << std::endl;
	tree_->bestDist = INF;
	tree_->NN(q_rand, tree_->root, 0,tree_->BB->bounds);
	std::cout << "bestPoint: " << tree_->bestPoint[0] << " " << tree_->bestPoint[1] << " " << tree_->bestPoint[2] << std::endl;

	q_near[0] = tree_->bestPoint[0];
	q_near[1] = tree_->bestPoint[1];
	q_near[2] = tree_->bestPoint[2];

	if(validNewConf(q_rand,q_near,q_new))
    {
    	//comparePt(q_rand,q_new,3)
		if(q_rand == q_new) 
		{
			return REACHED;
		}
		else
		{
			return ADVANCED;
		} 
	}
	return TRAPPED;
}

double RRT::Distance(vector<double> p1, vector<double> p2)
{

	double distance_tmp = 0;
	double distance = 0;	

	for(int i = 0; i < NUMOFDOF; i++)
	{
		distance_tmp =(p1[i] - p2[i]) * (p1[i] - p2[i]);
		distance += distance_tmp;
	}
	
	distance = sqrt(distance);
	return distance;
}

bool RRT::comparePt(vector<double>  p1,vector<double>  p2,int dim)
{
	bool retVal;
	int count = 0;
	for(int i=0;i<dim;i++)
	{
		if(p1[i]==p2[i])
		{
			count++;
		}
	}
	if(count == dim)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool RRT::check_result(octomap::OcTreeNode* node) {
	if (node != NULL) {
		if (node->getOccupancy() > 0.5) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}


std::vector<std::vector<double> > RRT::exportPlan()
{
	vector< vector<double> > vector_plan;
	tree_->NN(goal_,tree_->root,0,tree_->BB->bounds);
	Node* tmp = tree_->nearestNode;
    
    vector_plan.push_back(goal_);
	
	while(tmp != NULL)
	{
		vector_plan.push_back(tmp->data);
		tmp = tmp->parent;
	}
	
	return vector_plan;
}
