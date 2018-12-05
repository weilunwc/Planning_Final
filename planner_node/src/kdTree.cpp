/*

	KD-tree Class
	By Peter Jan
	Oct. 15 2018

*/


#include "kdTree.h"
#include <unistd.h>


using namespace std;

kdTree::kdTree(int dimension,boundingBox* BBox):
dim(dimension),
BB(BBox)
{
	root = NULL;
	bestDist = INF;
}

kdTree::~kdTree()
{
	deleteTree(root);
}

Node* kdTree::insert(vector<double> point,Node* n,int cd)
{	
	// cout << "Inserting!" << endl;
	if(n == NULL)
	{
		// cout <<"NULL!" << endl;
		n = new Node();
		n->data = point;
		n->parent = nearestNode;
		size++;
	}
	else if(point == n->data)
	{
		// cout << "DUPLICATE ENTRY" << endl;	
	}
	else if(point[cd] < n->data[cd])
	{
		// cout << "nLeft: "<< n->left << endl;
		n->left = insert(point,n->left,(cd+1)%dim);
	}
	else
	{
		// cout << "nRight: "<< n->right << endl;
		n->right = insert(point,n->right,(cd+1)%dim);
	}
	// cout << "n: "<<n << endl;
	return n;
}

void kdTree::kNN(std::vector<double> point, Node* n, int cd, std::vector<double> nodeBound,double r)
{
	vector< Node* > kNearestNodes;
	vector< vector<double> > kNearest;
	while(kNearestNodes.size()<k)
	{
		NN(point,root,0,BB->bounds);
		kNearestNodes.push_back(nearestNode);
		nearestNode->alreadyFound = true;
	}

}

void  kdTree::NN(vector<double>point, Node* n, int cd, vector<double>nodeBound)
{
	double dist;
	// cout << "n: " << n << " dist2Box: " << distance2Box(point,nodeBound)<< " bestDist: "<< bestDist << endl;
	if(n == NULL || distance2Box(point,nodeBound) > bestDist) 
		{
			return;
		}	

	dist = INF;
	dist = distance(point,n->data);
	// cout << "dist: " << dist << " best: " << bestDist << endl;
	if(dist < bestDist && n->alreadyFound == false)
	{
		nearestNode = n;
		bestPoint = n->data;
		bestDist = dist;
		// std::cout << "bestPoint: " << bestPoint[0] << " " << bestPoint[1] << " " << bestPoint[2] << std::endl;
	}

	if(point[cd]<=n->data[cd])
	{
		NN(point,n->left,(cd+1)%dim,BB->trimLeft(cd,point));
		NN(point,n->right,(cd+1)%dim,BB->trimRight(cd,point));
	}
	else
	{
		NN(point,n->right,(cd+1)%dim,BB->trimRight(cd,point));
		NN(point,n->left,(cd+1)%dim,BB->trimLeft(cd,point));
	}
	return;
}

void kdTree::deleteTree(Node* n)
{
	if(n == NULL) return;

	deleteTree(n->left);
	deleteTree(n->right);
	delete(n);
}

double kdTree::distance(vector<double>p1,vector<double>p2)
{
	double tmp;
	double wrapTmp;
	double shortest;

	for(int i=0;i<dim;i++)
	{
		tmp += (p1[i] - p2[i])*(p1[i] - p2[i]);
	}
	return sqrt(tmp);
}

double kdTree::distance2Box(vector<double>p1, vector<double>BB)
{
	vector <double> tmp;
	for(int i=0;i<dim;i++)
	{
		if(p1[i] < BB[2*i])
		{
			tmp.push_back(p1[i]-BB[2*i]);
		}
		else if(p1[i]>BB[2*i+1])
		{
			tmp.push_back(p1[i]-BB[2*i+1]);
		}
	}

	if(tmp.size() == 0)
	{
		return 0;
	}
	else 
	{
		double distance;
		double wrapTmp;

		for(int i=0;i<tmp.size();i++)
		{
			distance += (tmp[i]*tmp[i]);
		}
		return sqrt(distance);
	}
}

//bounduingBox class member fcns
boundingBox::boundingBox(vector<double>init):
bounds(init)
{

}

boundingBox::~boundingBox()
{

}

vector<double> boundingBox::trimLeft(int cd,vector<double>point)
{
	bounds[2*cd+1] = point[cd];
	return bounds;
}

vector<double> boundingBox::trimRight(int cd,vector<double>point)
{
	bounds[2*cd] = point[cd];
	return bounds;
}

//Node class member fcns
Node::Node()
{
	left = NULL;
	right = NULL;
}

Node::~Node()
{

}

bool kdTree::comparePt(vector<double>p1,vector<double>p2,int dim)
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