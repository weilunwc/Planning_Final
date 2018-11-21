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

Node* kdTree::insert(double* point,Node* n,int cd)
{	
	if(n == NULL)
	{
		n = new Node();
		double* nData = new double[dim];

		for(int i=0;i<dim;i++)
		{
			nData[i] = point[i];
		}

		n->data = nData;
		n->parent = nearestNode;
		size++;
	}
	else if(point == n->data)
	{
		cout << "DUPLICATE ENTRY" << endl;	
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
	return n;
}

//Need to fix
void  kdTree::kNN(double* point, Node* n, int cd, double* nodeBound,double r)
{
	double dist;
	bool alreadyFound = false;

	if(n == NULL || distance2Box(point,nodeBound) > bestDist) 
		{
			return;
		}	

	dist = INF;
	dist = distance(point,n->data);

	// cout << "dist: " << dist << endl;
	// cout << "bestDist: " << bestDist << endl;
	// cout << "alreadyFound: " << alreadyFound << endl;
	// cout << "r: " << r << endl;

	if(dist < bestDist && n->alreadyFound == false && dist < r)
	{
		// cout <<" dist: " << dist << endl;
		nearestNode = n;
		bestPoint = n->data;
		bestDist = dist;
		n->alreadyFound = true;
		kNNvec.push_back(bestPoint);
	}

	// cout << "bestDist: " << bestDist << endl;
	if(point[cd]<=n->data[cd])
	{
		kNN(point,n->left,(cd+1)%dim,BB->trimLeft(cd,point),r);
		kNN(point,n->right,(cd=1)%dim,BB->trimRight(cd,point),r);
	}
	else
	{
		kNN(point,n->right,(cd=1)%dim,BB->trimRight(cd,point),r);
		kNN(point,n->left,(cd+1)%dim,BB->trimLeft(cd,point),r);
	}
}


void  kdTree::NN(double* point, Node* n, int cd, double* nodeBound)
{
	double dist;

	if(n == NULL || distance2Box(point,nodeBound) > bestDist) 
		{
			return;
		}	

	dist = INF;
	dist = distance(point,n->data);

	if(dist < bestDist && n->alreadyFound == false && dist < r)
	{
		nearestNode = n;
		bestPoint = n->data;
		bestDist = dist;
	}

	if(point[cd]<=n->data[cd])
	{
		NN(point,n->left,(cd+1)%dim,BB->trimLeft(cd,point));
		NN(point,n->right,(cd=1)%dim,BB->trimRight(cd,point));
	}
	else
	{
		NN(point,n->right,(cd=1)%dim,BB->trimRight(cd,point));
		NN(point,n->left,(cd+1)%dim,BB->trimLeft(cd,point));
	}
}

void kdTree::deleteTree(Node* n)
{
	if(n == NULL) return;

	deleteTree(n->left);
	deleteTree(n->right);
	delete(n->data);
	delete(n);
}

double kdTree::distance(double* p1,double*p2)
{
	double tmp;
	double wrapTmp;
	double shortest;

	for(int i=0;i<dim;i++)
	{
		tmp += (p1[i] - p2[i])*(p1[i] - p2[i]);
		wrapTmp += (p1[i] + 2*PI-p2[i])*(p1[i] + 2*PI-p2[i]);
	}
	cout << "" ;
	// usleep(1);
	shortest = MIN(tmp,wrapTmp);
	// usleep(1);
	return sqrt(shortest);
}

double kdTree::distance2Box(double* p1, double* BB)
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
boundingBox::boundingBox(double* init):
bounds(init)
{

}

boundingBox::~boundingBox()
{

}

double* boundingBox::trimLeft(int cd,double* point)
{
	bounds[2*cd+1] = point[cd];
	return bounds;
}

double* boundingBox::trimRight(int cd,double* point)
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

bool kdTree::comparePt(double* p1,double* p2,int dim)
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
	cout << "P1: ";
	for(int i=0;i<dim;i++)
	{
		cout << p1[i] << " ";
	}
	cout << endl;

	cout << "P2: ";
	for(int i=0;i<dim;i++)
	{
		cout << p2[i] << " ";
	}
	cout << endl;

	if(count == dim)
	{
		return true;
	}
	else
	{
		return false;
	}
}