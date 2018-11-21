/*

	KD-tree Class
	By Peter Jan
	Oct. 15 2018

*/

#ifndef _KD_TREE_
#define _KD_TREE_

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#define PI 3.141592654
#define INF 1000000000.0


class Node
{
	public:
		int component;
		bool alreadyFound = false;
		Node* parent = NULL;
		Node* left = NULL;
		Node* right = NULL;
		double* data;

	public:
		Node();
		~Node();
		double distance();

};

class boundingBox
{
	public:
		boundingBox(double* init);
		~boundingBox();

		//Bounding Box
		double* bounds;
		double* trimLeft(int cd,double* point);
		double* trimRight(int cd,double* point);
};

class kdTree
{
	public:
		kdTree(int dimension,boundingBox* BBox);
		~kdTree();
		int dim;
		double* bestPoint=NULL;
		double bestDist;
		Node* nearestNode = NULL;
		int size = 0;
		std::vector<double*> kNNvec;

		Node* root;
		Node* insert(double* point,Node* n,int cd);
		void NN(double* point, Node* n, int cd, double* BB);
		void kNN(double* point, Node* n, int cd, double* nodeBound,double r);
		double distance(double* p1,double* p2);
		double distance2Box(double* p1, double* BB);
		void deleteTree(Node* n);
		bool comparePt(double* p1,double* p2,int dim);

		//BoundingBox for NN search
		boundingBox* BB;

		int r=100; //search radius
};

#endif