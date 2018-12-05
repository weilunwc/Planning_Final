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

#if !defined(MAX)
#define MAX(A,B) ((A)>(B)?(A):(B))
#endif

#if !defined(MIN)
#define MIN(A,B) ((A)>(B)?(A):(B))
#endif



class Node
{
	public:
		int component;
		bool alreadyFound = false;
		Node* parent = NULL;
		Node* left = NULL;
		Node* right = NULL;
		std::vector<double> data;

	public:
		Node();
		~Node();
		double distance();

};

class boundingBox
{
	public:
		boundingBox(std::vector<double> init);
		~boundingBox();

		//Bounding Box
		std::vector<double> bounds;
		std::vector<double> trimLeft(int cd,std::vector<double> point);
		std::vector<double> trimRight(int cd,std::vector<double> point);
};

class kdTree
{
	public:
		kdTree(int dimension,boundingBox* BBox);
		~kdTree();
		int dim;
		std::vector<double> bestPoint;
		double bestDist;
		Node* nearestNode = NULL;
		int size = 0;
		std::vector<double*> kNNvec;

		Node* root = NULL;
		Node* insert(std::vector<double> point,Node* n,int cd);
		void NN(std::vector<double> point, Node* n, int cd, std::vector<double> BB);
		void kNN(std::vector<double> point, Node* n, int cd, std::vector<double> nodeBound,double r,int k);
		double distance(std::vector<double> p1,std::vector<double> p2);
		double distance2Box(std::vector<double> p1, std::vector<double> BB);
		void deleteTree(Node* n);
		bool comparePt(std::vector<double> p1,std::vector<double> p2,int dim);

		//BoundingBox for NN search
		boundingBox* BB;

		int r=100; //search radius
};

#endif