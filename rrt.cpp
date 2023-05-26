// include the header file
#include "rrt.h"

// octomap related header files
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// fcl related header files
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/collision.h>

// header file for sqrt and pow
#include <bits/stdc++.h>


Tree::Tree(Node* node)
{
	root = node;
}

void Tree::insertNewNode(Node* node1, Node* node2)
{
	node2->parent = node1;
}

double Tree::distance(Node* node1, Node* node2)
{
	// make it eigen3 based
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}


RRT::RRT(std::pair<double, double> pairX,
		std::pair<double, double> pairY,
		std::pair<double, double> pairZ,
		Eigen::Vector3d start,
		Eigen::Vector3d goal,
		const std::string filename)
{
	this->lBx = pairX.first;
	this->uBx = pairX.second;
	this->lBy = pairY.first;
	this->uBy = pairY.second;
	this->lBz = pairX.first;
	this->uBz = pairX.second;

	this->start = start;
	this->goal = goal;

	this->ObstacleOctreeFilename = filename;

	// initialize tree with start node


}


