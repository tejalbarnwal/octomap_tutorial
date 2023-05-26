#include <iostream.h>
#include <Eigen/Dense.h>


struct Node{
	// parent node
	Node* parent=NULL;
	// x, y, z position
	Eigen::Vector3d coordinates

	Node(Node* parent_, Eigen::Vector3d coordinates_)
	{
		parent = parent_;
		coordinates = coordinates_;
	}
};


class Tree{
	Node* root;
public:
	// default constructor
	Tree(Node* node);

	// insert a node such that node1 is parent of node2
	void insertNewNode(Node* node1, Node* node2);

	// delete the present node, make its parent NULL
	void deleteNode(Node* node); // is it needed?

	// distance between x1,y1,z1 and x2,y2,z2
	double distance(Node* node1, Node* node2);

	// find the nearest node in tree according to parameter distance
	Node nearestNode(Node* node);

	~Tree();	
};


class RRT{
	//// Given
	// lower and upper bounds of x, y, z
	double lBx, uBx, lBy, uBy, lBz, uBz;
	// start position
	Eigen::Vector3d start;
	// goal position
	Eigen::Vector3d goal;
	// Obstacle 
	const std::string ObstacleOctreeFilename;

	// tuning parameters
	unsigned int nIterations;
	double stepSize;
	double goalProximityThreshold;

	// tree created from RRT
	Tree tree;

public:
	// constructor
	RRT(std::pair<double, double> pairX,
		std::pair<double, double> pairY,
		std::pair<double, double> pairZ,
		Eigen::Vector3d start_,
		Eigen::Vector3d goal_,
		const std::string filename);
	// use bounds to create a random x, y, z
	Eigen::Vector3d generateRandomPosition();

	// check using FCL and octree to check 
	// collision continuously between randomNode and nearestNode 
	bool hasCollisionWithObstacle(Node nearestNode, Node randomNode);

	// create a node at stepSize distance
	Node createNodeAtStepSize(Node nearestNode, Node randomNode);

	// set tuning parameters
	setTuningParameter(unsigned int nIterations, double stepSize, 
						double goalProximityThreshold);

	// RRT algorithm
	bool algorithm();

	~RRT();
}


