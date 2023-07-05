#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/collision.h>


int main(int argc, char const *argv[])
{
	// read an binary file to obtain an Octree
	const std::string filename = "../sample_octomap.bt";
	octomap::OcTree temp_tree(0.1);
	bool success = temp_tree.readBinary(filename);

	std::cout << "Read octree?: " << success << "\n";


	// // use octree to create a collision object map to be used with FCL
	std::shared_ptr<const octomap::OcTree> sharedTree = std::make_shared<const octomap::OcTree>(temp_tree);
	fcl::OcTree<float>* tree = new fcl::OcTree<float>(sharedTree);
	fcl::CollisionObject<float> treeCollisionObj((std::shared_ptr<fcl::CollisionGeometry<float>>(tree)));
	
	
	// // we model a sample robot with box and sphere collision  geometry
	std::shared_ptr<fcl::CollisionGeometry<float>> robot(new fcl::Box<float>(0.3, 0.3, 0.1));
	fcl::CollisionObject<float> robotCollisionObj(robot);

	// // perform collision checking between collision object tree and collision object robot
	fcl::Vector3f translation(1.0, 1.0, 1.0);
	fcl::Quaternionf rotation(0.988, 0.094, 0.079, 0.094);
	robotCollisionObj.setTransform(rotation, translation);
	fcl::CollisionRequest<float> requestType(1, false, 1, false);
	fcl::CollisionResult<float> collisionResult;
	fcl::collide(&robotCollisionObj, &treeCollisionObj, requestType, collisionResult);

	std::cout << "Is collision detected:\t" << collisionResult.isCollision() << "\n";


	return 0;
}