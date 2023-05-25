# octomap_tutorial

#### Map Accesssing
* Traverse nodes with iterators
```cpp
for(OcTree::leaf_iterator it=octree.begin_leafs(),
	end=octree.end_leafs(); it!=end; ++it)
{
	// access node, eg:
	std::cout << "Node center: " << it.getCoordinate();
	std::cout << "value: " << it->getValue() << "\n";
}
```
* Ray intersection queries
```cpp
octree.castRay(...)
```
* Access single nodes by searching
```cpp
OcTreeNode* n=octree.search(x, y, z);
if(n)
{
	std::cout << "value: " << n->getValue() << "\n";
}
```

#### Occupancy and Sensor Model
* Set occupancy parameters in octree
```cpp 
octree.setOccupancyThres(0.5);
octree.setProbHit(0.7);
octree.setProbMiss(0.3);
octree.setClampingThresMin(0.1);
octree.setClampingThresMax(0.95);
```
* Check if a node is free or occupied
```cpp
octree.isNodeOccupied(n);
```
* Check if a node is "clamped"
```cpp
octree.isNodeAtThreshold(n);
```

#### Reading Map Files(Deserialization)
* Read from .ot file(any kind of octree):
```cpp
AbstractOcTree* tree = AbstractOcTree::read(filename);
if(tree) // read error returs NULL
{ 
	OcTree* ot = dynamic_cast<OcTree*>(tree);
	if (ot) // cast succeeds if correct type
	{
		// do something
	}
}
```
* Read from .bt file(OcTree):
```cpp
OcTree* octree = new OcTree(filename);
```

#### (De-)Serialization in ROS
* `octomap_msgs/Octomap.msg` contains binary stream and header information
* Use `octomap_msgs/conversions.h` to convert between octrees and messages
* Serialize:
```cpp
octomap_msgs::Octomap map_msg, bmap_msg;
octomap_msgs::fullMapToMsg(octree, map_msg); //(.ot)
octomap_msgs::binaryMapToMsg(octree, bmap_msg) //(.bt)
```
* Deserialize
```cpp
AbstractOcTree* tree = octomap_msgs::msgToMap(map_msg);
OcTree octree* = dynamic_cast<OcTree*>(tree);
if (octree) // can be NULL
{
	...;
}
```

#### Map Visualization
* Native OctoMap Visualization: octovis
* RViz:
	* MarkerArray display from octomap_server
	* octomap_rviz_displays
	* MoveIt planning scene

#### 3D Mapping in ROS(outline)
* Build maps incrementally from point clouds with `octomap_server`
* Remap topic `cloud_in` to your sensor's PointCloud2
* Requires tf from map frame to sensor frame
* Example launch file in octomap_server

#### Using Octomap in your project
* Standard CMake(stand-alone or in ROS)
* In CMakeLists.txt:
```makefile
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})
```
* For ROS1:
	* `manifest.xml`(rosbuild): `<rosdep name="octomap"/>`
	* `package.xml`(catkin):
	```
	<build_depend>octomap</build_depend>
	<run_depend>octomap<run_depend>
	```

* Additional ROS packages for integration
	* `octomap_msgs`: ROS messages & serialization
	* `octomap_ros`: conversions from native ROS types



Reference:
1. [ROSCon2013 Video](https://vimeo.com/66577259) giving a bried overview about the working and available classes and methods.

2. https://github.com/ycaibb/octomap_rrt

3. https://github.com/ayushgaud/path_planning
