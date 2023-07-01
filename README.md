# octomap_tutorial

#### What is an Octree?
- An octree is a tree data structure in which each internal node can be sub-divided into exactly eight children. It is commonly used in computer graphics and mapping to efficiently represent 3D space.  
- The octree recursively subdivides the space into smaller eight cubic regions. Each of these cubic regions are refered to octant. Each octant can either be empty or contain object or could be further subdivided into eight smaller octants. The tree strucutre is formed by linking these octants tigether.

* How does Octree achieve space efficiency?
	- The octree exploits the fact that objects in 3D space tend to be clustered together. Therefore, by recusively subdiving the space, the octree can allocate more memory to regions containing high density of objects, while leaving empty regions sparsely represented or unrepresented entirely. 
	- As a result, the octree adapts to the distribution of objects in space. Regions with a high density of objects are represented by a higher number of tree nodes, while regions with no objects or low density are represented by fewer nodes or no nodes at all. This adaptive nature allows the octree to efficiently represent both dense and sparse regions, optimizing memory usage.

* How does this help to efficiently quering the nodes?
	- When you query nodes in a octree, you want to access, or perform operations on specific nodes in an octree
	- This could involve:
		- checking containment(determining whether a particular point or region lies within a specific node or set of nodes)
		- finding nearest neighbors(searching for the closest neighboring nodes to a given point or object)
		- retrieving information(attributes like occupancy probabilt, 3D coordinates of the associated node)
	- 


#### What is an octomap?
It is an extension of the octree data strucutre, widely used for repsentating and quering 3D occupancy maps. It is commonly employed in robotics application especially for 3D planning. In OctoMap, each node of the octree represents a cubic region of space, similar to the basic octree structure. However, in addition to the occupancy state (either occupied or free), OctoMap also stores probabilistic information about the occupancy of each node. This probabilistic representation allows for more nuanced and accurate modeling of uncertain or unknown regions.

Now, you maybe wondering why use octree to represent a map as quering data in 3D array would much simpler and direct, but octrees offer advantages in handling sparse data, representing complex geometry, and adapting to varying object density. The choice between a 3D array and an octree depends on the specific requirements of your application and the characteristics of the data and environment you are working with.


Some key points:
- Octomap model unknown, known unoccupied and known occupied spaces.
- The minimum voxel size determines the resolution of the octree
- Sensor model Probabilties
- Clamping


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

# OR link each executable as below
add_executable (name file.cpp)
target_link_libraries(name octomap)
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


#### Running OctoMap with ROS2
* In order to run octomap server to convert /PointCloud2 msgs into octomap
```bash
ros2 launch octomap_server octomap_mapping.launch.xml
```
Note: This requires `octomap-ros` and `octomap-msgs` as dependencies

* In order to save octomap
```bash
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=(path for saving octomap)
```
Note: The extension of octomap path should be `.bt` or `.ot`

#### Flexible Collision Checking(FCL)
Incase you also want to use FCL with Octrees, please follow the below steps:
```bash
git clone https://github.com/danfis/libccd
cd libccd
mkdir build && cd build
cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
make && sudo make install

git clone https://github.com/flexible-collision-library/fcl
cd fcl
mkdir build
cd build
cmake ..
sudo make install
```

Reference:
1. [ROSCon2013 Video](https://vimeo.com/66577259) giving a bried overview about the working and available classes and methods.
2. https://github.com/ycaibb/octomap_rrt
3. https://github.com/ayushgaud/path_planning