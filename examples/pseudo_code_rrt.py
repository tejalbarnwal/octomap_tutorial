class RRT():
  def __init__(self, xyzBounds, start_pos, goal_pos, nIter):
    self.lowerBoundx = 
    self.upperBoundx = 
    self.lowerBoundy = 
    self.upperBoundy = 
    self.lowerBoundz = 
    self.upperBoundz = 
    self.start_pose = 
    self.goal_pose = 
    
    
    self.tree = 
    
    # tunable parameters
    self.nIterations = 
    # if randomNode is collision free but far away, we would like to create a node in the same direction,
    # but at the distance of step size
    self.step_size = 
    # goal proximity distance, so that we can say we have reached the goal
    self.goal_threshold = 

  def generateRandomPosition(self, bounds):
    return x, y, z
  
  def createNode(self, x, y, z):
    return node
  
  def find_nearest_node(self, tree, randomNode):
    return nearestNode
  
  def hasObstaclecollision(self, randomNode, nearestNode):
    return bool
  
  def distance(self, node1, node2):
    return float
  
  def insert_new_node(tree, nearestNode, randomNode):
    return bool
  
  def create_node_at_step_size(randomNode, nearestNode):
    return node
  
  def algo():
    
    insertRootNode(tree, startPosition)
    for i=1 to nIter:
      randomPosition = generateRandomPosition(bounds)
      
      randomNode = createNode(randomPosition)
      
      nearestNode = find_nearest_node(tree, randomNode)
      
      if hasObstacleCollision():
        continue
      else:
        if distance(nearestNode, randomNode) > step_size:
          randomNode = createnewnodeatstepsize()
          
        insertNewNode(tree, nearestnode, randomNode)
        
        if (distance(randomNode, goalNode) < goalthreshold):
          return tree
    return tree
        
          
    
    