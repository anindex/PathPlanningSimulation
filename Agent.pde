class Agent
{
  private int sensorRange; // discrete range based on resolution of grid (range of 2 nodes)
  
  public Node start;
  public Node goal;
  
  Grid knownMap;
  
  Agent(int screenWidth, int screenHeight, int res, GridMode gridMode, int sensorRange, PVector start, PVector goal)
  {
    try
    {
      knownMap = new Grid(screenWidth, screenHeight, res, gridMode);
    }
    catch(InvalidResolution e)
    {
      println("Fail to initialize agent!");
      return;
    }
    
    this.start = knownMap.nodes[int(start.x / knownMap.res)][int(start.y / knownMap.res)];
    this.goal = knownMap.nodes[int(goal.x / knownMap.res)][int(goal.y / knownMap.res)];
    
    this.sensorRange = sensorRange;
  }
  
  Agent(Grid environment, int sensorRange, PVector start, PVector goal) // will deep copy the environment to Agent knownMap
  {
    knownMap = environment.deepCopy();
    
    this.start = knownMap.nodes[int(start.x / knownMap.res)][int(start.y / knownMap.res)];
    this.goal = knownMap.nodes[int(goal.x / knownMap.res)][int(goal.y / knownMap.res)];
    
    this.sensorRange = sensorRange;
  }
  
  public Cell[] scan(Grid environment, boolean update) // need check
  {
    ArrayList<Cell> changes = new ArrayList<Cell>();
    Cell[] knownNeighbors = knownMap.neighborCellsOfNode(start, sensorRange);
    Cell[] environmentNeighbors = environment.neighborCellsOfNode(start, sensorRange); // possible because neighborNodes function extract the position Id of Node start, which is the same with environment
    
    if (knownNeighbors.length != environmentNeighbors.length) return null; // expect two array must be equal
    
    for(int i = 0; i < knownNeighbors.length; i++)
    {
      if(knownNeighbors[i].cost != environmentNeighbors[i].cost)
      {
        if(update)
        {
          knownNeighbors[i].cost = environmentNeighbors[i].cost;
          knownNeighbors[i].gray = environmentNeighbors[i].gray;
        }
        
        changes.add(knownNeighbors[i]);
      }
    }
    
    Cell[] result = new Cell[changes.size()];
    changes.toArray(result);
    return result;
  }
}
  
  