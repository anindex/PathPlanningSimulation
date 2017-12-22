class Agent
{
  private int sensorRange; // discrete range based on resolution of grid (range of 2 nodes)
  private int step;
  
  private boolean hasPath;
  
  public Node start;
  public Node goal;
  
  Grid knownMap;
  
  Agent(int screenWidth, int screenHeight, GridMode gridMode, int sensorRange, int step, PVector start, PVector goal)
  {
    
  }
  
  Agent(Grid environment, int sensorRange, int step, PVector start, PVector goal) // will deep copy the environment to Agent knownMap
  {
    
  }
  
  public Node[] scan(Grid environment, boolean update)
  {
  }
  
  public void move()
  {
  }
  
  public void planning()
  {
  }
}
  
  