import java.util.Comparator;
import java.util.Vector;
import java.util.PriorityQueue;

class NodeComparator implements Comparator<Node>
{
  public int compare(Node source, Node target)
  { 
    if(source.f < target.f) return -1;
    else if(source.f > target.f) return 1;
    else return 0;
  }
}

interface PathPlanner
{
  public static final int DELAY = 10;
  public void run();
  public void start();
}

class BasicTheta extends Thread implements PathPlanner 
{ 
  public PriorityQueue<Node> open; // public for display purpose
  public Vector<Node> closed;
    
  public PVector[] path;
  
  Grid environment;
  Agent robot;
  
  public BasicTheta(Grid environment, int sensorRange, PVector start, PVector goal)
  {
    this.environment = environment;
    
    robot =  new Agent(environment, sensorRange, start, goal);
    
    Comparator<Node> nodeComparer = new NodeComparator();
    open = new PriorityQueue<Node>(10, nodeComparer);
    closed = new Vector<Node>();
  }
  
  private void UpdateVertex(Node s, Node t)
  {
    float gold = t.g;
    ComputeCost(s, t);
    if(t.g < gold)
    {
      if(open.contains(t)) open.remove(t);
      t.f = t.g + t.dist(robot.start);
      open.add(t);
    }
  }
  
  private void ComputeCost(Node s, Node t)
  {
    if(robot.knownMap.lineOfSight(s.parent, t))
    {
      float newCost = s.parent.g + robot.knownMap.dist(s.parent, t);
      if(newCost < t.g)
      {
        t.parent = s.parent;
        t.localParent = s;
        t.g = newCost;
      }
    }
    else
    {
      float newCost = s.g + robot.knownMap.dist(s, t);
      if(newCost < t.g)
      {
        t.parent = s;
        t.localParent = s;
        t.g = newCost;
      }
    }
  }
  
  private void ComputeShortestPath()
  {
    Node current;
    while(open.size() != 0 && open.peek().f < robot.start.g)
    {
      current = open.poll();
      closed.add(current); //<>//
      
      Node[] neighbors = robot.knownMap.neighborNodes(current, 1, true);
      if (neighbors.length == 0)
      {
        println("Compute Path failed because all paths are blocked!");
        return;
      }
      
      for(Node next : neighbors)
      {
        if(!closed.contains(next))
        {
          UpdateVertex(current, next);
        }
      }
    }
  }
  
  public void run()
  {
    robot.goal.g = 0.0;
    robot.goal.parent = robot.goal;
    robot.goal.localParent = robot.goal;
    robot.goal.f = robot.goal.g + robot.knownMap.dist(robot.goal, robot.start);
    open.add(robot.goal);

    ComputeShortestPath();
    path = robot.knownMap.extractPath(robot.start.coordinate, robot.goal.coordinate);
  }
}

class AStar extends Thread implements PathPlanner 
{ 
  public PriorityQueue<Node> open; // public for display purpose
  public Vector<Node> closed;
    
  public PVector[] path;
  
  Grid environment;
  Agent robot;
  
  public AStar(Grid environment, int sensorRange, PVector start, PVector goal)
  {
    this.environment = environment;
    
    robot =  new Agent(environment, sensorRange, start, goal);
    
    Comparator<Node> nodeComparer = new NodeComparator();
    open = new PriorityQueue<Node>(10, nodeComparer);
    closed = new Vector<Node>();
  }
  
  private void UpdateVertex(Node s, Node t)
  {
    float gold = t.g;
    ComputeCost(s, t);
    if(t.g < gold)
    {
      if(open.contains(t)) open.remove(t);
      t.f = t.g + t.dist(robot.start);
      open.add(t);
    }
  }
  
  private void ComputeCost(Node s, Node t)
  {
    float newCost = s.g + robot.knownMap.cost(s, t);
    if(newCost < t.g)
    {
      t.parent = s;
      t.localParent = s;
      t.g = newCost;
    }
  }
  
  private void ComputeShortestPath()
  {
    Node current;
    while(open.size() != 0 && open.peek().f < robot.start.g)
    {
      current = open.poll();
      closed.add(current);
      
      Node[] neighbors = robot.knownMap.neighborNodes(current, 1, true);
      if (neighbors.length == 0)
      {
        println("Compute Path failed because all paths are blocked!");
        return;
      }
      
      for(Node next : neighbors)
      {
        if(!closed.contains(next))
        {
          UpdateVertex(current, next);
        }
      }
    }
  }
  
  public void run()
  {
    robot.goal.g = 0.0;
    robot.goal.parent = robot.goal;
    robot.goal.localParent = robot.goal;
    robot.goal.f = robot.goal.g + robot.knownMap.dist(robot.goal, robot.start);
    open.add(robot.goal);

    ComputeShortestPath();
    path = robot.knownMap.extractPath(robot.start.coordinate, robot.goal.coordinate);
  }
}