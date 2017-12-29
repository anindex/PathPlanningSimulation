import java.util.Comparator;
import java.util.Vector;
import java.util.PriorityQueue;
import java.util.LinkedList;

class NodeComparator implements Comparator<Node>
{
  public int compare(Node source, Node target)
  { 
    if(source.f < target.f) return -1;
    else if(source.f > target.f) return 1;
    else return 0;
  }
}

class LexicalNodeComparator implements Comparator<Node> // fail to compare Lexical order
{
  public int compare(Node source, Node target)
  { 
    if(source.f < target.f) return -1;
    else if(source.f > target.f) return 1;
    else
    {
      if(source.minG < target.minG) return -1;
      else if(source.minG > target.minG) return 1;
      else return 0;
    }
  }
}


interface PathPlanner
{
  public static final int DELAY = 1000;
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
      t.f = t.g + robot.knownMap.dist(t, robot.start);
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
      t.f = t.g + robot.knownMap.dist(t, robot.start);
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



class Phi extends Thread implements PathPlanner 
{ 
  public PriorityQueue<Node> open; // public for display purpose
  public Vector<Node> closed;
    
  public PVector[] path;
  
  Grid environment;
  Agent robot;
  
  public Phi(Grid environment, int sensorRange, PVector start, PVector goal)
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
      t.f = t.g + robot.knownMap.dist(t, robot.start);
      open.add(t);
    }
  }
  
  private void ComputeCost(Node s, Node t)
  {
    float a = robot.knownMap.angle(s, s.parent, t);
    if(int(round(degrees(a))) % 45 != 0 && a >= s.range.min && a <= s.range.max && robot.knownMap.lineOfSight(s.parent, t))
    {
      
      float newCost = s.parent.g + robot.knownMap.dist(s.parent, t);
      if(newCost < t.g)
      {
        t.parent = s.parent;
        t.localParent = s;
        t.g = newCost;
          
        float l = Float.POSITIVE_INFINITY, h = -1;
        for(Node next : robot.knownMap.neighborNodes(t, 0, false))
        {
          float current = robot.knownMap.angle(t, s.parent, next);
          if (current > h) h = current;
          if (current < l) l = current;
        }
          
        t.range.min = max(l, s.range.min - a);
        t.range.max = min(h, s.range.max - a);
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
        t.range.min = -(PI * 25) / 100; // PI / 4 rounded to 2 decimal places
        t.range.max = (PI * 25) / 100;
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
      if (neighbors.length == 0) //<>//
      {
        println("Compute Path failed because all paths are blocked!");
        return;
      }
      
      for(Node next : neighbors)
      {
        if(!closed.contains(next))
        {
          UpdateVertex(current, next); //<>//
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

class DStarLite extends Thread implements PathPlanner 
{ 
  private float km;
  
  public PriorityQueue<Node> open; // public for display purpose
    
  public LinkedList<PVector> path;
  
  public ArrayList<PVector> processedNode; // for debugging and simulating purpose
  
  Grid environment;
  Agent robot;
  
  public DStarLite(Grid environment, int sensorRange, PVector start, PVector goal)
  {
    this.environment = environment;
    
    robot =  new Agent(environment, sensorRange, start, goal);
    
    Comparator<Node> nodeComparer = new LexicalNodeComparator();
    open = new PriorityQueue<Node>(10, nodeComparer);
    
    processedNode = new ArrayList<PVector>();
    
    km = 0.0;
  }
  
  private Key CalculateKey(Node s) // if update is true, update variable f of that Node
  {
    return new Key(min(s.g, s.rhs) + robot.knownMap.dist(s, robot.start) + km, min(s.g, s.rhs));
  }
  
  private boolean CompareKey(Key s, Key t) // Key(s) strictly less than Key(t) return true
  {
    if(s.k1 < t.k1) return true;
    else if (s.k1 == t.k1 && s.k2 < t.k2) return true;
    else return false;
  }
  
  private void UpdateVertex(Node s)
  {
    if(s != robot.goal) 
    {
      float m = Float.POSITIVE_INFINITY;
      for(Node next : robot.knownMap.neighborNodes(s, 1, false))
      {
        float c = robot.knownMap.cost(s, next) + next.g;
        if(m > c) m = c;
      }
      
      s.rhs = m;
    }
    
    if(open.contains(s)) open.remove(s);
    
    if(s.g != s.rhs)
    {
      s.updateKey(CalculateKey(s));
      open.add(s);
    }
  }
  
  private void ComputeShortestPath()
  {
    Node current;
    processedNode.clear();

    while(open.size() != 0 && (CompareKey(open.peek().returnKey(), CalculateKey(robot.start)) || robot.start.g != robot.start.rhs))
    {
      current = open.poll();
      processedNode.add(current.coordinate); 
      
      Key oldKey = current.returnKey();
      Key newKey = CalculateKey(current);
      if(CompareKey(oldKey, newKey))
      {
        current.updateKey(newKey);
        open.add(current);
      }
      else if (current.g > current.rhs)
      {
        current.g = current.rhs;
        for(Node next : robot.knownMap.neighborNodes(current, 1, false))
        {
          UpdateVertex(next);
        }
      }
      else
      {
        current.g = Float.POSITIVE_INFINITY;
        
        for(Node next : robot.knownMap.neighborNodes(current, 1, false))
        {
          UpdateVertex(next);
        }
        UpdateVertex(current);
      }
    }
  }
  
  public void run()
  {
    robot.goal.rhs = 0.0;
    robot.goal.updateKey(CalculateKey(robot.goal));
    open.add(robot.goal);
    
    Node last = robot.start;

    ComputeShortestPath();
    if(Float.isInfinite(robot.start.g)) 
    {
      println("[D*Lite]: Could not plan path!, All path are blocked!");
      return;
    }
    path = robot.knownMap.extractLinkedPath(robot.start.coordinate, robot.goal.coordinate);
    
    while(robot.start != robot.goal)
    {
      if(path == null)
      {
        println("[D*Lite] Replanning failed! Exiting!");
        return;
      }
      
      path.pollFirst();
      PVector nextPos = path.peekFirst();
      robot.start = robot.knownMap.nodes[int(nextPos.x / robot.knownMap.res)][int(nextPos.y / robot.knownMap.res)];
      
      
      Cell[] changes = robot.scan(environment, true);
      if(changes.length != 0)
      {
        km += robot.knownMap.dist(last, robot.start);
        last = robot.start;
        
        for(Cell cell : changes)
        {
          for(Node node : cell.corners)
          {
            UpdateVertex(node);
          }
        }
        
        ComputeShortestPath(); //<>//
        path = robot.knownMap.extractLinkedPath(robot.start.coordinate, robot.goal.coordinate); //<>//
      }
           
      try
      {
        Thread.sleep(DELAY);
      }
      catch(Exception e) {}
      
    }
  }
}