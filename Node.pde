class Node
{ 
  public class AngleRange
  {
    public float min;
    public float max;
    
    AngleRange()
    {
      min = -PI;
      max = PI;
    }
    
    AngleRange(float min, float max)
    {
      this.min = min;
      this.max = max;
    
    }
  }
  
  public PVector coordinate;
  public color nodeColor;
  
  public Node parent;
  public Node localParent;
  
  public float g;
  public float h;
  
  public AngleRange range;
  
  Node()
  {
    coordinate = new PVector();
    nodeColor = 0x000000;
    
    parent = null;
    localParent = null;
    
    g = Float.POSITIVE_INFINITY;
    h = Float.POSITIVE_INFINITY;
    
    range = new AngleRange();
  }
  
  Node(PVector coordinate) // default color will be black
  {
    this();
    this.coordinate = coordinate;
  }
  
  Node(PVector coordinate, color nodeColor)
  {
    this(coordinate);
    this.nodeColor = nodeColor;
  }
  
  float dist(Node target)
  {
    return target.coordinate != null ? coordinate.dist(target.coordinate) : -1.0;
  }
  
}