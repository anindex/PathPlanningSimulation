public class Key
  {
    public float k1;
    public float k2;
    
    Key(){}
    Key(float k1, float k2)
    {
      this.k1 = k1;
      this.k2 = k2;
    }
  }


public class Node
{ 
  public class AngleRange
  {
    public float min;
    public float max;
    
    AngleRange()
    {
      min = -(PI * 100) / 100;
      max = (PI * 100) / 100;
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
  public float rhs;
  public float f;
  public float minG;
  
  public AngleRange range;
  
  Node()
  {
    coordinate = new PVector();
    nodeColor = 0x000000;
    
    parent = null;
    localParent = null;
    
    g = Float.POSITIVE_INFINITY;
    rhs = Float.POSITIVE_INFINITY;
    f = Float.POSITIVE_INFINITY;
    minG = Float.POSITIVE_INFINITY;
    
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
  
  void updateKey(Key s)
  {
    f = s.k1;
    minG = s.k2;
  }
  
  Key returnKey()
  {
    return new Key(f, minG);
  }
  
  float dist(Node target) // round to one decimal place for the ease of computation, will output distance based on pixel
  {
    return target.coordinate != null ? (round(coordinate.dist(target.coordinate) * 10) / 10) : -1.0;
  }
  
}