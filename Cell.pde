class Cell
{
  public Node[] corners;
  
  public float cost; // cost per unit length of path
  public int gray; 
  
  Cell()
  { 
    cost = 1.0;
    gray = 255;
  }
  
  Cell(Node[] corners)
  {
    this();
    this.corners = corners;
  }
  
  Cell(Node tl, Node tr, Node bl, Node br)
  {
    this(new Node[]{tl, tr, bl, br});
  } 
  
  public Node[] getCopyCorners()
  {
    return corners.clone();
  }
}