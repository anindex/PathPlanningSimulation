import java.util.Random;
import java.util.LinkedList;

enum GridMode {BINARY, RANDOMCOST} // BINARY -> infinite cost untraversable cell, RANDOMCOST -> all cell are traversable but with specific cost
enum ArrowMode {LOCAL, REMOTE}
enum ObjectColor 
{
  START(0xFF0000FF), 
  GOAL(0xFF00FF00), 
  PROCESS(0xFFFF0000),
  ARROW(0xFFFFA500), 
  PATH(0xFFFF0000),
  THETAPATH(0xFFFA0A00),
  ASTARPATH(0xFFFA000A),
  PHIPATH(0xFFFA0A0A);
  
  private int value;
  private ObjectColor(int value)
  {
    this.value = value;
  }
  
  public int getColor()
  {
    return value;
  }
}

class InvalidResolution extends Exception
{
  public InvalidResolution()
  {
    println("Cannot create map due to invalid resolution. (Screen Width - 1) % resolution and (Screen Height - 1) % resolution must be zero");
  }
}   

class Grid
{
 GridMode gridMode;
 
 public static final int NODESIZE = 2;
 public static final float SIGHT_THRESHOLD = 50.0;
 public static final float COST_STEP = 1.0;
 public static final float MAX_COST = 50.0;
 
 public Node[][] nodes; // Node[col][row] means [x][y]
 public Cell[][] cells;
 
 private int res;
 
 private int cellRows;
 private int cellCols;
 
 private int lastGray; // a hack for faster drawing in Processing
 
 Grid(int screenWidth, int screenHeight, int res, GridMode gridMode) throws InvalidResolution
 {
   this.res = res;
   this.gridMode = gridMode; // could be binary cost or random cost
   
   lastGray = 255;
   
   if((screenWidth - 1) % res == 0 && (screenHeight - 1) % res == 0) // must be replaced by throwing an exception
   {
     cellCols = screenWidth / res;
     cellRows = screenHeight / res;
     
     // populate Nodes
     nodes = new Node[cellCols + 1][];
     for(int col = 0; col < (cellCols+1); col++)
     {
       nodes[col] = new Node[cellRows + 1];
       
       for(int row = 0; row < (cellRows + 1); row++)
       {
         nodes[col][row] = new Node(new PVector(col * res, row * res)); 
       }
     }
     
     // populate Cells
     cells = new Cell[cellCols][];
     for(int col = 0; col < cellCols; col++)
     {
       cells[col] = new Cell[cellRows];
       
       for(int row = 0; row < cellRows; row++)
       {
         cells[col][row] = new Cell(nodes[col][row], nodes[col + 1][row], nodes[col][row + 1], nodes[col + 1][row + 1]); //maybe unecessary
       }
     }
   }
   else
   {
     throw new InvalidResolution();
   }
 }
 
 private void drawCells()
 {
   stroke(0);
   fill(255); //set default color white
   for(Cell[] cellCol : cells)
   {
     for(Cell cell : cellCol)
     {
       if (lastGray != cell.gray)
       {
         fill(cell.gray);  
         lastGray = cell.gray;
       }
       rect(int(cell.corners[0].coordinate.x), int(cell.corners[0].coordinate.y), res, res);
     }   
   }
}
   
 
 private void drawNodes()
 {
   fill(0); //set default color black
   for(Node[] nodeCol : nodes)
   {
     for(Node node : nodeCol)
     {
       ellipse(int(node.coordinate.x), int(node.coordinate.y), NODESIZE, NODESIZE);
     }
   }
 }
 
 public void drawNodeColor(PVector pos, ObjectColor objColor)
 {
   fill(objColor.getColor());
   ellipse(int(pos.x), int(pos.y), NODESIZE * 3, NODESIZE * 3);
 }
 
 public PVector[] extractPath(PVector start, PVector goal) // all search algorithm from goal
 {  
   ArrayList<PVector> pathNodes = new ArrayList<PVector>();
   Node current = this.nodes[int(start.x / res)][int(start.y / res)];
   Node goalNode = this.nodes[int(goal.x / res)][int(goal.y / res)];
   while(current != goalNode)
   {
     pathNodes.add(current.coordinate);
     Node next = current.parent;
     if(next == null)
     {
       println("Could not extract path. Path is broken! Exiting");
       return null;
     }  
     
     if(pathNodes.contains(next.coordinate))
     {
       println("Path loop detected. Path is corrupted! Exiting");
       return null;
     }
     current = next;
   }
   
   pathNodes.add(goalNode.coordinate);
   
   PVector[] result = new PVector[pathNodes.size()];
   pathNodes.toArray(result);
   return result;
 }
 
 public LinkedList<PVector> extractLinkedPath(PVector start, PVector goal) // all search algorithm from goal
 {  
   LinkedList<PVector> pathNodes = new LinkedList<PVector>();
   Node current = this.nodes[int(start.x / res)][int(start.y / res)];
   Node goalNode = this.nodes[int(goal.x / res)][int(goal.y / res)];
   while(current != goalNode)
   {
     pathNodes.add(current.coordinate);
     
     Node next = followDownCostField(current);
     if(next == null)
     {
       println("Could not extract path. Path is broken! Exiting");
       return null;
     }  
     
     if(pathNodes.contains(next.coordinate))
     {
       println("Path loop detected at" + next.coordinate + ". Path is corrupted! Exiting");
       return null;
     }
     
     current = next;
   }
   
   pathNodes.add(goalNode.coordinate);
   return pathNodes;
 }
 
 public void drawProcessedNodes(ArrayList<PVector> processedNode)
 {
   Iterator<PVector> it = processedNode.iterator();
   while(it.hasNext())
   {
      drawNodeColor(it.next(), ObjectColor.PROCESS);
   }
 }
 
 public void drawPath(PVector[] path, color pathColor)
 {
   if (path == null) return;
   
   stroke(pathColor);
   for(int i = 0; i < path.length - 1; i++)
   {
     line(int(path[i].x), int(path[i].y), int(path[i + 1].x), int(path[i + 1].y));
   }
 }
 
 public void drawPath(LinkedList<PVector> path, color pathColor)
 {
   if (path == null) return;
   
   stroke(pathColor);
   Iterator<PVector> it = path.iterator();
   PVector last = it.next();
   while(it.hasNext())
   {
     PVector next = it.next();
     line(int(last.x), int(last.y), int(next.x), int(next.y));
     last = next;
   }
 }
  
 public void drawArrows(ArrowMode mode) //  0 <= ratio < 1 
 {
   stroke(ObjectColor.ARROW.getColor());
   for(Node[] nodeCol : nodes)
   {
     for(Node node : nodeCol)
     {
       Node p = new Node();
       if(mode == ArrowMode.LOCAL)
       {
         p = node.localParent;
       }
       else if(mode == ArrowMode.REMOTE)
       {
         p = node.parent;
       }
       
       if(p == null) continue;
       
       PVector dist = PVector.sub(p.coordinate, node.coordinate);
       float angle = dist.heading();
       int x = int(node.coordinate.x) + int(2 * res * cos(angle) / 3);
       int y = int(node.coordinate.y) + int(2 * res * sin(angle) / 3);
       line(int(node.coordinate.x), int(node.coordinate.y), x, y);
     }
   }
     
 }
 
 public void drawMap()
 {
   drawCells();
   drawNodes();
 }
 
 public Node[] neighborNodes(Node node, int mode, boolean visible) // mode = 0 -> NEIGHBOR 4, mode = 1 -> NEIGHBOR 8
 {
   int colId = int(node.coordinate.x / res);
   int rowId = int(node.coordinate.y / res);
   return neighborNodes(colId, rowId, mode, visible);
 }
 
 public Node[] neighborNodes(int colId, int rowId, int mode, boolean visible)
 {
   ArrayList<Node> neighbors = new ArrayList<Node>();
   for(int i = -1; i < 2; i++)
   {
       for(int j = -1; j < 2; j++)
       {
         if (abs(i) == abs(j))
         {
           if(i == 0 || mode == 0)
           {
             continue;
           }
         }
         
         if((colId + i) < 0 || (rowId + j) < 0 || (colId + i) >= (cellCols + 1) || (rowId + j) >= (cellRows + 1))
         {
           continue;
         }
         
         if(visible && cost(nodes[colId][rowId], nodes[colId + i][rowId + j]) > SIGHT_THRESHOLD)
         {
           continue;
         }
         
         neighbors.add(nodes[colId + i][rowId + j]);
       }
   }
   
   Node[] result = new Node[neighbors.size()];
   neighbors.toArray(result);
   return result;
 }
 
 public Cell[] neighborCells(Cell cell, int mode)
 {
   int colId = int(cell.corners[0].coordinate.x / res);
   int rowId = int(cell.corners[0].coordinate.y / res);
   return neighborCells(colId, rowId, mode);
 }
 
 public Cell[] neighborCells(int rowId, int colId, int mode)
 {
   ArrayList<Cell> neighbors = new ArrayList<Cell>();
   for(int i = -1; i < 2; i++)
   {
       for(int j = -1; j < 2; j++)
       {
         if (abs(i) == abs(j))
         {
           if(i == 0 || mode == 0)
           {
             continue;
           }
         }
         
         if((colId + i) < 0 || (rowId + j) < 0 || (colId + i) >= cellCols || (rowId + j) >= cellRows)
         {
           continue;
         }
         
         neighbors.add(cells[colId + i][rowId + j]);
       }
   }
   
   Cell[] result = new Cell[neighbors.size()];
   neighbors.toArray(result);
   return result;
 }
 
 public Cell[] neighborCellsOfNode(Node pos, int range) // range should > 0
 {
   ArrayList<Cell> neighbors = new ArrayList<Cell>();
   int posX = int(pos.coordinate.x / res);
   int posY = int(pos.coordinate.y / res);
   
   for(int col = posX - range; col <= posX + range - 1; col++) // must offset -1 because cell coordinate at top left corner
   {
     if (col < 0 || col >= cellCols) continue;
     
     for(int row = posY - range; row <= posY + range - 1; row++)
     {
       if (row < 0 || row >= cellRows) continue;
       
       neighbors.add(cells[col][row]);
     }
   }
   
   Cell[] result = new Cell[neighbors.size()];
   neighbors.toArray(result);
   return result;
 }
 
 public Node followDownCostField(Node s)
 {
   float minCost = Float.POSITIVE_INFINITY;
   Node result = null;
   for(Node next : neighborNodes(s, 1, false))
   {
     float c = cost(s, next) + next.g;
     if(minCost > c) 
     {
       minCost = c;
       result = next;
     }
   }
   
   if (result == null)
   {
     println("Potential field at " + s.coordinate + " is corrupted! Could not get next lowest cost node!");
   }
   
   return result;
 }
 
 public Cell retrieveCell(int x, int y)
 {
  return cells[x / res][y / res]; 
 }  
 
 public boolean lineOfSight(Node source, Node target) // infinite cost value of any cell along the line will disconnect the line of sight
 {
   if (source == target) return true; //base case
   
   if(target.coordinate.x < source.coordinate.x || (abs(target.coordinate.x - source.coordinate.x) < 0.0001 && target.coordinate.y < source.coordinate.y)) // swap source and target if it is not the convention
   {
     Node temp = source;
     source = target;
     target = temp;
   }
   
   int sourceX = int(source.coordinate.x / res); // id of source node
   int sourceY = int(source.coordinate.y / res);
   
   int targetX = int(target.coordinate.x / res); // id of target node
   int targetY = int(target.coordinate.y / res);
   
   float slop = float(targetY - sourceY) / (targetX - sourceX);
   float b = source.coordinate.y - slop * source.coordinate.x;
   
   int distX = int((target.coordinate.x - source.coordinate.x) / res); // id interval 
   int distY = int((target.coordinate.y - source.coordinate.y) / res); // id interval 
   
   if(slop < 0) sourceY--; //because coordinate of each cell is the top left corner, so must offset by -1
   
   if(distX == 0) // check cases when nodes connect horizontally or vertically
   { 
     if(sourceX == cellCols)
     {
       for(int row = sourceY; row < sourceY + distY; row++)
       {
         if(cells[sourceX - 1][row].cost > SIGHT_THRESHOLD) return false;
       }
     }
     else if (sourceX - 1 < 0)
     {
       for(int row = sourceY; row < sourceY + distY; row++)
       {
         if(cells[sourceX][row].cost > SIGHT_THRESHOLD) return false;
       }
     }
     else
     {
       for(int row = sourceY; row < sourceY + distY; row++)
       {
         if(cells[sourceX][row].cost > SIGHT_THRESHOLD && cells[sourceX - 1][row].cost > SIGHT_THRESHOLD) return false;
       }
     }
   }
   else if(distY == 0)
   {
     if(sourceY == cellRows)
     {
       for(int col = sourceX; col < sourceX + distX; col++)
       {
         if(cells[col][sourceY - 1].cost > SIGHT_THRESHOLD) return false;
       }
     }
     else if (sourceY - 1 < 0)
     {
       for(int col = sourceX; col < sourceX + distX; col++)
       {
         if(cells[col][sourceY].cost > SIGHT_THRESHOLD) return false;
       }
     }
     else
     {
       for(int col = sourceX; col < sourceX + distX; col++)
       {
         if(cells[col][sourceY - 1].cost > SIGHT_THRESHOLD && cells[col][sourceY].cost > SIGHT_THRESHOLD) return false;
       }
     }
   }
   else
   {
     int nextY = 0; 
     for(int col = sourceX; col < sourceX + distX; col++)
     {
       float y = slop * nodes[col + 1][sourceY].coordinate.x + b;
       nextY = int(y / res); 

       for(int row = sourceY; ;)
       {
         if(cells[col][row].cost > SIGHT_THRESHOLD) return false;
       
         if(slop < 0)
         {
           row--;
           if(row < nextY) break;
         }
         else
         {
           row++;
           if(abs(y - nodes[col + 1][nextY].coordinate.y) < 0.0001)
           {
             if(row >= nextY) break;  
           }
           else
           {
             if(row > nextY) break;
           }
         }      
       }
     
       if(abs(y - nodes[col + 1][nextY].coordinate.y) < 0.0001 && slop < 0)
       {
         sourceY = nextY - 1;
       }
       else
       {
         sourceY = nextY;
       }
     }
   } 
  
   return true;
 }
 
 public boolean checkNeighborNodeDiagonal(Node source, Node target)
 {
   PVector dist = PVector.sub(target.coordinate, source.coordinate);
   int distX = int(dist.x / res);
   int distY = int(dist.y / res);
   
   if(abs(distX) < 2 && abs(distY) < 2 && abs(distX) == abs(distY))
   {
     return true;
   }
   
   return false;
 }
 
  public boolean checkNeighborNodeCross(Node source, Node target)
 {
   PVector dist = PVector.sub(target.coordinate, source.coordinate);
   int distX = int(dist.x / res);
   int distY = int(dist.y / res);
   
   if(abs(distX) < 2 && abs(distY) < 2 && abs(distX) != abs(distY))
   {
     return true;
   }
   
   return false;
 }
 
 public boolean checkNeighborNode(Node source, Node target, int mode)
 {
   PVector dist = PVector.sub(target.coordinate, source.coordinate);
   int distX = int(dist.x / res);
   int distY = int(dist.y / res);
   
   if(abs(distX) < 2 && abs(distY) < 2)
   {
     if(abs(distX) == abs(distY))
     {
       return mode == 1 ? true : false;
     }  
     
     return true;
   }
   
   return false;
 }
 
 public float cost(Node source, Node target) // Runtime O(1), however it may need to be improved (may swap source and target by comparing x value)
 {
   if (source == target) return 0.0; //base case
   
   if(target.coordinate.x < source.coordinate.x || (abs(target.coordinate.x - source.coordinate.x) < 0.0001 && target.coordinate.y < source.coordinate.y))
   {
     Node temp = source;
     source = target;
     target = temp;
   }
   
   PVector dist = PVector.sub(target.coordinate, source.coordinate);
   int distX = int(dist.x / res);
   int distY = int(dist.y / res);
   
   int sourceX = int(source.coordinate.x / res); // id of source node
   int sourceY = int(source.coordinate.y / res);
   
   if(abs(distX) < 2 && abs(distY) < 2)
   {
     if(abs(distX) == abs(distY)) // diagonal neighbor
     {
       int idX, idY;        
       if(distX == distY)
       {
         idX = sourceX;
         idY = sourceY;
       }
       else
       {
         idX = sourceX;
         idY = sourceY - 1;
       }
       
       return 1.4 * cells[idX][idY].cost;
     }
     else
     {
       float c1 = Float.POSITIVE_INFINITY,  c2 = Float.POSITIVE_INFINITY;
       if(distX == 0)
       {
         if(sourceX == cellCols)
         {
            c1 = 1.0 * cells[sourceX - 1][sourceY].cost;
         }
         else if (sourceX - 1 < 0)
         {
            c2 = 1.0 * cells[sourceX][sourceY].cost;
         }
         else
         {
           c1 = 1.0 * cells[sourceX - 1][sourceY].cost;
           c2 = 1.0 * cells[sourceX][sourceY].cost;
         }    
       }
       else //distY = 0
       {
         if(sourceY == cellRows)
         {
            c1 = 1.0 * cells[sourceX][sourceY - 1].cost;
         }
         else if (sourceY - 1 < 0)
         {
           c2 = 1.0 * cells[sourceX][sourceY].cost;
         }
         else
         {
           c1 = 1.0 * cells[sourceX][sourceY - 1].cost;
           c2 = 1.0 * cells[sourceX][sourceY].cost;
         } 
       }
       
       return min(c1, c2);
     }     
   }
   else
   {
     println("Source and Target Node are not neighbors! Return -1");
     return -1;
   }
 }
 
 public float dist(Node source, Node target)
 {
   return source.dist(target) / res;
 }
 
 public float angle(Node s, Node p, Node t)
 {
   PVector line1 = PVector.sub(s.coordinate, p.coordinate);
   PVector line2 = PVector.sub(t.coordinate, p.coordinate);
   
   return round(PVector.angleBetween(line1, line2) * 100) / 100.0;
 }
 
 public float angle(Node s, Node t)
 {
   return round(PVector.angleBetween(s.coordinate, t.coordinate) * 100) / 100.0;
 }
 
 public void leftMouseClicked(int mX, int mY)
 {
   Cell cell = retrieveCell(mX, mY);
   if(gridMode == GridMode.BINARY)
   {
     cell.cost = Float.POSITIVE_INFINITY;
     cell.gray = 0;
   }
   else if(gridMode == GridMode.RANDOMCOST)
   {
     if(cell.cost < MAX_COST)
     {
       cell.cost += COST_STEP; 
       cell.gray = int(((MAX_COST - cell.cost) * 255 / (MAX_COST - 1.0))) ;
     }
     else
     {
       cell.cost = MAX_COST; 
       cell.gray = 0;
     }
   }
 }
 
 public void rightMouseClicked(int mX, int mY)
 {
    Cell cell = retrieveCell(mX, mY);
    cell.gray = 255;
    cell.cost = 1.0;
 }
 
 public void randomCost(int percentage)
 {
   Random r = new Random();
   int max = int(float(percentage) * cellCols * cellRows / 100);
   int total = 0;
   for(int col = 0; col < cellCols; col++)
   {
     for(int row = 0; row < cellRows; row++)
     {
       int value = r.nextInt(100) + 1; // [1..100]
       if(value > percentage) continue;
       
       if(gridMode == GridMode.RANDOMCOST)
       {
         int cost = r.nextInt(int(MAX_COST)) + 1;
         cells[col][row].cost = cost;
         cells[col][row].gray = int(((cost - 1.0) * 255 / (MAX_COST - 1.0))) ;
       }
       else if(gridMode == GridMode.BINARY)
       {
         cells[col][row].cost = Float.POSITIVE_INFINITY;
         cells[col][row].gray = 0;
       }
       
       total++;
       if(total >= max) return;
     }
   }
 }
 
 public Grid deepCopy()
 {
   Grid newGrid;
   try
   {
     newGrid = new Grid(this.cellCols * this.res + 1, this.cellRows * this.res + 1, this.res, this.gridMode);
   }
   catch(InvalidResolution e)
   {
     return null;
   }
   
   for(int col = 0; col < this.cellCols + 1; col++) // no need to copy coordinate because they are the same
   {
     for(int row = 0; row < this.cellRows + 1; row++)
     {
       newGrid.nodes[col][row].nodeColor = this.nodes[col][row].nodeColor;
       
       if(this.nodes[col][row].parent != null)
       {
         int parentIdX = int(this.nodes[col][row].parent.coordinate.x / res);
         int parentIdY = int(this.nodes[col][row].parent.coordinate.y / res);
         newGrid.nodes[col][row].parent = newGrid.nodes[parentIdX][parentIdY];
       }
       
       if(this.nodes[col][row].localParent != null)
       {
         int parentIdX = int(this.nodes[col][row].localParent.coordinate.x / res);
         int parentIdY = int(this.nodes[col][row].localParent.coordinate.y / res);
         newGrid.nodes[col][row].localParent = newGrid.nodes[parentIdX][parentIdY];
       }
       
       newGrid.nodes[col][row].g = this.nodes[col][row].g;
       newGrid.nodes[col][row].rhs = this.nodes[col][row].rhs;
       newGrid.nodes[col][row].f = this.nodes[col][row].f;
       newGrid.nodes[col][row].minG = this.nodes[col][row].minG;
       
       newGrid.nodes[col][row].range.min = this.nodes[col][row].range.min;
       newGrid.nodes[col][row].range.max = this.nodes[col][row].range.max;
     }
   }
   
   for(int col = 0; col < this.cellCols; col++) // no need to copy coordinate because they are the same
   {
     for(int row = 0; row < this.cellRows; row++)
     {
       newGrid.cells[col][row].gray = this.cells[col][row].gray;
       newGrid.cells[col][row].cost = this.cells[col][row].cost;
     }
   }
   
   return newGrid;
 }
}