import java.util.Iterator;

Grid grid;
BasicTheta thetaPlanner;
AStar astarPlanner;
Phi phiPlanner;
DStarLite dlitePlanner;

public static final int RESOLUTION = 20;
boolean started = true;
Node start, goal;

void setup()
{
  size(301, 301);
  
  try
  {
    grid = new Grid(width, height, RESOLUTION, GridMode.BINARY);
  }
  catch (InvalidResolution e)
  {
    started = false;
  }
  
  grid.randomCost(20);
  start = grid.nodes[1][1];
  goal = grid.nodes[14][14];
  
  //thetaPlanner = new BasicTheta(grid, 2, start.coordinate, goal.coordinate);
  //astarPlanner = new AStar(grid, 2, start.coordinate, goal.coordinate);
  //phiPlanner = new Phi(grid, 2, start.coordinate, goal.coordinate);
  
  
  //thetaPlanner.start();
  //astarPlanner.start();
  //phiPlanner.start();
  
  try
  {
    //thetaPlanner.join();
    //astarPlanner.join();
    //phiPlanner.join();
  }
  catch(Exception e)
  {}
}

void draw()
{
  if(started)
  { 
    grid.drawMap();
    //if(thetaPlanner.path != null) grid.drawPath(thetaPlanner.path, ObjectColor.THETAPATH.getColor());
    //if(astarPlanner.path != null) grid.drawPath(astarPlanner.path);
    //if(phiPlanner.path != null) grid.drawPath(phiPlanner.path, ObjectColor.PHIPATH.getColor());
    if(dlitePlanner != null)
    {
      grid.drawPath(dlitePlanner.path, ObjectColor.PATH.getColor());
      grid.drawNodeColor(dlitePlanner.robot.start.coordinate, ObjectColor.START);
      grid.drawNodeColor(dlitePlanner.robot.goal.coordinate, ObjectColor.GOAL);
      grid.drawProcessedNodes(dlitePlanner.processedNode);   
    } 
  }
}

void mouseClicked()
{
  if(mouseButton == LEFT)
  {
    grid.leftMouseClicked(mouseX, mouseY);
  }
  else if(mouseButton == RIGHT)
  {
    grid.rightMouseClicked(mouseX, mouseY);
  }
}

void keyPressed()
{
  if(key == 'd')
  {
    dlitePlanner = new DStarLite(grid, 2, start.coordinate, goal.coordinate);
    dlitePlanner.start();
  }
}