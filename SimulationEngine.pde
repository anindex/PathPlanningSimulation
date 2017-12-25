import java.util.Iterator;

Grid grid;
BasicTheta thetaPlanner;
AStar astarPlanner;

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
  
  thetaPlanner = new BasicTheta(grid, 2, start.coordinate, goal.coordinate);
  astarPlanner = new AStar(grid, 2, start.coordinate, goal.coordinate);
  
  thetaPlanner.start();
  astarPlanner.start();
  
  try
  {
    thetaPlanner.join();
    astarPlanner.join();
  }
  catch(Exception e)
  {}
  
}

void draw()
{
  if(started)
  {
     grid.drawMap();
     if(thetaPlanner.path != null) grid.drawPath(thetaPlanner.path);
     if(astarPlanner.path != null) grid.drawPath(astarPlanner.path);
     grid.drawNodeColor(thetaPlanner.robot.start.coordinate, ObjectColor.START);
     grid.drawNodeColor(thetaPlanner.robot.goal.coordinate, ObjectColor.GOAL);
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