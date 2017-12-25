import java.util.Iterator;

Grid grid;
BasicTheta planner;

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
  
  planner = new BasicTheta(grid, 2, start.coordinate, goal.coordinate);
  planner.start();
  
  try
  {
    planner.join();
  }
  catch(Exception e)
  {}
  
}

void draw()
{
  if(started)
  {
     grid.drawMap();
     if(planner.path != null) grid.drawPath(planner.path);
     grid.drawNodeColor(planner.robot.start.coordinate, ObjectColor.START);
     grid.drawNodeColor(planner.robot.goal.coordinate, ObjectColor.GOAL);
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