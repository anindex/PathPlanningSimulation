Grid grid;
public static final int RESOLUTION = 20;
boolean started = true;
Node start, goal;

void setup()
{
  size(301, 301);
  
  //noStroke();
  
  try
  {
    grid = new Grid(width, height, RESOLUTION, GridMode.RANDOMCOST);
  }
  catch (InvalidResolution e)
  {
    started = false;
  }
  
  grid.randomCost(20);
  
  start = grid.nodes[15][2];
  goal = grid.nodes[15][10];
  
  goal.parent = start;
}

void draw()
{
  if(started)
  {
     grid.drawMap();
     grid.drawPath(start, goal);
     //grid.drawArrows(ArrowMode.REMOTE);
  }
 
}

void mouseClicked()
{
  if(mouseButton == LEFT)
  {
    grid.leftMouseClicked(mouseX, mouseY);
    println(grid.lineOfSight(start, goal));
  }
  else if(mouseButton == RIGHT)
  {
    grid.rightMouseClicked(mouseX, mouseY);
  }
}