# Path Planning Grid World

The Path Planning Grid World is a simulation engine developed in Java, harnessing the rich graphical and event handler libraries of the Processing environment.  
## Motivation
This simulation engine serves two purposes:
- Develop a grid world simulation environment for the evaluation of various path planning algorithms. Researchers can add their algorithms to test the engine.
- Various criteria evaluation of path planning algorithms for the book chapter titled [Search-Based Planning and Replanning in Robotics and Autonomous Systems](https://www.intechopen.com/books/advanced-path-planning-for-mobile-entities/search-based-planning-and-replanning-in-robotics-and-autonomous-systems).

Soon, the path planning simulation engine will be a standalone framework to serve as test-bed for the Motion Planning community. If you use this work for your research, please kindly cite it as [![DOI](https://zenodo.org/badge/115075354.svg)](https://zenodo.org/badge/latestdoi/115075354).
## Implementation

### Implemented Modules:

**Simulation engine**
- Grid Engine
- Drawer
- Agent Handler
- Random Environment Generator
- Random Maze Generator

**Planning algorithms**
- [A*](https://en.wikipedia.org/wiki/A*_search_algorithm) well-known heuristic planning algorithm
- [ARA*](https://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf): Anytime A* with Provable Bounds on Sub-Optimality
- [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
- [AD*](http://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf)
- [Phi*](https://www.sciencedirect.com/science/article/pii/S187770581403149X)
- [Incremental Phi*](http://www.cs.cmu.edu/~./maxim/files/inctheta_ijcai09.pdf)

Detail UML for classes and architectures will be generated soon.

### High-level Component Diagram

![PathPlanningSim](https://user-images.githubusercontent.com/18066876/59590532-c8a4ec80-90db-11e9-8b28-842c451164e8.png)

## Usages

### Prerequisites
Please download Processing [here](https://processing.org/download/) and open the engine as a project by opening the main file *PathPlanningSimulation.pde*.

Users can interactively click on the grid to add or clear blocked cells represented as the black square (see in demo). The start and goal poses are always top-left cell and bottom-right cell, respectively.

### Changing grid size and resolution
The resolution is the width of each cell. For instance, if the map is 15x15 cells and each cell has 20 pixels, then the corresponding configuration is as followed in *PathPlanningSimulation.pde*. The size of the drawing window would be 15*20 + 1 = 301.

```java

...
public static final int RESOLUTION = 20;
...

void setup()
{
  size(301, 301);
  ...
}
...
```

### Starting planners
There is a planning algorithm for each agent planner, please start a thread corresponding to an agent by pressing the following key:
- 'a' for ARA* planner
- 'd' for D* Lite planner
- 'q' for AD* planner
- 'i' for incremental Phi* planner

Pressing multiple keys will start multiple individual threads and all current positions of all agents will be drawn in the simulation grid.

## Demo

Please click on the below image for the demo of the simulation engine.
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/nAne4CkpFkY/0.jpg)](https://www.youtube.com/watch?v=nAne4CkpFkY)
