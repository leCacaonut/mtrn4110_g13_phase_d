// File:          main.cpp
// Date:
// Description:   with features
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <memory>

#include "TrajectoryPlanning.cpp"
#define PATHFINDING_CPP
#include "../phases/phase_b.cpp"
#include "Exploration.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

int main(int argc, char **argv) {
    Epuck robot = Epuck();
    
    unique_ptr<ExploreMap> emap(new ExploreMap());
  
    cout << ">>> EXPLORING MAP <<<" << endl;
    emap->explore(robot);
    cout << "--- Explored map ---\n";
    emap->print2DVector(emap->getExplored());
    cout << "--- Horizontal walls ---\n";
    emap->print2DVector(emap->getHWalls());
    cout << "--- Vertical walls ---\n";
    emap->print2DVector(emap->getVWalls());
    
    cout << ">>> MOVING TO GOAL <<<" << endl;
    PathFinding::generatePath(robot.getHeading(), robot.getPosition(), emap->getGoal(), emap->getHWalls(), emap->getVWalls());
    robot.runSim(true);
    cout << ">>> GOAL REACHED <<<" << endl;
    return 0;
}
