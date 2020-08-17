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
  
    cout << "Exploring map" << endl;
    emap->explore(robot);
    cout << "EXPLORED:\n";
    emap->print2DVector(emap->getExplored());
    // cout << "H WALLS:\n";
    // emap->print2DVector(emap->getHWalls());
    // cout << "V WALLS:\n";
    // emap->print2DVector(emap->getVWalls());
    
    // cout << "Map Explored" << endl;
    // emap->print2DVector(emap->getExplored());
    // cout << "Finding path" << endl;
    // PathFinding::generatePath();
    // cout << "Moving" << endl;
    // epuck.runSim(true);
    cout << "Complete" << endl;
    return 0;
}
