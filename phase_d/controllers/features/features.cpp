// File:          main.cpp
// Date:
// Description:   with features
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <memory>

#include "TrajectoryPlanning.cpp"
#include "../phases/phase_b.cpp"
#include "Exploration.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

int main(int argc, char **argv) {
    Epuck epuck = Epuck();
    unique_ptr<ExploreMap> emap(new ExploreMap());
  
    cout << "Exploring map" << endl;
    emap->explore(epuck);
    cout << "Map Explored" << endl;
    // emap->print2DVector(emap->getExplored());
    cout << "Finding path" << endl;
    // PathFinding::generatePath();
    cout << "Moving" << endl;
    // epuck.runSim(true);
    cout << "Complete" << endl;
    return 0;
}
