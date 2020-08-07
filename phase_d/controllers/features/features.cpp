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
  cout << "Finding path" << endl;
  PathFinding::generatePath();
  cout << "Moving..." << endl;
  Epuck r = Epuck(); 
  r.runSim(true);
  
  unique_ptr<Map> map(new Map());
  
  return 0;
}
