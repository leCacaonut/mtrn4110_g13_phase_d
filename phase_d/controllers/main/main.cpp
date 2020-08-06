// File:          main.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include "../phases/phase_a.cpp"
#include "../phases/phase_b.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  cout << "Finding path" << endl;
  runProcess();
  cout << "Moving..." << endl;
  Epuck r = Epuck(); 
  r.runSim();

  return 0;
}
