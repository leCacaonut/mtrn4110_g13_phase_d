
#include "MapBuild.cpp"
#include "../phases/phase_b.cpp"

using namespace webots;
using namespace std;

#define START_R 0
#define START_C 0
#define START_H 'S'

int main(int argc, char **argv) {
    Epuck robot = Epuck(START_R, START_C, START_H);  
    robot.exploreManual();
    cout << "Exploration Complete" << endl;
    robot.writeMap();
    PathFinding::generatePath();
    robot.moveManual();
    cout << "Destination Reached!" << endl ;
    cout << "Good Job!" << endl;
    return 0;
}
