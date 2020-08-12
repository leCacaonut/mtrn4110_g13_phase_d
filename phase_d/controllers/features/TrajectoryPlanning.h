// webots include files
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

// other includes
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

// Namespaces
using namespace webots;
using namespace std;

// general constants
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_SENSORS 3
#define WALL_DETECTED 1000
#define YAW_INDEX 2
#define DEVIATION 0.03

// initialising constants (index of instructions in file)
#define INIT_ROW 0
#define INIT_COL 1
#define INIT_HEAD 2
#define INITIAL_COMMAND 3

// Vehicle and world geometry constants
#define PI 3.1419
#define WHEEL_RAD 0.020
#define AXLE_LEN 0.0566
#define LEN_SQUARE 0.165

// Rotating and forward motion formulas
#define DIST_FORWARD (LEN_SQUARE / WHEEL_RAD)
#define DIST_ROTATE (PI / 4 * AXLE_LEN / WHEEL_RAD)
#define SPEED_FORWARD MAX_SPEED
#define SPEED_ROTATE 0.4 * MAX_SPEED
#define GRID_GAP (LEN_SQUARE - AXLE_LEN)
#define LR_WHEEL_ROTATE_RATIO
#define SMALL_ROTATE 4.2568
#define LARGE_ROTATE SMALL_ROTATE*2

// Defined path to command file provided
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"

enum WallIDs { LEFT,
               RIGHT,
               FRONT } wIDs;  // wall position indexing
enum PositionIDs { ROW,
                   COLUMN } pIDs;  // used to index position array
enum MotorIDs { LMOTOR,
                RMOTOR } mIDs;  // index left and right motors

class Epuck {
   private:
    Robot *robot;
    Motor *motors[2];
    DistanceSensor *distSensors[3];
    PositionSensor *posSensors[2];

    string commands;
    int currCommandIndex;
    char currCommand;
    int endCommand;
    double distSensorReadings[3];
    double posSensorReadings[2];
    char heading;
    int gridPosition[2];
    double motorPosition[2];
    char walls[3];

   public:
    Epuck();

    ~Epuck();
    

    // run simulation
    void runSim();
    // initialisation
    void readPath();
    void enableSensors();
    void setInitStatus();
    // readings
    void getDistSensorReadings();
    void getPosSensorReadings();
    bool validDistReadings();
    bool validPosReadings();
    // updates
    void updateWalls();
    void updatePosition();
    void updateHeading();
    void updateSurroundings();
    // navigation
    void moveRobot(); // moves grid length
    void moveRobot(unsigned int numberOfMotions); // smooth move grid length
    void moveRobot(unsigned int numberOfMotions, bool moveHalfGrid); // smooth move half grid length
    void rotateRobot();
    void rotateRobot(bool smoothGridTurn);
    void displayStatus();
};