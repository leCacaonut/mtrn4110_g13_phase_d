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

// initialising constants
#define INIT_ROW 0
#define INIT_COL 1
#define INIT_HEAD 2
#define INITIAL_COMMAND 3

// Vehicle and world geometry constants
#define PI 3.1419
#define WHEEL_RAD 20
#define AXLE_LEN 56.6
#define LEN_SQUARE 165.0

// Rotating and forward motion formulas
#define DIST_FORWARD (LEN_SQUARE / WHEEL_RAD)
#define DIST_ROTATE (PI / 4 * AXLE_LEN / WHEEL_RAD)
#define SPEED_FORWARD MAX_SPEED
#define SPEED_ROTATE 0.4 * MAX_SPEED

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
    void moveRobot();
    void rotateRobot();
    void displayStatus();
};