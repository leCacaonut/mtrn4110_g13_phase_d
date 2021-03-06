// Epuck class header file
// webots include files
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>

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
#define DEVIATION 0.01
#define COLLISION 750
#define DEVIATION_YAW 0.03

// initialising constants (index of instructions in file)
#define INIT_ROW 0
#define INIT_COL 1
#define INIT_HEAD 2
#define INITIAL_COMMAND 3

// Vehicle and world geometry constants
#define NUM_ROWS 5
#define NUM_COLS 9
#define MAP_ROWS 11
#define PI 3.1419
#define WHEEL_RAD 0.020
#define AXLE_LEN 0.0566
#define LEN_SQUARE 0.165

// Rotating and forward motion formulas
#define DIST_FORWARD (LEN_SQUARE / WHEEL_RAD)
// #define CORR_FACTOR_1 0.15
#define DIST_ROTATE (PI / 4 * AXLE_LEN / WHEEL_RAD)
#define SPEED_FORWARD MAX_SPEED
#define SPEED_ROTATE (0.4 * MAX_SPEED)
#define GRID_GAP ((LEN_SQUARE - AXLE_LEN) / 2)  // the gap between the wheel and the edge of the grid when robot placed in the centre of the grid
#define LR_WHEEL_ROTATE_RATIO (GRID_GAP / (GRID_GAP + AXLE_LEN))
#define CORR_FACTOR_2 0.05
#define SMALL_ROTATE ((PI / 4 * 2 * GRID_GAP / WHEEL_RAD) + CORR_FACTOR_2 * LR_WHEEL_ROTATE_RATIO)  // with rough correction factor
#define LARGE_ROTATE ((PI / 4 * 2 * (GRID_GAP + AXLE_LEN) / WHEEL_RAD) + CORR_FACTOR_2)             // with rough correction factor

// Defined path to command file provided
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define MAP_FILE_NAME "../../MapFound.txt"

enum WallIDs { LEFT,
               RIGHT,
               FRONT } wIDs;  // wall position indexing
typedef enum PositionIDs { ROW,
                           COLUMN } pIDs;  // used to index position array
enum MotorIDs { LMOTOR,
                RMOTOR } mIDs;  // index left and right motors

struct Map {
  int numExplored;
  string map[MAP_ROWS];
};

class Epuck {
   private:
    Robot *robot;
    Motor *motors[2];
    
    DistanceSensor *distSensors[3];
    PositionSensor *posSensors[2];
    InertialUnit *IMU;
    Keyboard *keyboard;
    Map m;

    string commands;
    int startPosition[2];
    int startHeading;
    int currCommandIndex;
    char currCommand;
    int endCommand;
    double distSensorReadings[3];
    double posSensorReadings[2];
    char heading;
    int gridPosition[2];
    double motorPosition[2];
    char walls[3];
    double yaw;

   public:
    Epuck(int start_row, int start_col, char start_heading);

    ~Epuck();
    // initialisation
    void readPath();
    void enableSensors();
    void getCommands();
    // readings
    void getDistSensorReadings();
    void getPosSensorReadings();
    void getIMUReadings();
    bool validDistReadings();
    bool validPosReadings();
    bool validIMUReadings();
    // updates
    void updateWalls();
    void updatePosition();
    // void updateHeading(char command);
    void moveRobot();                                                 // moves grid length

    void adjustRotation();
    
    void genMap();
    void exploreManual();
    void dispMap();
    void writeMap();
    bool checkPossible(int key);
    void mapUpdateHeading(int key);
    void mapRotateRobot(int key);
    void rotateReq(char req);
    void moveManual();

};