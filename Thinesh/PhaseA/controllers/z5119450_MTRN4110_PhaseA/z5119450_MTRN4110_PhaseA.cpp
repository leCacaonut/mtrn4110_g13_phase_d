/*
 * File: z5119450_MTRN4110_PhaseA.cpp
 * Date: 09/06/2020
 * Description: Robot controller reads and simulates commands provided
 * Author: Thinesh Manisekaran, z5119450
 * Modifications: 
*/


// webots include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <Python.h>

// other includes
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <iomanip>

// general constants
#define TIME_STEP       64
#define MAX_SPEED       6.28
#define NUM_SENSORS     3
#define WALL_DETECTED   1000
#define YAW_INDEX       2

// initialising constants
#define INIT_ROW        0
#define INIT_COL        1
#define INIT_HEAD       2
#define INITIAL_COMMAND 3

// Vehicle and world geometry constants
#define PI              3.1419
#define WHEEL_RAD       20
#define AXLE_LEN        56.6
#define LEN_SQUARE      165.0

// Rotating and forward motion formulas
#define DIST_FORWARD    (LEN_SQUARE/WHEEL_RAD)
#define DIST_ROTATE     (PI/4*AXLE_LEN/WHEEL_RAD)
#define SPEED_FORWARD   MAX_SPEED
#define SPEED_ROTATE    0.4*MAX_SPEED

// Defined path to command file provided
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt" 

// Namespaces
using namespace webots;
using namespace std;
  
enum WallIDs     {LEFT, RIGHT, FRONT} wIDs; // wall position indexing
enum PositionIDs {ROW , COLUMN}       pIDs; // used to index position array
enum MotorIDs    {LMOTOR, RMOTOR}     mIDs; // index left and right motors

class Epuck {
  private: 
    Robot          *robot;
    Motor          *motors[2];
    DistanceSensor *distSensors[3];
    PositionSensor *posSensors[2];
    
    string          commands;
    int             currCommandIndex;
    char            currCommand;
    int             endCommand;
    double          distSensorReadings[3];
    double          posSensorReadings[2];
    char            heading;
    int             gridPosition[2];
    double          motorPosition[2];
    char            walls[3];
    
  public:
    Epuck();
    
    ~Epuck() 
    {
      delete robot;
    };

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

// constructor creates new robot and sets sensors
Epuck::Epuck() {
  robot = new Robot();
  // motors
  motors[LMOTOR] = robot->getMotor("left wheel motor");
  motors[RMOTOR] = robot->getMotor("right wheel motor");
  // distance sensors
  distSensors[LEFT]  = robot->getDistanceSensor("Left Distance Sensor");
  distSensors[RIGHT] = robot->getDistanceSensor("Right Distance Sensor");
  distSensors[FRONT] = robot->getDistanceSensor("Front Distance Sensor");
  // position sensors
  posSensors[LMOTOR] = robot->getPositionSensor("left wheel sensor");
  posSensors[RMOTOR] = robot->getPositionSensor("right wheel sensor"); 
}


// reads robot commands from PathPlan.txt
void Epuck::readPath() 
{
  ifstream PathFile(PATH_PLAN_FILE_NAME);
  getline(PathFile, commands);
  PathFile.close();
  
  // remove initial position
  endCommand = commands.length();
  currCommandIndex = INITIAL_COMMAND;
  
  // print information from file and path to file
  cout << "Start - Read path plan from " << PATH_PLAN_FILE_NAME << ":" << endl;
  cout << commands << endl;
  cout << "Done - Path plan read!" << endl;
  cout << "Start - Execute path plan!" << endl;
}


// enable epuck sensors
void Epuck::enableSensors() 
{
  // enable distance sensors
  distSensors[LEFT]->enable(TIME_STEP);
  distSensors[RIGHT]->enable(TIME_STEP);
  distSensors[FRONT]->enable(TIME_STEP);
  // enable position sensors
  posSensors[LMOTOR]->enable(TIME_STEP);
  posSensors[RMOTOR]->enable(TIME_STEP);  
}

// get values from distance sensors
void Epuck::getDistSensorReadings()
{
  while (robot->step(TIME_STEP) != -1) {
    // cout << "here" << endl;
    distSensorReadings[LEFT]  = distSensors[LEFT]->getValue();
    distSensorReadings[RIGHT] = distSensors[RIGHT]->getValue();
    distSensorReadings[FRONT] = distSensors[FRONT]->getValue(); 

    if (validDistReadings()) {
      break;
    }
   }
}

// get values from position sensors
void Epuck::getPosSensorReadings()
{
  while (robot->step(TIME_STEP) != -1) {
    posSensorReadings[LMOTOR] = posSensors[LMOTOR]->getValue();
    posSensorReadings[RMOTOR] = posSensors[RMOTOR]->getValue();   
    
    if (validPosReadings()) {
      break;
    }
  }
}


// check if distance sensor readings are valid
bool Epuck::validDistReadings() 
{
  int sensorIndex = 0;

  // check if distance sensor readings are numbers
  for (sensorIndex = 0; sensorIndex < NUM_SENSORS; sensorIndex++) {
    if (distSensorReadings[sensorIndex] == NAN) {
      cout << "Invalid distance sensor readings" << endl;                            
      return false;
    }
  }
 
  return true;
}

// check if position sensor readings are valid
bool Epuck::validPosReadings() 
{
  // check if position sensor readings are numbers
  if (posSensorReadings[LMOTOR] == NAN || posSensorReadings[RMOTOR] == NAN) {
    cout << "Invalid position sensor readings" << endl;                            
    return false;
  }
 
  return true;
}


// get sensor readings and update if walls are present or not
void Epuck::updateWalls()
{
  // update sensor readings
  getDistSensorReadings();
  
  distSensorReadings[LEFT]  < WALL_DETECTED ? walls[LEFT]  = 'Y': walls[LEFT]  = 'N';
  distSensorReadings[RIGHT] < WALL_DETECTED ? walls[RIGHT] = 'Y': walls[RIGHT] = 'N';
  distSensorReadings[FRONT] < WALL_DETECTED ? walls[FRONT] = 'Y': walls[FRONT] = 'N';
}


// update current row and column position and heading
void Epuck::updatePosition()
{ 
  switch (heading) {
    case 'N':
      gridPosition[ROW]    -= 1; 
      break;     
    case 'S':
      gridPosition[ROW]    += 1; 
      break;  
    case 'E':
      gridPosition[COLUMN] += 1; 
      break;   
    case 'W':
      gridPosition[COLUMN] -= 1;  
      break;  
    default:
      cout << "Invalid Heading in updatePosition" << endl;
      break;
  }
}

// update current heading based on left or right command
void Epuck::updateHeading()
{
  switch (heading) {
    case 'N': 
      (currCommand == 'R') ? heading = 'E' : heading = 'W'; 
      break;     
    case 'S':
      (currCommand == 'R') ? heading = 'W' : heading = 'E';
      break;  
    case 'E':
      (currCommand == 'R') ? heading = 'S' : heading = 'N';
      break;   
    case 'W':
      (currCommand == 'R') ? heading = 'N' : heading = 'S';
      break;  
    default:
      cout << "Invalid Heading in updateHeading" << endl;
      break;
  }
}

void Epuck::moveRobot()
{
  double lTarget = motorPosition[LMOTOR] + DIST_FORWARD;
  double rTarget = motorPosition[RMOTOR] + DIST_FORWARD;

  // set motor speed
  motors[LMOTOR]->setVelocity(SPEED_FORWARD);
  motors[RMOTOR]->setVelocity(SPEED_FORWARD);
  // set new position
  motors[LMOTOR]->setPosition(lTarget);
  motors[RMOTOR]->setPosition(rTarget);
  
  getPosSensorReadings();
  // wait for robot to reach position
  while (robot->step(TIME_STEP) != -1) {
    double lPos = posSensorReadings[LMOTOR];
    double rPos = posSensorReadings[RMOTOR];
    // check if position has been reached
    if (lPos >= lTarget && rPos >= rTarget) {
      motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
      motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
      break;
    }
    
    getPosSensorReadings();
  } 
  
  // update grid position and walls
  updatePosition();     
  updateWalls(); 
}

void Epuck::rotateRobot()
{
  double lTarget = motorPosition[LMOTOR] + DIST_ROTATE;
  double rTarget = motorPosition[RMOTOR] + DIST_ROTATE;
  bool breakCondition = false;
  
  // adjust for direction
  currCommand == 'R' ? (rTarget -= 2*DIST_ROTATE) : (lTarget -= 2*DIST_ROTATE);
  
  // set motor speed
  motors[LMOTOR]->setVelocity(SPEED_ROTATE);
  motors[RMOTOR]->setVelocity(SPEED_ROTATE);
  // set new position
  motors[LMOTOR]->setPosition(lTarget);
  motors[RMOTOR]->setPosition(rTarget);  
  
  getPosSensorReadings();
  // wait for robot to reach position
  while (robot->step(TIME_STEP) != -1) {
    double rPos = posSensorReadings[RMOTOR];
    // check if position has been reached, set break condiion
    currCommand == 'R' ? (breakCondition = (rPos <= rTarget)) \
                       : (breakCondition = (rPos >= rTarget));
    
    if (breakCondition) {
      motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
      motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
      break;
    }
    
    getPosSensorReadings();
  } 
  
  // update grid position and walls
  updateHeading();     
  updateWalls();  
}

void Epuck::setInitStatus()
{
  readPath();             // extract commands from file  
  enableSensors();        // enable sensors 
  updateWalls();          // get wall status
  getPosSensorReadings(); // get current position coordinates
  //set initial position, covert from char to int
  gridPosition[ROW]    = commands[INIT_ROW] - '0';
  gridPosition[COLUMN] = commands[INIT_COL] - '0'; 
  //set coordinate position
  motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
  motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
  //set heading
  heading = commands[INIT_HEAD];   
}

// display walls, position, heading
void Epuck::displayStatus()
{
  cout << "Step: " << setw(2) << setfill('0') << (currCommandIndex - INITIAL_COMMAND) \
       << ", Row: "        << gridPosition[ROW]    \
       << ", Column: "     << gridPosition[COLUMN] \
       << ", Heading: "    << heading              \
       << ", Left Wall: "  << walls[LEFT]          \
       << ", Front Wall: " << walls[FRONT]         \
       << ", Right Wall: " << walls[RIGHT]         \
       << endl;
}

/* 
 * Runs simulation by incrementing through provided commands
 * and navigating robot
*/
void Epuck::runSim() 
{  
  setInitStatus();
  displayStatus();      

  while(robot->step(TIME_STEP) != -1) {
    currCommand = commands[currCommandIndex];
    currCommand == 'F' ? moveRobot() : rotateRobot();
    currCommandIndex += 1;   

    displayStatus();
    
    // if final command executed
    if (currCommandIndex == endCommand) {
      break;
    }
  }
  
  cout << "Done - Path plan executed!" << endl;
}


int main(int argc, char **argv) 
{
  Epuck r = Epuck(); 
  r.runSim();

  return 0;
}