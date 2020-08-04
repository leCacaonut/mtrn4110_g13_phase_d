// File:          z5129009_MTRN4110_PhaseA.cpp
// Date of submission: 21/6/2020
// Description: MTRN4110 Assignment, Phase A
// Author: Mei Yan Tang
// zID: z5129009

//include files
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//define constants
#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define PI 3.14159
#define FORWARD 'F'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'
#define POSITION_STEP_LINEAR 165.0/20.0
#define MAX_SPEED 6.28

//function for getting the heading in (N,S,E,W) by inputting the yaw value
std::string getHeading(const double yaw) {
  std::string heading = " ";
    if(yaw < -1.50 && yaw > -1.60) {
      heading = "E";
    } else if (yaw > 3.10 && yaw <3.20) {
      heading = "S";
    } else if (yaw > -3.20 && yaw < -3.10) {
      heading = "S";
    } else if (yaw > 1.50 && yaw < 1.70) {
      heading = "W";
    } else if (yaw > -0.1 && yaw < 0.1) {
      heading = "N";
    }  
  return heading;
}

//function to convert bool to string 'Y' or 'N' (yes or no)
std::string bool2string(bool wall) {
  std::string YN;
  if (wall == true) {
    YN = "Y";
  } else {
    YN = "N";
  }
  return YN;
}

//main function
int main(int argc, char **argv) {

  //read PathPlan.txt
  std::ifstream file(PATH_PLAN_FILE_NAME);
  std::string path;
  int nSteps = 0;
  int n = 0;
  
  //check file is successfully open
  if(file.is_open()) {
    getline(file,path);
    std::cout << "START - Read path plan from " << PATH_PLAN_FILE_NAME << ':' << std::endl;
    std::cout << path << std::endl;
    std::cout << "Done - Path plan read!" << std::endl;
    std::cout << "Start - Execute path plan!" << std::endl;
  } else {
    std::cout << "Unable to read file :(" << std::endl;
  }
   
  //get number of steps
  n = path.length();
  nSteps = n-3;
  
  //copy the path (each motion step) into char array
  char pathArray[n+1];
  path.copy(pathArray,n,0);

  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  //initialize devices (inertial unit, motors, position sensors, distance sensors)
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  PositionSensor *leftEncoder = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightEncoder = robot->getPositionSensor("right wheel sensor");
  InertialUnit *IMU = robot->getInertialUnit("inertial unit");  
  DistanceSensor *leftWallDS = robot->getDistanceSensor("left wall sensor");
  DistanceSensor *rightWallDS = robot->getDistanceSensor("right wall sensor");
  DistanceSensor *frontWallDS = robot->getDistanceSensor("front wall sensor");
  leftEncoder->enable(timeStep);
  rightEncoder->enable(timeStep);
  IMU->enable(timeStep);
  frontWallDS->enable(timeStep);
  leftWallDS->enable(timeStep);
  rightWallDS->enable(timeStep);
  
  int i = 3;
  double leftPosition = 0.0;
  double rightPosition = 0.0;
  double leftError = 0.0;
  double rightError = 0.0;
  int step = 0;
  //get initial row and col
  int row = (int)pathArray[0]-48;
  int col = (int)pathArray[1]-48;
  double turn=0;
  double turnError=0;
  double leftSpeed  = 0.5 * MAX_SPEED;
  double rightSpeed = 0.5 * MAX_SPEED;
  std::string heading = " ";

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) { 
  
    //get the yaw value (to determine the heading)   
    const double yaw = IMU->getRollPitchYaw()[2];
    
    // detect obstacles (walls)
    bool rightWall = rightWallDS->getValue() > 250.0;
    bool leftWall = leftWallDS->getValue() > 250.0;
    bool frontWall = frontWallDS->getValue() > 250.0; 
    
    //read the motion step from path plan and execute 
    //if error is small enough and path plan have not ended
    //note: i starts from 3 to skip the first 3 characters which specifies the initial location and heading
    if (leftError < 0.005 && rightError <0.005 && turnError <=0.005 && step <= nSteps) {
      if (pathArray[i] == FORWARD) {
        leftSpeed  = 0.5 * MAX_SPEED;
        rightSpeed = 0.5 * MAX_SPEED;
        leftPosition = leftEncoder->getValue() + POSITION_STEP_LINEAR;
        rightPosition = rightEncoder->getValue() + POSITION_STEP_LINEAR; 
      } else if (pathArray[i] == LEFT_TURN) {
        leftSpeed = 0; 
        rightSpeed = 0;
        //heading south, can either be around -pi or pi
        if(heading == "S") {
          turn =-90*(PI/180);
        } else {
          turn = yaw + 90*(PI/180);
        }     
        leftSpeed -= 0.1* MAX_SPEED;
        rightSpeed += 0.1 * MAX_SPEED;
      } else if (pathArray[i] == RIGHT_TURN) {
        leftSpeed = 0;
        rightSpeed = 0;
        //heading south, can either be around -pi or pi
        if(heading == "S") {
          turn = 90*(PI/180);
        } else {
          turn = yaw - 90*(PI/180);
        }   
        leftSpeed  += 0.1 * MAX_SPEED;
        rightSpeed -= 0.1 * MAX_SPEED;
      }
      
      //get the heading of the robot
      heading = getHeading(yaw);
      
      //get the robot position in rows and columns
      //example: if the robot have moved forward, and robot is facing south, 
      // it means that the robot have moved down by one row.
      if (pathArray[i-1] == FORWARD) {
        if(heading == "S") {
          row++;
        } else if (heading == "E") {
          col++;
        } else if (heading == "N") {
          row--;
        } else if (heading == "W") {
          col--;
        }     
      }
      
      if (step < 10) {
        std::cout << "Step: 0" << step <<  ", Row: " << row << ", Column: " << col <<", Heading: " << heading << 
        " , Left Wall: " << bool2string(leftWall) << " , Front Wall: " << bool2string(frontWall) << 
        " , Right Wall: " << bool2string(rightWall) << std::endl;   
      } else {
        std::cout << "Step: " << step <<  ", Row: " << row << ", Column: " << col <<", Heading: " << heading << 
        " , Left Wall: " << bool2string(leftWall) << " , Front Wall: " << bool2string(frontWall) << 
        " , Right Wall: " << bool2string(rightWall) << std::endl;   
      }
      
      step++;
      i++;
      
      //print the statement when path plan has finished executing 
      if(step == nSteps +1) {
        std::cout << "Done - Path plan executed!" << std::endl;
       }
    }   
      
    //calculate error  
    if(pathArray[i-1] == FORWARD) {
      leftError = leftPosition - leftEncoder->getValue() ;
      rightError = rightPosition - rightEncoder->getValue();
    } else if(pathArray[i-1] == LEFT_TURN) {
      if((yaw < 0 && turn > 0) || (yaw>0 && turn <0)) {
        turnError = abs(yaw + turn);
      } else {
        turnError = abs(yaw - turn);
      }
    } else if(pathArray[i-1] == RIGHT_TURN) {
      if((yaw < 0 && turn > 0) || (yaw>0 && turn <0)) {
        turnError = abs(yaw + turn);
      } else {
        turnError = abs(yaw - turn);
      }
    }
     
    //stop robot from moving when path plan ended
    if(step == nSteps+1 && leftError < 0.005 && rightError <0.005) {
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0);
    //otherwise do not stop
    } else {
      leftMotor->setPosition(INFINITY);
      rightMotor->setPosition(INFINITY);
      leftMotor->setVelocity(leftSpeed);
      rightMotor->setVelocity(rightSpeed);
    }

  };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}

