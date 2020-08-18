/* Adapted from phase A
*  Trajectory planning for movement of robot
*
*
*/
#include "TrajectoryPlanning.h"

// constructor creates new robot and sets sensors
Epuck::Epuck() {
    robot = new Robot();
    // motors
    motors[LMOTOR] = robot->getMotor("left wheel motor");
    motors[RMOTOR] = robot->getMotor("right wheel motor");
    // distance sensors
    distSensors[LEFT] = robot->getDistanceSensor("Left Distance Sensor");
    distSensors[RIGHT] = robot->getDistanceSensor("Right Distance Sensor");
    distSensors[FRONT] = robot->getDistanceSensor("Front Distance Sensor");
    distSensors[FRONTLEFT] = robot->getDistanceSensor("Front Left Distance Sensor");
    distSensors[FRONTRIGHT] = robot->getDistanceSensor("Front Right Distance Sensor");
    // position sensors
    posSensors[LMOTOR] = robot->getPositionSensor("left wheel sensor");
    posSensors[RMOTOR] = robot->getPositionSensor("right wheel sensor");
    // IMU
    IMU = robot->getInertialUnit("inertial unit");  
}

Epuck::~Epuck() {
    delete robot;
};

// reads robot commands from PathPlan.txt
void Epuck::readPath() {
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
void Epuck::enableSensors() {
    // enable distance sensors
    distSensors[LEFT]->enable(TIME_STEP);
    distSensors[RIGHT]->enable(TIME_STEP);
    distSensors[FRONT]->enable(TIME_STEP);
    //added for obstacle avoidance
    distSensors[FRONTRIGHT]->enable(TIME_STEP);
    distSensors[FRONTLEFT]->enable(TIME_STEP);  
    // enable position sensors
    posSensors[LMOTOR]->enable(TIME_STEP);
    posSensors[RMOTOR]->enable(TIME_STEP);
    // enable IMU
    IMU->enable(TIME_STEP);
}

// get values from distance sensors
void Epuck::getDistSensorReadings() {
    while (robot->step(TIME_STEP) != -1) {
        // cout << "here" << endl;
        distSensorReadings[LEFT] = distSensors[LEFT]->getValue();
        distSensorReadings[RIGHT] = distSensors[RIGHT]->getValue();
        distSensorReadings[FRONT] = distSensors[FRONT]->getValue();
        distSensorReadings[FRONTRIGHT] = distSensors[FRONTRIGHT]->getValue();
        distSensorReadings[FRONTLEFT] = distSensors[FRONTLEFT]->getValue();

        if (validDistReadings()) {
            break;
        }
    }
}

// get values from position sensors
void Epuck::getPosSensorReadings() {
    while (robot->step(TIME_STEP) != -1) {
        posSensorReadings[LMOTOR] = posSensors[LMOTOR]->getValue();
        posSensorReadings[RMOTOR] = posSensors[RMOTOR]->getValue();

        if (validPosReadings()) {
            break;
        }
    }
}

void Epuck::getIMUReadings() {
    while (robot->step(TIME_STEP) != -1) {
        yaw = IMU->getRollPitchYaw()[2];

        if (validIMUReadings()) {
            break;
        }
    }
}

// check if distance sensor readings are valid
bool Epuck::validDistReadings() {
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
bool Epuck::validPosReadings() {
    // check if position sensor readings are numbers
    if (posSensorReadings[LMOTOR] == NAN || posSensorReadings[RMOTOR] == NAN) {
        cout << "Invalid position sensor readings" << endl;
        return false;
    }

    return true;
}

bool Epuck::validIMUReadings() {
    // check if position sensor readings are numbers
    if (yaw == NAN) {
        cout << "Invalid IMU sensor readings" << endl;
        return false;
    }

    return true;
}

// get sensor readings and update if walls are present or not
void Epuck::updateWalls() {
    // update sensor readings
    getDistSensorReadings();

    distSensorReadings[LEFT] < WALL_DETECTED ? walls[LEFT] = 'Y' : walls[LEFT] = 'N';
    distSensorReadings[RIGHT] < WALL_DETECTED ? walls[RIGHT] = 'Y' : walls[RIGHT] = 'N';
    distSensorReadings[FRONT] < WALL_DETECTED ? walls[FRONT] = 'Y' : walls[FRONT] = 'N';
}

// update current row and column position and heading
void Epuck::updatePosition() {
    switch (heading) {
        case 'N':
            gridPosition[ROW] -= 1;
            break;
        case 'S':
            gridPosition[ROW] += 1;
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
void Epuck::updateHeading() {
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

// move robot half a step
void Epuck::moveHalfStep() {
    cout << "MOVE " << endl;
    getPosSensorReadings();
    double lTarget = posSensorReadings[LMOTOR] + DIST_FORWARD/5;
    double rTarget = posSensorReadings[RMOTOR] + DIST_FORWARD/5;

    // set motor speed
    motors[LMOTOR]->setVelocity(SPEED_FORWARD);
    motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    // set new position
    motors[LMOTOR]->setPosition(lTarget);
    motors[RMOTOR]->setPosition(rTarget);

    while (robot->step(TIME_STEP) != -1) {
        getPosSensorReadings();
        double lPos = posSensorReadings[LMOTOR];
        double rPos = posSensorReadings[RMOTOR];

        // check if position has been reached
        if (lPos >= lTarget - DEVIATION && rPos >= rTarget - DEVIATION) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }

}

// avoid the obstacle
void Epuck::avoidObstacles(int obstacleLocation) {
    char right = 'R';
    char left = 'L';
    while (robot->step(TIME_STEP) != -1) {
        if(obstacleLocation == FRONTRIGHT) {
            // sequence for moving the robot when obstacle on front right
            cout << "HI" << endl;
            rotateRobotWithCommand(left);
            moveHalfStep();
            rotateRobotWithCommand(right);
            moveRobot(0.75, false, true);
            rotateRobotWithCommand(right);
            moveHalfStep();
            rotateRobotWithCommand(left);
            motors[LMOTOR]->setVelocity(SPEED_FORWARD);
            motors[RMOTOR]->setVelocity(SPEED_FORWARD);
            break;
        } else if (obstacleLocation == FRONTLEFT) {
            // sequence for moving the robot when obstacle on front left
            rotateRobotWithCommand(right);
            moveHalfStep();
            rotateRobotWithCommand(left);
            moveRobot(0.75,false,true);
            rotateRobotWithCommand(left);
            moveHalfStep();
            rotateRobotWithCommand(right);
            motors[LMOTOR]->setVelocity(SPEED_FORWARD);
            motors[RMOTOR]->setVelocity(SPEED_FORWARD);
            break;

        }
    }
}

// correct robot position if too near or too far from the wall
void Epuck::correctRobot() {
    // too near to the wall
    if (distSensorReadings[LEFT] < 750) {
        motors[LMOTOR]->setVelocity(SPEED_FORWARD);
        motors[RMOTOR]->setVelocity(SPEED_FORWARD-0.05);
    }
    if (distSensorReadings[RIGHT] < 750) {
        motors[LMOTOR]->setVelocity(SPEED_FORWARD-0.05);
        motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    }
    // too far from the wall
    if (distSensorReadings[LEFT] != 1000 && distSensorReadings[LEFT] >800) {
        motors[LMOTOR]->setVelocity(SPEED_FORWARD-0.05);
        motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    }
    if (distSensorReadings[RIGHT] != 1000 && distSensorReadings[RIGHT] >800) {
        motors[LMOTOR]->setVelocity(SPEED_FORWARD);
        motors[RMOTOR]->setVelocity(SPEED_FORWARD-0.05);
    }
    // if at centre, same speed
    if (distSensorReadings[RIGHT] < 780 && distSensorReadings[LEFT] < 780) {
        motors[LMOTOR]->setVelocity(SPEED_FORWARD);
        motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    }
}

void Epuck::moveRobot() {
    double lTarget = motorPosition[LMOTOR] + DIST_FORWARD;
    double rTarget = motorPosition[RMOTOR] + DIST_FORWARD;

    // set motor speed
    motors[LMOTOR]->setVelocity(SPEED_FORWARD);
    motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    // set new position
    motors[LMOTOR]->setPosition(lTarget);
    motors[RMOTOR]->setPosition(rTarget);

    // wait for robot to reach position
    while (robot->step(TIME_STEP) != -1) {
        getPosSensorReadings();
        double lPos = posSensorReadings[LMOTOR];
        double rPos = posSensorReadings[RMOTOR];
        // check if position has been reached
        if (lPos >= lTarget - DEVIATION && rPos >= rTarget - DEVIATION) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();

    // update grid position and walls
    updatePosition();
    updateWalls();
}

void Epuck::moveRobot(unsigned int numberOfMotions) {
    double lTarget = motorPosition[LMOTOR] + DIST_FORWARD * numberOfMotions;
    double rTarget = motorPosition[RMOTOR] + DIST_FORWARD * numberOfMotions;

    // set motor speed
    motors[LMOTOR]->setVelocity(SPEED_FORWARD);
    motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    // set new position
    motors[LMOTOR]->setPosition(lTarget);
    motors[RMOTOR]->setPosition(rTarget);
    
    // wait for robot to reach position
    while (robot->step(TIME_STEP) != -1) {
        getPosSensorReadings();
        double lPos = posSensorReadings[LMOTOR];
        double rPos = posSensorReadings[RMOTOR];
        // check if position has been reached
        if (lPos >= lTarget - DEVIATION && rPos >= rTarget - DEVIATION) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();

    // update grid position and walls
    updatePosition();
    updateWalls();
}

void Epuck::moveRobot(double numberOfMotions, bool moveHalfGrid, bool avoidingObstacle){
    double lTarget = motorPosition[LMOTOR] + DIST_FORWARD * (numberOfMotions);
    double rTarget = motorPosition[RMOTOR] + DIST_FORWARD * (numberOfMotions);
    if (moveHalfGrid) {
        lTarget += DIST_FORWARD / 2;
        rTarget += DIST_FORWARD / 2;
    }
    // set motor speed
    motors[LMOTOR]->setVelocity(SPEED_FORWARD);
    motors[RMOTOR]->setVelocity(SPEED_FORWARD);
    // set new position
    motors[LMOTOR]->setPosition(lTarget);
    motors[RMOTOR]->setPosition(rTarget);
    int num =0;
    // wait for robot to reach position
    while (robot->step(TIME_STEP) != -1) {
        getPosSensorReadings();
        double lPos = posSensorReadings[LMOTOR];
        double rPos = posSensorReadings[RMOTOR];

        getDistSensorReadings();
        // calculate how many steps left 
        if (avoidingObstacle == false) {
            num = round((lTarget - lPos)/DIST_FORWARD);
            correctRobot();
        }
        // if front has obstacles
        if(distSensorReadings[FRONTLEFT] < WALL_DETECTED && distSensorReadings[FRONT] == WALL_DETECTED && avoidingObstacle == false) {
            cout << "YESSS123" << endl;
            avoidObstacles(FRONTLEFT);
            getPosSensorReadings();
            lTarget = posSensorReadings[LMOTOR] + DIST_FORWARD * (num-1);
            rTarget = posSensorReadings[RMOTOR] + DIST_FORWARD * (num-1);
            motors[LMOTOR]->setPosition(lTarget);
            motors[RMOTOR]->setPosition(rTarget);
            
        } else if (distSensorReadings[FRONTRIGHT] < WALL_DETECTED && distSensorReadings[FRONT] == WALL_DETECTED && avoidingObstacle == false) {
            cout << "YESSS" << endl;
            avoidObstacles(FRONTRIGHT);
            getPosSensorReadings();
            lTarget = posSensorReadings[LMOTOR] + DIST_FORWARD * (num-1);
            rTarget = posSensorReadings[RMOTOR] + DIST_FORWARD * (num-1);
            motors[LMOTOR]->setPosition(lTarget);
            motors[RMOTOR]->setPosition(rTarget);
        }
        // check if position has been reached
        if (lPos >= lTarget - DEVIATION && rPos >= rTarget - DEVIATION) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();

    // update grid position and walls
    updatePosition();
    updateWalls();
}

void Epuck::rotateRobot(bool smoothGridTurn){
    if (!smoothGridTurn) {
        rotateRobotWithCommand(currCommand);
    } else {
        double lTarget, rTarget;
        // adjust for direction
        if (currCommand == 'R') {
            lTarget = motorPosition[LMOTOR] + LARGE_ROTATE;
            rTarget = motorPosition[RMOTOR] + SMALL_ROTATE;
            // set motor speed
            motors[LMOTOR]->setVelocity(SPEED_ROTATE);
            motors[RMOTOR]->setVelocity(SPEED_ROTATE * LR_WHEEL_ROTATE_RATIO);
        } else {
            lTarget = motorPosition[LMOTOR] + SMALL_ROTATE;
            rTarget = motorPosition[RMOTOR] + LARGE_ROTATE;
            // set motor speed
            motors[LMOTOR]->setVelocity(SPEED_ROTATE * LR_WHEEL_ROTATE_RATIO);
            motors[RMOTOR]->setVelocity(SPEED_ROTATE);
        }
        
        // set new position
        motors[LMOTOR]->setPosition(lTarget);
        motors[RMOTOR]->setPosition(rTarget);

        // wait for robot to reach position
        while (robot->step(TIME_STEP) != -1) {
            getPosSensorReadings();
            double lPos = posSensorReadings[RMOTOR];
            double rPos = posSensorReadings[RMOTOR];
            // check if position has been reached, set break condiion
            if (lPos > rTarget - DEVIATION && rPos > rTarget - DEVIATION) {
                motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
                motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
                break;
            }
        }
        getPosSensorReadings();
        // update grid position and walls
        updateHeading();
        updateWalls();

        // adjust heading 
        getIMUReadings();
        cout << yaw << endl;
        adjustRotation();

    }
}

void Epuck::adjustRotation() {

    //North      East     West      South
    // >0        >-1.57   >1.57     >-3.14
    // <0        <-1.57   <1.57     <3.14
    cout << "IN " << endl;
    char direction[4] = {'N','E','W','S'};
    double biggerValues[4] = {0,-PI/2,PI/2,-PI};
    double smallerValues[4] = {0,-PI/2,PI/2,PI};
    int i=0;
    int j=0;
    double lTarget, rTarget;
    lTarget = motorPosition[LMOTOR];
    rTarget = motorPosition[RMOTOR];
    while (robot->step(TIME_STEP) != -1) {
        getIMUReadings();
        cout << yaw << endl;
        for (i=0;i<3;i++) {
            if (heading == direction[i]) {
                if (yaw > biggerValues[i]) {
                    // turn right
                    cout <<"TURN RIGHT " << endl;
                    lTarget = lTarget + DIST_ROTATE/180;
                    rTarget = rTarget - DIST_ROTATE/180;
                    motors[LMOTOR]->setPosition(lTarget);
                    motors[RMOTOR]->setPosition(rTarget);
                    j=i;
                } else if (yaw < smallerValues[i]) {
                    // turn left
                    cout <<"TURN LEFT " << endl;
                    lTarget = lTarget - DIST_ROTATE/180;
                    rTarget = rTarget + DIST_ROTATE/180;
                    motors[LMOTOR]->setPosition(lTarget);
                    motors[RMOTOR]->setPosition(rTarget);
                    j=i;
                }
            }
        }
        if (heading == direction[3]) {
            if (yaw > biggerValues[3] && yaw < 0) {
                cout <<"TURN RIGHT " << endl;
                lTarget = lTarget + DIST_ROTATE/180;
                rTarget = rTarget - DIST_ROTATE/180;
                motors[LMOTOR]->setPosition(lTarget);
                motors[RMOTOR]->setPosition(rTarget);
                j=3;
            } else if (yaw < smallerValues[3] && yaw > 0) {
                cout <<"TURN LEFT " << endl;
                lTarget = lTarget - DIST_ROTATE/180;
                rTarget = rTarget + DIST_ROTATE/180;
                motors[LMOTOR]->setPosition(lTarget);
                motors[RMOTOR]->setPosition(rTarget);
                j=3;
            }
        }
        i=0;
        if (abs(yaw - biggerValues[j]) < DEVIATION_YAW || abs(yaw - smallerValues[j]) < DEVIATION_YAW) {
            getIMUReadings();
            getPosSensorReadings(); 
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
}

void Epuck::rotateRobotWithCommand(char direction) {
    cout << direction << endl;
    getPosSensorReadings();
    double lTarget, rTarget;
    if (direction == 'R') {
        lTarget = posSensorReadings[LMOTOR] + DIST_ROTATE;
        rTarget = posSensorReadings[RMOTOR] - DIST_ROTATE;
    } else {
        lTarget = posSensorReadings[LMOTOR] - DIST_ROTATE;
        rTarget = posSensorReadings[RMOTOR] + DIST_ROTATE;
    }
    bool breakCondition = false;

    // adjust for direction

    // set motor speed
    motors[LMOTOR]->setVelocity(SPEED_ROTATE);
    motors[RMOTOR]->setVelocity(SPEED_ROTATE);
    // set new position
    motors[LMOTOR]->setPosition(lTarget);
    motors[RMOTOR]->setPosition(rTarget);

    // wait for robot to reach position
    while (robot->step(TIME_STEP) != -1) {
        getPosSensorReadings();
        double rPos = posSensorReadings[RMOTOR];
        // check if position has been reached, set break condiion
        direction == 'R' ? (breakCondition = (rPos <= rTarget + DEVIATION))
                           : (breakCondition = (rPos >= rTarget - DEVIATION));

        if (breakCondition) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();
    // update grid position and walls
    updateHeading();
    updateWalls();
}

void Epuck::setInitStatus() {
    readPath();              // extract commands from file
    enableSensors();         // enable sensors
    updateWalls();           // get wall status
    getPosSensorReadings();  // get current position coordinates
    //set initial position, covert from char to int
    gridPosition[ROW] = commands[INIT_ROW] - '0';
    gridPosition[COLUMN] = commands[INIT_COL] - '0';
    //set coordinate position
    motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
    motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
    //set heading
    heading = commands[INIT_HEAD];
}

// display walls, position, heading
void Epuck::displayStatus() {
    cout << "1Step: " << setw(2) << setfill('0') << (currCommandIndex - INITIAL_COMMAND)
         << ", Row: " << gridPosition[ROW]
         << ", Column: " << gridPosition[COLUMN]
         << ", Heading: " << heading
         << ", Left Wall: " << walls[LEFT]
         << ", Front Wall: " << walls[FRONT]
         << ", Right Wall: " << walls[RIGHT]
         << endl;
}

void Epuck::smoothPath() {
    for (unsigned int i = 0; i < commands.length(); ++i) {
        if (commands[i] == 'L' || commands[i] == 'R') {
            commands = commands.erase(i - 1, 1);
        }
    }
    endCommand = commands.length();
}


/* 
 * Runs simulation by incrementing through provided commands
 * and navigating robot
*/
void Epuck::runSim(bool smooth) {
    setInitStatus();
    displayStatus();

    // initially, turn to the correct heading, then go half grid to get started
    // generate a smooth path
    if (smooth) {
        smoothPath();
        cout << commands << endl;
        currCommand = commands[currCommandIndex];
        rotateRobotWithCommand(currCommand);
        moveRobot(0, true, false);
        while (true) {
            unsigned int numberOfMotions = 0;
            currCommand = commands[currCommandIndex];
            // the following code is voodoo magic, do not attempt to understand
            cout << currCommandIndex << ": " << commands[currCommandIndex] << endl;
            if (currCommand == 'F') {
                while (commands[currCommandIndex] == 'F') {
                    ++numberOfMotions;
                    ++currCommandIndex;
                }
                (currCommandIndex == endCommand) ? moveRobot(numberOfMotions - 1, true, false) : moveRobot(numberOfMotions, false, false);
            } else {
                currCommand = commands[currCommandIndex];
                ++currCommandIndex;
                rotateRobot(smooth);
            }


            // if final command executed
            if (currCommandIndex == endCommand) break;
        }
    } else {
        while(robot->step(TIME_STEP) != -1) {
            currCommand = commands[currCommandIndex];
            currCommand == 'F' ? moveRobot() : rotateRobotWithCommand(currCommand);
            currCommandIndex += 1;   

            displayStatus();
            
            // if final command executed
            if (currCommandIndex == endCommand) break;
        }
    }

    cout << "Done - Path plan executed!" << endl;
}