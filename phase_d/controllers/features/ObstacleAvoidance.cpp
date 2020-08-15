/* Adapted from phase A
*  Trajectory planning for movement of robot
*
*
*/
#include "ObstacleAvoidance.h"

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
    // camera
    camera = robot->getCamera("camera");
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

    // get map
    ifstream MapFile(MAP_FILE_NAME);
    for (int i = 0; i<TOTAL_ROW ; i++) {
        getline(MapFile,map[i]);
        std::cout<<map[i]<<std::endl;
    }
    MapFile.close();
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
    // enable camera
    camera->enable(TIME_STEP);
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

        cout << "LEFT: " << distSensorReadings[LEFT] << endl;
        cout << "RIGHT: " <<distSensorReadings[RIGHT] << endl;
        cout << "FRONT: " <<distSensorReadings[FRONT] << endl;
        cout << "FRONTLEFT: " <<distSensorReadings[FRONTLEFT] << endl;
        cout << "FRONTRIGHT: " <<distSensorReadings[FRONTRIGHT] << endl;
        cout << endl;

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

// get sensor readings and update if walls are present or not
void Epuck::updateWalls() {
    // update sensor readings
    getDistSensorReadings();

    distSensorReadings[LEFT] < WALL_DETECTED ? walls[LEFT] = 'Y' : walls[LEFT] = 'N';
    distSensorReadings[RIGHT] < WALL_DETECTED ? walls[RIGHT] = 'Y' : walls[RIGHT] = 'N';
    distSensorReadings[FRONT] < WALL_DETECTED ? walls[FRONT] = 'Y' : walls[FRONT] = 'N';
}

// update current row and column position and heading
void Epuck::updatePosition(double numberOfMotions) {
    switch (heading) {
        case 'N':
            gridPosition[ROW] -= numberOfMotions*1;
            break;
        case 'S':
            gridPosition[ROW] += numberOfMotions*1;
            break;
        case 'E':
            gridPosition[COLUMN] += numberOfMotions*1;
            break;
        case 'W':
            gridPosition[COLUMN] -= numberOfMotions*1;
            break;
        default:
            cout << "Invalid Heading in updatePosition" << endl;
            break;
    }
    cout <<"row: " << gridPosition[ROW] << " col: " << gridPosition[COLUMN] << endl;
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
            rotateRobot(left);
            moveHalfStep();
            rotateRobot(right);
            moveRobot(0.75,true);
            rotateRobot(right);
            moveHalfStep();
            rotateRobot(left);
            motors[LMOTOR]->setVelocity(SPEED_FORWARD);
            motors[RMOTOR]->setVelocity(SPEED_FORWARD);
            break;
        } else if (obstacleLocation == FRONTLEFT) {
            // sequence for moving the robot when obstacle on front left
            rotateRobot(right);
            moveHalfStep();
            rotateRobot(left);
            moveRobot(0.75,true);
            rotateRobot(left);
            moveHalfStep();
            rotateRobot(right);
            motors[LMOTOR]->setVelocity(SPEED_FORWARD);
            motors[RMOTOR]->setVelocity(SPEED_FORWARD);
            break;

        }
    }
}

void Epuck::moveRobot(double numberOfMotions, bool avoidingObstacle) {
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
        getDistSensorReadings();
        // if front has obstacles
        if(distSensorReadings[FRONTLEFT] < WALL_DETECTED && distSensorReadings[FRONT] == WALL_DETECTED && avoidingObstacle == false) {
            avoidObstacles(FRONTLEFT);
            getPosSensorReadings();
            lTarget = posSensorReadings[LMOTOR] + DIST_FORWARD * (numberOfMotions-1);
            rTarget = posSensorReadings[RMOTOR] + DIST_FORWARD * (numberOfMotions-1);
            motors[LMOTOR]->setPosition(lTarget);
            motors[RMOTOR]->setPosition(rTarget);
            
        } else if (distSensorReadings[FRONTRIGHT] < WALL_DETECTED && distSensorReadings[FRONT] == WALL_DETECTED && avoidingObstacle == false) {
            avoidObstacles(FRONTRIGHT);
            getPosSensorReadings();
            lTarget = posSensorReadings[LMOTOR] + DIST_FORWARD * (numberOfMotions-1);
            rTarget = posSensorReadings[RMOTOR] + DIST_FORWARD * (numberOfMotions-1);
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
    updatePosition(numberOfMotions);
    updateWalls();
}

void Epuck::rotateRobot(char direction) {
    getPosSensorReadings();
    double lTarget = posSensorReadings[LMOTOR] + DIST_ROTATE;
    double rTarget = posSensorReadings[RMOTOR] + DIST_ROTATE;
    bool breakCondition = false;
    // adjust for direction
    direction == 'R' ? (rTarget -= 2 * DIST_ROTATE) : (lTarget -= 2 * DIST_ROTATE);

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
        direction == 'R' ? (breakCondition = (rPos <= rTarget + DEVIATION))
                           : (breakCondition = (rPos >= rTarget - DEVIATION));

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
    cout << "Step: " << setw(2) << setfill('0') << (currCommandIndex - INITIAL_COMMAND)
         << ", Row: " << gridPosition[ROW]
         << ", Column: " << gridPosition[COLUMN]
         << ", Heading: " << heading
         << ", Left Wall: " << walls[LEFT]
         << ", Front Wall: " << walls[FRONT]
         << ", Right Wall: " << walls[RIGHT]
         << endl;
}

/* 
 * Runs simulation by incrementing through provided commands
 * and navigating robot
*/
void Epuck::runSim() {
    setInitStatus();
    displayStatus();

    while (robot->step(TIME_STEP) != -1) {
        unsigned int numberOfMotions = 0;
        currCommand = commands[currCommandIndex];
        if (currCommand == 'F') {
            while (commands[currCommandIndex] == 'F') {
                ++numberOfMotions;
                ++currCommandIndex;
            }
            moveRobot(numberOfMotions,false);
        } else {
            currCommand = commands[currCommandIndex];
            ++currCommandIndex;
            rotateRobot(currCommand);
        }

        // displayStatus();

        // if final command executed
        if (currCommandIndex == endCommand) {
            break;
        }
    }

    cout << "Done - Path plan executed!" << endl;
}