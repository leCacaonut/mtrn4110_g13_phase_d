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
    // position sensors
    posSensors[LMOTOR] = robot->getPositionSensor("left wheel sensor");
    posSensors[RMOTOR] = robot->getPositionSensor("right wheel sensor");

    enableSensors();         // enable sensors
    updateWalls();           // get wall status
    getPosSensorReadings();  // get current motor position

    //set motor position just in case weird things
    motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
    motorPosition[RMOTOR] = posSensorReadings[RMOTOR];

    //set default heading and location
    heading = 'S';
    gridPosition[ROW] = 0;
    gridPosition[COLUMN] = 0;
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
    // enable position sensors
    posSensors[LMOTOR]->enable(TIME_STEP);
    posSensors[RMOTOR]->enable(TIME_STEP);
}

// get values from distance sensors
void Epuck::getDistSensorReadings() {
    while (robot->step(TIME_STEP) != -1) {
        // cout << "here" << endl;
        distSensorReadings[LEFT] = distSensors[LEFT]->getValue();
        distSensorReadings[RIGHT] = distSensors[RIGHT]->getValue();
        distSensorReadings[FRONT] = distSensors[FRONT]->getValue();

        // cout << distSensorReadings[FRONT] << endl;
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

void Epuck::updateHeading(char command) {
    switch (heading) {
        case 'N':
            (command == 'R') ? heading = 'E' : heading = 'W';
            break;
        case 'S':
            (command == 'R') ? heading = 'W' : heading = 'E';
            break;
        case 'E':
            (command == 'R') ? heading = 'S' : heading = 'N';
            break;
        case 'W':
            (command == 'R') ? heading = 'N' : heading = 'S';
            break;
        default:
            cout << "Invalid Heading in updateHeading" << endl;
            break;
    }
}

void Epuck::moveRobot() {
    double initialPosition = motorPosition[LMOTOR];
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
        getDistSensorReadings();
        double lPos = posSensorReadings[LMOTOR];
        double rPos = posSensorReadings[RMOTOR];
        // check if position has been reached
        bool collision = distSensorReadings[FRONT] < WALL_DETECTED * 0.9;
        // cout << distSensorReadings[FRONT] << ":" << COLLISION << ":" << collision << endl;
        if ((lPos >= lTarget - DEVIATION && rPos >= rTarget - DEVIATION) || collision) {
            motors[LMOTOR]->setPosition(posSensorReadings[LMOTOR]);
            motors[RMOTOR]->setPosition(posSensorReadings[RMOTOR]);
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();

    // update grid position and walls
    // dont update position if the move is considered improper
    // cout << lTarget - initialPosition << "\t:\t" << (posSensorReadings[LMOTOR] - initialPosition) / 2 << endl;

    if (posSensorReadings[LMOTOR] - initialPosition > (lTarget - initialPosition) / 3) {
        updatePosition();
    }
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

void Epuck::moveRobot(unsigned int numberOfMotions, bool moveHalfGrid) {
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

void Epuck::rotateRobot(bool smoothGridTurn) {
    if (!smoothGridTurn) {
        rotateRobot();
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
    }
}

void Epuck::rotateRobot() {
    double lTarget, rTarget;
    if (currCommand == 'R') {
        lTarget = motorPosition[LMOTOR] + DIST_ROTATE;
        rTarget = motorPosition[RMOTOR] - DIST_ROTATE;
    } else {
        lTarget = motorPosition[LMOTOR] - DIST_ROTATE;
        rTarget = motorPosition[RMOTOR] + DIST_ROTATE;
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
        currCommand == 'R' ? (breakCondition = (rPos <= rTarget + DEVIATION))
                           : (breakCondition = (rPos >= rTarget - DEVIATION));

        if (breakCondition) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();
    // update grid position and walls
    updateWalls();
}

void Epuck::rotateRobot(char command) {
    double lTarget, rTarget;
    if (command == 'R') {
        lTarget = motorPosition[LMOTOR] + DIST_ROTATE;
        rTarget = motorPosition[RMOTOR] - DIST_ROTATE;
    } else if (command == 'L') {
        lTarget = motorPosition[LMOTOR] - DIST_ROTATE;
        rTarget = motorPosition[RMOTOR] + DIST_ROTATE;
    } else {
        cout << "Unknown command" << endl;
        return;
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
    // while (robot->step(TIME_STEP) != -1) {
    while(true) {
        getPosSensorReadings();
        double rPos = posSensorReadings[RMOTOR];
        // check if position has been reached, set break condiion
        command == 'R' ? (breakCondition = (rPos <= rTarget + DEVIATION))
                        : (breakCondition = (rPos >= rTarget - DEVIATION));

        if (breakCondition) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();
    // update grid position and walls
    updateHeading(command);
    updateWalls();
}

void Epuck::setHeading(char h) {
    heading = h;
}

void Epuck::setPosition(int pos[2]) {
    gridPosition[ROW] = pos[ROW];
    gridPosition[COLUMN] = pos[COLUMN];
}

void Epuck::setPosition(vector<int> pos) {
    gridPosition[ROW] = pos[ROW];
    gridPosition[COLUMN] = pos[COLUMN];
}

char* Epuck::getWalls() {
    return walls;
}

char Epuck::getHeading() {
    return heading;
}

int* Epuck::getPosition() {
    return gridPosition;
}

void Epuck::getCommands() {
    readPath();              // extract commands from file
    //set initial position, covert from char to int
    gridPosition[ROW] = commands[INIT_ROW] - '0';
    gridPosition[COLUMN] = commands[INIT_COL] - '0';
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

void Epuck::smoothPath() {
    for (unsigned int i = 0; i < commands.length(); ++i) {
        if (commands[i] == 'L' || commands[i] == 'R') {
            commands = commands.erase(i - 1, 1);
        }
    }
    endCommand = commands.length();
}

void Epuck::followWallStep() {
    static bool turned = true;
    updateWalls();
    // if (walls[RIGHT] == 'Y') {
    //     turned = true;
    //     if (walls[FRONT] == 'N') {
    //         moveRobot();
    //     } else if (walls[FRONT] == 'Y') {
    //         rotateRobot('L');
    //     }
    // } else if (walls[RIGHT] == 'N') {
    //     if (turned == true) {
    //         rotateRobot('R');
    //         turned = false;
    //     } else {
    //         moveRobot();
    //         turned = true;
    //     }
    // }
    if (walls[LEFT] == 'Y') {
        turned = true;
        if (walls[FRONT] == 'N') {
            moveRobot();
        } else if (walls[FRONT] == 'Y') {
            rotateRobot('R');
        }
    } else if (walls[LEFT] == 'N') {
        if (turned == true) {
            rotateRobot('L');
            turned = false;
        } else {
            moveRobot();
            turned = true;
        }
    }
}

/* 
 * Runs simulation by incrementing through provided commands
 * and navigating robot
*/
void Epuck::runSim(bool smooth) {
    getCommands();
    displayStatus();

    // initially, turn to the correct heading, then go half grid to get started
    // generate a smooth path
    if (smooth) {
        smoothPath();
        // cout << commands << endl;
        currCommand = commands[currCommandIndex - 1];
        
        if (currCommand == 'L' || currCommand == 'R') rotateRobot();
        moveRobot(0, true);

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
                (currCommandIndex == endCommand) ? moveRobot(numberOfMotions - 1, true) : moveRobot(numberOfMotions, false);
            } else {
                currCommand = commands[currCommandIndex];
                ++currCommandIndex;
                rotateRobot(smooth);
            }

            // if final command executed
            if (currCommandIndex == endCommand) break;
        }
        
    } else {
        while (robot->step(TIME_STEP) != -1) {
            currCommand = commands[currCommandIndex];
            if (currCommand == 'F') moveRobot();
            else rotateRobot();

            currCommandIndex += 1;

            displayStatus();

            // if final command executed
            if (currCommandIndex == endCommand) break;
        }
    }

    cout << "Done - Path plan executed!" << endl;
}