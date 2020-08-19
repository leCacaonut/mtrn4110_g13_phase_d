/* Adapted from phase A
*  Trajectory planning for movement of robot
*  Written by Thinesh Manisekaran
*  Edited by Gordon Chen, Mei Yan Tang
*/
#include "MapBuild.h"

#define KEY_RIGHT  316
#define KEY_LEFT   314
#define KEY_UP     315
#define KEY_DOWN   317 

void dispKey(int key);

// constructor creates new robot and sets sensors
Epuck::Epuck(int start_row, int start_col, char start_heading) {
    startPosition[ROW] = start_row;
    startPosition[COLUMN] = start_col;
    startHeading = start_heading;
    
    robot = new Robot();
    genMap();
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
    // IMU
    IMU = robot->getInertialUnit("inertial unit");
    // keyboard
    keyboard = new Keyboard();
    keyboard->enable(TIME_STEP);
    enableSensors();         // enable sensors
    getPosSensorReadings();  // get current motor position

    // //set motor position just in case weird things
    motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
    motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
    
    heading = start_heading;

    m.numExplored = 0;

    
    updateWalls();           // get wall status

    //set default heading and location

    gridPosition[ROW] = 0;
    gridPosition[COLUMN] = 0;  
         
    dispMap();
}

Epuck::~Epuck() {
    delete robot;
};

// =======================================================
// MAP BUILD FUNCTIONS
// =======================================================

void Epuck::exploreManual() {
    int key = 0;
    
    while (robot->step(TIME_STEP) != -1) {
        key = keyboard->getKey();
        
        if (key != -1) {
            dispKey(key);
            
            if (checkPossible(key)) { 
                (key == KEY_UP) ? moveRobot() : mapRotateRobot(key);   
                dispMap();
            } else {
                cout << "Invalid Command" << endl;
            }
        }
        
        if (m.numExplored == NUM_ROWS * NUM_COLS) {
            if (gridPosition[ROW] == startPosition[ROW] && gridPosition[COLUMN] == startPosition[COLUMN]) {
                break;
            } else {
                cout << "Exploring Complete - Return To Start Position [" << startPosition[0] << ", " << startPosition[1] << "]" << endl;
            }
        }
    }    
}

bool Epuck::checkPossible(int key) {    
    int currRowIndex = 2*gridPosition[ROW] + 1;
    int currColIndex = 4*gridPosition[COLUMN] + 2;
    
    bool up    = (m.map[currRowIndex-1][currColIndex] == '-');
    bool down  = (m.map[currRowIndex+1][currColIndex] == '-');
    bool left  = (m.map[currRowIndex][currColIndex-2] == '|');
    bool right = (m.map[currRowIndex][currColIndex+2] == '|');

    switch (heading) {
        case 'N':
            if (key == KEY_DOWN)           return false;
            if (key == KEY_UP && up)       return false;            
            break;
        case 'S':
            if (key == KEY_DOWN)           return false;
            if (key == KEY_UP && down)     return false;             
            break;
        case 'E':
            if (key == KEY_DOWN)           return false;
            if (key == KEY_UP && right)    return false;            
            break;
        case 'W':
            if (key == KEY_DOWN)           return false;
            if (key == KEY_UP && left)     return false; 
          
            break;
        default:
            cout << "Invalid Heading in updatePosition" << endl;
            break;
    }  
    
    
    return true;
}

void Epuck::genMap() {
    string topBottomWall = " --- --- --- --- --- --- --- --- --- ";
    string endSquareWall = "                                     ";
    string midSquare     = "| X   X   X   X   X   X   X   X   X |";
  
    m.map[MAP_ROWS - 1] = (m.map[0] = topBottomWall);
  
    for (int i = 1; i < MAP_ROWS - 1; i++) {
        if   (i%2) {m.map[i] = midSquare;}
        else       {m.map[i] = endSquareWall;}; 
    }
}

void Epuck::dispMap() {

    cout << "Current Status" << endl << endl;

    for (int i = 0; i < MAP_ROWS; i++) {
        cout << m.map[i] << endl;
    }  
    
    cout << "Num Explored = " << m.numExplored << endl;
}


// get sensor readings and update if walls are present or not
void Epuck::updateWalls() {
    // update sensor readings
    getDistSensorReadings();    
    int currRowIndex = 2*gridPosition[ROW] + 1;
    int currColIndex = 4*gridPosition[COLUMN] + 2;

    switch (heading) {
        case 'N':
            if (distSensorReadings[LEFT] < WALL_DETECTED) m.map[currRowIndex][currColIndex-2] = '|';
            if (distSensorReadings[RIGHT] < WALL_DETECTED) m.map[currRowIndex][currColIndex+2] = '|';
            if (distSensorReadings[FRONT] < WALL_DETECTED) {
                for (int i = 0; i < 3; i++) {
                    m.map[currRowIndex-1][currColIndex-1+i] = '-';
                }
            }
                        
            break;
        case 'S':
            if (distSensorReadings[LEFT] < WALL_DETECTED) m.map[currRowIndex][currColIndex+2] = '|';
            if (distSensorReadings[RIGHT] < WALL_DETECTED) m.map[currRowIndex][currColIndex-2] = '|';
            if (distSensorReadings[FRONT] < WALL_DETECTED) {
                for (int i = 0; i < 3; i++) {
                    m.map[currRowIndex+1][currColIndex-1+i] = '-';
                }
            }
            
            break;
        case 'E':
            if (distSensorReadings[LEFT] < WALL_DETECTED) { 
                for (int i = 0; i < 3; i++) {
                    m.map[currRowIndex-1][currColIndex-1+i] = '-';
                }
            }
            
            if (distSensorReadings[RIGHT] < WALL_DETECTED) {
                for (int i = 0; i < 3; i++) {
                    m.map[currRowIndex+1][currColIndex-1+i] = '-';
                }
            }
            
            if (distSensorReadings[FRONT] < WALL_DETECTED) m.map[currRowIndex][currColIndex+2] = '|';
                        
            break;
        case 'W':
            if (distSensorReadings[LEFT] < WALL_DETECTED) { 
                for (int i = 0; i < 3; i++) {
                    m.map[currRowIndex+1][currColIndex-1+i] = '-';
                }
            }
            
            if (distSensorReadings[RIGHT] < WALL_DETECTED) {
                for (int i = 0; i < 3; i++) {
                    m.map[currRowIndex-1][currColIndex-1+i] = '-';
                }
            }
            
            if (distSensorReadings[FRONT] < WALL_DETECTED) m.map[currRowIndex][currColIndex-2] = '|';
            
            break;
        default:
            cout << "Invalid Heading in updatePosition" << endl;
            break;
    }

    if (m.map[currRowIndex][currColIndex] == 'X') {
        m.numExplored++;
    }
    
    bool start = (gridPosition[ROW] == startPosition[ROW] && gridPosition[COLUMN] == startPosition[COLUMN]);
    if (m.numExplored == 1 || start) {
        switch (startHeading) {
            case 'N':
                m.map[currRowIndex][currColIndex] = '^';
                break;
            case 'S':
                m.map[currRowIndex][currColIndex] = 'v';
                break;
            case 'E':
                m.map[currRowIndex][currColIndex]  = '>';
                break;
            case 'W':
                m.map[currRowIndex][currColIndex]  = '<';
                break;
            default:
                cout << "Invalid Heading in mapUpdateHeading" << endl;
                break;
        }
    } else {
      m.map[currRowIndex][currColIndex] = ' ';
    }
}

// update current heading based on left or right key
void Epuck::mapUpdateHeading(int key) {
    switch (heading) {
        case 'N':
            (key == KEY_RIGHT) ? heading = 'E' : heading = 'W';
            break;
        case 'S':
            (key == KEY_RIGHT) ? heading = 'W' : heading = 'E';
            break;
        case 'E':
            (key == KEY_RIGHT) ? heading = 'S' : heading = 'N';
            break;
        case 'W':
            (key == KEY_RIGHT) ? heading = 'N' : heading = 'S';
            break;
        default:
            cout << "Invalid Heading in mapUpdateHeading" << endl;
            break;
    }
}

void Epuck::mapRotateRobot(int command) {
    double lTarget, rTarget;
    if (command == KEY_RIGHT) {
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
        command == KEY_RIGHT ? (breakCondition = (rPos <= rTarget + DEVIATION))
                             : (breakCondition = (rPos >= rTarget - DEVIATION));

        if (breakCondition) {
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
    getPosSensorReadings();
    // update grid position and walls
    // Scout << "ROTATE" << endl;
    // adjust rotation
    mapUpdateHeading(command);
    updateWalls();
    adjustRotation();

}

void Epuck::adjustRotation() {
    //North      East     South    West
    double rBiggerValues[4] = {PI / 4, -PI / 4, (-3 * PI) / 4, (3 * PI) / 4};
    double rSmallerValues[4] = {0, -PI / 2, -PI, PI / 2};
    double lBiggerValues[4] = {0, -PI / 2, PI, PI / 2};
    double lSmallerValues[4] = {-PI / 4, -3 * PI / 4, 3 * PI / 4, PI / 4};
    double biggerValues[4] = {0, -PI / 2, -PI, PI / 2};
    double smallerValues[4] = {0, -PI / 2, PI, PI / 2};
    int i = 0;
    int j = 0;
    double lTarget, rTarget;
    lTarget = motorPosition[LMOTOR];
    rTarget = motorPosition[RMOTOR];
    while (robot->step(TIME_STEP) != -1) {
        getIMUReadings();
        for (i = 0; i < 4; i++) {
            if (yaw < rBiggerValues[i] && yaw > rSmallerValues[i]) {
                // cout << "Adjusting heading to the RIGHT " << endl;
                lTarget = lTarget + DIST_ROTATE / 180;
                rTarget = rTarget - DIST_ROTATE / 180;
                motors[LMOTOR]->setPosition(lTarget);
                motors[RMOTOR]->setPosition(rTarget);
                j = i;
            } else if (yaw < lBiggerValues[i] && yaw > lSmallerValues[i]) {
                // cout << "Adjusting heading to the LEFT " << endl;
                lTarget = lTarget - DIST_ROTATE / 180;
                rTarget = rTarget + DIST_ROTATE / 180;
                motors[LMOTOR]->setPosition(lTarget);
                motors[RMOTOR]->setPosition(rTarget);
                j = i;
            }
        }
        i = 0;
        if (abs(yaw - biggerValues[j]) < DEVIATION_YAW || abs(yaw - smallerValues[j]) < DEVIATION_YAW) {
            getIMUReadings();
            getPosSensorReadings();
            // cout << "Final Yaw " << yaw << endl;
            motorPosition[LMOTOR] = posSensorReadings[LMOTOR];
            motorPosition[RMOTOR] = posSensorReadings[RMOTOR];
            break;
        }
    }
}

void Epuck::writeMap() {
    cout << "File: ./MapFound.txt" << endl;
    cout << "Path: " << m.map << endl;
  
    ofstream Path;
    Path.open(MAP_FILE_NAME);
    
    for (int i = 0; i < MAP_ROWS; i++) {
        Path << m.map[i] << "\n";
    }
    
    Path.close();
}

// =======================================================
// =======================================================

// =======================================================
// MANUAL MOVE FUNCTIONS
// =======================================================

void dispKey(int key) {
    switch (key) {
       case KEY_UP:
          cout << "Keypress = [" << key <<"], INSTRUCTION = [GO FORWARD]" << endl;
 
          break;
      case KEY_DOWN:
          cout << "Keypress = [" << key <<"], INSTRUCTION = [GO BACK]" << endl;
   
          break;
      case KEY_LEFT:
          cout << "Keypress = [" << key <<"], INSTRUCTION = [ROTATE LEFT]" << endl;
   
          break;
      case KEY_RIGHT:
          cout << "Keypress = [" << key <<"], INSTRUCTION = [ROTATE RIGHT]" << endl;
          
          break;
      default:
          cout << "Keypress = [" << key <<"], Invalid Heading in updatePosition" << endl;
          break;
    }  
}

bool checkCorrespond(int key, char command) {
    switch (command) {
        case 'F':
            if (key == KEY_UP) return true;
            break;
        case 'R':
            if (key == KEY_RIGHT) return true;
            break;
        case 'L':
            if (key == KEY_LEFT) return true;
            break;        
        default:
            return false;
            break;    
    }
    
    return false;
}

void Epuck::moveManual() {
    readPath();
    
    char reqHeading = commands[2];
    rotateReq(reqHeading);
    int key = 0;

    currCommand = commands[currCommandIndex];
    cout << "CURRENT INSTRUCTION = [" << currCommand << "]" << endl;

    while (robot->step(TIME_STEP) != -1) {
        key = keyboard->getKey();
        
        if (key != -1) {
            if (checkPossible(key) && checkCorrespond(key, currCommand)) { 
                dispKey(key);           
                currCommandIndex += 1;       
                (key == KEY_UP) ? moveRobot() : mapRotateRobot(key); 
                if (currCommandIndex < endCommand) {
                    currCommand = commands[currCommandIndex];
                    cout << "CURRENT INSTRUCTION = [" << currCommand << "]" << endl;
                }
            } else {
                cout << "Invalid Command, expecting [" << currCommand << "]" << endl;     
            }
        }
        
        if (currCommandIndex == endCommand) {
            break;
        }
    }      
}


void Epuck::rotateReq(char req) {
    int key = 0;
    
    if (heading == req) return;
    
    cout << "Please Rotate To Required Heading [" << req << "]" << endl;
    
    while (robot->step(TIME_STEP) != -1) {
        if (heading == req) break;
        
        key = keyboard->getKey();
        
        if (key != -1) {
            dispKey(key);           
            if (checkPossible(key)) { 
                if (key != KEY_UP) { mapRotateRobot(key);}
                else               { cout << "Invalid Command" << endl;};   
            } else {
                cout << "Invalid Command" << endl;
            }
        }
    }   
}

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

void Epuck::moveRobot() {
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