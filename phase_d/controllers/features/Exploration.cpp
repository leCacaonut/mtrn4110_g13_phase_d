/*
*  Exploring a map
*
*
*/

// includes
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// Namespaces
using namespace std;

#include "Exploration.h"
#ifndef PATHFINDING_CPP
#define PATHFINDING_CPP
#include "../phases/phase_b.cpp"  // pathfinding
#endif
// Defined path to command file provided
#ifndef MAP_FILE_NAME
#define MAP_FILE_NAME "../../Map.txt"
#endif

#ifndef ROW
#define ROW 0
#endif
#ifndef COL
#define COL 1
#endif

#define LEFT 0
#define RIGHT 1
#define FRONT 2

#define TOP_LEFT 0
#define TOP_RIGHT 1

ExploreMap::ExploreMap() {
    exploredMap.resize(1, vector<bool>(1, false));
    hWalls.resize(2, vector<bool>(1, false));
    vWalls.resize(1, vector<bool>(2, false));
    setDefaultGoal();
}

ExploreMap::ExploreMap(int row, int col) {
    exploredMap.resize(row, vector<bool>(col, false));
    hWalls.resize(row + 1, vector<bool>(col, false));
    vWalls.resize(row, vector<bool>(col + 1, false));
    setDefaultGoal();
}

ExploreMap::~ExploreMap() {}

void ExploreMap::setDefaultGoal() {
    int* s = {getMapSize()};
    if (s[ROW] % 2 == 1) --s[ROW];
    if (s[COL] % 2 == 1) --s[COL];
    goal[ROW] = s[ROW] / 2;
    goal[COL] = s[COL] / 2;
}

void ExploreMap::setGoal(int row, int col) {
    goal[ROW] = row;
    goal[COL] = col;
}

void ExploreMap::setExplored(int v[2], char heading, char* walls) {
    int& row = v[0];
    int& col = v[1];
    if (abs(row) >= getMapSize()[ROW] || abs(col) >= getMapSize()[COL]) {
        resize2DVector(exploredMap, abs(row) + 1, abs(col) + 1);
        resize2DVector(hWalls, abs(row) + 2, abs(col) + 1);
        resize2DVector(vWalls, abs(row) + 1, abs(col) + 2);
    }
    exploredMap[abs(row)][abs(col)] = true;
    setWalls(v, heading, walls);
}

int* ExploreMap::getMapSize() {
    mapSize[ROW] = exploredMap.size();
    mapSize[COL] = exploredMap[0].size();
    return mapSize;
}

int* ExploreMap::getGoal() {
    return goal;
}

bool ExploreMap::getExplored(int row, int col) {
    return exploredMap[row][col];
}

vector<vector<bool>> ExploreMap::getExplored() {
    return exploredMap;
}

vector<vector<bool>> ExploreMap::getHWalls() {
    return hWalls;
}

vector<vector<bool>> ExploreMap::getVWalls() {
    return vWalls;
}

int* ExploreMap::size2DVector(int* s, vector<vector<bool>> v) {
    s[0] = v.size();
    s[1] = v[0].size();
    return s;
}

void ExploreMap::resize2DVector(vector<vector<bool>>& v, unsigned int numRows, unsigned int numCols) {
    //change v.push_back(vector<bool>(numCols, false)) -> v.push_back(vector<bool>(v[0].size(), false));
    while (v.size() < numRows) {
        v.push_back(vector<bool>(v[0].size(), false));
    }

    for (unsigned int i = 0; i < v.size(); ++i) {
        while (v[i].size() < numCols) {
            v[i].push_back(false);
        }
    }
}

void ExploreMap::print2DVector(vector<vector<bool>> v) {
    // cout << "Vector:\n";
    for (unsigned int i = 0; i < v.size(); i++) {
        for (unsigned int j = 0; j < v[i].size(); j++) {
            cout << v[i][j];
        }
        cout << "\n";
    }
    cout << "\n";
}

void ExploreMap::setWalls(int position[2], char heading, char* walls) {
    switch (heading) {
        case 'N':
            if (position[1] < 0) {
                if (walls[LEFT] == 'Y') addVWall(position[0], abs(position[1]) + 1);
                if (walls[RIGHT] == 'Y') addVWall(position[0], position[1]);
                if (walls[LEFT] == 'N') removeVWall(position[0], abs(position[1]) + 1);
                if (walls[RIGHT] == 'N') removeVWall(position[0], position[1]);
            } else {
                if (walls[LEFT] == 'Y') addVWall(position[0], position[1]);
                if (walls[RIGHT] == 'Y') addVWall(position[0], position[1] + 1);
                if (walls[LEFT] == 'N') removeVWall(position[0], position[1]);
                if (walls[RIGHT] == 'N') removeVWall(position[0], position[1] + 1);
            }
            if (walls[FRONT] == 'Y') addHWall(position[0], position[1]);
            if (walls[FRONT] == 'N') removeHWall(position[0], position[1]);
            break;

        case 'S':
            if (position[1] < 0) {
                if (walls[LEFT] == 'Y') addVWall(position[0], position[1]);
                if (walls[RIGHT] == 'Y') addVWall(position[0], abs(position[1]) + 1);
                if (walls[LEFT] == 'N') removeVWall(position[0], position[1]);
                if (walls[RIGHT] == 'N') removeVWall(position[0], abs(position[1]) + 1);
            } else {
                if (walls[LEFT] == 'Y') addVWall(position[0], position[1] + 1);
                if (walls[RIGHT] == 'Y') addVWall(position[0], position[1]);
                if (walls[LEFT] == 'N') removeVWall(position[0], position[1] + 1);
                if (walls[RIGHT] == 'N') removeVWall(position[0], position[1]);
            }
            if (walls[FRONT] == 'N') removeHWall(position[0] + 1, position[1]);
            if (walls[FRONT] == 'Y') addHWall(position[0] + 1, position[1]);
            break;

        case 'W':
            if (position[1] < 0) {
                if (walls[FRONT] == 'Y') addVWall(position[0], abs(position[1]) + 1);
                if (walls[FRONT] == 'N') removeVWall(position[0], abs(position[1]) + 1);
            } else {
                if (walls[FRONT] == 'Y') addVWall(position[0], position[1]);
                if (walls[FRONT] == 'N') removeVWall(position[0], position[1]);
            }
            if (walls[LEFT] == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[RIGHT] == 'Y') addHWall(position[0], position[1]);
            if (walls[LEFT] == 'N') removeHWall(position[0] + 1, position[1]);
            if (walls[RIGHT] == 'N') removeHWall(position[0], position[1]);
            break;

        case 'E':
            if (position[1] < 0) {
                if (walls[FRONT] == 'Y' && position[1] <= 0) addVWall(position[0], position[1]);
                if (walls[FRONT] == 'N' && position[1] <= 0) removeVWall(position[0], position[1]);
            } else {
                if (walls[FRONT] == 'Y') addVWall(position[0], position[1] + 1);
                if (walls[FRONT] == 'N') removeVWall(position[0], position[1] + 1);
            }
            if (walls[LEFT] == 'Y') addHWall(position[0], position[1]);
            if (walls[RIGHT] == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[LEFT] == 'N') removeHWall(position[0], position[1]);
            if (walls[RIGHT] == 'N') removeHWall(position[0] + 1, position[1]);
            break;

        default:
            cout << "Failed to set walls" << endl;
    }
}

void ExploreMap::addHWall(int row, int col) {
    hWalls[abs(row)][abs(col)] = true;
}

void ExploreMap::addVWall(int row, int col) {
    vWalls[abs(row)][abs(col)] = true;
}

void ExploreMap::removeHWall(int row, int col) {
    hWalls[abs(row)][abs(col)] = false;
}

void ExploreMap::removeVWall(int row, int col) {
    vWalls[abs(row)][abs(col)] = false;
}

vector<vector<bool>> ExploreMap::flipMap(vector<vector<bool>> v) {
    int rowNum = v.size();
    int colNum = v[0].size();

    vector<vector<bool>> flipped = v;

    // flip map
    for (int i = 0; i < rowNum; i++) {
        for (int j = 0; j < colNum; j++) {
            flipped[i][j] = v[i][colNum - 1 - j];
            cout << flipped[i][j];
        }
        cout << endl;
    }
    return flipped;
}

vector<vector<bool>> ExploreMap::swapColumns(vector<vector<bool>> v) {
    int rowNum = v.size();

    for (int i = 0; i < rowNum; i++) {
        bool tmp = v[i][0];
        v[i][0] = v[i][1];
        v[i][1] = tmp;
    }

    return v;
}

bool ExploreMap::gridValid(int row, int col) {
    if ((hWalls[row][col] && 
    hWalls[row + 1][col] && 
    vWalls[row][col] && 
    vWalls[row][col + 1]) ||
    exploredMap[row][col]) {
        return false;
    } else {
        return true;
    }
}

bool ExploreMap::findEmptyGrid() {
    bool foundEmptyGrid = false;
    int rows = getMapSize()[ROW] - 1;
    int cols = getMapSize()[COL] - 1;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if(gridValid(i, j)) {
                emptyGrid[ROW] = i;
                emptyGrid[COL] = j;
                foundEmptyGrid = true;
                goto PRADATHEDEVIL;
            }
        }
    }
    PRADATHEDEVIL:
    // if no empty grids found
    return foundEmptyGrid;
}

template <typename T>
void ExploreMap::explore(T& robot) {
    vector<int> initialPosition{0, 0};
    char initialHeading = robot.getHeading();
    bool actualInitialPosition = TOP_LEFT;  // assume a default top left corner

    int* gridPosition = robot.getPosition();
    char* walls = robot.getWalls();
    char heading = robot.getHeading();
    setExplored(gridPosition, heading, walls);

    // make a first move without while loop conditional
    robot.followWallStep();
    gridPosition = robot.getPosition();
    walls = robot.getWalls();
    heading = robot.getHeading();
    // cout << ": " << gridPosition[ROW] << ": " << gridPosition[COL] << ": " << heading << endl;
    setExplored(gridPosition, heading, walls);

    do {
        robot.followWallStep();
        gridPosition = robot.getPosition();
        walls = robot.getWalls();
        heading = robot.getHeading();
        // cout << ": " << gridPosition[ROW] << ": " << gridPosition[COL] << ": " << heading << endl;
        setExplored(gridPosition, heading, walls);
        if (robot.getPosition()[1] < 0) {
            actualInitialPosition = TOP_RIGHT;
        }
    } while (!(initialPosition[ROW] == gridPosition[ROW] && initialPosition[COL] == gridPosition[COL]));
    // for (int i = 0; i < 21; ++i) {
    //     robot.followWallStep();
    //     gridPosition = robot.getPosition();
    //     walls = robot.getWalls();
    //     heading = robot.getHeading();
    //     cout << ": " << gridPosition[ROW] << ": " << gridPosition[COL] << ": " << heading << endl;
    //     setExplored(gridPosition, heading, walls);
    //     if (robot.getPosition()[1] < 0) {
    //         actualInitialPosition = TOP_RIGHT;
    //     }
    // } % use this loop for limited number of steps

    // Extra rotations to return to initial heading
    while (heading != initialHeading) {
        robot.rotateRobot('L');
        heading = robot.getHeading();
    }

    // if col is negative
    if (actualInitialPosition == TOP_RIGHT) {
        initialPosition[COL] = getMapSize()[COL] - 1;
        robot.setPosition(initialPosition);

        exploredMap = flipMap(exploredMap);
        hWalls = flipMap(hWalls);
        vWalls = swapColumns(vWalls);
        vWalls = flipMap(vWalls);
    }
    // cout << "Position: " << robot.getPosition()[ROW] << robot.getPosition()[COL] << endl;

    // map is now appropriately created
    // use a path planner to get to the closest empty grid
    // initial position will always be explored
    int* targetPosition = &emptyGrid[0];
    while (findEmptyGrid()) {
        cout << "Target: [" << targetPosition[0] << ", " << targetPosition[1] << "]" << endl;
        
        
        break;
    }
    
    // use a left wall follower.
    // if returned to the start position
    //      find closest unexplored grid
    //      use path finding to get there
}