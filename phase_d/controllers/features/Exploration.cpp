/*
*  Exploring a map
*
*
*/
#include "Exploration.h"
#ifndef PATHFINDING_CPP
#define PATHFINDING_CPP
#include "../phases/phase_b.cpp"
#endif

ExploreMap::ExploreMap() {
    explored.resize(1, vector<bool>(1, false));
    hWalls.resize(2, vector<bool>(1, false));
    vWalls.resize(1, vector<bool>(2, false));
    setDefaultGoal();
}

ExploreMap::ExploreMap(int row, int col) {
    explored.resize(row, vector<bool>(col, false));
    setDefaultGoal();
}

ExploreMap::~ExploreMap() {}

void ExploreMap::setDefaultGoal() {
    int *s = {getMapSize()};
    if (s[ROW] % 2 == 1) --s[ROW];
    if (s[COL] % 2 == 1) --s[COL];
    goal[ROW] = s[ROW] / 2;
    goal[COL] = s[COL] / 2;
}

void ExploreMap::setGoal(int row, int col) {
    goal[ROW] = row;
    goal[COL] = col;
}

void ExploreMap::setExplored(int row, int col) {
    explored[row][col] = true;
}

int* ExploreMap::getMapSize() {
    mapSize[ROW] = explored.size();
    mapSize[COL] = explored[0].size();
    return mapSize;
}

int* ExploreMap::getGoal() {
    return goal;
}

vector<vector<bool>> ExploreMap::getExplored() {
    return explored;
}

bool ExploreMap::getExplored(int row, int col) {
    return explored[row][col];
}

vector<vector<bool>> ExploreMap::getHWalls() {
    return hWalls;
}

vector<vector<bool>> ExploreMap::getVWalls() {
    return vWalls;
}

void ExploreMap::setWalls(int position[2], char heading, char* walls) {
    cout << "BEFORE: " << endl;
    print2DVector(hWalls);
    print2DVector(vWalls);

    switch (heading) {
        case 'N':
            if (walls[FRONT] == 'Y') addHWall(position[0], position[1]);
            if (walls[LEFT]  == 'Y') addVWall(position[0], position[1]);
            if (walls[RIGHT] == 'Y') addVWall(position[0], position[1] + 1);
            if (walls[FRONT] == 'N') removeHWall(position[0], position[1]);
            if (walls[LEFT]  == 'N') removeVWall(position[0], position[1]);
            if (walls[RIGHT] == 'N') removeVWall(position[0], position[1] + 1);
            break;
        case 'S':
            if (walls[FRONT] == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[LEFT]  == 'Y') addVWall(position[0], position[1] + 1);
            if (walls[RIGHT] == 'Y') addVWall(position[0], position[1]);
            if (walls[FRONT] == 'N') removeHWall(position[0], position[1]);
            if (walls[LEFT]  == 'N') removeVWall(position[0], position[1]);
            if (walls[RIGHT] == 'N') removeVWall(position[0], position[1] + 1);
            break;
        case 'W':
            if (walls[FRONT] == 'Y') addVWall(position[0], position[1]);
            if (walls[LEFT]  == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[RIGHT] == 'Y') addHWall(position[0], position[1]);
            if (walls[FRONT] == 'N') removeVWall(position[0], position[1]);
            if (walls[LEFT]  == 'N') removeHWall(position[0], position[1]);
            if (walls[RIGHT] == 'N') removeHWall(position[0], position[1] + 1);
            break;
        case 'E':
            if (walls[FRONT] == 'Y') addVWall(position[0], position[1] + 1);
            if (walls[LEFT]  == 'Y') addHWall(position[0], position[1]);
            if (walls[RIGHT] == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[FRONT] == 'N') removeVWall(position[0], position[1]);
            if (walls[LEFT]  == 'N') removeHWall(position[0], position[1]);
            if (walls[RIGHT] == 'N') removeHWall(position[0], position[1] + 1);
            break;
        default:
            cout << "Failed to set walls" << endl;
    }
    cout << "AFTER: " << endl;
    print2DVector(hWalls);
    print2DVector(vWalls);
}

// set a wall to be true
// if outside of the size of the map, pushback
void ExploreMap::addHWall(int row, int col) {
    hWalls[row][col] = true;
}

void ExploreMap::addVWall(int row, int col) {
    vWalls[row][col] = true;
}

void ExploreMap::removeHWall(int row, int col) {
    hWalls[row][col] = false;
}

void ExploreMap::removeVWall(int row, int col) {
    vWalls[row][col] = false;
}

void ExploreMap::print2DVector(vector<vector<bool>> p) {
    cout << "Vector:\n";
    for (unsigned int i = 0; i < p.size(); i++) {
        for (unsigned int j = 0; j < p[i].size(); j++) {
            cout << p[i][j];
        }
        cout << "\n";
    }
    cout << "\n";
}

template <typename T>
void ExploreMap::explore(T& robot) {
    int currentLocation[2] = {0, 0};
    char heading; // assume an initial heading of south
    char* walls;

    setExplored(currentLocation[0], currentLocation[1]);
    heading = 'S';
    robot.getDistSensorReadings();
    walls = robot.getWalls();
    cout << walls << endl;
    setWalls(currentLocation, heading, walls);

    robot.rotateRobot('R');
    
    heading = 'W';
    robot.getDistSensorReadings();
    walls = robot.getWalls();
    cout << walls << endl;
    setWalls(currentLocation, heading, walls);
    // use a left wall follower. 
    // if returned to the start position
    //      find closest unexplored grid
    //      use path finding to get there
}