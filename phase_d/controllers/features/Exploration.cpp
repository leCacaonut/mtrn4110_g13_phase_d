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
    explored.resize(5, vector<bool>(9, false));
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
       switch (heading) {
        case 'N':
            // if (lWall) hWalls = true;
            break;
        case 'S':
            break;
        case 'W':
            break;
        case 'E':
            break;
        default:
            break;
    }
}

// set a wall to be true
// if outside of the size of the map, pushback
bool ExploreMap::addWall(int row, int col) {
    return true;
}

bool ExploreMap::removeWall(int row, int col) {
    return true;
}

void ExploreMap::print2DVector(vector<vector<bool>> p) {
    cout << "Vector H:\n";
    for (unsigned int i = 0; i < p.size(); i++) {
        for (unsigned int j = 0; j < p[i].size(); j++) {
            cout << p[i][j];
        }
        cout << "\n";
    }
    cout << "\n";
}

template<typename T>
void ExploreMap::explore(T& robot) {
    // instead of defining variables here, integrate with the robot class using its getter functions
    int currentLocation[2] = {0, 0};
    char heading;
    char* walls;

    setExplored(currentLocation[0], currentLocation[1]);

    robot.getDistSensorReadings();
    walls = robot.getWalls();
    setWalls(currentLocation, heading, walls);
    robot.rotateRobot('R');
    robot.getDistSensorReadings();
    walls = robot.getWalls();
    // use a left wall follower. 
    // if returned to the start position
    //      find closest unexplored grid
    //      use path finding to get there
}