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
    hWalls.resize(row + 1, vector<bool>(col, false));
    vWalls.resize(row, vector<bool>(col + 1, false));
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

void ExploreMap::resize2DVector(vector<vector<bool>>& v, unsigned int numRows, unsigned int numCols) {
    while (v.size() < numRows) {
        v.push_back(vector<bool>(numCols, false));
    }
    for (unsigned int i = 0; i < v.size(); ++i) {
        while (v[i].size() < numCols) {
            v[i].push_back(false);
        }
    }
}

void ExploreMap::setExplored(int v[2], char heading, char* walls) {
    int& row = v[0];
    int& col = v[1];
    if (abs(row) >= getMapSize()[0] || abs(col) >= getMapSize()[1]) {
        resize2DVector(explored, abs(row) + 1, abs(col) + 1);
        resize2DVector(hWalls, abs(row) + 2, abs(col) + 1);
        resize2DVector(vWalls, abs(row) + 1, abs(col) + 2);
    }
    explored[abs(row)][abs(col)] = true;
    setWalls(v, heading, walls);
}

vector<vector<bool>> ExploreMap::rotateMap(vector<vector<bool>> matrix) {

    int rowNum = matrix.size();
    int colNum = matrix[0].size();

    vector<vector<bool>> flipped = matrix;

    // flip map
    for(int i = 0; i < rowNum; i++) {
        for(int j = 0; j < colNum; j++) {
            flipped[i][j] = matrix[i][colNum-1-j];
            cout << flipped[i][j];
        }
        cout << endl;
    }
    // matrix = flipped;
    return flipped;
}

vector<vector<bool>> ExploreMap::swapColumns(vector<vector<bool>> matrix) {

    int rowNum = matrix.size();

    for (int i = 0; i < rowNum; i++) { 
        bool tmp = matrix[i][0]; 
        matrix[i][0] = matrix[i][1]; 
        matrix[i][1] = tmp; 
    } 

    return matrix;
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

void ExploreMap::setRotatedExplored(vector<vector<bool>> p) {
    explored = p;
}

void ExploreMap::setRotatedhWalls(vector<vector<bool>> p) {
    hWalls = p;
}

void ExploreMap::setRotatedvWalls(vector<vector<bool>> p) {
    vWalls = p;
}

void ExploreMap::setWalls(int position[2], char heading, char* walls) {

    switch (heading) {
        case 'N':
            if (position[1] < 0 ) {
                if (walls[LEFT]  == 'Y') addVWall(position[0], abs(position[1])+1);
                if (walls[RIGHT] == 'Y') addVWall(position[0], position[1]);
                if (walls[LEFT]  == 'N') removeVWall(position[0], abs(position[1])+1);
                if (walls[RIGHT] == 'N') removeVWall(position[0], position[1]);
            } else {
                if (walls[LEFT]  == 'Y') addVWall(position[0], position[1]);
                if (walls[RIGHT] == 'Y') addVWall(position[0], position[1] + 1);
                if (walls[LEFT]  == 'N') removeVWall(position[0], position[1]);
                if (walls[RIGHT] == 'N') removeVWall(position[0], position[1] + 1);
            }
    
            if (walls[FRONT] == 'Y') addHWall(position[0], position[1]);
            if (walls[FRONT] == 'N') removeHWall(position[0], position[1]);
           
            break;
        case 'S':
            if (position[1] < 0 ) {
                if (walls[LEFT]  == 'Y') addVWall(position[0], position[1]);
                if (walls[RIGHT] == 'Y') addVWall(position[0], abs(position[1])+1);
                if (walls[LEFT]  == 'N') removeVWall(position[0], position[1]);
                if (walls[RIGHT] == 'N') removeVWall(position[0], abs(position[1])+1);
            } else {
                if (walls[LEFT]  == 'Y') addVWall(position[0], position[1] + 1);
                if (walls[RIGHT] == 'Y') addVWall(position[0], position[1]); 
                if (walls[LEFT]  == 'N') removeVWall(position[0], position[1] + 1);
                if (walls[RIGHT] == 'N') removeVWall(position[0], position[1]);
            }
            
            if (walls[FRONT] == 'N') removeHWall(position[0] + 1, position[1]);
            if (walls[FRONT] == 'Y') addHWall(position[0] + 1, position[1]);
            
            break;
        case 'W':
            if (position[1] < 0) {
                if (walls[FRONT] == 'Y') addVWall(position[0], abs(position[1])+1);
                if (walls[FRONT] == 'N') removeVWall(position[0], abs(position[1])+1);
            } else {
                if (walls[FRONT] == 'Y') addVWall(position[0], position[1]);
                if (walls[FRONT] == 'N') removeVWall(position[0], position[1]);
            }
            
            if (walls[LEFT]  == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[RIGHT] == 'Y') addHWall(position[0], position[1]);
            if (walls[LEFT]  == 'N') removeHWall(position[0] + 1, position[1]);
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

            if (walls[LEFT]  == 'Y') addHWall(position[0], position[1]);
            if (walls[RIGHT] == 'Y') addHWall(position[0] + 1, position[1]);
            if (walls[LEFT]  == 'N') removeHWall(position[0], position[1]);
            if (walls[RIGHT] == 'N') removeHWall(position[0] + 1, position[1]);
            break;
        default:
            cout << "Failed to set walls" << endl;
    }
}

// set a wall to be true
// if outside of the size of the map, pushback
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

void ExploreMap::print2DVector(vector<vector<bool>> p) {
    // cout << "Vector:\n";
    for (unsigned int i = 0; i < p.size(); i++) {
        for (unsigned int j = 0; j < p[i].size(); j++) {
            cout << p[i][j];
        }
        cout << "\n";
    }
    cout << "\n";
}

int* ExploreMap::size2DVector(int* s, vector<vector<bool>> p) {
    s[0] = p.size();
    s[1] = p[0].size();
    return s;
}

template <typename T>
void ExploreMap::explore(T& robot) {
    int initialPosition[2] = {0, 0};
    int* gridPosition = robot.getPosition();
    char* walls = robot.getWalls();
    char heading = robot.getHeading();
    setExplored(gridPosition, heading, walls);
    int robotLocation = TOPLEFT;

    // make a first move without while loop conditional
    robot.followWallStep();
    gridPosition = robot.getPosition();
    walls = robot.getWalls();
    heading = robot.getHeading();
    cout << ": " << gridPosition[ROW] << ": " << gridPosition[COL] << ": " << heading << endl;
    setExplored(gridPosition, heading, walls);
    
    do {
        robot.followWallStep();
        gridPosition = robot.getPosition();
        walls = robot.getWalls();
        heading = robot.getHeading();
        cout << ": " << gridPosition[ROW] << ": " << gridPosition[COL] << ": " << heading << endl;
        setExplored(gridPosition, heading, walls);
        if (robot.getPosition()[1] < 0) {
            robotLocation = TOPRIGHT;
        }
    } while (!(initialPosition[0] == gridPosition[0] && initialPosition[1] == gridPosition[1]));
    // for (int i = 0; i < 21; ++i) {
    //     robot.followWallStep();
    //     gridPosition = robot.getPosition();
    //     walls = robot.getWalls();
    //     heading = robot.getHeading();
    //     cout << ": " << gridPosition[ROW] << ": " << gridPosition[COL] << ": " << heading << endl;
    //     setExplored(gridPosition, heading, walls);
    //     if (robot.getPosition()[1] < 0) {
    //         robotLocation = TOPRIGHT;
    //     }
    // }


    // if col is negative
    if(robotLocation == TOPRIGHT) {
        vector<vector<bool>> rExplored;
        vector<vector<bool>> rvWalls;
        vector<vector<bool>> rhWalls;

        rExplored = rotateMap(explored);
        setRotatedExplored(rExplored);
        rhWalls = rotateMap(hWalls);
        print2DVector(rhWalls);
        setRotatedhWalls(rhWalls);
        rvWalls = swapColumns(vWalls);
        rvWalls = rotateMap(rvWalls);
        setRotatedvWalls(rvWalls);
    }

    // use a left wall follower. 
    // if returned to the start position
    //      find closest unexplored grid
    //      use path finding to get there
}