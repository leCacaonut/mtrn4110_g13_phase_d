/*
*  Exploring a map
*
*
*/
#include "Exploration.h"

ExploreMap::ExploreMap() {
    explored.resize(5, vector<bool>(9, false));
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

vector<vector<bool>> ExploreMap::getHWalls() {
    return hWalls;
}

vector<vector<bool>> ExploreMap::getVWalls() {
    return vWalls;
}

// set a wall to be true
// if outside of the size of the map, pushback
bool ExploreMap::addWall(int position[2], char heading, bool lWall, bool fWall, bool rWall) {
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
            return false;
    }
    return true;
}

bool ExploreMap::removeWall(int position[2], char heading, bool lWall, bool fWall, bool rWall) {
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