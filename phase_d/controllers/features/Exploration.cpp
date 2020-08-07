/*
*  Exploring a map
*
*
*/
#include "Exploration.h"


Map::Map() {
    setMapSize(1, 1);
    setDefaultGoal();
}

Map::~Map(){}

void Map::setMapSize(int row, int col) {
    mapSize[ROW] = row;
    mapSize[COL] = col;
}

void Map::setDefaultGoal() {
    int x = mapSize[ROW];
    int y = mapSize[COL];
    if (x % 2 == 1) --x;
    if (y % 2 == 1) --y;
    goal[ROW] = x / 2;
    goal[COL] = y / 2;
}

void Map::setGoal(int row, int col) {
    goal[ROW] = row;
    goal[COL] = col;
}

int* Map::getMapSize() {
    return mapSize;
}

int* Map::getGoal() {
    return goal;
}

void Map::addHWall(int row, int col){
    // set a wall to be true
    // if outside of the size of the map, pushback
}

void Map::addVWall(int row, int col){

}

void Map::removeHWall(int row, int col){

}

void Map::removeVWall(int row, int col){

}