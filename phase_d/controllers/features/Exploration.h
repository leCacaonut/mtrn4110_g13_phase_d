/*
*  Wall detection will have to include border wall
*/


// includes
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

// Namespaces
using namespace std;

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

class ExploreMap {
private:
    int mapSize[2];
    int goal[2];

    vector<vector<bool>> explored;
    vector<vector<bool>> hWalls;
    vector<vector<bool>> vWalls;

public:
    ExploreMap();
    ExploreMap(int row, int col);
    ~ExploreMap();

    void setDefaultGoal();
    void setGoal(int row, int col);
    void setExplored(int row, int col);

    void resize2DVector(vector<vector<bool>> &v, unsigned int numRows, unsigned int numCols);

    int* getMapSize();
    int* getGoal();
    vector<vector<bool>> getExplored();
    bool getExplored(int row, int col);
    vector<vector<bool>> getHWalls();
    vector<vector<bool>> getVWalls();
    int* size2DVector(int* s, vector<vector<bool>> p);

    void setWalls(int position[2], char heading, char* walls);
    void addHWall(int row, int col);
    void addVWall(int row, int col);
    void removeHWall(int row, int col);
    void removeVWall(int row, int col);
    void print2DVector(vector<vector<bool>> p);

    template <typename T>
    void explore(T& robot);
};
