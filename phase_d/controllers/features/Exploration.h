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

    int* getMapSize();
    int* getGoal();
    vector<vector<bool>> getExplored();
    bool getExplored(int row, int col);
    vector<vector<bool>> getHWalls();
    vector<vector<bool>> getVWalls();

    bool addWall(int position[2], char heading, bool lWall, bool fWall, bool rWall);
    bool removeWall(int position[2], char heading, bool lWall, bool fWall, bool rWall);
    void print2DVector(vector<vector<bool>> p);

    void explore(Epuck &epuck);
};
