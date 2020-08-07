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

class Map {
    private:
        int mapSize[2];
        int goal[2];
        vector<vector<bool>> explored;
        vector<vector<bool>> hWalls;
        vector<vector<bool>> vWalls;

    public:
        Map();
        ~Map();
        void setMapSize(int row, int col);
        void setDefaultGoal();
        void setGoal(int row, int col);
        
        int* getMapSize();
        int* getGoal();

        void addHWall(int row, int col);
        void addVWall(int row, int col);
        void removeHWall(int row, int col);
        void removeVWall(int row, int col);
};
