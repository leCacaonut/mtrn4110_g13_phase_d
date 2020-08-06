// File:          z5133929_MTRN4110_PhaseBc.cpp
// Date:          23/6/2020
// Description:   Some flood fill path planner
// Author:        Gordon
// Modifications: 
// Hard-coding: Nope

// Question
// maintain a list of short paths?

#include <webots/Robot.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace webots;

#define TIME_STEP 64
#define MAP_FILE_NAME "../../MapFound.txt"
// #define PATH_PLAN_FILE_NAME "../../PathPlanFound.txt"
#define MAP_ROW 5
#define MAP_COL 9
#define GOAL {2, 4}

enum {N, E, S, W};

class Pathing {
  public:
    string directionalPath;
    string instructionalPath;
    int expense = 0;
    int currentPosition[2];
    int currVal;
    vector<vector<bool>> cellOccupied;
    
    Pathing();
    Pathing(string dirPath, int currPos[2], int cv) {
      directionalPath = dirPath;
      currentPosition[0] = currPos[0];
      currentPosition[1] = currPos[1];
      currVal = cv;
      instructionalPath = "";
      cellOccupied.resize(MAP_COL,vector<bool>(MAP_ROW, false));
      for(int i = 0; i < MAP_ROW; ++i) {
        for(int j = 0; j < MAP_COL; ++j) {
          cellOccupied[i][j] = false;
        }
      }
    }
    Pathing(string dirPath, vector<int> currPos, int cv) {
      directionalPath = dirPath;
      currentPosition[0] = currPos[0];
      currentPosition[1] = currPos[1];
      currVal = cv;
      instructionalPath = "";
      cellOccupied.resize(MAP_COL,vector<bool>(MAP_ROW, false));
      for(int i = 0; i < MAP_ROW; ++i) {
        for(int j = 0; j < MAP_COL; ++j) {
          cellOccupied[i][j] = false;
        }
      }
    }
    
    char num2heading(int heading) {
      char cHeading = 'N';
      while(heading < 0) heading += 4;
      while(heading > 3) heading -= 4;
      switch(heading) {
        case N: cHeading = 'N'; break;
        case E: cHeading = 'E'; break;
        case S: cHeading = 'S'; break;
        case W: cHeading = 'W'; break;
        default: cout << "Error converting\n";
      }
      return cHeading;
    }
    
    void calculateExpense() {
      expense = 0;
      for(unsigned int i = 0; i < instructionalPath.size(); ++i) {
        if(instructionalPath[i] == 'L' || instructionalPath[i] == 'R') ++expense;
      }
    }
    
    void processPath(int heading) {
      // passed heading is a copy
      // do anything with it without changes out of scope
      // go through each heading and scenarios
      for(unsigned int i = 0; i < directionalPath.size(); ++i) {
        if(directionalPath[i] == num2heading(heading)) {
          // forwards!
          instructionalPath.append("F");
        } else if(directionalPath[i] == num2heading(heading - 1)) {
          // left and forwards!
          --heading;
          instructionalPath.append("LF");
        } else if(directionalPath[i] == num2heading(heading + 1)) {
          // right and forwards!
          ++heading;
          instructionalPath.append("RF");
        }
      }
      calculateExpense();
    }
};

void processMap(std::ifstream& mapfile, bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int &heading);
void floodFillMap(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int goal[2]);
void printMap(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int heading);
void printMap(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int heading, Pathing* path);
vector<Pathing*> findPath(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int heading, int goal[2]);

// ==================
// ||     MAIN     ||
// ==================

// int main(int argc, char **argv) {
//   runProcess();
//   return 0;
// }

void runProcess(){
  Robot *robot = new Robot();
  
  bool hWalls[MAP_ROW + 1][MAP_COL] = {0};
  bool vWalls[MAP_ROW][MAP_COL + 1] = {0};
  int cValues[MAP_ROW][MAP_COL] = {0};
  // initialise a high value
  for(int i = 0; i < MAP_ROW; ++i) {
    for(int j = 0; j < MAP_COL; ++j) {
      cValues[i][j] = MAP_ROW*MAP_COL+1;
    }
  }
  
  int heading;
  int startingPos[2] = {0, 0};
  int goal[2] = GOAL;
  
  // read map file
  ifstream mapfile(MAP_FILE_NAME);
  if(!mapfile.is_open()) {
    cout << "Error opening file\n";
    exit(-1);
  }
  cout << "--- Task 1 ---" << endl;
  processMap(mapfile, hWalls, vWalls, startingPos, heading);

  mapfile.close();
  floodFillMap(cValues, hWalls, vWalls, startingPos, goal);
  cout << "--- Task 2 ---" << endl;
  vector<Pathing*> path = findPath(cValues, hWalls, vWalls, startingPos, heading, goal);

  
  // process each path
  for(unsigned int i = 0; i < path.size(); ++i) {
    // translate raw NESW instructions to FLR instructions
    // processPath(path[i], heading);
    path[i]->processPath(heading);
    
    cout << "--- Path " << i + 1 << " ---" << endl;
    // print path on map
    printMap(cValues, hWalls, vWalls, startingPos, heading, path[i]);
  }
  
  // find the path with the least amount of turns (MAY NOT BE SHORTEST OVERALL INSTRUCTIONS)
  // in this case, least 'expense'
  char h = N;
  switch(heading) {
    case N: h = 'N'; break;
    case E: h = 'E'; break;
    case S: h = 'S'; break;
    case W: h = 'W'; break;
  }
  int pathID = 0;
  int exp = 99;
  for(unsigned int i = 0; i < path.size(); ++i) {
    if(path[i]->expense < exp) {
      pathID = i;
      exp = path[i]->expense;
    }
  }
  
  cout << "--- Task 3 ---" << endl;
  printMap(cValues, hWalls, vWalls, startingPos, heading, path[pathID]);
  cout << "Steps: " << path[pathID]->instructionalPath.length() << endl;
  cout << "Path: " << startingPos[0] << startingPos[1] << h << path[pathID]->instructionalPath << endl;
  
  cout << "--- Task 4 ---" << endl;
  cout << "File: " << PATH_PLAN_FILE_NAME << endl;
  // export path to a text file
  ofstream outputFile(PATH_PLAN_FILE_NAME);
  outputFile << startingPos[0] << startingPos[1] << h << path[pathID]->instructionalPath << endl;
  outputFile.close();
  
  ifstream _outputFile(PATH_PLAN_FILE_NAME);
  string output;
  getline(_outputFile, output);
  cout << "Path: " << output << endl;
  _outputFile.close();
  
  while (robot->step(TIME_STEP) != -1) {
  };
  
  delete robot;
}

void processMap(std::ifstream& mapfile, bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int &heading) {
  char ch;
  int i = 0, j = 0; // character counter and line counter
  int ih = 0, jh = 0, iv = 0, jv = 0; // wall counters
  while(mapfile.get(ch)) {
    cout << ch;
    // line manipulation
    if(ch == '\n') {
      if(j % 2 == 0) {
        ++jh;
      } else {
        ++jv;
      }
      // reset
      i = 0;
      ih = 0;
      iv = 0;
      ++j;  // new line
    }
    // read every second character
    if(i % 2 == 0) {
      // horizontal row 
      if(j % 2 == 0) {
        if(i % 4 == 2) {
          if(ch == '-') {
            hWalls[jh][ih] = true;
          }
          ++ih;
        }
      }
    } 
    // vertical row
    else {
      if(i % 4 == 1) {
        if(ch == '|') {
          vWalls[jv][iv] = true;
        }
        ++iv;
      } else {
        // find a heading character anywhere
        switch(ch) {
          case '^': heading = N; startingPos[0] = jv; startingPos[1] = iv - 1; break;
          case '>': heading = E; startingPos[0] = jv; startingPos[1] = iv - 1; break;
          case 'v': heading = S; startingPos[0] = jv; startingPos[1] = iv - 1; break;
          case '<': heading = W; startingPos[0] = jv; startingPos[1] = iv - 1; break;
        }
      }
    }
    ++i; // increment character counter
  }
  cout << endl;
}

void floodFillMap(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int goal[2]) {
  cValues[goal[0]][goal[1]] = 0;
  int currentVal = 0;
  int robotReached = false;
  while(!robotReached) {
    for(int i = 0; i < MAP_ROW; ++i) {
      for(int j = 0; j < MAP_COL; ++j) {
        if(cValues[i][j] == currentVal) {
          if(i == startingPos[0] && j == startingPos[1]) robotReached = true;
          // up
          if(hWalls[i][j] == false && cValues[i - 1][j] > currentVal) {
            cValues[i - 1][j] = currentVal + 1;
          }
          // left
          if(vWalls[i][j] == false && cValues[i][j - 1] > currentVal) {
            cValues[i][j - 1] = currentVal + 1;
          }
          // down
          if(hWalls[i + 1][j] == false && cValues[i + 1][j] > currentVal) {
            cValues[i + 1][j] = currentVal + 1;
          }
          // right
          if(vWalls[i][j + 1] == false && cValues[i][j + 1] > currentVal) {
            cValues[i][j + 1] = currentVal + 1;
          }
        }
      }
    }
    ++currentVal;
  }
}

void printMap(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int heading) {
  char _h;
  if(heading == N) _h = '^';
  else if(heading == E) _h = '>';
  else if(heading == S) _h = 'v';
  else _h = '<';
  for (int i = 0; i < MAP_ROW; ++i) {
    // print horizontals
    for(int j = 0; j < MAP_COL; ++j) {
      if(hWalls[i][j] == true) printf(" ---");
      else cout << "    ";
    }
    printf(" \n");
    // print verticals
    for (int j = 0; j < MAP_COL; ++j) {
      if(vWalls[i][j] == true) cout << "|";
      else cout << " ";
      if(i == startingPos[0] && j == startingPos[1]) cout << " " << _h << " ";
      else if(cValues[i][j] == MAP_ROW*MAP_COL+1) cout << "   ";
      else if(cValues[i][j] < 10) cout << " " << cValues[i][j] << " ";
      else cout << setw(3) << cValues[i][j];
    }
    cout << "|\n";
  }
  for(int i = 0; i < MAP_COL; ++i) {
    cout << " ---";
  }
  cout << " " << endl;
}

void printMap(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int heading, Pathing* path) {
  char _h;
  if(heading == N) _h = '^';
  else if(heading == E) _h = '>';
  else if(heading == S) _h = 'v';
  else _h = '<';
  // For each path, create a path on the map
  for(int i = 0; i < MAP_ROW; ++i) {
    // print horizontals
    for(int j = 0; j < MAP_COL; ++j) {
      if(hWalls[i][j] == true) printf(" ---");
      else cout << "    ";
    }
    printf(" \n");
    // print verticals
    for (int j = 0; j < MAP_COL; ++j) {
      // vertical wall space
      if(vWalls[i][j] == true) cout << "|";
      else cout << " ";
      // print path
      if(i == startingPos[0] && j == startingPos[1]) cout << " " << _h << " ";
      else if(cValues[i][j] == MAP_ROW*MAP_COL+1) cout << "   ";
      else {
        if(path->cellOccupied[i][j]) {
          if(cValues[i][j] < 10) cout << " " << cValues[i][j] << " ";
          else cout << setw(3) << cValues[i][j];
        } else {
          cout << "   ";
        }
      }
    }
    cout << "|\n";
  }
  for(int i = 0; i < MAP_COL; ++i) {
    cout << " ---";
  }
  cout << " " << endl;
}

vector<Pathing*> findPath(int cValues[MAP_ROW][MAP_COL], bool hWalls[MAP_ROW + 1][MAP_COL], bool vWalls[MAP_ROW][MAP_COL + 1], int startingPos[2], int heading, int goal[2]) {
  vector<Pathing*> path;
  
  int currentVal = cValues[startingPos[0]][startingPos[1]];
  path.push_back(new Pathing("", startingPos, currentVal));
  
  int splitCounter;
  while(currentVal > 0) {
    unsigned int n = path.size();
    for(unsigned int i = 0; i < n; ++i) {
      splitCounter = 0;
      // do an initial check for multiple paths
      if(hWalls[path[i]->currentPosition[0]][path[i]->currentPosition[1]] == false &&
         cValues[path[i]->currentPosition[0] - 1][path[i]->currentPosition[1]] == currentVal - 1) {
        ++splitCounter;
      }
      if(vWalls[path[i]->currentPosition[0]][path[i]->currentPosition[1]] == false &&
         cValues[path[i]->currentPosition[0]][path[i]->currentPosition[1] - 1] == currentVal - 1) {
        ++splitCounter;
      }
      if(hWalls[path[i]->currentPosition[0] + 1][path[i]->currentPosition[1]] == false &&
         cValues[path[i]->currentPosition[0] + 1][path[i]->currentPosition[1]] == currentVal - 1) {        
        ++splitCounter;
      }
      if(vWalls[path[i]->currentPosition[0]][path[i]->currentPosition[1] + 1] == false &&
         cValues[path[i]->currentPosition[0]][path[i]->currentPosition[1] + 1] == currentVal - 1) {
        ++splitCounter;
      }
      // copy current path to new path(s)
      if(splitCounter > 1) {
        for(int ctr = 1; ctr < splitCounter; ++ctr) {
          path.push_back(new Pathing(path[i]->directionalPath, path[i]->currentPosition, path[i]->currVal));
        }
      }
    }
    // add a direction to the path
    // then if the path is exactly the same, pick another path
    for(unsigned int i = 0; i < path.size(); ++i) {
      if(path[i]->currVal >= currentVal &&
         hWalls[path[i]->currentPosition[0]][path[i]->currentPosition[1]] == false &&
         cValues[path[i]->currentPosition[0] - 1][path[i]->currentPosition[1]] == currentVal - 1) {
        // create a path step
        path[i]->directionalPath.push_back('N');
        --path[i]->currVal;
        --path[i]->currentPosition[0];
        // if there are matches for each path
        for(unsigned int j = 0; j < path.size(); ++j) {
          if(i != j && path[i]->directionalPath.compare(path[j]->directionalPath) == 0) {
            path[i]->directionalPath.pop_back();   // revert all changes
            ++path[i]->currVal;
            ++path[i]->currentPosition[0];
            break;   // only want to delete ONCE
          }
        }
      }
      if(path[i]->currVal >= currentVal &&
         vWalls[path[i]->currentPosition[0]][path[i]->currentPosition[1]] == false &&
         cValues[path[i]->currentPosition[0]][path[i]->currentPosition[1] - 1] == currentVal - 1) {
        
        path[i]->directionalPath.push_back('W');
        --path[i]->currVal;
        --path[i]->currentPosition[1];
        
        for(unsigned int j = 0; j < path.size(); ++j) {
          if(i != j && path[i]->directionalPath.compare(path[j]->directionalPath) == 0) {
            path[i]->directionalPath.pop_back();
            ++path[i]->currVal;
            ++path[i]->currentPosition[1];
            break;
          }
        }
      }
      if(path[i]->currVal >= currentVal &&
         hWalls[path[i]->currentPosition[0] + 1][path[i]->currentPosition[1]] == false &&
         cValues[path[i]->currentPosition[0] + 1][path[i]->currentPosition[1]] == currentVal - 1) {
         
        path[i]->directionalPath.push_back('S');
        --path[i]->currVal;
        ++path[i]->currentPosition[0];
        
        for(unsigned int j = 0; j < path.size(); ++j) {
          if(i != j && path[i]->directionalPath.compare(path[j]->directionalPath) == 0) {
            path[i]->directionalPath.pop_back();
            ++path[i]->currVal;
            --path[i]->currentPosition[0];
            break;
          }
        }
      }
      
      if(path[i]->currVal >= currentVal &&
         vWalls[path[i]->currentPosition[0]][path[i]->currentPosition[1] + 1] == false &&
         cValues[path[i]->currentPosition[0]][path[i]->currentPosition[1] + 1] == currentVal - 1) {
         
        path[i]->directionalPath.push_back('E');
        --path[i]->currVal;
        ++path[i]->currentPosition[1];
        
        for(unsigned int j = 0; j < path.size(); ++j) {
          if(i != j && path[i]->directionalPath.compare(path[j]->directionalPath) == 0) {
            path[i]->directionalPath.pop_back();
            ++path[i]->currVal;
            --path[i]->currentPosition[1];
            break;
          }
        }
      }
    }
    --currentVal;
  }
  // now save a path onto a map style
  for(unsigned int i = 0; i < path.size(); ++i) {
    path[i]->currentPosition[0] = startingPos[0];
    path[i]->currentPosition[1] = startingPos[1];
    
    for(unsigned int j = 0; j < path[i]->directionalPath.size(); ++j) {
      if(path[i]->directionalPath[j] == 'N') {
        --path[i]->currentPosition[0];
      } else if(path[i]->directionalPath[j] == 'W') {
        --path[i]->currentPosition[1];
      } else if(path[i]->directionalPath[j] == 'S') {
        ++path[i]->currentPosition[0];
      } else {
        ++path[i]->currentPosition[1];
      }
      path[i]->cellOccupied[path[i]->currentPosition[0]][path[i]->currentPosition[1]] = true;
    }
  }
  return path;
}













