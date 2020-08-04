// File:          z5129009_MTRN4110_PhaseB.cpp
// Date:
// Description:
// Author:
// Modifications:


#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <queue>
#include <algorithm>

#define MAP_FILE_NAME "../../Map.txt"
#define PATH_PLAN_FILE_NAME "../../PathPlanFound.txt"
#define HORIZONTAL_ROW 6
#define HORIZONTAL_COLUMN 9
#define VERTICAL_ROW 5
#define VERTICAL_COLUMN 10
#define CELL_ROW 5
#define CELL_COLUMN 9
#define GOAL_CELL_ROW 2
#define GOAL_CELL_COLUMN 4 
#define MAP_ROW 11
#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

int getHeading(char inputDirection) {
  int direction = 0;
  switch(inputDirection) {
    case 'v' :
      direction = SOUTH;
      break;
    case '^' :
      direction = NORTH;
      break;
    case '<' :
      direction = WEST;
      break;
    case '>' :
      direction = EAST;
      break;
   }
  return direction;
}

void findAllShortestPaths(std::vector<std::vector<int> >&paths, std::vector<int>& path, 
std::vector<int> parent[], int u) {
  
  if (u == -1) {
    paths.push_back(path);
    return;
  }
  
  for(int par: parent[u]) {
    
    path.push_back(u);

    findAllShortestPaths(paths,path,parent,par);
    
    path.pop_back();
    
  }
}


void BFS(int cellValue[CELL_ROW][CELL_COLUMN], int initialRow, int initialCol, 
  int horizontalWall[HORIZONTAL_ROW][HORIZONTAL_COLUMN], int verticalWall[VERTICAL_ROW][VERTICAL_COLUMN], 
  std::vector<int> parent[]) {

  std::queue<std::pair<int,int>> q;
  bool visited[CELL_ROW][CELL_COLUMN];
  
  for (int x = 0; x<CELL_ROW; x++) {
    for(int y = 0; y<CELL_COLUMN; y++) {
      visited[x][y] = false;
     }
  }
      

  int currentRow = initialRow;
  int currentCol = initialCol;
  q.push({currentRow,currentCol});
  parent[(currentRow*CELL_COLUMN)+currentCol] = {-1};
  visited[currentRow][currentCol] = true;
  int j = q.front().first;
  int i = q.front().second;  
  
  while(!q.empty() && cellValue[j][i] != 0) {
    j = q.front().first;
    i = q.front().second;
    q.pop();
    for(int k=0; k<4; k++) {
        //k=0 means its check north
        switch(k) {
          case NORTH :
            if(cellValue[j-1][i] == cellValue[j][i]-1 && (horizontalWall[j][i] != 1)) {
            //push parent node
              parent[((j-1)*CELL_COLUMN)+i].push_back((j*CELL_COLUMN)+i);
              if(visited[j-1][i] == false) {
                q.push({j-1,i}); 
                visited[j-1][i] = true;
              }
            }
            break;
          case SOUTH :
            if(cellValue[j+1][i] == cellValue[j][i]-1 && (horizontalWall[j+1][i] != 1)) {
              parent[((j+1)*CELL_COLUMN)+i].push_back((j*CELL_COLUMN)+i);
              if(visited[j+1][i] == false) {
                q.push({j+1,i});
                visited[j+1][i] = true;
              }
            }
            break;
          case EAST :
            if(cellValue[j][i+1] == cellValue[j][i]-1 && (verticalWall[j][i+1] != 1)) {
              parent[(j*CELL_COLUMN)+i+1].push_back(((j)*CELL_COLUMN)+i);
              if(visited[j][i+1] == false) {
                q.push({j,i+1});   
                visited[j][i+1] = true; 
              }    
            }
            break;
          case WEST :
            if(cellValue[j][i-1] == cellValue[j][i]-1 && (verticalWall[j][i] != 1)) {
              parent[(j*CELL_COLUMN)+i-1].push_back(((j)*CELL_COLUMN)+i);
              if(visited[j][i-1] == false) {
                q.push({j,i-1});    
                visited[j][i-1] = true;
              }
            }
            break;
        }  
      }
  }

}

void printMapPaths(int cellValue[CELL_ROW][CELL_COLUMN], std::string map[MAP_ROW],int initialRow, int initialCol,
                   std::vector<int> path) {
                   
    int j=0;
    int i=2;
    int n = map[0].length();
                 
    while (j<MAP_ROW) {
    if(j%2 == 0) {
      std::cout << map[j] << std::endl;
    } else {
      while (i<n) {
        for(int y : path) {
          int row = y/CELL_COLUMN;
          int col = y%CELL_COLUMN;
          if(row == initialRow && col == initialCol) {
            //if it is the initial location, skip
          } else if(row == (j-1)/2 && col == (i-2)/4) {
            if(cellValue[row][col] > 9) {
              map[j][i] = (cellValue[row][col]/10)+48;
              map[j][i+1] = (cellValue[row][col] % 10)+48;
            } else {
              map[j][i] = cellValue[row][col]+48;
            }
          } 
        }
        i=i+4; 
      }
      std::cout << map[j] <<std::endl;
      i=2;
    }
    j++;
  }  

}

std::vector<std::vector<int>> printPaths(int cellValue[CELL_ROW][CELL_COLUMN], int initialRow, int initialCol, 
  int horizontalWall[HORIZONTAL_ROW][HORIZONTAL_COLUMN], int verticalWall[VERTICAL_ROW][VERTICAL_COLUMN],
  std::string map[MAP_ROW], int nSteps) {
  
  std::vector<std::vector<int>> paths;
  std::vector<int> path;
  std::vector<int> parent[(CELL_ROW*CELL_COLUMN)];
  int goal;
  
  goal = (GOAL_CELL_ROW*CELL_COLUMN)+GOAL_CELL_COLUMN;
  
  BFS(cellValue, initialRow, initialCol, horizontalWall, verticalWall, parent);
  
  findAllShortestPaths(paths, path, parent, goal);
  
  int nPaths = 0;
  //print the map 
  std::string mapCopy[MAP_ROW];
  
  std::cout << "--- Task 2 ---" << std::endl;
  for (auto x : paths) {
    std::reverse(x.begin(),x.end());
    nPaths++;
    std::cout << "--- Path " <<nPaths<< " ---" << std::endl;
    //duplicate map
    for(int k = 0; k<MAP_ROW; k++) {
      mapCopy[k] = map[k];
    }
    printMapPaths(cellValue, mapCopy,initialRow,initialCol,x);

  }
  return paths;
}   

std::vector<std::pair<int,std::string>> findLeastSteps(std::vector<std::vector<int>> paths, int initialHeading, int initialRow, int initialCol) {
  
  int nSteps = 0;
  int previousRow =0;
  int previousCol =0;
  int row = 0;
  int col = 0;
  int nextRow = 0;
  int nextCol =0;
  int y =0;
  int turn = 0;
  std::vector<std::pair<int,std::string>> n;
  std::string command;

  for(auto x: paths) {
    std::reverse(x.begin(),x.end());
    turn = 0;
    nSteps = x.size()-1;
    command = "";
    command.append(std::to_string(initialRow));
    command.append(std::to_string(initialCol));
    for(y =1; y <= nSteps; y++) {
      previousRow = x[y-1]/CELL_COLUMN;
      previousCol = x[y-1]%CELL_COLUMN;
      row = x[y]/CELL_COLUMN;
      col = x[y]%CELL_COLUMN;  
      nextRow = x[y+1]/CELL_COLUMN;
      nextCol = x[y+1]%CELL_COLUMN; 
      //get turns  
      if(y==1) { //initial heading
         if(initialHeading == NORTH) {
           command.append("N");
           //turn left
           if(col == previousCol -1) {
             turn++;
             command.append("L");
           //turn right
           }else if (col == previousCol + 1) {
             turn++; 
             command.append("R");
           //turn right twice || turn left twice
           }else if(row == previousRow + 1) {
             turn = turn + 2;
             command.append("RR"); 
           }
         } else if (initialHeading == SOUTH) {
           command.append("S");
           //turn left
           if(col == previousCol + 1) {
             turn++;
             command.append("L");
           //turn right
           } else if (col == previousCol -1) {
             turn++;
             command.append("R");
           //turn right twice || turn left twice
           } else if (row == previousRow - 1) {
             turn = turn +2;
             command.append("RR");
           } 
         } else if (initialHeading == EAST) {
           command.append("E");
           //turn right
           if(row == previousRow + 1) {
             turn++;
             command.append("R");
           //turn left
           } else if (row == previousRow - 1){
             turn++;   
             command.append("L");
           } else if (col == previousCol-1) {
             turn = turn + 2;
             command.append("RR");
           } 
         } else if (initialHeading == WEST) {
           command.append("W");
           //turn left
           if(row == previousRow + 1) {
             turn++;
             command.append("L");
           //turn right
           } else if (row == previousRow -1) {
             turn++;
             command.append("R");
           } else if (col == previousCol+1) {
             turn = turn + 2;
             command.append("RR");
           } 
         }
       }
      //go forward
      if((previousRow == row + 1) || (previousRow == row - 1) || (previousCol == col +1) || (previousCol == col -1)) {
        command.append("F");
      }
      //turn right
      if((row == previousRow && nextRow == row + 1 && col == previousCol +1) || 
         (row == previousRow && nextRow == row - 1 && col == previousCol -1) ||
         (col == previousCol && nextCol == col - 1 && row == previousRow +1) ||
         (col == previousCol && nextCol == col + 1 && row == previousRow -1)) {
        turn++;
        command.append("R");
      //turn left
      } else if ((row == previousRow && nextRow == row - 1 && col == previousCol +1) ||
                 (row == previousRow && nextRow == row + 1 && col == previousCol -1) ||
                 (col == previousCol && nextCol == col + 1 && row == previousRow +1) ||
                 (col == previousCol && nextCol == col - 1 && row == previousRow -1)) {
        turn++;
        command.append("L");
      }
    }     
    n.push_back(make_pair((turn+nSteps) ,command));
  }

  return n;
}  

void printAndStoreLeastStep(int cellValue[CELL_ROW][CELL_COLUMN], std::string map[MAP_ROW],int initialRow, int initialCol,
                   std::vector<std::pair<int,std::string>> n, std::vector<std::vector<int>> paths) {
                   
  int nCommand = 45;
  int k = 0;
  int nPaths = n.size();
  int counter = 0;
  
  for(int tmp=0;tmp<nPaths;tmp++) {
    if(n[tmp].first < nCommand) {
      nCommand = n[tmp].first;
      k = tmp;
    }
  }
  
  std::cout << "--- Task 3 ---" << std::endl;
                   
  for (auto x : paths) {
    std::reverse(x.begin(),x.end());
    if(counter == k) {
      printMapPaths(cellValue,map,initialRow,initialCol,x);
    }
    counter++;
  }
  std::cout << "Steps: "<< nCommand << std::endl;
  std::cout << "Path: "<< n[k].second << std::endl;
  
  //write file
  std::cout << "--- Task 4 ---" <<std::endl;
  std::cout << "File: " << PATH_PLAN_FILE_NAME << std::endl;
  std::cout << "Path: "<< n[k].second << std::endl;
  std::ofstream pathPlanFile;
  pathPlanFile.open(PATH_PLAN_FILE_NAME);
  pathPlanFile << n[k].second;

}         

int main(int argc, char **argv) {

  std::ifstream file(MAP_FILE_NAME);
  int horizontalWall[HORIZONTAL_ROW][HORIZONTAL_COLUMN];
  int verticalWall[VERTICAL_ROW][VERTICAL_COLUMN];
  int cellValue[CELL_ROW][CELL_COLUMN];
  std::string map[MAP_ROW];
  std::string mapCopy[MAP_ROW];
  int i = 0;
  // [0] and [1] store row and column, [2] store heading
  int initialLocation[3] = {0};
   
  if(file.is_open()) {
    std::cout << "--- Task 1 ---"<< std::endl;
    while(getline(file,map[i])) {
      std::cout<<map[i]<<std::endl;
      i++;
    }
  } else {
    std::cout << "Unable to read file :(" << std::endl;
  }
  
  
  //duplicate map
  for(i = 0; i<MAP_ROW; i++) {
    mapCopy[i] = map[i];
  }
  
  int n = map[0].length();
  int wall = 0;
  int noWall = 0;
  int a =0;
  int j =0;
  int b =0;
  i=0;
  
  //get horizontal wall
  while (j<MAP_ROW && b<HORIZONTAL_ROW) {
    i=0;
    a=0;
    while (i<n && a<HORIZONTAL_COLUMN) {
      if(map[j][i] == '-') {
        wall++;
        if(wall == 3) {
          horizontalWall[b][a] = 1;
          //i++ because after --- or 3 spaces, the next is always a space bar
          i++;
          a++;
          wall = 0;
        }
      } else {
        wall = 0;
        noWall++;
        if(noWall == 3) {
          horizontalWall[b][a] = 0;
          i++;
          a++;
          noWall = 0;
        }
      }
      i++;
    }
    j=j+2;
    b++;
  }
  
  //get Vertical wall
  j=1;
  a=0;
  b=0;
  i=0;
  while (j<MAP_ROW && b<VERTICAL_ROW) {
   i=0;
   a=0;
   while (i<n && a<VERTICAL_COLUMN) {
     if(map[j][i] == '|') {
       //get initial location of robot and its heading
       if(map[j][i+2] == 'v' || map[j][i+2] == '^' ||map[j][i+2] == '<' || map[j][i+2] == '>') {
         initialLocation[0] = b;
         initialLocation[1] = a;
         initialLocation[2] = getHeading(map[j][i+2]);
       }
       verticalWall[b][a] = 1;
       //i++ because after --- or 3 spaces, the next is always a space bar
       a++;
       i=i+3;
     } else {
       //get initial location of robot and its heading
       if(map[j][i+2] == 'v'|| map[j][i+2] == '^' ||map[j][i+2] == '<' || map[j][i+2] == '>') {
         initialLocation[0] = b;
         initialLocation[1] = a;
         initialLocation[2] = getHeading(map[j][i+2]);
       }
       verticalWall[b][a] = 0;
       i=i+3;
       a++;   
     }
     i++;
   }
   j=j+2;
   b++;
  } 
   
  //initialise cell value to N (rowxcolumn) = 45
  int N = 45;
  for(i=0;i<CELL_ROW;i++) {
    for(j=0;j<CELL_COLUMN;j++) {
      cellValue[i][j] = N;
    }
  }
  
  //set goal cell value to 0
  cellValue[GOAL_CELL_ROW][GOAL_CELL_COLUMN]=0;

  //Flood fill algorithm
  int currentExploredValue = 0;
  int mazeValueChanged = 1;
  int k = 0;
  
  while (mazeValueChanged !=0) {
    mazeValueChanged = 0;
    for (j=0; j<CELL_ROW; j++) {
      for(i=0; i<CELL_COLUMN; i++) {
        if(cellValue[j][i] == currentExploredValue) {
          //check 4 directions
          for(k=0; k<4; k++) {
            //k=0 means its check north 
            switch(k) {
              case NORTH :
                if(horizontalWall[j][i] == 0) {
                  if(cellValue[j-1][i] == N) {
                    cellValue[j-1][i] = cellValue[j][i]+1;
                    mazeValueChanged = 1;
                  }
                }
                break;
              case SOUTH :
                if(horizontalWall[j+1][i] == 0) {
                  if(cellValue[j+1][i] == N) {
                    cellValue[j+1][i] = cellValue[j][i]+1;
                    mazeValueChanged = 1;
                  }
                }
                break;
              case EAST :
                if(verticalWall[j][i+1] == 0) {
                  if(cellValue[j][i+1] == N) {
                    cellValue[j][i+1] = cellValue[j][i]+1;
                    mazeValueChanged = 1;
                  }
                }
                break;
              case WEST :
                if(verticalWall[j][i] == 0) {
                  if(cellValue[j][i-1] == N) {
                    cellValue[j][i-1] = cellValue[j][i]+1;
                    mazeValueChanged = 1;
                  }
                }
                break;
            }  
          }
        }
      }
    }
    currentExploredValue++;
  }
 
  int initialRow = initialLocation[0];
  int initialCol = initialLocation[1];
  int nSteps = cellValue[initialRow][initialCol];
  std::vector<std::vector<int>> paths;
  std::vector<std::pair<int,std::string>> pathList;
  
  paths = printPaths(cellValue,initialRow,initialCol,horizontalWall,verticalWall,mapCopy,nSteps);

  pathList =findLeastSteps(paths,initialLocation[2],initialLocation[0], initialLocation[1]);
  
  printAndStoreLeastStep(cellValue,map,initialRow,initialCol,pathList,paths);
  
  return 0;
}