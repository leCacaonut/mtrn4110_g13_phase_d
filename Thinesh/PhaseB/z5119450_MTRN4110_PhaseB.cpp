// File:          z5119450_MTRN4110_PhaseB.cpp
// Date:          11/07/2020
// Description:   Phase B program, reads a map and finds the
//                lowest cost path to the center of the maze
// Author:        Thinesh Manisekaran, z5119450
// Modifications:

#include <webots/Robot.hpp>

#include <sstream>  
#include <fstream>
#include <iostream>
#include <list>
#include <string>

// grid information
#define SQUARE_SIZE        4
#define NUM_ROWS           5
#define NUM_COLS           9
#define ROW_CHARS          38

// generating parent information
#define VALID_PARENT       0
#define NEW_PARENT         1
#define IGNORE             2
#define UNEXPLORED         100

// start and stop information
#define DEST               22
#define LSTART             2
#define RSTART             34
#define NUM_VERTS          (NUM_ROWS * NUM_COLS)

// instruction info
#define TL                 0
#define TR                 8
#define BL                 36
#define BR                 44

#define TOP_LEFT           "00"
#define TOP_RIGHT          "08"
#define BOT_LEFT           "40"
#define BOT_RIGHT          "48"


// file path
#define MAP_FILE_NAME       "../../Map.txt"
#define PATH_PLAN_FILE_NAME "../../PathPlanFound.txt"

// namespaces
using namespace webots;
using namespace std;

// Map display variables
string PathView; // current path
string MapView;  // initial map read

list<int>       vertices[NUM_VERTS]; // list of adjacent vertices
list<int>       parents[NUM_VERTS];  // list of parents for each vertex 
list<int>       currPath;            // current path being generated
list<list<int>> paths;               // all generated paths 
list<int>       bestPath;            // most efficient path

int distances[NUM_VERTS]; // Distances from source to each vertex          
 
// starting information
string heading;      // current heading
bool   horiz;        // current direction 
int    source = -1;  // source vertex
int    startRow = 0; // starting row  
int    startCol = 0; // starting column

void checkSource(string line, int row);
void updateVertices(string border, string squares, int row);
void addEdge(int col, int row, bool top);
void replace(int vertex, string val);

// debugging
void debug();
void displayEdges();
void displayParents();
void displayDistances();

void readDisplayMap()
{
  string border;
  string squares;
  
  ifstream Map(MAP_FILE_NAME);
    
  for (int row = 0; row < NUM_ROWS; row++) {
    getline(Map, border);
    getline(Map, squares);
    
    if (source == -1) {
      if (row == 0 || row == NUM_ROWS - 1) {
        checkSource(squares, row);
      }
    }

    MapView += border + "\n";
    MapView += squares + "\n";

    updateVertices(border, squares, row);
  }
  
  getline(Map, border);
  MapView += border + "\n";
  cout << MapView;
  Map.close();
}

void setHeading(char direction)
{
  switch (direction) {
    case '^':
      heading = "N";
      break;
    case '>':
      heading = "E";
      break;
    case 'v':
      heading = "S";
      break;
    case '<':
      heading = "W";
      break;
    default:
      break;
  }
}

void updateSource(char start, bool left, int row)
{
  left ? source = row * NUM_COLS : source = (row + 1) * NUM_COLS - 1;  
  (start == '>' || start == '<') ? horiz = true : horiz = false;
}

void checkSource(string line, int row)
{
  if (line[LSTART] != ' ') {
    setHeading(line[LSTART]);
    updateSource(line[LSTART], 1, row);
    return;
  }
  
  if (line[RSTART] != ' ') {
    setHeading(line[RSTART]);
    updateSource(line[RSTART], 0, row);
    return;
  }
}

void updateVertices(string border, string squares, int row)
{  
  
  for (int col = 0; col < NUM_COLS; col++) {
    int rightBorderIndex = (col + 1) * SQUARE_SIZE;
    int topBorderIndex = rightBorderIndex - 1;

    if (squares[rightBorderIndex] != '|') {addEdge(col, row, 0);};
    if (border[topBorderIndex] != '-') {addEdge(col, row, 1);};
  }
}

void addEdge(int col, int row, bool top)
{
  int source = row * NUM_COLS + col;
  int dest;
  
  top ? (dest = source - NUM_COLS) : (dest = source + 1);

  vertices[source].push_back(dest);
  vertices[dest].push_back(source);
}

void bfs()
{
  list<int> bfsQueue;

  bfsQueue.push_back(source);
  parents[source].push_back(-1);
  
  for (int i = 0; i < NUM_VERTS; i++) {
    distances[i] = UNEXPLORED;
  }
  
  distances[source] = 0;
  
  while (!bfsQueue.empty()) {
    int currVertex = bfsQueue.front();
    bfsQueue.pop_front();
    
    for (auto adj: vertices[currVertex]) {
      int difference = distances[adj] - distances[currVertex];
      int newParent;
      
      if      (difference == 1)  {newParent = VALID_PARENT;}  //another valid parent
      else if (difference > 1)   {newParent = NEW_PARENT;}    //more efficient path
      else                       {newParent = IGNORE;};       //ignore avoids repeats
                 
      switch (newParent) {
        case NEW_PARENT: // unexplored or larger distance
          distances[adj] = distances[currVertex] + 1;
          parents[adj].clear();
          parents[adj].push_back(currVertex);
          bfsQueue.push_back(adj);
          break;
          
        case VALID_PARENT: // explored and same distance
          parents[adj].push_back(currVertex);
          break;
          
        default:
          break;
      }
    }
  }
}

void findShortestPaths(int dest)
{
  if (dest == -1) {
    paths.push_back(currPath); // found source
    return;
  }

  for (auto parent : parents[dest]) {
    currPath.push_back(dest);  // add vertex to path
    findShortestPaths(parent); // find path to parent
    currPath.pop_back();       // remove explored possibility
  }
}

void updateMap(list<int> path)
{
  stringstream ss;
  string val;
  int distanceCounter = 0;
  
  for (auto vertex : path) {
    if (vertex == source) break;
    
    val = to_string(distanceCounter);
    
    (val.length() == 2) ? val = " " + val : val = " " + val + " ";

    replace(vertex, val);
    distanceCounter += 1;
  }
}

void displayPaths() 
{
  PathView = MapView;
  int pathCounter = 1;
  
  for (auto path : paths) {
  
    cout << "--- Path " << pathCounter << " ---" << endl;

    updateMap(path);

    cout << PathView;
    PathView = MapView;
    
    pathCounter += 1;
  }
  
}

void replace(int vertex, string val)
{
  int row = 2 * (vertex / (NUM_COLS)) + 1;
  int col = vertex - ((row-1)/2) * NUM_COLS;
  
  int row_offset = row * ROW_CHARS;
  int col_offset = col * SQUARE_SIZE + 1;
  int total_offset = row_offset + col_offset;

  PathView[total_offset] = val[0];
  PathView[total_offset + 1] = val[1];
  PathView[total_offset + 2] = val[2];
}

void updateCost(int *cost, bool *currHoriz, int diff, bool horiz)
{
  if ((horiz && diff == NUM_COLS) || (!horiz && diff == 1)) {
    *currHoriz = !(*currHoriz); 
    *cost += 2;
  } else {
    *cost += 1;
  }
}

int getCost(list<int> path)
{
  int cost = 0;
    
  int currVertex = path.front();
  path.pop_front();
  
  int compareVertex = 0;
  bool currHoriz = horiz;

  while (!path.empty()) {
    compareVertex = path.front();
    int diff = abs(compareVertex - currVertex);
    
    switch (currHoriz) {
      case true:
        updateCost(&cost, &currHoriz, diff, true);
        break;
        
      default:
        updateCost(&cost, &currHoriz, diff, false);
        break;     
    }
    
    currVertex = compareVertex;
    path.pop_front();
  }
     
  return cost;
}

void setInitInfo(string *instructions)
{
  switch(source) {
    case TL:
      *instructions += TOP_LEFT;
      break;
    case TR:
      *instructions += TOP_RIGHT;
      break;
    case BL:
      *instructions += BOT_LEFT;
      break;
    case BR:
      *instructions += BOT_RIGHT;
      break;
    default:
      break;  
  }
  
  *instructions += heading;
}

string generatePath()
{
  string currHead = heading;
  string instructions = "";
  string directions = "";
  
  setInitInfo(&instructions);
 
  bestPath.reverse();
   
  int currVertex = bestPath.front();
  bestPath.pop_front();
 
  int compareVertex = 0;
  
  while (!bestPath.empty()) {
    compareVertex = bestPath.front();
    int diff = compareVertex - currVertex;
    
    switch (diff) {
      case 1:         // heading east
        if      (currHead == "N") {currHead = "E"; directions += "RF";}
        else if (currHead == "S") {currHead = "E"; directions += "LF";}
        else                      {directions += "F";};
  
        break;
        
      case -1:        // heading west
        if      (currHead == "N") {currHead = "W"; directions += "LF";}
        else if (currHead == "S") {currHead = "W"; directions += "RF";}
        else                      {directions += "F";};
  
        break;
        
      case NUM_COLS:  // heading north
        if      (currHead == "E") {currHead = "S"; directions += "RF";}
        else if (currHead == "W") {currHead = "S"; directions += "LF";}
        else                      {directions += "F";};
  
        break; 
             
      case -NUM_COLS: // heading south
        if      (currHead == "E") {currHead = "N"; directions += "LF";}
        else if (currHead == "W") {currHead = "N"; directions += "RF";}
        else                      {directions += "F";};
  
        break;
            
      default:
        break;
    }
    
    currVertex = compareVertex;
    bestPath.pop_front();
  }
  instructions += directions;

  return instructions;
}

string analysePaths() 
{
  int leastCost = INT_MAX;
  int counter = 1;

  for (auto path: paths) {
    int cost = getCost(path);

    if (cost <= leastCost) {
      bestPath = path;    
      leastCost = cost;
    }
    counter++;
  }
  
  PathView = MapView;
  updateMap(bestPath);
  cout << PathView;
  cout << "Steps: " << leastCost << endl;

  string shortestPath = generatePath();
  cout << "Path: " << shortestPath << endl;
  
  return shortestPath;
}

void writeToFile(string path)
{
  cout << "File: ./PathPlanFound.txt" << endl;
  cout << "Path: " << path << endl;

  ofstream Path;
  Path.open(PATH_PLAN_FILE_NAME);
  Path << path;
  Path.close();
}


int main(int argc, char **argv) {

  cout << "--- Task 1 ---" << endl;
  readDisplayMap();  

  bfs();
  
  cout << "--- Task 2 ---" << endl;
  findShortestPaths(DEST);
  displayPaths();

  cout << "--- Task 3 ---" << endl;
  string shortestPath = analysePaths();
 
  cout << "--- Task 4 ---" << endl;
  writeToFile(shortestPath);
 
  return 0;
}

// DEBUGGING ======================================

void debug() 
{
  displayEdges();
  displayParents();
  displayDistances();
}


void displayDistances() {
  int i = 0;
  for (auto it:distances) {
    cout << "[" << it << "]" << "<-" << i << endl;
    i++;
  }
}

void displayParents() {
  for (int i = 0; i < 45; i++) {
    cout << i << " -> ";
    for(auto const& it : parents[i]){
      cout << it << " - ";
    }
    cout << endl;
  }
}

void displayEdges()
{
  for (int i = 0; i < 45; i++) {
    cout << i << " -> ";
    for(auto const& it : vertices[i]){
      cout << it << " - ";
    }
    cout << endl;
  }
}


