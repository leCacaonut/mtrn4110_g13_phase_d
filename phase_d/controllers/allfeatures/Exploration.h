// Exploration header file

class ExploreMap {
   private:
    int mapSize[2];
    int goal[2];
    vector<int> emptyGrid{2, 0};

    vector<vector<bool>> exploredMap;
    vector<vector<bool>> hWalls;
    vector<vector<bool>> vWalls;

   public:
    ExploreMap();
    ExploreMap(int row, int col);
    ~ExploreMap();

    // setters
    void setDefaultGoal();
    void setGoal(int row, int col);
    void setExplored(int v[2], char heading, char* walls);

    // getters
    int* getMapSize();
    int* getGoal();
    bool getExplored(int row, int col);
    vector<vector<bool>> getExplored();
    vector<vector<bool>> getHWalls();
    vector<vector<bool>> getVWalls();

    // sizing functions
    int* size2DVector(int* s, vector<vector<bool>> v);
    void resize2DVector(vector<vector<bool>>& v, unsigned int numRows, unsigned int numCols);
    void print2DVector(vector<vector<bool>> v);

    // functions
    void setWalls(int position[2], char heading, char* walls);
    void addHWall(int row, int col);
    void addVWall(int row, int col);
    void removeHWall(int row, int col);
    void removeVWall(int row, int col);
    void addWallBorder();
    vector<vector<bool>> flipMap(vector<vector<bool>> v);
    vector<vector<bool>> swapColumns(vector<vector<bool>> v);
    bool gridValid(int row, int col);
    bool findEmptyGrid();

    // main function
    template <typename T>
    void explore(T& robot);
};
