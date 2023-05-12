//
// Created by Daniel on 04/05/2023.
//

#ifndef PROJETO2_DA_GRAPH_H
#define PROJETO2_DA_GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>




using namespace std;

class Graph {
public:

    bool readEdges(std::vector<vector<int>> &paths);

    void tsp(vector<int> &currPath, double currDist, int currInd, vector<vector<double>> &graph);

    void matrixForm();
    void printPath();
protected:


    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const int &id) const;

    void initialize(const std::string &filename);

    void readVertex(const std::string &filename);
    

    bool isVisited(int city, vector<int> &path);

};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);


#endif //PROJETO2_DA_GRAPH_H
