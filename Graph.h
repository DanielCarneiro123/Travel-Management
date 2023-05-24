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
#include "MutablePriorityQueue.h"

#include "VertexEdge.h"



class Graph {
public:
    ~Graph();
    /*
    * Auxiliary function to find a vertex with a given ID.
    */
    Vertex *findVertex(const int &id) const;
    /*
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const int &id);

    /*
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const int &sourc, const int &dest, double w);
    bool addBidirectionalEdge(const int &sourc, const int &dest, double w);

    int getNumVertex() const;
    std::vector<Vertex *> getVertexSet() const;

    void initialize(const std::string &filename);


    void printPath();

    double PathCost(const std::vector<int> &path);

    void permute(std::vector<int> &path, int start, double &minCost, std::vector<int> &bestPath);

    std::vector<int> solveTSP();

    void
    tsp(Vertex* currVertex, int currInd, double currDist, double &minDist,
        std::vector<Vertex*> &path, std::vector<Vertex*> &minpath);

    double tspBT(int initialNode, std::vector<Vertex*> &path);

protected:
    std::vector<Vertex *> vertexSet;    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const int &id) const;

    bool isVisited(int currInd, std::vector<int> &path);



};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);


#endif //PROJETO2_DA_GRAPH_H