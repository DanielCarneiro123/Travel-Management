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
#include <unordered_set>
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

    double PathCost(const std::vector<int> path);

    void permute(std::vector<int> &path, int start, double &minCost, std::vector<int> &bestPath);

    std::vector<int> solveTSP();

    void
    tsp(Vertex* currVertex, int currInd, double currDist, double &minDist,
        std::vector<Vertex*> &path, std::vector<Vertex*> &minpath);

    double tspBT(int initialNode, std::vector<Vertex*> &path);

    void TriangleApprox();


    std::vector<Vertex *> tsp_v2();

    void Prim();

    double calculatePathDistance(const std::vector<Vertex *> &path);

    void dfs(Vertex *currentVertex, std::vector<Vertex *> &path);

    std::vector<Vertex *> preOrderTraversal(Vertex *vertex);

    bool addVertexV2(const int &id, double longitude = 0, double latitude = 0);

    std::vector<std::vector<Vertex*>> divideGraph(const std::vector<Vertex*> vertexSet);
    std::vector<std::vector<Vertex*>> clusteringFunction(const std::vector<Vertex*>& subPath);

    void tspCombined();

    std::vector<Vertex *> OddVertex(Graph mst);

    std::vector<int> findMinimumCostPerfectMatching(const std::vector<Vertex *> &oddVertices);

    std::vector<Vertex*> findEulerianCircuit();

    Graph createSubgraph(const std::vector<Vertex*> &oddV);


    bool addBidirectionalEdge2(Vertex *&v1, Vertex *&v2, double w);

    std::vector<Vertex*> convertToHamiltonianPath();

    std::vector<Edge*> MinimumPerfectMatching(std::vector<Vertex*>oddVert);

    void uniteGraphs(std::vector<Edge*>finalEdges);

    Graph createMST();

    std::vector<Vertex *> findEulerianCycle(Vertex *startVertex);

    double hamiltonPath(std::vector<Vertex *> eurlerian);

    void final4_3(Graph &gextra);

    void aux4_2(Graph &gextra);
    void aux4_1(Graph &gextra);



protected:
    std::vector<Vertex *> vertexSet;    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const int &id) const;

    bool isVisited(int currInd, std::vector<int> &path);


    std::vector<Vertex *> prim();


    double haversine(double lat1, double lon1, double lat2, double lon2);

    bool hasCon(Vertex* v1, Vertex* v2);



    std::vector<Vertex *> preOrderTraversal2(Vertex *vertex, std::vector<Vertex *> cluster);

    std::vector<Vertex *> tspTwoStepApproximation(const std::vector<Vertex *> &cluster);

    double getDistance(Vertex *v1, Vertex *v2);

    Vertex *findNearestVertex(Vertex *&vertex, Graph &graph);


    std::vector<std::pair<Vertex *, Vertex *>>
    findMissingPoints(const Graph &oddDegreeSubgraph, Graph &minimumSpanningTree);

    void DFS(Vertex* v, std::vector<Edge*>& visitedEdges, std::vector<std::vector<Vertex*>>& cycles);

    bool IsOdd(Vertex *v, std::vector<Vertex*>oddVertices);


    double convert_to_radians(double coord);


};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);


#endif //PROJETO2_DA_GRAPH_H