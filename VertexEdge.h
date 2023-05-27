#ifndef DA_TP_CLASSES_VERTEX_EDGE
#define DA_TP_CLASSES_VERTEX_EDGE

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

class Edge;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

class Vertex {
public:
    Vertex(int id) : id(id) {}

    int getId() const;
    void addEdge(Vertex* dest, double weight);
    bool isVisited() const;
    const std::vector<Edge*>& getAdj() const;
    void setVisited(bool visited);
    void setPath(Edge *path);
    void setDist(double dist);
    double getDist() const;

    void addChild(Vertex* child);

    const std::vector<Vertex*>& getChildren() const;

    double dist = 0;
protected:
    int id; // identifier
    std::vector<Edge*> adj; // outgoing edges

    // auxiliary fields
    bool visited = false;
    bool processing = false;
    Edge* path = nullptr;

    std::vector<Edge*> incoming; // incoming edges

    int queueIndex = 0; // required by MutablePriorityQueue and UFDS

    std::vector<Vertex*> children;
};

/********************** Edge  ****************************/

class Edge {
public:
    Edge(Vertex* orig, Vertex* dest, double w) : orig(orig), dest(dest), weight(w) {}

    Vertex* getDestination() const;
    Vertex* getOrg() const;

    double getWeight() const;

    Edge *getReverse() const;

    void setReverse(Edge *reverse);


protected:
    Vertex* orig; // origin vertex
    Vertex* dest; // destination vertex
    double weight; // edge weight, can also be used for capacity
    Edge *reverse = nullptr;
};


#endif /* DA_TP_CLASSES_VERTEX_EDGE */