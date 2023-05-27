#include <iostream>
#include <vector>
#include <queue>
#include "VertexEdge.h"

#include <functional>

using namespace std;

/********************** Graph  ****************************/

class Graph {
public:
    ~Graph();

    vector<Vertex*> prim();
    void dfsPreOrder(Vertex* vertex);
    void initialize();

    void addVertex(Vertex* vertex) {
        vertexSet.push_back(vertex);
    }

    vector<Vertex*> vertexSet;
protected:
    void visit(Vertex* vertex);

    vector<Vertex*> mst;
};

Graph::~Graph() {
    for (Vertex* v : vertexSet)
        delete v;
}

vector<Vertex*> Graph::prim() {
    initialize();

    priority_queue<pair<double, Vertex*>, vector<pair<double, Vertex*>>, greater<pair<double, Vertex*>>> pq;

    for (Vertex* v : vertexSet) {
        if (!v->isVisited()) {
            v->dist = 0;
            pq.push({0, v});
        }
    }

    while (!pq.empty()) {
        Vertex* v = pq.top().second;
        pq.pop();

        if (!v->isVisited()) {
            mst.push_back(v);
            visit(v);

            for (Edge* e : v->getAdj()) {
                Vertex* u = e->getDestination();
                double weight = e->getWeight();
                if (!u->isVisited() && weight < u->dist) {
                    u->dist = weight;
                    u->setPath(e);
                    pq.push({weight, u});
                }
            }
            v->setVisited(true);  // Mark the current vertex as visited
        }
    }

    return mst;
}

void Graph::visit(Vertex* vertex) {
    vertex->setVisited(true);

    for (Edge* e : vertex->getAdj()) {
        Vertex* v = e->getDestination();
        if (!v->isVisited())
            visit(v);
    }
}

void Graph::dfsPreOrder(Vertex* vertex) {
    if (!vertex->isVisited()) {
        vertex->setVisited(true);
        cout << vertex->getId() << " ";

        for (Edge* e : vertex->getAdj()) {
            Vertex* child = e->getDestination();
            if (!child->isVisited()) {
                // Store the edge used in the father vertex
                child->setPath(e);
                dfsPreOrder(child);
            }
        }
    }
}

void Graph::initialize() {
    for (Vertex* v : vertexSet) {
        v->setVisited(false);
        v->setDist(std::numeric_limits<double>::max());
        v->setPath(nullptr);
    }
    mst.clear();
}


int main() {
    // Create Vertex objects
    Graph graph;

    // Add vertices to the graph
    // Create Vertex objects
    Vertex* v0 = new Vertex(0);
    Vertex* v1 = new Vertex(1);
    Vertex* v2 = new Vertex(2);
    Vertex* v3 = new Vertex(3);
    Vertex* v4 = new Vertex(4);
    Vertex* v5 = new Vertex(5);
    Vertex* v6 = new Vertex(6);
    Vertex* v7 = new Vertex(7);
    Vertex* v8 = new Vertex(8);
    Vertex* v9 = new Vertex(9);
    Vertex* v10 = new Vertex(10);

// Add vertices to the graph
    graph.addVertex(v0);
    graph.addVertex(v1);
    graph.addVertex(v2);
    graph.addVertex(v3);
    graph.addVertex(v4);
    graph.addVertex(v5);
    graph.addVertex(v6);
    graph.addVertex(v7);
    graph.addVertex(v8);
    graph.addVertex(v9);
    graph.addVertex(v10);

// Create Edge objects and add them to the corresponding vertices
    v0->addEdge(v1, 4.6);
    v0->addEdge(v2, 20.6);
    v0->addEdge(v3, 37.8);
    v0->addEdge(v4, 46.3);
    v0->addEdge(v5, 39.5);
    v0->addEdge(v6, 27.1);
    v0->addEdge(v7, 114.1);
    v0->addEdge(v8, 43.4);
    v0->addEdge(v9, 25.9);
    v0->addEdge(v10, 34.2);

    v1->addEdge(v2, 24.7);
    v1->addEdge(v3, 41.1);
    v1->addEdge(v4, 48.3);
    v1->addEdge(v5, 42.8);
    v1->addEdge(v6, 28.8);
    v1->addEdge(v7, 118.2);
    v1->addEdge(v8, 43.8);
    v1->addEdge(v9, 24.3);
    v1->addEdge(v10, 37.9);

    v2->addEdge(v3, 39.3);
    v2->addEdge(v4, 32.6);
    v2->addEdge(v5, 20.6);
    v2->addEdge(v6, 18.4);
    v2->addEdge(v7, 93.5);
    v2->addEdge(v8, 36.9);
    v2->addEdge(v9, 31.4);
    v2->addEdge(v10, 14.6);

    v3->addEdge(v4, 71.5);
    v3->addEdge(v5, 57.1);
    v3->addEdge(v6, 56.8);
    v3->addEdge(v7, 112.1);
    v3->addEdge(v8, 75.4);
    v3->addEdge(v9, 63.0);
    v3->addEdge(v10, 50.9);

    v4->addEdge(v5, 16.8);
    v4->addEdge(v6, 19.5);
    v4->addEdge(v7, 83.0);
    v4->addEdge(v8, 16.2);
    v4->addEdge(v9, 34.1);
    v4->addEdge(v10, 21.8);

    v5->addEdge(v6, 19.2);
    v5->addEdge(v7, 77.5);
    v5->addEdge(v8, 29.3);
    v5->addEdge(v9, 37.9);
    v5->addEdge(v10, 6.4);

    v6->addEdge(v7, 96.3);
    v6->addEdge(v8, 18.6);
    v6->addEdge(v9, 18.7);
    v6->addEdge(v10, 18.0);

    v7->addEdge(v8, 99.0);
    v7->addEdge(v9, 114.8);
    v7->addEdge(v10, 80.9);

    v8->addEdge(v9, 23.1);
    v8->addEdge(v10, 31.9);

    v9->addEdge(v10, 36.3);

    graph.initialize();
    vector<Vertex*> mst = graph.prim();

    cout << "MST vertices: ";
    for (Vertex* vertex : mst) {
        cout << vertex->getId() << " ";
    }
    cout << endl;

    cout << "Starting vertex ID: " << mst[0]->getId() << endl;

    cout << "Adjacent vertices of starting vertex: ";
    for (Edge* edge : mst[0]->getAdj()) {
        cout << edge->getDestination()->getId() << " ";
    }
    cout << endl;

    // Print the MST in pre-order traversal
    cout << "MST in pre-order traversal: ";
    for (Vertex* v : mst) {
        graph.dfsPreOrder(v);
    }  // Start the DFS from the first vertex

    return 0;
}

