//
// Created by gkoch on 25/05/2023.
//

#include "VertexEdge.h"

int Vertex::getId() const {
    return id;
}

void Vertex::addEdge(Vertex* dest, double weight) {
    Edge* edge = new Edge(this, dest, weight);
    adj.push_back(edge);
    dest->incoming.push_back(edge);
}

void Vertex::addChild(Vertex* child) {
    children.push_back(child);
}

const std::vector<Vertex*>& Vertex::getChildren() const {
    return children;
}

bool Vertex::isVisited() const {
    return this->visited;
}

double Vertex::getDist() const {
    return dist;
}

void Vertex::setPath(Edge *path) {
    this->path = path;
}

void Vertex::setDist(double dist) {
    this->dist = dist;
}

const std::vector<Edge*>& Vertex::getAdj() const {
    return adj;
}

void Vertex::setVisited(bool visited) {
    this->visited = visited;
}

Vertex* Edge::getDestination() const {
    return dest;
}

Vertex* Edge::getOrg() const {
    return orig;
}

double Edge::getWeight() const {
    return weight;
}

Edge *Edge::getReverse() const {
    return this->reverse;
}

void Edge::setReverse(Edge *reverse) {
    this->reverse = reverse;
}