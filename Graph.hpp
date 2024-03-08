#ifndef GRAPH_H
#define GRAPH_H

#include "Edge.hpp"
#include "Vertex.hpp"
#include "GraphBase.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <list>
#include <limits>
#include <iterator>
#include <map>
#include <unordered_map>
#include <stack>

class Graph : public GraphBase {

//Functions as given to us for the project
public:
    Graph();
    ~Graph();
    void addVertex(std::string label);
    void removeVertex(std::string label);
    void addEdge(std::string label1, std::string label2, unsigned long weight);
    void removeEdge(std::string label1, std::string label2);
    unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path);

//Our additional functions
protected:
    Vertex* findVertex(std::string label);
    bool vertexExist(std::string label);
    bool checkEdge(std::string label1, std::string label2);
    void getPath(std::vector<std::string> &path, std::unordered_map<std::string, std::string> parent, std::string start, std::string end);

//Deque/Map
private:
    //list to keep verteces
    std::deque<Vertex*> vertexList;
    //list to keep edges
    std::deque<Edge*> edgeList;
    std::map<std::string, std::vector<std::pair<unsigned long, std::string>>> adjList;

};


#endif /* GRAPH_H */