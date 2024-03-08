#ifndef EDGE_H
#define EDGE_H

#include <string>

//Edge Class
class Edge {
public:
    Edge(std::string a, std::string b, unsigned long w) //Constructor
    : edge1(a), edge2(b), weight(w) {}
    ~Edge() {} //Deconstructor

private:
    std::string edge1; //Edge 1
    std::string edge2; //Edge 2
    unsigned long weight; //Weight of the edge

    friend class Graph;

};

#endif /* EDGE_H */