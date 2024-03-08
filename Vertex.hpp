#ifndef VERTEX_H
#define VERTEX_H

#include <string>
#include <vector>

//Vertext class
class Vertex {
public:
    Vertex(std::string label) : label(label) {} //Constructor

private:
    std::string label; //Node
    friend class Graph;

};

#endif /* VERTEX_H */