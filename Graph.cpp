/*
Name: Felicia Drysdale, John Rojas, Abylkhair Zholdybay
Class: COP 4530 - Section 3
Project 4
Summary: This project implements a undirected weighted Graph ADT and aslo forms Dijkstra's Algorithm (shortestPath)
to find the shortest path between two vertices. This code uses an adjacency list to perform these operations.
*/

#include "Graph.hpp"

//Constructor
Graph::Graph(){
    adjList = std::map<std::string, std::vector<std::pair<unsigned long, std::string>>>();
}

//Deconstructor
Graph::~Graph(){
    for (const auto& vertex : vertexList){
        delete vertex;
    }
    for (const auto& edge : edgeList) { 
        delete edge;
    }
    vertexList.clear();
    edgeList.clear();
}

//Finds vertex by its label in the vertexList in the deque
Vertex* Graph::findVertex(std::string label){
    for (Vertex* vertex : vertexList){
        if (vertex->label == label){
            return vertex;
        }
    }
    return nullptr;
}

//Checks if the vertex with label name exists already
bool Graph::vertexExist(std::string label){
    return findVertex(label) != nullptr;
}

//Checks if the edge exists already
bool Graph::checkEdge(std::string label1, std::string label2){

     for (const auto& edge : edgeList) {
        if ((edge->edge1 == label1 && edge->edge2 == label2) ||
            (edge->edge1 == label2 && edge->edge2 == label1)) {
            return true;
        }
    }
    return false;
}

/*Creates and addsvertex graph with label.*/
void Graph::addVertex(std::string label) {
    //checkes that no other vertex has the same label
    if (vertexExist(label)){
        return;
    }
    //Creates new label
    Vertex* new_vertex = new Vertex(label);

    //Adds to the deque
    vertexList.push_back(new_vertex);

}

/*Removes the vertex with label and edges b/w vertex and other vertices*/
void Graph::removeVertex(std::string label) {
    //remove vertex
    for (std::deque<Vertex*>::iterator itrVer = vertexList.begin(); itrVer != vertexList.end();){
        if ((*itrVer)->label == label){
            vertexList.erase(itrVer);
        } else {
            ++itrVer;
        }
    }
    //Remove edges
    for (std::deque<Edge*>::iterator itrEdge = edgeList.begin(); itrEdge != edgeList.end();){
        if (((*itrEdge)->edge1 == label) || ((*itrEdge)->edge2 == label)){
            edgeList.erase(itrEdge);
        } else { 
            ++itrEdge;
        }
    }

    // Remove vertex and associated edges from adjList
    adjList.erase(label);

    // Remove references to the vertex in the adjacency lists of other vertices, use remove_if to do this all in one line
    for (auto& element : adjList) {
        element.second.erase(std::remove_if(element.second.begin(), element.second.end(),[label](const std::pair<unsigned long, std::string>&x) {
            return x.second == label;}),
            element.second.end());
    }
}

/*Adds edge of val weight b/w vertex with label1 and vertex with label2.
there must not already be an edge between those vertices, and a vertex cannot have an edge to itself*/
void Graph::addEdge(std::string label1, std::string label2, unsigned long weight) {

    //If vertex with label 1 and label with label 2 do not exist, return
    if (!vertexExist(label1) || !vertexExist(label2)){
        return;
    }
    //checks if edge to itself
    if (label1 == label2){
        return;
    }
    //checks if already an edge between vertices
    if (checkEdge(label1, label2)){
        return;
    }

    // Add the edge to the adjacency list of both vertices
    //The first element of the pair is the weight, the second element is the label of the vertex
    //Emplace back adds the element to the end of the vector (push_back was not working for some reason)
    adjList[label1].emplace_back(weight, label2);
    adjList[label2].emplace_back(weight, label1);

}

/*Removes the edge from the graph between the vertex with label1 and the vertex with label2. A
vertex with label1 and a vertex with label2 must both exist and there must be an edge between
those vertices*/
void Graph::removeEdge(std::string label1, std::string label2) {

    //If vertex with label 1 and label with label 2 do not exist, return
    if (!vertexExist(label1) || !vertexExist(label2)){
        return;
        
    }
    //Checks if edge is between both vertices
    if (checkEdge(label1, label2)){
        return;
    }

    //Removes edge
    for (std::deque<Edge*>::iterator itrEdge = edgeList.begin(); itrEdge != edgeList.end();){
        if (((*itrEdge)->edge1 == label1) && ((*itrEdge)->edge2 == label2)){
            edgeList.erase(itrEdge);
        } else {
            itrEdge++;
        }

    }
}

/*Calculates the shortest path between the vertex with startLabel and the vertex with endLabel
using Dijkstra's Algorithm. A vector is passed into the method that stores the shortest path
between the vertices. The return value is the sum of the edges between the start and end
vertices on the shortest path.*/
unsigned long Graph::shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path) {

    //Declare priority queue
    std::priority_queue<std::pair<unsigned long, std::string>, std::vector<std::pair<unsigned long, std::string>>, 
    std::greater<std::pair<unsigned long, std::string>>> pq;

    //Create a map to store distances from the source vertex to all other vertices
    std::unordered_map<std::string, unsigned long> dist;
    std::unordered_map<std::string, std::string> parent; // Map to store parent information for constructing the path

    //Initialize distances, first; all to infinity, second; first to empty
    for (const auto& node : adjList) {
        dist[node.first] = INT_MAX;
        parent[node.first] = "";
    }

    //Insert the source vertex into the priority queue and initialize its distance as 0
    pq.push({0, startLabel});

    //Set the distance of the source vertex to 0
    dist[startLabel] = 0;

    //Loop until the priority queue is empty
    while (!pq.empty()) {

        // Extract the minimum distance vertex from the priority queue
        auto current = pq.top();
        pq.pop();

        //Get the vertex and distance of the current vertex
        std::string v1 = current.second;

        for (const auto& edgePair : adjList[v1]) {
            // Get the vertex and weight of the current adjacent vertex
            // Extract the second element of the pair, which is the vertex
            std::string v2 = edgePair.second; 
            unsigned long weight = edgePair.first;

            // If the distance to the adjacent vertex through the current vertex is less than its current distance, update the distance
            if (dist[v2] > dist[v1] + weight) {
                dist[v2] = dist[v1] + weight;
                pq.push({dist[v2], v2});

                //Update parent information for constructing the path
                parent[v2] = v1; 
            }
        }
    }

    // Construct the path from source to destination
    getPath(path, parent, startLabel, endLabel);

    //Return the distance from source to destination
    return dist[endLabel]; 
}

// Gets the shortest path from source to destination and inserts pathNodes into a vector
void Graph::getPath(std::vector<std::string> &path, std::unordered_map<std::string, std::string> parent, std::string start, std::string end)
{
    //Declare stack and final string
    std::stack<std::string> mystack;
    std::string final = end;

    //Check if the destination is reachable
    if (parent[end] == "") {
        path.clear();  // Clear the path if the destination is not reachable
        return;
    }

    //Insert all nodes between source and target into stack
    while (parent[end] != start)
    {
        end = parent[end];
        mystack.push(end);
    }

    //Push source and target into path
    path.push_back(start);

    //Empty stack and push into path
    while (!mystack.empty())
    {
        //Push into path
        path.push_back(mystack.top());
        mystack.pop();
    }

    //Push target into path
    path.push_back(final);
}