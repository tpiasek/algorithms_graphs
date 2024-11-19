#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <iostream>
#include <vector>
#include <sstream>
#include <queue>


constexpr int DEFAULT_WEIGHT = INT_MAX;


class Vertex
{
public:
    int id;

	Vertex(int id) : id(id) {}
};


class Edge
{
public:
    Vertex* start;
    Vertex* end;
    int weight;

    Edge(Vertex* start, Vertex* end, int weight) : start(start), end(end), weight(weight) {}
};

class SP_Node {
public:
    Vertex* v_end;
    int dist;
    std::vector<Vertex*> path;

    SP_Node(Vertex* v_end, int distance, std::vector<Vertex*> path) : v_end(v_end), dist(distance), path(path) {}
};


class Graph
{
public:
    int no_vertices = 0;
    int no_edges = 0;
    Vertex* sp_vertex = nullptr;


    Graph() = default;
    Graph(int no_vertices, int no_edges) : no_vertices(no_vertices), no_edges(no_edges) {}
    Graph(int no_vertices, int no_edges, Vertex* v_sp) : no_vertices(no_vertices), no_edges(no_edges), sp_vertex(v_sp) {}

    // TODO: implement all required methods
    virtual int addVertex(int id) = 0;
    virtual void updateVertex(int id, int v) = 0;
    virtual void removeVertex(int id) = 0;

    virtual void addEdge(int idStart, int idEnd, int w) = 0;
    virtual void updateEdge(int idStart, int idEnd, int w) = 0;
    virtual void removeEdge(int idStart, int idEnd) = 0;

    virtual std::vector<Vertex*> getNeighbours(int v) = 0;
    //virtual void checkNeighbour(int id, int n) = 0;

    virtual int findVertexPos(int v_id) = 0;
    virtual Edge* findEdge(int idStart, int idEnd) = 0;
    virtual void print() = 0;

    virtual std::vector<SP_Node*> spDijkstra(int idStart) = 0;
    virtual std::vector<SP_Node*> spBellmanFord(int idStart) = 0;

    int getNoVertices() const {
		return no_vertices;
	}
    int getNoEdges() const {
		return no_edges;
	}
    int getSPVertexId() const {
        if(sp_vertex == nullptr) return -1;
        return sp_vertex->id;
    }
    Vertex* getSPVertex() const {
		return sp_vertex;
	}
};

#endif /* GRAPH_HPP_ */
