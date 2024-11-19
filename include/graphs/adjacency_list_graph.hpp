#ifndef ADJACENCY_LIST_GRAPH_HPP_
#define ADJACENCY_LIST_GRAPH_HPP_

#include <memory>
#include <list>

#include "graphs/graph.hpp"
#include "graphs/node.hpp"


class AdjacencyListGraph : public Graph
{
private:
	std::vector<Vertex*> vertices;
    std::vector<std::list<Edge*>> adjacencyList;

public:
    AdjacencyListGraph() = default;
    AdjacencyListGraph(int no_vertices, int no_edges);
    AdjacencyListGraph(int no_vertices, int no_edges, std::vector<Vertex*> vertices, std::vector<std::list<Edge*>> adjacencyList);
    AdjacencyListGraph(int no_vertices, int no_edges, std::vector<Vertex*> vertices, std::vector<std::list<Edge*>> adjacencyList, Vertex* v_sp);

    static std::unique_ptr<Graph> createGraph(std::istream& is); // Done
    
    int addVertex(int id) override; // Done
    void updateVertex(int id, int v) override; // Done
    void removeVertex(int id) override; // Done

    void addEdge(int idStart, int idEnd, int w) override; // Done
    void updateEdge(int idStart, int idEnd, int w) override; // Done
    void removeEdge(int idStart, int idEnd) override; // Done

    std::vector<Vertex*> getNeighbours(int id) override; // Done
    bool checkNeighbour(int id, int n); // Done

    Vertex* findVertex(int id); // Done
    int findVertexPos(int v_id) override; // Done
    Edge* findEdge(int idStart, int idEnd) override; // Done
    int findEdgePos(int idStart, int idEnd); // Done
    void print() override; //Done

    std::vector<SP_Node*> spDijkstra(int idStart); // Done
    std::vector<SP_Node*> spBellmanFord(int idStart); // Done
};

#endif /* ADJACENCY_LIST_GRAPH_HPP_ */
