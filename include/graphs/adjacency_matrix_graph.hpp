#ifndef ADJACENCY_MATRIX_GRAPH_HPP_
#define ADJACENCY_MATRIX_GRAPH_HPP_

#include <memory>

#include "graphs/graph.hpp"

class AdjacencyMatrixGraph : public Graph
{
    std::vector<Vertex*> vertices;
    std::vector<std::vector<int>> adjacencyMatrix;

  public:
    AdjacencyMatrixGraph() = default;
    AdjacencyMatrixGraph(int no_vertices, int no_edges);
    AdjacencyMatrixGraph(int no_vertices, int no_edges, std::vector<Vertex*> vertices, std::vector<std::vector<int>> adjacencyMatrix);
    AdjacencyMatrixGraph(int no_vertices, int no_edges, std::vector<Vertex*> vertices, std::vector<std::vector<int>> adjacencyMatrix, Vertex* v_sp);

    static std::unique_ptr<Graph> createGraph(std::istream& is); // Done

    int addVertex(int id) override; // Done (check)
    void updateVertex(int id, int v) override; // Done (check)
    void removeVertex(int id) override; // Done (check)

    void addEdge(int idStart, int idEnd, int w) override; // Done
    void updateEdge(int idStart, int idEnd, int w) override; // Done
    void removeEdge(int idStart, int idEnd) override; // Done

    std::vector<Vertex*> getNeighbours(int id) override; // Done
    bool checkNeighbour(int id, int n); // Done

    int findVertexPos(int v_id) override; // Done (check)
    Edge* findEdge(int idStart, int idEnd) override; // Done (consider returned type)
    void print() override; // Done
    void printMatrix(); // Done
    
    std::vector<SP_Node*> spDijkstra(int idStart) override; // Done
    std::vector<SP_Node*> spBellmanFord(int idStart); // Done

public:
    std::vector<Vertex*>* getVertices() { return &vertices; }
    std::vector<std::vector<int>>* getAdjacencyMatrix() { return &adjacencyMatrix; }
};

#endif /* ADJACENCY_MATRIX_GRAPH_HPP_ */
