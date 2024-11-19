#ifndef MINIMUM_SPANNING_TREE_ALGORITHMS_HPP_
#define MINIMUM_SPANNING_TREE_ALGORITHMS_HPP_

#include "graphs/graph.hpp"
#include <vector>

struct MinimumSpanningEdge {
    int v1;
    int v2;
    int weight;

    bool operator<(const MinimumSpanningEdge &edge) const {
        return std::min(v1, v2) != std::min(edge.v1, edge.v2) ? std::min(v1, v2) < std::min(edge.v1, edge.v2) :
               std::max(v1, v2) < std::max(edge.v1, edge.v2);
    }

    bool operator==(const MinimumSpanningEdge &edge) const {
        return std::min(v1, v2) == std::min(edge.v1, edge.v2) && std::max(v1, v2) == std::max(edge.v1, edge.v2) &&
               weight == edge.weight;
    }
};

using MinimumSpanningTreeResult = std::vector<MinimumSpanningEdge>;

void kruskal(Graph &graph, MinimumSpanningTreeResult &result);

void prim(Graph &graph, MinimumSpanningTreeResult &result);

#endif /* MINIMUM_SPANNING_TREE_ALGORITHMS_HPP_ */

