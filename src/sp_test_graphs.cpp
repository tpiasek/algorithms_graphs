#include <sstream>
#include <string>
#define CATCH_CONFIG_MAIN

#include "catch2/catch.hpp"

#include "graphs/adjacency_list_graph.hpp"
#include "graphs/adjacency_matrix_graph.hpp"
#include "graphs/shortest_path_algorithms.hpp"
#include <filesystem>
#include <fstream>

using namespace std::string_literals;

const std::filesystem::path dataDirectoryPath{DATA_DIR_PATH};

void readShortestPathResult(std::istream& is, ShortestPathResult& result)
{
    result.clear();

    while(is)
    {
        std::string line;
        std::getline(is, line);
        std::istringstream iss{line};

        int endVertexIndex, cost;
        iss >> endVertexIndex >> cost;

        if(is)
        {
            std::vector<int> vertices;
            while(iss)
            {
                int vertexIndex;
                iss >> vertexIndex;

                if(iss)
                {
                    vertices.push_back(vertexIndex);
                }
            }

            result[endVertexIndex] = std::make_pair(cost, vertices);
        }
    }
}

void checkShortestPathResult(const ShortestPathResult& result, const ShortestPathResult& refResult)
{
    REQUIRE(refResult.size() == result.size());
    for(auto& [refVertexIndex, refValue] : refResult)
    {
        auto it = result.find(refVertexIndex);
        INFO("Checking shortest path to vertex " << refVertexIndex);
        REQUIRE(it != result.end());

        auto& vertices = it->second.second;
        auto& refVertices = refValue.second;

        INFO("Checking cost == refCost: " << it->second.first << " == " << refValue.first);
        REQUIRE(it->second.first == refValue.first);
        REQUIRE(refVertices.size() == vertices.size());

        for(int v = 0; v < refVertices.size(); ++v)
        {
            INFO("Checking vertices[" << v << "] == refVertices[" << v << "]: " << vertices[v]
                                      << " == " << refVertices[v]);

            REQUIRE(vertices[v] == refVertices[v]);
        }
    }
}

TEST_CASE("Adjacency Matrix Graph -- Dijkstra")
{
    auto [inputFile, refFile] = GENERATE(std::make_tuple(dataDirectoryPath / "graph" / "graphV10D0.5.txt",
                                                         dataDirectoryPath / "sp_result" / "spV10D0.5.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV30D0.25.txt",
                                                         dataDirectoryPath / "sp_result" / "spV30D0.25.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV200D0.75.txt",
                                                         dataDirectoryPath / "sp_result" / "spV200D0.75.txt"));

    std::ifstream inputStream{inputFile}, refStream{refFile};
    auto graph = AdjacencyMatrixGraph::createGraph(inputStream);

    ShortestPathResult result, refResult;
    readShortestPathResult(refStream, refResult);

    int sourceIndex;
    inputStream >> sourceIndex;

    dijkstra(*graph, sourceIndex, result);

    checkShortestPathResult(result, refResult);
}

TEST_CASE("Adjacency List Graph -- Dijktra")
{
    auto [inputFile, refFile] = GENERATE(std::make_tuple(dataDirectoryPath / "graph" / "graphV10D0.5.txt",
                                                         dataDirectoryPath / "sp_result" / "spV10D0.5.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV30D0.25.txt",
                                                         dataDirectoryPath / "sp_result" / "spV30D0.25.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV200D0.75.txt",
                                                         dataDirectoryPath / "sp_result" / "spV200D0.75.txt"));

    std::ifstream inputStream{inputFile}, refStream{refFile};
    auto graph = AdjacencyListGraph::createGraph(inputStream);

    ShortestPathResult result, refResult;
    readShortestPathResult(refStream, refResult);

    int sourceIndex;
    inputStream >> sourceIndex;

    dijkstra(*graph, sourceIndex, result);

    checkShortestPathResult(result, refResult);
}

TEST_CASE("Adjacency Matrix Graph -- Bellman-Ford")
{
    auto [inputFile, refFile] = GENERATE(std::make_tuple(dataDirectoryPath / "graph" / "graphV10D0.5Negative.txt",
                                                         dataDirectoryPath / "sp_result" / "spV10D0.5Negative.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV30D0.25Negative.txt",
                                                         dataDirectoryPath / "sp_result" / "spV30D0.25Negative.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV200D0.75.txt",
                                                         dataDirectoryPath / "sp_result" / "spV200D0.75.txt"));

    std::ifstream inputStream{inputFile}, refStream{refFile};
    auto graph = AdjacencyMatrixGraph::createGraph(inputStream);

    ShortestPathResult result, refResult;
    readShortestPathResult(refStream, refResult);

    int sourceIndex;
    inputStream >> sourceIndex;

    REQUIRE(bellmanFord(*graph, sourceIndex, result));

    checkShortestPathResult(result, refResult);
}

TEST_CASE("Adjacency List Graph -- Bellman-Ford")
{
    auto [inputFile, refFile] = GENERATE(std::make_tuple(dataDirectoryPath / "graph" / "graphV10D0.5Negative.txt",
                                                         dataDirectoryPath / "sp_result" / "spV10D0.5Negative.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV30D0.25Negative.txt",
                                                         dataDirectoryPath / "sp_result" / "spV30D0.25Negative.txt"),
                                         std::make_tuple(dataDirectoryPath / "graph" / "graphV200D0.75.txt",
                                                         dataDirectoryPath / "sp_result" / "spV200D0.75.txt"));

    std::ifstream inputStream{inputFile}, refStream{refFile};
    auto graph = AdjacencyListGraph::createGraph(inputStream);

    ShortestPathResult result, refResult;
    readShortestPathResult(refStream, refResult);

    int sourceIndex;
    inputStream >> sourceIndex;

    REQUIRE(bellmanFord(*graph, sourceIndex, result));

    checkShortestPathResult(result, refResult);
}
